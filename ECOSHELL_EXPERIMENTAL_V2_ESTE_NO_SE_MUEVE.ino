#define THROTTLE_PIN A3       // Throttle pin
#define THROTTLE_LOW 290      // These LOW and HIGH values are used to scale the ADC reading. More on this below
#define THROTTLE_HIGH 580

#define HALL_1_PIN A0
#define HALL_2_PIN A1
#define HALL_3_PIN A2

#define AHpin 6             // Pins from the Teensy to the gate drivers. AH = A high, etc
#define ALpin 11
#define BHpin 5
#define BLpin 10
#define CHpin 3
#define CLpin 9

#define HALL_OVERSAMPLE 4     // Hall oversampling count. More on this in the getHalls() function

int estado_motor,Ciclo,Estado_Pasado;
int EstadoAnt = 10;
uint8_t hallToMotor[8] = {255, 1, 3, 2, 6, 4, 5, 255};

void setup() {                // The setup function is called ONCE on boot-up
  Serial.begin(115200);

  // Pins D5 and D6 - 7.8 kHz
  TCCR0B = 0b00000010; // x8
  TCCR0A = 0b00000011; // fast pwm
  // Pins D9 and D10 - 7.8 kHz
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00001010; // x8 fast pwm
  // Pins D3 and D11 - 8 kHz
  TCCR2B = 0b00000010; // x8
  TCCR2A = 0b00000011; // fast pwm


  analogWrite(ALpin, LOW);          //escribe los estados recibidos en el pin de cada fase
  analogWrite(BLpin, LOW);
  analogWrite(CLpin, LOW);
  digitalWrite(AHpin, LOW);
  digitalWrite(BHpin, LOW);
  digitalWrite(CHpin, LOW);

  pinMode(AHpin, OUTPUT);    // Set all PWM pins as output
  pinMode(ALpin, OUTPUT);
  pinMode(BHpin, OUTPUT);
  pinMode(BLpin, OUTPUT);
  pinMode(CHpin, OUTPUT);
  pinMode(CLpin, OUTPUT);

  //analogWriteFrequency(AHpin, 8000); // Set the PWM frequency. Since all pins are on the same timer, this sets PWM freq for all

  pinMode(HALL_1_PIN, INPUT_PULLUP);         // Set the hall pins as input
  pinMode(HALL_2_PIN, INPUT_PULLUP);
  pinMode(HALL_3_PIN, INPUT_PULLUP);

  pinMode(THROTTLE_PIN, INPUT);
  
}

void loop() {                         // The loop function is called repeatedly, once setup() is done
  
  uint8_t throttle = readThrottle();  // readThrottle() is slow. So do the more important things 200 times more often
  //Serial.println(throttle);
  for(uint8_t i = 0; i < 150; i++)
  {  
    uint8_t hall = getHalls();              // Read from the hall sensors
    //Serial.println(hall);
    //delay(1000);
    //uint8_t motorState = hallToMotor[hall]; // Convert from hall values (from 1 to 6) to motor state values (from 0 to 5) in the correct order. This line is magic
    writePWM(hall, throttle);         // Actually command the transistors to switch into specified sequence and PWM value
  }
}


  

void writePWM(uint8_t Estado, uint8_t CicloPre){
  Ciclo = CicloPre;
  //Serial.println(Estado);
  //Estado = 5;
  if(Estado != EstadoAnt){
   // EstadoAnt = Estado;
    switch(Estado){
      case 1:   //1 0 0
        analogWrite(ALpin, 0);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, 0);
        analogWrite(CLpin, Ciclo);
        digitalWrite(AHpin, 0);
        digitalWrite(BHpin, 1);
        digitalWrite(CHpin, 1);
        break;
      case 2:  //0 1 0
        analogWrite(ALpin, Ciclo);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, 0);
        analogWrite(CLpin, 0);
        digitalWrite(AHpin, 1);
        digitalWrite(BHpin, 0);
        digitalWrite(CHpin, 1);
        break;
      case 3:  //1 1 0
        analogWrite(ALpin, 0);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, 0);
        analogWrite(CLpin, Ciclo);
        digitalWrite(AHpin, 1);
        digitalWrite(BHpin, 0);
        digitalWrite(CHpin, 1);
        break;
      case 4:  //0 0 1
        analogWrite(ALpin, 0);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, Ciclo);
        analogWrite(CLpin, 0);
        digitalWrite(AHpin, 1);
        digitalWrite(BHpin, 1);
        digitalWrite(CHpin, 0);
        break;
      case 5:  // 1 0 1
        analogWrite(ALpin, 0);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, Ciclo);
        analogWrite(CLpin, 0);
        digitalWrite(AHpin, 0);
        digitalWrite(BHpin, 1);
        digitalWrite(CHpin, 1);
        break;
      case 6:  // 0 1 1
        analogWrite(ALpin, Ciclo);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, 0);
        analogWrite(CLpin, 0);
        digitalWrite(AHpin, 1);
        digitalWrite(BHpin, 1);
        digitalWrite(CHpin, 0);
        break;
      default:
        analogWrite(ALpin, 0);          //escribe los estados recibidos en el pin de cada fase
        analogWrite(BLpin, 0);
        analogWrite(CLpin, 0);
        digitalWrite(AHpin, 0);
        digitalWrite(BHpin, 0);
        digitalWrite(CHpin, 0);
        break;
    }
  }
}

/* Read hall sensors WITH oversamping. This is required, as the hall sensor readings are often noisy.
 * This function reads the sensors multiple times (defined by HALL_OVERSAMPLE) and only sets the output
 * to a 1 if a majority of the readings are 1. This really helps reject noise. If the motor starts "cogging" or "skipping"
 * at low speed and high torque, try increasing the HALL_OVERSAMPLE value
 * 
 * Outputs a number, with the last 3 binary digits corresponding to hall readings. Thus 0 to 7, or 1 to 6 in normal operation
 */

uint8_t getHalls(){
  /*
  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) // Read all the hall pins repeatedly, tally results 
  {
    hallCounts[0] = digitalRead(HALL_1_PIN);
    hallCounts[1] = digitalRead(HALL_2_PIN);
    hallCounts[2] = digitalRead(HALL_3_PIN);
  }
  
  uint8_t hall = 0;

  if((hallCounts[0] == 1 && hallCounts[1] == 0) && hallCounts[2] == 0){
    hall = 1;
  }
  else if ((hallCounts[0] == 1 && hallCounts[1] == 1) && hallCounts[2] == 0){
    hall = 2;
  }
  else if ((hallCounts[0] == 0 && hallCounts[1] == 1) && hallCounts[2] == 0){
    hall = 3;
  }
  else if ((hallCounts[0] == 0 && hallCounts[1] == 1) && hallCounts[2] == 1){
    hall = 4;
  }
  else if ((hallCounts[0] == 0 && hallCounts[1] == 0) && hallCounts[2] == 1){
    hall = 5;
  }
  else if ((hallCounts[0] == 1 && hallCounts[1] == 0) && hallCounts[2] == 1){
    hall = 6;
  }
  else hall = 255;
  */

  
  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) // Read all the hall pins repeatedly, tally results 
  {
    hallCounts[0] += digitalRead(HALL_1_PIN);
    hallCounts[1] += digitalRead(HALL_2_PIN);
    hallCounts[2] += digitalRead(HALL_3_PIN);
  }

  uint8_t hall = 0;
  
  if (hallCounts[0] >= HALL_OVERSAMPLE / 2)     // If votes >= threshold, call that a 1
    hall |= (1<<0);                             // Store a 1 in the 0th bit
  if (hallCounts[1] >= HALL_OVERSAMPLE / 2)
    hall |= (1<<1);                             // Store a 1 in the 1st bit
  if (hallCounts[2] >= HALL_OVERSAMPLE / 2)
    hall |= (1<<2);                             // Store a 1 in the 2nd bit
  //Serial.println(hall);
  
  return hall;                            // Just to make sure we didn't do anything stupid, set the maximum output value to 7
}

/* Read the throttle value from the ADC. Because our ADC can read from 0v-3.3v, but the throttle doesn't output over this whole range,
 * scale the throttle reading to take up the full range of 0-255
 */

uint8_t readThrottle(){
  int32_t adc = analogRead(THROTTLE_PIN); // Note, analogRead can be slow!
  adc = (adc - THROTTLE_LOW) << 8;
  adc = adc / (THROTTLE_HIGH - THROTTLE_LOW);

  if (adc > 255) // Bound the output between 0 and 255
    return 255;

  if (adc < 0)
    return 0;
  
  return adc;
}