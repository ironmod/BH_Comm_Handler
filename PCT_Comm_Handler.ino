#include <Wire.h>
#include <AltSoftSerial.h>
#include "Maxbotix.h"
Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV, Maxbotix::BEST, 9);

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Arduino Uno        9         8         10

AltSoftSerial MECANUM_UART;
int LD_RED      = 10; //Red LED
int LD_BLUE     = 11; //Blue LED
int BEDINI      = 12;
int PERIPH_I2C_ID  = 4; 
int PIR         = 2;  //PIR Motion Sensor
int i=0;
int ARRAY_VAL[] = {1, 2, 3, 4, 1, 0, 0, 1, 3, 4, 5};
  int range = 0;
  int range_old = 0;
  int PIR_STATE = 0;
// Variables will change:
boolean BEDINI_STATE = LOW;             // state of bedini motor
long previousMillis = 0;        // will store last time data transmitted
long interval = 300000;           // interval at which to keep bedini off
long sampleMillis = 0;
long sampling_rate = 100;

/************************************************************************/
/*            Initial Setup Routine                                     */
/************************************************************************/
void setup() {
  Serial.begin(57600);
  //Serial.println("DRONENT BIT Controller"); //take this out
  MECANUM_UART.begin(57600);

  Wire.begin(); // join i2c bus (address optional for master)

  pinMode(PIR,          INPUT);
  pinMode(LD_BLUE,      OUTPUT);
  pinMode(LD_RED,       OUTPUT);
  pinMode(BEDINI,       OUTPUT);
  digitalWrite(LD_BLUE, HIGH);
  digitalWrite(LD_RED,  LOW);
  digitalWrite(BEDINI,  LOW);
  // Set the delay between AD readings to 10ms
  rangeSensorAD.setADSampleDelay(10);
}

/************************************************************************/
/*            Infinite Loop/Run Mode                                    */
/************************************************************************/
void loop() {
  char c;
  int val;

  if (Serial.available()) {
    c = Serial.read();
    switch(c)
    {

      //To Periph Controller
      case 'P':
          Wire.beginTransmission(PERIPH_I2C_ID); // transmit to device #4
          Wire.println(Serial.readStringUntil('\r'));      
          Wire.endTransmission();    // stop transmitting
      break;

      //To Motor Controller
      case 'M':
        MECANUM_UART.println(Serial.readStringUntil('\r'));
        if(BEDINI_STATE == LOW)
        {
            BEDINI_STATE = HIGH;
            digitalWrite(BEDINI, HIGH);
            digitalWrite(LD_RED, HIGH);
            Serial.write(8);
            Serial.write(1);
        }
      break;

	 }//end fsm
  }//end Serial available


  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval)
  {
      previousMillis = currentMillis;   
      digitalWrite(BEDINI, LOW);
      digitalWrite(LD_RED, LOW);

      Serial.write(8);
      Serial.write(0);

      BEDINI_STATE = LOW;
  }

if(currentMillis - sampleMillis > sampling_rate)
{
  sampleMillis = currentMillis;
if(!Serial.available())
{
    //PIR Motion Sensor Read
    if(digitalRead(PIR) == HIGH && PIR_STATE == LOW)
    {
      PIR_STATE = HIGH;
      digitalWrite(LD_BLUE, HIGH);
      Serial.write(7);
      Serial.write(0);
    }

    else if (PIR_STATE == HIGH && digitalRead(PIR) == LOW)
    {
      PIR_STATE = LOW;
      digitalWrite(LD_BLUE, LOW);
      Serial.write(7);
      Serial.write(1);
    }

//Range Finder Reads
      range = (rangeSensorAD.getRange());
      if(range_old != range)
      {
        Serial.write(6);
       Serial.write(range);
      }

      range_old = range;
  }//end range
}
}//end loop


