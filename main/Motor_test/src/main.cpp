#include <Arduino.h>
#include <Wire.h>

void SerialCommunication(int sensorValue,int channelA, int channelB, int speedValue);

// defines pins numbers
#define pwm_pin 3

// define variables
int counter = 0;
int angle = 0; 
bool channelA = 0;
bool channelB = 0;

bool aState = 0;
bool aLastState = 0;
 

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Setup");

  // Setup Pins
  pinMode(pwm_pin, OUTPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  int speedValue = 0;
  int sensorValue = analogRead(A0);

  speedValue = map(sensorValue, 0, 1023, 0, 255);
  analogWrite(pwm_pin, speedValue);

  // Decoder
  channelA = digitalRead(A1);
  channelB = digitalRead(A2);
 
  aState = digitalRead(A1);
  
  if(aState != aLastState){     
    if(digitalRead(A2) != aState) { 
      counter ++;
      angle ++;
      if(angle >= 360){
        angle = 0;
      }
    }
    else{
      counter--;
      angle --;
      if(angle <= 0){
        angle = 360;
      }
    }
    if(counter >=30 || counter <= -30)
    {
      counter = 0;
    } 
    
    Serial.println("Position: ");
    Serial.print(int(angle));
    Serial.println(" deg");


    Serial.println("counter: ");
    Serial.println(int(counter));
  }
  aLastState = aState;
            


  //SerialCommunication(sensorValue, channelA, channelB, speedValue);
  //delay(1000);
}

// micosecond to mescure the ofset von the a nad b channel to mesure the speed

void SerialCommunication(int sensorValue, int channelA, int channelB, int speedValue){
  Serial.println("sensorValue");
  Serial.println(sensorValue);

  Serial.println("channelA");
  Serial.println(channelA);
  Serial.println("channelB");
  Serial.println(channelB);

  Serial.println("speedValue");
  Serial.println(speedValue);
}