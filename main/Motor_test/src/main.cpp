#include <Arduino.h>
#include <Wire.h>


void SerialCommunication(int sensorValue,int channelA, int channelB, int speedValue);

#define pwm_pin 3

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Setup");

    pinMode(pwm_pin, OUTPUT);

}

// the loop routine runs over and over again forever:
void loop() {
  int speedValue = 0;
  int sensorValue = analogRead(A0);
  int channelA = analogRead(A1);
  int channelB = analogRead(A2);
  
  speedValue = map(sensorValue, 0, 1023, 0, 255);
  analogWrite(pwm_pin, speedValue);

                  
  SerialCommunication(sensorValue, channelA, channelB, speedValue);
  delay(1000);
}

// micosecond to mescure the ofset von the a nad b channel to mesure the speed

void SerialCommunication(int sensorValue, int channelA, int channelB, int speedValue){
  //Serial.println("sensorValue");
  //Serial.println(sensorValue);

  Serial.println("channelA");
  Serial.println(channelA);

  Serial.println("channelB");
  Serial.println(channelB);

  //Serial.println("speedValue");
  //Serial.println(speedValue);
}






