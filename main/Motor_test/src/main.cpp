#include <Arduino.h>
#include <Wire.h>


#define MD22ADDRESS 0xBC
#define SOFTREG 0x07
#define ACCELLREG 0x03




void SerialCommunication(int sensorValue,int channelA, int channelB, int speedValue);
void setMode();
void getSoftware();


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Setup");
  Wire.begin();

  //getSoftware();                                              // Function that gets and prints software revision to screen
  //setMode();                                                  // Function that sets mode to 2 and sets acceleration

}

// the loop routine runs over and over again forever:
void loop() {
  int speedValue = 0;
  int sensorValue = analogRead(A0);
  int channelA = analogRead(A1);
  int channelB = analogRead(A2);
  
  speedValue = map(sensorValue, 0, 1023, 0, 255);
  speedValue = 128;

  Wire.beginTransmission(MD22ADDRESS);                // Set first motor to speed 0
  Wire.write(0x01);
  Wire.write(0);
  Wire.endTransmission();
  delay(3000);                              

  Wire.beginTransmission(0xBC);
  Wire.write(0x01);
  Wire.write(speedValue);
  Wire.endTransmission();
  delay(3000); 

  SerialCommunication(sensorValue, channelA, channelB, speedValue);
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

void getSoftware(){                                   // Reads and displays the software version of MD22
  Wire.beginTransmission(MD22ADDRESS);                // Calles software register
  Wire.write(SOFTREG);
  Wire.endTransmission();
  
  Wire.requestFrom(MD22ADDRESS, 1);                   // Requests one byte
  while(Wire.available() < 1);                        // Wait for it to arrive
  int software = Wire.read();                         // Get byte
  Serial.println("MD22 Example   V:");
  Serial.println(software, DEC);                        // Print byte to LCD03
}

void setMode(){
  Wire.beginTransmission(MD22ADDRESS);                // Set a value of 255 to the acceleration register
  Wire.write(ACCELLREG);
  Wire.write(255);
  Wire.endTransmission();
}

