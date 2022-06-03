#include <Arduino.h>
#include <Wire.h>

void SerialCommunication(int sensorValue,int channelA, int channelB, int speedValue);

// defines pins numbers
#define pwm_pin_motor_output 3
#define pwm_pin_speed_from_enocder 5

// define variables
int counter = 0;
int angle = 0;
int round_counter = 0;
int last_round = 0;
int last_5_counter = 0;
int median = 0;

int speedValue = 0;
int sensorValue  = 0;
int position = 0;


int test  = 0;

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
  pinMode(pwm_pin_motor_output, OUTPUT);
  pinMode(pwm_pin_speed_from_enocder, OUTPUT);

  pinMode(A1,INPUT);  // channelA
  pinMode(A2,INPUT);  // channelB
  pinMode(A2,INPUT);  // Poti-Posit.
  pinMode(A4,INPUT);  // Switch on/off motor
}

// the loop routine runs over and over again forever:
void loop() {

  sensorValue = analogRead(A0);
  test = analogRead(A4);
  
  position = analogRead(A3);
  position = map(position, 0, 1023, 0, 360);

  if(test >= 1000){
    //Just a testswitch
  }


  speedValue = 110;
  analogWrite(pwm_pin_motor_output, speedValue);

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
  }


  if(position == 0 && last_round >= 10){
    round_counter++;
    Serial.println("****** ");
    Serial.print("round_counter: ");
    Serial.println(round_counter);
    Serial.print("encoder_ounter: ");
    Serial.println(counter);
    Serial.println("****** ");
  }

  if (round_counter == 5 && last_round >= 10){
    Serial.println("-----");
    Serial.print("last_5_counter: ");
    Serial.println(last_5_counter);
    Serial.print("median: ");
    Serial.println(median);
    Serial.println("-----");
    last_5_counter = counter;
    median += counter;
    median = median/2;
    counter = 0;
    round_counter = 0;
    speedValue = 127;
    analogWrite(pwm_pin_motor_output, speedValue);
    delay(10000);
    speedValue = 50;
    analogWrite(pwm_pin_motor_output, speedValue);
  }

 /*
  if(position >= 0 && position <= 5 && last_round >= 10){
    speedValue = 127;
    analogWrite(pwm_pin_motor_output, speedValue);
    Serial.println("****** ");
    Serial.println("counter: ");
    Serial.println(counter);
    Serial.println("****** ");
    delay(10000);
    counter = 0;
   }

  */
  last_round = position;
  //Serial.println(position);
  aLastState = aState;

  //SerialCommunication(sensorValue, channelA, channelB, speedValue);
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