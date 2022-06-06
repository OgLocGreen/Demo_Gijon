#include <Arduino.h>
#include <Wire.h>

void SerialExecutionSpeed();
void A_Pin_Up();
void B_Pin_Up();

// defines pins numbers
#define pin_pwm__motor_output 6
#define pin_channelA 2
#define pin_channelB 3

// define variables
volatile int counter = 0;
int angle = 0;
int round_counter = 0;
int last_round = 0;
int last_5_counter = 0;
int median = 0;

int speedValue = 0;
int sensorValue  = 0;
int position = 0;


int test  = 0;
bool aState = 0;
bool aLastState = 0;

bool a_up = false;
bool b_up = false;
 

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Setup");

  // Setup Pins
  pinMode(pin_pwm__motor_output, OUTPUT);

  pinMode(pin_channelA, INPUT);  // channelA
  pinMode(pin_channelB, INPUT);  // channelB
  //pinMode(pin_test, INPUT);

  attachInterrupt(digitalPinToInterrupt(pin_channelA), A_Pin_Up, RISING);
  attachInterrupt(digitalPinToInterrupt(pin_channelB), B_Pin_Up, CHANGE);


  pinMode(A3,INPUT);  // Poti-Posit.
  //pinMode(A4,INPUT);  // Switch on/off motor
}

// the loop routine runs over and over again forever:
void loop() {

  sensorValue = analogRead(A0);
  speedValue = map(sensorValue, 0, 1023, 0, 254);

  position = analogRead(A3);
  position = map(position, 0, 1023, 0, 360);

  speedValue = 90;
  analogWrite(pin_pwm__motor_output, speedValue);

 
 /*
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
  */


  if(position == 0 && last_round >= 10){
    round_counter++;
    Serial.println("****** ");
    Serial.print("round_counter: ");
    Serial.println(round_counter);
    Serial.print("encoder_counter: ");
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
    analogWrite(pin_pwm__motor_output, speedValue);
    delay(10000);
    speedValue = 50;
    analogWrite(pin_pwm__motor_output, speedValue);
  }

  last_round = position;
  aLastState = aState;

  Serial.println(counter);
  //SerialExecutionSpeed();
}


void SerialExecutionSpeed(){
  unsigned int cycles = TCNT1;
  Serial.print("Cycles: ");
  Serial.println(cycles - 1);
  Serial.print("Microseconds: ");
  Serial.println((float)(cycles - 1) / 16);
}


void A_Pin_Up(){
  Serial.println("A_Pin_Up +++");
  counter++;
}

void B_Pin_Up(){
  Serial.println("B_Pin_Up ---");
  counter--;
}



