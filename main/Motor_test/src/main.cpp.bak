#include <Arduino.h>
#include <Wire.h>

void SerialExecutionSpeed();
void CanalA();
void CanalB();
void SPrint();
// defines pins numbers
#define pin_pwm__motor_output 6
#define PIN_D1 2 
#define PIN_D2 3 

// define variables
volatile int contador = 0;
volatile bool A, B, Ao, Bo = false;

int i = 0;
int angle = 0;

int speed_value = 0;
int postion_value = 0;

int tiempo2, tiempo2_old, tiempo2_new = 0;
int vel_enc = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.write("Setup");

  // Setup Pins
  pinMode(pin_pwm__motor_output, OUTPUT);
  pinMode(A0,INPUT);   // Input Speed_Value
  pinMode(A3,INPUT);  // Poti-Posit.


  pinMode(PIN_D2, INPUT_PULLUP);  // channelA
  pinMode(PIN_D1, INPUT_PULLUP);  // channelB
  //pinMode(pin_test, INPUT);

  A=digitalRead(PIN_D2);
  B=digitalRead(PIN_D1);
  Ao=A;
  Bo=B;

  attachInterrupt(digitalPinToInterrupt(PIN_D1), CanalB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D2), CanalA, CHANGE);






  tiempo2_old = micros(); 
  //pinMode(A4,INPUT);  // Switch on/off motor
}

// the loop routine runs over and over again forever:
void loop() {
  speed_value = analogRead(A0);
  speed_value = map(speed_value, 0, 1023, 0, 254);

  postion_value = analogRead(A3);
  postion_value = map(postion_value, 0, 1023, 0, 360);

  analogWrite(pin_pwm__motor_output, speed_value);

  
  tiempo2_new = micros();            // Nuevo valor de los microsegundos del reloj.
   // Se controla el tiempo que ha pasado desde la anterior ejecución para calcular la velocidad del encoder    
    if (contador != 0){
      tiempo2 = tiempo2_new - tiempo2_old; // Calcula los microsegundos que han pasado.
                // "tiempo2" debería ser positivo salvo OVERFLOW de las variables que son "unsigned long"
                // (no se ha programado el tratamiento del OVERFLOWque se produciría cada 70horas aproximadamente)
      vel_enc=234375*contador/tiempo2;   //Cálculo de la velocidad por el contador del encoder cada n micros
                // N[pulsos/micros]*[1000000micros/1s]*[60s/1min]*[1vuelta/(8*32)pulsos]=N*1000000*60/(8*32)[vuelta/min]=N*234375[rpm]
      contador=0;     // Se pone a cero el contador de pulsos incrementado por las interrupciones de los canales A y B
      tiempo2_old=tiempo2_new;
    }
    else{  // Poner la velocidad del encoder a cero si no llegan pulsos en 0.25 segundos
      if ((tiempo2_new-tiempo2_old)>250000){
        vel_enc=0;   // La interrupción no se ha activado en ese tiempo
        tiempo2_old=tiempo2_new;
      }
   }

  if(i==10000){
    SPrint();
    i = 0;
  }
  i++;



  //SerialExecutionSpeed();
}


void SerialExecutionSpeed(){
  unsigned int cycles = TCNT1;
  Serial.print("Cycles: ");
  Serial.println(cycles - 1);
  Serial.print("Microseconds: ");
  Serial.println((float)(cycles - 1) / 16);
}


void CanalA(){                // Flanco en el canal A
//  A=!A;                     // estas dos líneas no funcionan bien ¿por qué?
//  contador=contador-A*B;    // sería un código más óptimo que el actual
  A=digitalRead(PIN_D2);      // Actualizar el estado correcto del pin del encoder
  if  (A==!Ao)  contador=contador-A*B;   // pulso en sentido CCW si A=B=1 cuando cambia A
  Ao=A;
}

void CanalB(){                // Flanco en el canal B,
//  B=!B;
//  contador=contador-A*B;
  B=digitalRead(PIN_D1); 
  if  (B==!Bo) contador=contador+A*B;   // pulso en sentido CW si A=B=1 cuando cambia B
  Bo=B;
}

void SPrint(){
  Serial.print("Input Velocidad: ");
  Serial.println(speed_value);
  Serial.print("Velocidad: ");
  Serial.println(vel_enc);
  Serial.print("Position: ");
  Serial.println(postion_value);
  Serial.println("----------------------");
}