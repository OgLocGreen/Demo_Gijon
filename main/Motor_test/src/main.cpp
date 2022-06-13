//////////////////////////////////////////////////////////////////
//  ##########################################################  //
//  ################### controlArduino #######################  //
//  ##########################################################  //
//                                                              //
//   Versión 4.3 20/05/2022 10:00                               //
//                                                              //
//  Programa para el control de Velocidad y Posición de los     //
//  módulos de Feedback a través del interfaz gráfico en QT     //
//                                                              //
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//   Versión para los Arduino Micro. Para los Arduino MEGA      //
//   de Mecatrónica con la placa de adaptación de la Taco       //
//   con R2=56K R3=R4=100K, se deben modificar los valores de   //
//   las variables RPM_max, V_max y V_0 en conversionAD() y     //
//   descomentar la línea que introduce un retardo de un        //
//   milisegundo en leerSerie().                                //
//                                                              //
//  Para el Arduino MEGA, hay varias partes del código          //
//  que no se utilizan porque no están cableadas las señales    //
//  las señales digitales del encoder ni del                    //
//  del disco Gray. Por lo tanto, las funciones para las        //
//  interrupciones de los Canales A y B y la que calcula la     //
//  posición mediante el código Gray no se utilizan.            //
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//                                                              //
//     DENOMINACIÓN DE PINES                                    //
//                                                              //
//////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>

void envioDatos();
void envioestado();
void regulador();
void leerSerie();
unsigned int gray2binary(unsigned int gray);
void conversionDA();
void conversionAD();
void CanalA();
void CanalB();

#define PIN_posicion A3   //Pin para el potenciómetro de posición
#define PIN_referencia A0 //Pin para el potenciómetro de referencia
// #define PIN_PWM 3         //Pin para la salida de la señal PWM. La frecuencia de PWM 3 y 11 controlada con Timer0 
#define PIN_PWM 6         //Pin para la salida de la señal PWM para poder usar el 2 y el 3 para las interrupciones del encoder

//#define PIN_D1 0          //Pin para el bit D1 del encoder Pin 0 interrupción RX. Entra en conflicto con el USB para interrupciones
//#define PIN_D2 1          //Pin para el bit D2 del encoder Pin 1 interrupción TX. Entra en conflicto con el USB para interrupciones
#define PIN_D1 2          //Pin para el bit D1 del encoder. Usar estos porque el Pin 0 y el 1 del puerto serie tienen conflicto con el USB
#define PIN_D2 3          //Pin para el bit D2 del encoder. 


//////////////////////////////////////////////////////////////////
//                                                              //
//     DECLARACIÓN DE VARIABLES                                 //
//                                                              //
//////////////////////////////////////////////////////////////////

//Variables para temporización
unsigned long tiempo_old;    // Variable que guarda el último valor de los milisegundos del reloj del Arduino.
unsigned long tiempo_new;    // Variable que guarda el nuevo valor de los milisegundos del reloj del Arduino.
unsigned long tiempo;        // Variable en la que se calculan los microsegundos que han pasado.
unsigned long tiempo2_old;    // Variable que guarda el último valor de los microsegundos del reloj del Arduino.
unsigned long tiempo2_new;    // Variable que guarda el nuevo valor de los microsegundos del reloj del Arduino.
float tiempo2;        // Variable en la que se calculan los microsegundos que han pasado.

// Variables para medida digital de velocidad y posición
// *** Las variables usadas en interrupciones se recomienda declararlas como volátiles ***
volatile int contador; // Contador de pulsos + -> CW, - -> CCW
volatile bool A, B, Ao, Bo;  // Estado de los bits del encoder
float vel_enc=0;         // Velocidad medida por el encoder
float pos_gray=0;        // Posición medida por el disco de Gray

//VARIABLES GLOBALES
int posicion;           // Variable para el valor del potenciómetro de posición (int, 0...1023)
int referencia;         // Variable para el valor del potenciómetro de referencia (int, 0...1023)
int velocidad;          // Variable para el valor del tacómetro (int, 0...1023)
unsigned int binario;   // Variable para transmitir los bits de los sensores digitales
unsigned int gray;      // Variable para el código binario de Gray
bool bit1;               // Bit D1 del disco del Encoder
bool bit2;               // Bit D2 del disco del Encoder
bool bit3;               // Bit D3 del disco de Gray
bool bit4;               // Bit D4 del disco de Gray
bool bit5;               // Bit D5 del disco de Gray
bool bit6;               // Bit D6 del disco de Gray
bool bit7;               // Bit D7 del disco de Gray
bool bit8;               // Bit D8 del disco de Gray
bool bit9;               // Bit D9 del disco de Gray (índice de vuelta)

float ref=0;        // Valor de la referencia en voltios
float pos=0;        // Valor de la posición en grados +-180º
float pos1=0;
float pos2=0;
float ref_pos=0;    //Referencia posición en grados +-180º
float vel=0;        // Valor de la velocidad en rpm del eje de salida (eje de baja) 
float vel1=0;
float ref_vel=0;    //Referencia velocidad en rpm del eje de salida (eje de baja)
int duty=127;       // El Duty va de 0 (0%) a 255 (100%)
float pwm;          // Valor del Duty en un float
float duty_0 = 127.5;   //Duty 50%, es decir, motor parado
float err_v[3];     //Error entre la velocidad actual y la referencia
float err_p[3];     //Error entre la posición actual y la referencia
float uk[3];        //Variables de salida del regulador PID

float W=1;          //Frecuencia para el generador de senoides en Hercios
float Amp=1;          //Amplitud de la senoide en voltios

// VARIABLES DE LAS CARACTERÍSTICAS DE LA TACODINAMO
float    RPM_max;         // Máximas RPM CW del motor       
float    V_max;          // Tensión de la tacodinamo a las máximas RPM
float    V_0;           // Tensión de la tacodinamo a 0 RPM

//VARIABLES PARA EL REGULADOR
float Tm=20;      //Tiempo de muestreo en milisegundos
float T;          //Tiempo real en ms desde la última ejecución de algoritmo de control()
//PID Velocidad
float kp_v = 0.1;     //Kp del PID
float ki_v = 4.0;     //Ki del PID
float kd_v = 0.0;     //Kd del PID
float integral_v=0;   //Integral del error
float derivada_v=0;   //Derivada del error
//PID Posición
float kp_p = 0.5;     //Kp del PID
float ki_p = 0.0;     //Ki del PID
float kd_p = 0.0;     //Kd del PID
float integral_p=0;   //Integral del error
float derivada_p=0;   //Derivada del error

//VARIABLES PARA RECIBIR DATOS SERIE
const byte buffSize = 32; //Tamaño de la cadena de caracteres
char input[buffSize];     //Cadena de caracteres recibida
byte maxChars = 31;       //31 caracteres de máximo
int inputInt = 0;         //Valor Int recibido
float inputFloat = 0.0;   //Valor Float recibido
char inputCommand[12];    //Texto recibido

//VARIABLES PARA ELEGIR MODO DE CONTROL, REFERENCIA, MODELO, etc.
int modocontrol=4;        // 1 vel lazo abierto, 2 vel lazo cerrado, 3 pos, 4 pos cascada
bool Pot_PC=0;            //Referencai del Potenciómetro=0 o del PC=1
bool modelo=1;            //Modelo de unidad Feedback: 0->electrónica antigua, 1->electrónica modificada
bool sim=0;               //Modo NORMAL=0, Modo SIMULACION=1
bool fb_e=0;              //Realimentación de velocidad por taco=0 o por encoder=1
bool fb_g=0;              //Realimentación de posición por potenciómetro=0 o por disco Gray=1
bool senoide=0;           //Generador de funciones 0->desactivado, 1->activado



void setup() {
//////////////////////////////////////////////////////////////////
//  ##########################################################  //
//  #####################  SETUP() ###########################  //
//  ##########################################################  //
//////////////////////////////////////////////////////////////////

  Serial.begin(115200);  // Velocidad en bits por segundo del puerto serie USB. Resto de parámetros por defecto

  //Iniciar a cero las tablas
  err_v[0]=0;
  err_v[1]=0;
  err_v[2]=0;
  err_p[0]=0;
  err_p[1]=0;
  err_p[2]=0;
  uk[0]=0;
  uk[1]=0;
  uk[2]=0;

  //Modo entrada/salida de cada PIN
  pinMode(PIN_posicion, INPUT);
  pinMode(PIN_referencia, INPUT);
  pinMode(PIN_PWM, OUTPUT);
  
//  pinMode(PIN_D1, INPUT);  // Si se declaran simplemente como INPUT no funcionan con el encoder !!!
//  pinMode(PIN_D2, INPUT);  // Si se declaran simplemente como INPUT no funcionan con el encoder !!!
  pinMode(PIN_D1, INPUT_PULLUP);  // Para usarlos en interrupciones: INPUT_PULLUP
  pinMode(PIN_D2, INPUT_PULLUP);  // Incluso si no se usan interrupciones es necesario!!!
                // Si se usan interrupciones hay que declarar el PIN asociado con las dos siguientes líneas  
  attachInterrupt(digitalPinToInterrupt(PIN_D1),CanalB,CHANGE); // Función que atiende la interrupción CanalB()
  attachInterrupt(digitalPinToInterrupt(PIN_D2),CanalA,CHANGE); // Función que atiende la interrupción CanalA()

  A=digitalRead(PIN_D2);
  B=digitalRead(PIN_D1);
  Ao=A;
  Bo=B;

  tiempo_old = millis();      // Inicia el primer valor de los milisegundos del reloj del Arduino.
  tiempo2_old = micros();      // Inicia el primer valor de los microsegundos del reloj del Arduino.

  // FRECUENCIA DEL PWM
  // SOLO PARA Arduino MICRO con ATMega32U4. Ver para otros Arduinos y Procesadores
  // AHORA SE HA CAMBIADO EL PWM AL PIN 13 PARA USAR EL 2 Y EL 3 PARA LAS INTERRUPCIONES DEL ENCODER
  
  // TCCR0B = TCCR0B & 0b11111000 | 0x03;  
  
  //Frecuencia del PWM para pin 3 y 11: Timer0
  // 0x01 62500 Hz   16MHz/(1*256)    (16MHz system Clock)
  // 0x02  7812 Hz   16MHz/(8*256)
  // 0x03   976 Hz   16MHz/(64*256)   (por defecto)
  // 0x04   244 Hz   16MHz/(256*256)
  // 0x05    61 Hz   16MHz/(1024*256)
  //
  // Nota: -La electrónica de Feedback recomienda una frecuencia de 300Hz (244Hz es bastante ruidosa).
  //        Funciona mejor a 7812Hz o 976Hz a pesar de emitir un pitido molesto. 
  //       -El controlador de motores MD03 solicita una frecuencia de 20KHz o más. 
  //        Sin embargo funciona correctamente con el modo por defecto (976Hz) y no produce ruido.
  //       -El controlador de motores MD22 no especifica que soporte señal PWM.
  //        Funciona correctamente con el modo por defecto pero es conveniente poner un filtro RC para evitar el ruido.
  //
}



void loop() {
//////////////////////////////////////////////////////////////////
//  ##########################################################  //
//  ######################  LOOP() ###########################  //
//  ##########################################################  //
//////////////////////////////////////////////////////////////////

  //Bucle de ejecución continua
  leerSerie();     // Comprobar si han llegado órdenes por el puerto serie

  tiempo2_new = micros();            // Nuevo valor de los microsegundos del reloj.
   // Se controla el tiempo que ha pasado desde la anterior ejecución para calcular la velocidad del encoder    
   if (contador != 0)      
   {
     tiempo2 = tiempo2_new - tiempo2_old; // Calcula los microsegundos que han pasado.
                // "tiempo2" debería ser positivo salvo OVERFLOW de las variables que son "unsigned long"
                // (no se ha programado el tratamiento del OVERFLOWque se produciría cada 70horas aproximadamente)
     vel_enc=234375*contador/tiempo2;   //Cálculo de la velocidad por el contador del encoder cada n micros
                // N[pulsos/micros]*[1000000micros/1s]*[60s/1min]*[1vuelta/(8*32)pulsos]=N*1000000*60/(8*32)[vuelta/min]=N*234375[rpm]
     contador=0;     // Se pone a cero el contador de pulsos incrementado por las interrupciones de los canales A y B
     tiempo2_old=tiempo2_new;
   }
   else
   {  // Poner la velocidad del encoder a cero si no llegan pulsos en 0.25 segundos
      if ((tiempo2_new-tiempo2_old)>250000)
      {
        vel_enc=0;   // La interrupción no se ha activado en ese tiempo
        tiempo2_old=tiempo2_new;
      }
   }   

  // Se controla el tiempo que ha pasado desde la anterior ejecución del "control()" para ejecutar el 
  // algoritmo de control cada Tm ms aproximadamente y a ajustar el "T" de cálculo del algoritmo de control    
  tiempo_new = millis();            // Se mira el nuevo valor de los milisegundos del reloj.
  tiempo = tiempo_new - tiempo_old; // Se calculan los milisegundos que han pasado.
  if (tiempo > Tm-1)                // "tiempo" debería ser positivo salvo OVERFLOW de las variables que son "long int"
    {                               // (no se ha programado el tratamiento del OVERFLOW  que se produciría cada 5 días aproximadamente)
    T=tiempo;         //Se ajusta el tiempo del cálculo del PID al tiempo transcurrido en ms que, en principio, debería ser Tm ms
    conversionAD();                 // Lectura de valores de sensores y referencia
    regulador();                    // Ejecución del algoritmo de control que proceda
    conversionDA();                 // Envío de la señal de control a la salida PWM
    tiempo_old=tiempo_new;
  }
}



void leerSerie() {
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTO leerSerie()                                //
//                                                              //
//////////////////////////////////////////////////////////////////
  /* Esta función lee los caracteres recibidos por el puerto serie (USB)
     Espera tres cadenas de caracteres separados por comas,
     los separa y los convierte al formato correcto de cada valor:
     una cadena de caracteres, un valor entero y un valor real.
     Todos serán guardados en diferentes variables. */


  input[0] = 0;
  maxChars = buffSize - 1;
  byte charCont = 0;  
  byte i = 0;        

  //Recibe los datos carácter a carácter y lo almacena en "input"
  if (Serial.available() > 0) {
    // delay(1);   // Para dar tiempo a que llegue el comando completo 1ms
                  // (descomentar para el Arduino MEGA, en el MICRO no hace falta)
    while (Serial.available() > 0) { 
      if (i > maxChars - 1) {
        i = maxChars;
      } 
      input[i] = Serial.read();
      i ++;        
      charCont ++;
    }
    if (i > maxChars) { 
      i = maxChars;
    }
    input[i] = 0; 

  // DEVOLVER LO QUE ACABA DE RECIBIR PARA DEPURACION    
  // Serial.print(input);
  // Serial.print("\n");
  // Serial.flush();
  
  //Ahora se debe dividir la cadena recibida en partes
  char * partirString;  //Este puntero indicará donde cortar la cadena
  
  partirString = strtok(input,",");       //Se separa la primera parte - el texto
  strcpy(inputCommand, partirString);   //Lo copia en inputCommand
  
  partirString = strtok(NULL, ",");       //Se separa la segunda parte - el Int
  inputInt = atoi(partirString);          //Lo convierte a int
  
  partirString = strtok(NULL, ",");       //Se separa la segunda parte - el Float
  inputFloat = atof(partirString);        //Lo convierte a float

  //////////////////////////////////////////////////////////////////
  //     Interpretación y ejecución del comando recibido          //
  //////////////////////////////////////////////////////////////////
  /* Interpretación de los datos recibidos y operación según el comando recibido desde el PC.

     Comandos aceptados: Todos los comandos pueden ir seguidos de un "int" y un "float" separados
                         por comas:

                         cmd,entero,real,
                         
     Comandos sin parámetros:                 
     dat - Petición de datos desde el PC. Se le enviarán los valores de las variables.
     rst - Se pone el programa con los parámetros por defecto. Petición desde el PC al iniciar o cerrar la conexión.
     est - Petición desde el PC del estado de los modos de control, referencia, realimentacion, etc.
     
     Comandos con parámetro entero binario (0<>1):
     mod - Selecciona el modelo de Feedback original=0, modificado=1
     sim - Funcionamiento en modo normal=0, simulación=1
     ppc - Selecciona la señal de referencia desde el potenciómetro de la unidad Feedback=0 o desde el PC=1
     fbv - Realimentación de velocidad por tacodinamo=0 o encoder=1
     fbp - Realimentación de posición por potenciometro=0 o disco Gray=1

     Comandos con parámetro entero:     
     con - Modo de control: vel lazo abierto=1, vel lazo cerrado=2, pos un lazo simple=3, pos en cascada=4
    
     Comandos con parámetros entero y real: 
     ref - Valor de la referencia en el valor "float" desde el PC. El "int" indica referencia de velocidad=0 o posición=1
     pid - Valores de las constantes de los reguladores PID en el valor "float". El "int" indica la constante a modificar:
           1=Kp del regulador de velocidad
           2=Ki del regulador de velocidad
           3=Kd del regulador de velocidad
           4=Kp del regulador de posición
           5=Ki del regulador de posición
           6=Kd del regulador de posición
      sin - Generador de referencias senoidales. Valor del "int":
           0=desactivar
           1=activar
           2=valor de W en hercios (el valor "float" indica la W frecuencia en hercios)
           3=valor de A en voltios (el valor "float" indica la A amplitud en voltios)
  */

  //PETICION DE DATOS
  if (strcmp (inputCommand,"dat") == 0){
    envioDatos();
  }
  //PETICION DE ESTADO PARA DEPURACION
  if (strcmp (inputCommand,"est") == 0){
    envioestado();
  }
  //INICIO O CIERRE DE COMUNICACIÓN
  //Paso a control de Posición con el Potenciómetro
  if (strcmp (inputCommand,"rst") == 0){
    Pot_PC=0;
    modocontrol=4;    // Control de posición con lazo en cascada
    sim=0;            //Quitar modo SIMULACION si está puesto
    senoide=0;        // Desactiva el generador de senoides
    fb_e=0;
    fb_g=0;
    kp_v=0.1;         //Valores por defecto para los reguladores
    ki_v=4;
    kd_v=0.0;
    kp_p=0.5;
    ki_p=0.0;
    kd_p=0.0;
    integral_v=0;
    integral_p=0;
  }
  //MODELO DE UNIDAD FEEDBACK ANTIGUO (0) O NUEVO (1)
  if (strcmp (inputCommand,"mod") == 0){
    modelo=inputInt;
  } 
  //FUNCIONAMIENTO NORMAL (0) O EN SIMULACIÓN (1)
  if (strcmp (inputCommand,"sim") == 0){
    sim=inputInt;
  } 
  //REFERENCIA POTENCIOMETRO (0) O PC (1)
  if (strcmp (inputCommand,"ppc") == 0){
    Pot_PC=inputInt;
    ref_pos=0;
    ref_vel=0;
    integral_v=0;
    integral_p=0;
  }
  //REALIMENTACIÓN DE VELOCIDAD POR TACODINAMO (0) O ENCODER (1)
  if (strcmp (inputCommand,"fbv") == 0){
    fb_e=inputInt;
  }
  //REALIMENTACIÓN DE POSICIÓN POR POTENCIÓMETRO (0) O GRAY (1)
  if (strcmp (inputCommand,"fbp") == 0){
    fb_g=inputInt;
  }
  //MODO DE CONTROL 
  if (strcmp (inputCommand,"con") == 0){
    if ((Pot_PC==1) & (modocontrol>2) & (inputInt<3)) ref_vel=0.25*ref_pos;  // Cambio de referencia de posición a referencia de velocidad enviada por el PC
    if ((Pot_PC==1) & (modocontrol<3) & (inputInt>2)) ref_pos=4*ref_vel;   // Cambio de referencia de velocidad a referencia de posición enviada por el PC
    modocontrol=inputInt;   // vel lazo abierto=1, vel lazo cerrado=2, pos un lazo simple=3, pos en cascada=4
    integral_v=0;
    integral_p=0;
  }
  //NUEVA REFERENCIA
  if (strcmp (inputCommand,"ref") == 0){
    if (inputInt) ref_pos=inputFloat;  // El entero indica si cambia la referencia de velocidad (0) o de posición (1)
    else ref_vel=inputFloat;           // Valor de la nueva referencia
  }
  //CAMBIO VALORES DE LOS REGULADORES PID
  if (strcmp (inputCommand,"pid") == 0){
    switch (inputInt) {        // El entero indica que constante se va a cambiar
        case 1:
          kp_v=inputFloat;     // Regulador PID de velocidad
          break;
        case 2:
          ki_v=inputFloat;
          break;
        case 3:
          kd_v=inputFloat;
          break;
        case 4:
          kp_p=inputFloat;    // Regulador PID de posición
          break;
        case 5:
          ki_p=inputFloat;
          break;
        case 6:
          kd_p=inputFloat;
          break;
        default:
          // if nothing else matches, do the default
          // default is optional
        break;
      }
    }
    
  //GENERADOR DE SENOIDES
  if (strcmp (inputCommand,"sin") == 0){
    switch (inputInt) {        // El entero indica que valor se va a cambiar
        case 0:
          senoide=0;     // Regulador PID de velocidad
          break;
        case 1:
          senoide=1;     // Regulador PID de velocidad
          break;
        case 2:
          W=inputFloat;
          break;
        case 3:
          Amp=inputFloat;
          break;
        default:
          // if nothing else matches, do the default
          // default is optional
        break;
      }
    }
  }  
}


void envioDatos(){
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTO envioDatos()                               //
//                                                              //
//////////////////////////////////////////////////////////////////
  /* Con esta función se envían los datos por el puerto serie USB hacia el PC.
     Los datos van en una cadena separados por comas:

     "referencia,ref_vel,pwm,velocidad,velocidad_enc,ref_pos,posición,posición_gray,error_v,int_v,der_v,error_p,int_p,der_p,binario"

     referencia - valor de referencia del Potenciómetro en voltios 
     ref_vel - valor de referencia del velocidad en rpm
     pwm - salida del regulador hacia el accionamiento del motor
     velocidad - velocidad del eje de baja en rpm
     velocidad_enc - velocidad calculada con el código digital del encoder   
     posición - posición en grados del eje de baja (-180º a +180º)
     posición_gray - posición calculada con el código digital de Gray
     error_v - error a la entrada del regulador de velocidad
     int_v - integral del error de velocidad
     der_v- derivada del error de velocidad
     error_p - error a la entrada del regulador de posición
     int_p - integral del error de posición
     der_p- derivada del error de posición
     binario- valor binario de las entradas digitales (Encoder,Gray,Bit de paso por vuelta)

     La función se ejecuta cada vez que se piden los datos desde el PC con el comando "dat"
  */


  //Enviar el dato de REFERENCIA DEL POTENCIÓMETRO en voltios
  Serial.print(ref);  
  Serial.print(",");
  //Enviar el dato de REFERENCIA DE VELOCIDAD
  Serial.print(ref_vel);                
  Serial.print(",");
  //Enviar el dato de Control del Motor (salida del PID)
  Serial.print(uk[0]);
  Serial.print(",");
  //Enviar el dato de VELOCIDAD
  Serial.print(vel);
  Serial.print(",");
  //Enviar datos digitales de velocidad encoder
  Serial.print(vel_enc);
  Serial.print(",");  
  //Enviar el dato de REFERENCIA de POSICION
  Serial.print(ref_pos);  
  Serial.print(",");
  //Enviar el dato de POSICIÓN
  Serial.print(pos);
  Serial.print(",");
  //Enviar datos digitales de Posición Gray
  Serial.print(pos_gray);
  Serial.print(",");
  //Enviar el dato de error de velocidad
    Serial.print(err_v[0]);
    Serial.print(",");
    //Enviar el dato de integral
    Serial.print(integral_v);
    Serial.print(",");
    //Enviar el dato de derivada
    Serial.print(derivada_v);
    Serial.print(",");
  //Enviar el dato de error de posición
    Serial.print(err_p[0]);
    Serial.print(",");
    //Enviar el dato de integral
    Serial.print(integral_p);
    Serial.print(",");
    //Enviar el dato de derivada
    Serial.print(derivada_p);
    Serial.print(",");
  //Enviar el dato de entradas digitales
  Serial.print(binario);
  Serial.print(",");
  // Enviar todo
  Serial.flush();  
}


void envioestado(){
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTO envioestado()                              //
//                                                              //
//////////////////////////////////////////////////////////////////
  /* Envío del estado de las variables que controlan el modo de funcionamiento
                             (Para depuración)
  */

  //Enviar el dato de estado
  Serial.print("Modo_Control=");  
  Serial.print(modocontrol);          // Control: vel lazo abierto=1, vel lazo cerrado=2, pos un lazo simple=3, pos en cascada=4
  Serial.print(",Potenciometro_PC=");
  Serial.print(Pot_PC);               // Referencia desde el potenciómetro de la unidad Feedback=0 o desde el PC=1
  Serial.print(",modelo=");
  Serial.print(modelo);               // Modelo de Feedback original=0, modificado=1
  Serial.print(",simulacion=");
  Serial.print(sim);                  // Funcionamiento en modo normal=0, simulación=1
  Serial.print(",encoder=");
  Serial.print(fb_e);                 // Realimentación de velocidad por tacodinamo=0 o encoder=1
  Serial.print(",gray=");
  Serial.print(fb_g);                 // Realimentación de posición por potenciometro=0 o disco Gray=1
  Serial.print(",");
  // Enviar todo
  Serial.flush();  
}


unsigned int gray2binary(unsigned int gray){
//////////////////////////////////////////////////////////////////
//                                                              //
//     FUNCIÓN gray2binary()                                    //
//                                                              //
//////////////////////////////////////////////////////////////////
/*
  Función para convertir un entero código de GRAY en el entero binario correspondiente:

  gray - entero del código de GRAY
*/

    unsigned int num, mask;
    num = gray;
    mask = num >> 1;       // Desplaza un bit a la derecha
    while (mask > 0) {
       num = num^mask;     // XOR entre num y mask
       mask = mask >> 1;   // Desplaza un bit a la derecha
    }
    return num;
}


void CanalA(){
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTOS CanalA() y CanalB()                       //
//                                                              //
//////////////////////////////////////////////////////////////////
// Rutinas de interrupción para el encoder digital

// Flanco en el canal A
//  A=!A;                     // estas dos líneas no funcionan bien ¿por qué?
//  contador=contador-A*B;    // sería un código más óptimo que el actual
  A=digitalRead(PIN_D2);      // Actualizar el estado correcto del pin del encoder
  if  (A==!Ao)  contador=contador-A*B;   // pulso en sentido CCW si A=B=1 cuando cambia A
  Ao=A;
}


void CanalB(){                
// Flanco en el canal B
//  B=!B;
//  contador=contador-A*B;
  B=digitalRead(PIN_D1); 
  if  (B==!Bo) contador=contador+A*B;   // pulso en sentido CW si A=B=1 cuando cambia B
  Bo=B;
}


void conversionAD(){
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTO conversionAD()                             //
//                                                              //
//////////////////////////////////////////////////////////////////
  /* Función mediante la cual se leerán las entradas
     analógicas y digitales con las que se trabaja.
  */

  /* La velocidad y posición del equipo se estiman a partir de la medida de los sensores, pero si 
     se trabaja en modo simulación ambas se calculan simulando la función de transferencia del sistema
   */
  
  if (sim) {                        // Modo SIMULACION el Arduino simula los valores de velocidad y posición.
    if (uk[0]>2.5) uk[0]=2.5; // Limitado para simulación
    if (uk[0]<-2.5) uk[0]=-2.5;
    if (uk[1]>2.5) uk[1]=2.5; // Limitado para simulación
    if (uk[1]<-2.5) uk[1]=-2.5;
    if (uk[2]>2.5) uk[2]=2.5; // Limitado para simulación
    if (uk[2]<-2.5) uk[2]=-2.5;
    vel=0.8521*vel1+(255/5)*0.04639*uk[1];    // Ecuación en diferencias de la G(z) del accionador/motor/reductora para velocidad
    if (vel>45) vel=45;              // Discretizada para Tm=20ms
    if (vel<-45) vel=-45;
    pos=pos+6*vel1*T/1000;
//    pos=1.8521*pos1-0.8521*pos2+(255/5)*(0.02857*uk[1]+0.02709*uk[2]);
    if (pos>180) pos=pos-360;       // Ecuación en diferencias de la G(z) del accionador/motor/reductora para posición
    if (pos<-180) pos=pos+360;      // Discretizada para Tm=20ms
    vel1=vel;
    pos1=pos;
    pos2=pos1;
  }
  else {                            // Si no está en modo simulación, valores medidos de los sensores.

    /* Entradas analógicas 0...5V a un entero 0...1023 (10bits) */
    referencia= analogRead(PIN_referencia); // Valor entero sin signo 0...1023
    posicion= analogRead(PIN_posicion);     // Valor entero sin signo 0...1023

    /* Se tratarán la señales analógicas y se transforman de valores enteros (int) 
     a valores reales (float) relacionados con la magnitud que se mide. */

    //POSICION
    pos=(float) posicion*(360.0/1023.0)-180.0;     // "posicion" en int 0...1023 que hay que pasar a "pos" en grados +-180º
    if (fb_g) pos=pos_gray;                        // Realimentación de posición por código de GRAY

    //VELOCIDAD
    /* El valor estimado de la velocidad depende de las carácteristicas de cada tacodinamo
    y de los valores de las resistencias de la placa que adapta su tensión de salida (+-10V teóricos)
    a la tensión que admite el ARDUINO (0...5V).
    La mejor forma de que la conversión se aproxime a la velocidad real es medir la tensión del borne 
    "velocidad" de la placa cuando el motor está parado "V_0" y cuando este gira a velocidad máxima CW
    "V_max", anotando cual es esa velocidad máxima en RPM "RPM_max*/   
    
    // Valores aproximados para R2=47K R3=R4=100K en placas con Arduino MICRO
    RPM_max=45;         // Máximas RPM CW del motor       
    V_max=3.5;          // Tensión del motor a las máximas RPM
    V_0=2.55;           // Tensión de la tacodinamo a 0 RPM
    
    // Valores aproximados para R2=56K R3=R4=100K en placas con Arduino MEGA 
    // RPM_max=45;         // Máximas RPM CW del motor       
    // V_max=3.35;         // Tensión del motor a las máximas RPM
    // V_0=2.36;           // Tensión de la tacodinamo a 0 RPM    

    // Valores aproximados si en las placas R2=50K R3=R4=100K
    // RPM_max=45;         // Máximas RPM CW del motor       
    // V_max=3.44;         // Tensión del motor a las máximas RPM
    // V_0=2.5;            // Tensión de la tacodinamo a 0 RPM

    vel=(float) velocidad*(5.0/1023.0);   // "vel" en voltios (de 0...1023 a 0V...5V) que hay que pasar a RPM
    vel=(vel-V_0)*(RPM_max/(V_max-V_0));  // paso de voltios a RPM
      
    if (fb_e) vel=vel_enc;                         // Realimentación de velocidad por encoder
  }

  //REFERENCIA
  ref=(float) referencia*(5.0/1023.0);           // referencia (0...1023) a ref en voltios (0V...5V).
   
  //Se comprueba el modo de control (posición o velocidad) y si la referencia viene del potenciómetro o del PC
  //Si la referencia la envía el PC, Pot_PC==1, no se actualiza con la de la entrada A/D "referencia".
  if (Pot_PC==0 && modocontrol<3){               // modos de control de velocidad
      ref_vel=(float) referencia*(90.0/1023.0)-45.0; // Paso 0...1023 a -45...+45 rpm
  }
  if (Pot_PC==0 && modocontrol>2){               // modos de control de posición
      ref_pos=(float) referencia*(360.0/1023.0)-180.0; // Paso 0...1023 a -180º...+180º
  }

  // ENTRADA SENOIDAL
  if (senoide==1) {       //Si se selecciona senoidal senoide=1
    // float Amp=2;         //Amplitud en voltios (entre +-2.5V)
    // float W=1;         //Frecuencia en hercios: 1[hercio]=1[ciclos/s]=2*pi[rad/s]
    ref=Amp*sin(2*3.1416*W*tiempo_new/1000.0)+2.5;  // tiempo_new en milisegundos, ref 0...5V
    referencia=ref*1023.0/5.0;
    ref_vel=(float) referencia*(90.0/1023.0)-45.0;
    ref_pos=(float) referencia*(360.0/1023.0)-180.0; 
  }

  /* Entradas digitales del Encoder, Gray e Indice de vuelta */
   bit1=digitalRead(PIN_D1); // Encoder
   bit2=digitalRead(PIN_D2);

  // Se combinan todas en una única variable con todos los bits para el envío al PC
   binario=bit1+2*bit2+4*bit3+8*bit4+16*bit5+32*bit6+64*bit7+128*bit8+256*bit9;
   gray=bit8+2*bit7+4*bit6+8*bit5+16*bit4+32*bit3;   // Código de GRAY
   pos_gray=-(gray2binary(gray)-31.5)*360/64;        // Posición en grados del código de GRAY
}


void conversionDA(){
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTO conversionDA()                             //
//                                                              //
//////////////////////////////////////////////////////////////////
  /* Función mediante la cual se genera el Duty para la salida PWM
     a partir de la salida del regulador.  */

  // El duty va de 0 a 255 (0% a 100%, duty_0=127.5 => motor parado=50%) mientras que 
  // la salida del regulador uk[0] varía de -2.5 a +2.5V (o más, no está limitada).

  pwm=uk[0]*255/5;             // Paso de -2.5...+2.5V a -127.5...+127.5
  if (pwm>127.5) pwm=127.5;
  if (pwm<-127.5) pwm=-127.5;

  // Además, como el motor tiene una zona muerta de duty, se trata de evitar sumando (duty de giro CW) o
  // restando al duty (duty de giro CCW) una cantida aproximadamente equivalente a esa
  // zona muerta (+-20 para el modelo 1 y +-15 para el modelo 0) pero manteniendo la linealidad (127.5-20)/127.5.

  if (modelo) {         // Si el MODELO es el nuevo el duty al 100% gira CW => duty = uk[0]+duty_0;
    if (pwm>0) duty=(pwm*(127.5-20)/127.5)+20+duty_0;
    else if (pwm<0) duty=(pwm*(127.5-20)/127.5)-20+duty_0;   
    }
  else {                 // Si el MODELO es el antiguo el duty al 100% gira CCW => duty = -uk[0]+duty_0;
    if (pwm>0) duty=-((pwm*(127.5-15)/127.5)+15)+duty_0;
    else if (pwm<0) duty=-((pwm*(127.5-15)/127.5)-15)+duty_0;   
    }
  
  // Limitador de duty (mejor limitar "pwm" para evitar errores de paso de real (float) a entero (int)
  //if (duty<0) duty = 0;           // 0%
  //else if (duty>255)  duty=255;   // 100%

  // En control de velocidad, parada del motor si la referencia de velocidad es inferior a +-2 r.p.m.
  // Si no, el motor con ref_vel=0 gira ligeramente por errores de medida del tacogenerador
  if ((modocontrol<3) && (ref_vel<2) && (ref_vel>-2)) { 
        duty = duty_0;
        integral_v=0;
  }
  
  //Envío del valor del duty al puerto PWM conectado al driver del motor.
  analogWrite(PIN_PWM,duty);

}

                 
void regulador(){
//////////////////////////////////////////////////////////////////
//                                                              //
//     PROCEDIMIENTO regulador()                                //
//                                                              //
//////////////////////////////////////////////////////////////////
  /* En esta función se recogen los algoritmos de control.
     El control puede ser:

      +-LAZO ABIERTO (modocontrol=1)
      +
      +                         +-UN LAZO    (modocontrol=3)
      +              +-POSICIÓN-+        
      +              +          +-EN CASCADA (modocontrol=4) 
      +-LAZO CERRADO-+
                     +
                     +-VELOCIDAD ((modocontrol=2) o (modocontrol=4))
  */  

//MODO LAZO ABIERTO
  if (modocontrol==1) { 
    uk[0]=ref_vel*5/90;  // La consigna del "regulador" será la referencia de velocidad +-45 RPM pasada a +-2.5V
  }
 
//MODO LAZO CERRADO 

  // CONTROL DE POSICIÓN
  if (modocontrol>2){

    //CÁLCULO DEL ERROR
    // Referencia del potenciómetro
    if (Pot_PC==0) { 
      err_p[0] = ref_pos - pos;
    } 
    // Referencia del PC
    else {   
      if (((ref_pos-pos)<180.0)&&((ref_pos-pos)>-180.0)) {   // Problema del paso +180º -180º
        err_p[0] = ref_pos - pos;
      }                                 // Busca el camino más corto para llegar a la nueva posición
      else if ((ref_pos-pos)>180.0) {   // paso de -180º a +180º
        err_p[0] = ref_pos - pos - 360;
      }
      else if ((ref_pos-pos)<-180.0) {  // paso de +180º a -180º
        err_p[0] = ref_pos - pos + 360;
      }
    }

    //CÁLCULO DE LA INTEGRAL DEL ERROR
    //Sistema anti-windup
    if (duty<1 || duty>254 ) // Saturación
    {
      integral_p=integral_p;
    // integral_p=0;
    }
    else  // si no hay saturación se calcula el valor de la integral normalmente
    {
      integral_p=integral_p+err_p[1]*T/1000;    // T está en ms, hay que dividirlo por 1000 para pasarlo a s
    }
    // Si no hay acción integral, se resetea el valor de la integral
    if (ki_p==0)  integral_p=0; 
  
    //CÁLCULO DE LA DERIVADA DEL ERROR  
    derivada_p=1000*(err_p[0]-err_p[1])/T;      // T está en ms, hay que dividirlo por 1000 para pasarlo a s

    //CÁLCULO DEL ALGORITMO DEL PID
    if (modocontrol==3) {                           // Control de posición lazo simple: se calcula la consigna para el motor
      uk[0]=kp_p*(err_p[0]+ki_p*integral_p+kd_p*derivada_p);
    }
    else {                                      // Control de posición en cascada: se calcula la referencia de velocidad
      ref_vel=kp_p*(err_p[0]+ki_p*integral_p+kd_p*derivada_p);
    }
  }
    
  // CONTROL DE VELOCIDAD  (si se hace control de velocidad en lazo cerrado y si se hace control de posición en cascada)
  if ((modocontrol==2) || (modocontrol==4)) {        
    
    //CÁLCULO DEL ERROR
    err_v[0] = ref_vel - vel;   // Si el control es de posición en cascada ref_vel lo ha generado el regulador de posición
  
    //CÁLCULO DE LA INTEGRAL DEL ERROR
    //Sistema anti-windup
    if (duty<1 || duty>254 ) // Saturación
    {
      integral_v=integral_v;
      // integral_v=0;
    }
    else  // si no hay saturación se calcula el valor de la integral normalmente
    {
      integral_v=integral_v+err_v[1]*T/1000;    // T está en ms, hay que dividirlo por 1000 para pasarlo a s
    }
    // Si no hay acción integral, se resetea el valor de la integral
    if (ki_v==0) integral_v=0; 
    
    //CÁLCULO DE LA DERIVADA DEL ERROR  
    derivada_v=1000*(err_v[0]-err_v[1])/T;      // T está en ms, hay que dividirlo por 1000 para pasarlo a s
  
    //CÁLCULO DEL ALGORITMO DEL PID
    uk[0]=kp_v*(err_v[0]+ki_v*integral_v+kd_v*derivada_v);
  
  }

  // Desplazar valores
  err_v[2]= err_v[1];
  err_v[1]= err_v[0];
  err_p[2]= err_p[1];
  err_p[1]= err_p[0];
  uk[1]= uk[0];
  uk[2]= uk[1];
  
}

