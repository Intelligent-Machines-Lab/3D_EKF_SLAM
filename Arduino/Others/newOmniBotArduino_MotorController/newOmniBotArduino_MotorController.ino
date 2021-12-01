#include <PWMServo.h>
#include <Wire.h>
/*
#define TWI_FREQ_FAST 400000L
TWBR = ((CPU_FREQ / TWI_FREQ_FAST) - 16) / 2;
*/
#include <math.h>

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN          13   // ANALOG NUMBERING
#define TEMP_1_PIN          14   // ANALOG NUMBERING

// Pinos referentes ao eletroima
#define LED_PIN            13
#define HEATER_0_PIN       10
#define step_division       4   //This is the step divisin configuration at the step motor shield
#define SATURATION_VEL     2.0  //2.0 [m/s]


//Tamanho do robo
#define   C   0.26   // [m]
#define   L   0.265  // [m]

#define SERVO_LEFT_PIN 11
#define SERVO_RIGHT_PIN 6
 
//Frente esquerdo
unsigned long time_step_FE;
unsigned long conta_tempo_FE=0 ;

//Frente direito
unsigned long time_step_FD;
unsigned long conta_tempo_FD=0 ;

//Tras esquerdo
unsigned long time_step_TE;
unsigned long conta_tempo_TE=0 ;

//Tras direito
unsigned long time_step_TD;
unsigned long conta_tempo_TD=0 ;

boolean parar_robo=false;

float vel_X=0, vel_Y=0, vel_Wz=0;
float v1=0, v2=0, v3=0, v4=0;

unsigned int pos=0;
typedef void (*GeneralFunction) ();
char data[20], in_char;

double pulsesX = 0, pulsesZ = 0, pulsesY = 0, pulsesQ = 0;
int stateX = 0, stateZ = 0, stateY = 0, stateQ = 0;

// This is the time since the last rising edge in units of 0.5us.
//uint16_t volatile servoTime = 0;
 
// This is the pulse width we want in units of 0.5us.
//uint16_t volatile servoHighTime = 3000;
 
// This is true if the servo pin is currently high.
//boolean volatile servoHigh = false;

// Global holding desired servo position and its flag
float servo_ang = 0;
//float oldServoAng = 0; 
//float newServoAng = 0;
boolean servoFlag = false;
PWMServo myservo;


// Interrupt
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
  conta_tempo_FE += 100;
  conta_tempo_FD += 100;
  conta_tempo_TE += 100;
  conta_tempo_TD += 100;
  
  digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);   // toggle LED pin
  
  if ( conta_tempo_FE >= time_step_FE)
  {
      conta_tempo_FE=0;
      if(v1!=0){
        movimento_translacional('F','E');
      }
      else{ 
        parar('F','E');
      }
  }
  if (conta_tempo_FD >= time_step_FD)
  {
    conta_tempo_FD=0;
    if(v2!=0){ 
      movimento_translacional('F','D');
    }
    else{ 
      parar('F','D');
    }
  } 
  if ( conta_tempo_TE >= time_step_TE)
  {
      conta_tempo_TE=0;
      if(v3!=0){ 
        movimento_translacional('T','E');
      }
      else{ 
        parar('T','E');
      }
  }  
  if (conta_tempo_TD >= time_step_TD)
  {
    conta_tempo_TD=0;
    if(v4!=0){ 
      movimento_translacional('T','D');
    }
    else{ 
      parar('T','D');
    }
  }
}
/* old code, callback for servo
ISR(TIMER2_COMPA_vect)
{
  // The time that passed since the last interrupt is OCR2A + 1
  // because the timer value will equal OCR2A before going to 0.
  servoTime += OCR2A + 1;
   
  static uint16_t highTimeCopy = 3000;
  static uint8_t interruptCount = 0;
   
  if(servoHigh){
    if(++interruptCount == 2){
      OCR2A = 255;
    }
 
    // The servo pin is currently high.
    // Check to see if is time for a falling edge.
    // Note: We could == instead of >=.
    if(servoTime >= highTimeCopy){
      // The pin has been high enough, so do a falling edge.
      digitalWrite(SERVO_LEFT_PIN, LOW);
      servoHigh = false;
      interruptCount = 0;
    }
  }else{
    // The servo pin is currently low.
    if(servoTime >= 40000){
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = servoHighTime;
      digitalWrite(SERVO_LEFT_PIN, HIGH);
      servoHigh = true;
      servoTime = 0;
      interruptCount = 0;
      OCR2A = ((highTimeCopy % 256) + 256)/2 - 1;
    }
  }
}
*/
char cmdCamera;

void setup()
{
  Serial.begin(9600);
  
  Wire.begin(9);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  /*
  digitalWrite(SERVO_LEFT_PIN, LOW);
  pinMode(SERVO_LEFT_PIN, OUTPUT);
  */
  myservo.attach((int)SERVO_LEFT_PIN);
  
  inicia_motores();
  inicia_timer();
    
  time_step_FE=6000;
  time_step_FD=6000;
  time_step_TE=6000;
  time_step_TD=6000;

  
}

void loop () {
    if (v1>=0) sentido_frente('F','E');
    else sentido_tras('F','E');
    
    if (v2>=0) sentido_frente('F','D');
    else sentido_tras('F','D');
    
    if (v3>=0) sentido_frente('T','E');
    else sentido_tras('T','E');
    
    if (v4>=0) sentido_frente('T','D');
    else sentido_tras('T','D');

    if(v1>SATURATION_VEL) v1=SATURATION_VEL;
    if(v2>SATURATION_VEL) v2=SATURATION_VEL;
    if(v3>SATURATION_VEL) v3=SATURATION_VEL;
    if(v4>SATURATION_VEL) v4=SATURATION_VEL;
        
    if(v1<-SATURATION_VEL) v1=-SATURATION_VEL;
    if(v2<-SATURATION_VEL) v2=-SATURATION_VEL;
    if(v3<-SATURATION_VEL) v3=-SATURATION_VEL;
    if(v4<-SATURATION_VEL) v4=-SATURATION_VEL;
            
    calcula_time_step(abs(v1),abs(v2),abs(v3),abs(v4));
    //Serial.println(v1);Serial.println(v2);Serial.println(v3); Serial.println(v4);

    /* new code, not tested yet. suppose to slow down servo movement */
    if(servoFlag){
      //myservo.write(servo_ang);
      servoFlag = false;
      int icmd = (int) servo_ang;
      Serial.write(icmd);  
      
    }
    
    /* old code for fast servo movement
    if(servoFlag){
      servoSetPosition(map(servo_ang, 0, 180, 700, 2500));
      servoFlag = false;
    }
    */
    
//    sentido_tras('D');
//    sentido_frente('E');
//    calcula_time_step (0.2,0.01);delay(1000);

//    sentido_frente('D');
//    sentido_tras('E');
//    calcula_time_step (0.05,0.1);delay(1000); 
}

void inicia_timer ()
{
  noInterrupts();           // disable all interrupts
  // TIMER 3 FOR INTERNAL USE
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 1599;//799;            // compare match register 16MHz/1/10kHz
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS30);    // 1 prescaler 
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  /*
  // TIMER 2 FOR SERVO USE
  // Turn on CTC mode.  Timer 2 will count up to OCR2A, then
  // reset to 0 and cause an interrupt.
  TCCR2A = (1 << WGM21);
  // Set a 1:8 prescaler.  This gives us 0.5us resolution.
  TCCR2B = (1 << CS21);
   
  // Put the timer in a good default state.
  TCNT2 = 0;
  OCR2A = 255;
  
  TIMSK2 |= (1 << OCIE2A);  // Enable timer compare interrupt
  */
  interrupts();             // enable all interrupts
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/*
void servoSetPosition(uint16_t highTimeMicroseconds)
{
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  servoHighTime = highTimeMicroseconds * 2;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
}
*/
void calcula_time_step(float vel_roda_FE, float vel_roda_FD,float vel_roda_TE,float vel_roda_TD)
{
  time_step_FE = 198.4/(pow(vel_roda_FE, 1.082)*step_division);
  time_step_FD = 198.4/(pow(vel_roda_FD, 1.082)*step_division);
  time_step_TE = 198.4/(pow(vel_roda_TE, 1.082)*step_division);
  time_step_TD = 198.4/(pow(vel_roda_TD, 1.082)*step_division);

//  Serial.print("tsFE=");Serial.print(time_step_FE);Serial.println();
//  Serial.print("tsFD=");Serial.print(time_step_FD);Serial.println();
//  Serial.print("tsTE=");Serial.print(time_step_TE);Serial.println();
//  Serial.print("tsTD=");Serial.print(time_step_TD);Serial.println();
}
