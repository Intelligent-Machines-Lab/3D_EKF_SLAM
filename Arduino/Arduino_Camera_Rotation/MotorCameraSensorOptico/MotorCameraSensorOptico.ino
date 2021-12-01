#include <SoftwareSerial.h>

const int stepPin = 3; 
const int dirPin = 4; 
const int optpin = 5;
unsigned long dt = 5; //1/16
int lim = 1000;
int cmdRecebido;
float stepvalue = 1.8;

SoftwareSerial serial_robo(10, 11); // RX, TX

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(optpin,INPUT);
  Serial.begin(9600);
  serial_robo.begin(9600);


  
  
}
void loop() {
  if (serial_robo.available()) {
    delay(100); //allows all serial sent to be received together
    cmdRecebido = serial_robo.read();

  
  Serial.print(cmdRecebido);

  switch (cmdRecebido) {
  case 0:
    // não faz nada
    break;
  case 1:
    Serial.println("Calibra zero");
    findZero();
    break;
  case 2:
    Serial.println("Avança 90");
    move_motor(90.0, 0);
    break;
  case 3:
    move_motor(60.0, 0);
    Serial.println("Avança 60");
    break;
  case 4:
    Serial.println("Volta 90");
    move_motor(90.0, 1);
    break;
  case 5:
    Serial.println("Volta 360");
    move_motor(360.0, 1);
    break;
  default:
    Serial.println("def");
    break;
  }
    }
}


void findZero(){
  digitalWrite(dirPin,HIGH);
  int isZero = 0;
  int nsteps = 0;
  while(!isZero){
    if(nsteps > (360/stepvalue)){
      break;
    }
    digitalWrite(stepPin,HIGH); 
    delay(dt); 
    digitalWrite(stepPin,LOW); 
    delay(dt);
    isZero = digitalRead(optpin);   //set pin 8 HIGH, turning on LED
    Serial.println(isZero);
    nsteps = nsteps+1;
  }
}


void move_motor(float anguloDesejado, int dir){
  float anguloMovido = 0;
  digitalWrite(dirPin,dir);
  
  while(anguloMovido < anguloDesejado ){
    digitalWrite(stepPin,HIGH); 
    delay(dt); 
    digitalWrite(stepPin,LOW); 
    delay(dt);
    anguloMovido = anguloMovido + stepvalue;
  }
}
