const int stepPin = 3; 
const int dirPin = 4; 
const int optpin = 5;
unsigned long dt = 5; //1/16
int lim = 1000;
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(optpin,INPUT);
  Serial.begin(9600);
  
}
void loop() {
  digitalWrite(dirPin,HIGH);
  int isNotZero = 0;
  while(!isNotZero){
    digitalWrite(stepPin,HIGH); 
    delay(dt); 
    digitalWrite(stepPin,LOW); 
    delay(dt);
    isNotZero = digitalRead(optpin);   //set pin 8 HIGH, turning on LED
    Serial.println(isNotZero);
  }
  while(true){
  }
}
