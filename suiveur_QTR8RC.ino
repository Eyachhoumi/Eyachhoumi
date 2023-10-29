

#include <SoftwareSerial.h>
#include <QTRSensors.h>

#define in1 A1
#define in2 A0    // moteur gauche 
#define ENA 5
#define in3 A2
#define in4 A3    // moteur droite 
#define ENB 6
#define tcrd 2
#define tcrg 7
#define Speed  100 //180 //140


 QTRSensors qtr;
  
 const uint8_t SensorCount = 7;
 uint16_t sensorValues[SensorCount];
 const double kp = 0.4;    //--- LAST 0.8
 const double kd = 2; //0.5    // last 0.5
 uint16_t position ; 
 double lastError = 0 ;
 const int Goal = 2800;

 int S0,S7,S6,S5,S2;

void setup() {
  Serial.begin(9600);
   pinMode(tcrd,INPUT);
   pinMode(tcrg,INPUT);
  // pinMode(button2,INPUT);
   pinMode(in1,OUTPUT);
   pinMode(in2,OUTPUT);
   pinMode(ENA,OUTPUT);
   pinMode(in3,OUTPUT);
   pinMode(in4,OUTPUT);
   pinMode(ENB,OUTPUT);
   qtr.setTypeRC();
   qtr.setSensorPins((const uint8_t[]){3,4,13,12,11,10,9,8}, SensorCount);
   calibrateLineSensor();

}

void loop() {


if ((digitalRead(tcrd)) && (!digitalRead(tcrg))){
  
  droite();
  }
  
if ((!digitalRead(tcrd)) && (digitalRead(tcrg))){
  
  gauche();
  }
  if ((digitalRead(tcrd)) && (digitalRead(tcrg))){
  
  avant();
  
  }
  else{
 position = qtr.readLineBlack(sensorValues); 
 int error = Goal - position ;
 int adjustment = kp*error + kd*(error - lastError);
 lastError = error;
 move(0, ( constrain (Speed + adjustment ,0 , Speed )), 1);  
 move(1, ( constrain (Speed - adjustment ,0 , Speed )) , 1);
  }
  
}


 void calibrateLineSensor()
  {
  digitalWrite(LED_BUILTIN, HIGH); 
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();

  }
  }
  void move(int motor, int speed, int direction)
  {
    
   unsigned int inPin1=0;
   unsigned int inPin2=0;

  if(direction == 1){
    inPin1 = 255;
    inPin2 = 0;

  }  
  if(direction == 0){
    inPin1 = 0;
    inPin2 = 255;

  }

  if(motor == 0){
    analogWrite(in1,inPin1);
    analogWrite(in2,inPin2);
    analogWrite(ENA, speed);
  }
  if(motor == 1){
    analogWrite(in4,inPin1);
    analogWrite(in3,inPin2);
    analogWrite(ENB, speed);
  }
 
  }
   void droite(){
      analogWrite(in1,0);
    analogWrite(in2,255);
    analogWrite(ENA, 180);
        analogWrite(in3,0);
    analogWrite(in4,255);
    analogWrite(ENB, 180);
  
  }
    void gauche(){
      analogWrite(in2,0);
    analogWrite(in1,255);
    analogWrite(ENA, 180);
        analogWrite(in4,0);
    analogWrite(in3,255);
    analogWrite(ENB, 180);
  
  }
   void avant(){
      analogWrite(in2,0);
    analogWrite(in1,255);
    analogWrite(ENA, 90);
        analogWrite(in3,0);
    analogWrite(in4,255);
    analogWrite(ENB, 90);
  
  }
