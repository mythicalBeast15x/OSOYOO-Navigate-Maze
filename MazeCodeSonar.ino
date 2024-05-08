#include <Servo.h>
#include <TimerOne.h>
//motor pin definitions
#define rightSpeed  5 //3
#define leftSpeed  6
#define rightForwards  12
#define rightBackwards  11
#define leftForwards  7
#define leftBackwards  8
//#define loopCounter  2
#define servo  9
//sensor pin definitions
#define echo_pin  4//2
#define trig_pin  10

#define buzzer  13
// constant speeds
#define FAST_SPEED 250
#define SPEED 200
#define TURN_SPEED 200
#define BACK_SPEED1 255
#define BACK_SPEED2 90

const byte MOTOR1 = 2; // Motor 1 Interrupt Pin - INT 0 (Encoder)
const byte MOTOR2 = 3; // Motor 2 Interrupt Pin - INT 1 (Encoder)

unsigned int counter1 = 0;
unsigned int counter2 = 0;
unsigned int m1Counter = 0;
unsigned int m2Counter = 0;

void ISR_count1()
{
  counter1++; // increment Motor 1 counter value
  m1Counter++;
}
// Motor 2 pulse count ISR
void ISR_count2()
{
  counter2++; // increment Motor 2 counter value
  m2Counter++;
}

int thereis;
const int testNum = 40;
Servo head;
const int threshold = 8;
void forwards(void)  //Forward
{
  digitalWrite(rightForwards, HIGH);
  digitalWrite(rightBackwards,LOW);
  digitalWrite(leftForwards,HIGH);
  digitalWrite(leftBackwards,LOW);
}
void rotateLeft()  //Turn left
{
  digitalWrite(rightForwards, HIGH);
  digitalWrite(rightBackwards,LOW);
  digitalWrite(leftForwards,LOW);
  digitalWrite(leftBackwards,HIGH);
}
void rotateRight()  //Turn right
{
  digitalWrite(rightForwards, LOW);
  digitalWrite(rightBackwards,HIGH);
  digitalWrite(leftForwards,HIGH);
  digitalWrite(leftBackwards,LOW);
}
void backwards()  //Reverse
{
  digitalWrite(rightForwards, LOW);
  digitalWrite(rightBackwards,HIGH);
  digitalWrite(leftForwards,LOW);
  digitalWrite(leftBackwards,HIGH);
}
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(leftSpeed,speed_L); 
  analogWrite(rightSpeed,speed_R);   
}
void stop_Stop()    //Stop
{
  digitalWrite(rightForwards, LOW);
  digitalWrite(rightBackwards,LOW);
  digitalWrite(leftForwards,LOW);
  digitalWrite(leftBackwards,LOW);
  set_Motorspeed(0,0);
}

void disableBuzzer(){
  digitalWrite(buzzer, HIGH);
}

void soundAlarm(){
  digitalWrite(buzzer, LOW);
  delay(500);
  disableBuzzer();
}

int getDistance(){
  long avgDist = 0;
  for(int x = 0; x < testNum; x++){
    long elapsed_time;
    digitalWrite(trig_pin, LOW);
    //Serial.println("Sensed");
    delayMicroseconds(5);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(15);
    digitalWrite(trig_pin, LOW);
    
    elapsed_time = pulseIn(echo_pin, HIGH);
    long distance = (elapsed_time * 0.0135039)/2;
    avgDist += distance;
    //Serial.println(avgDist);
  }
  //Serial.println(avgDist);
  avgDist /= testNum;
  return avgDist;

}


void setup() {
  // put your setup code here, to run once:
  /*setup L298N pin mode*/
  pinMode(rightForwards, OUTPUT); 
  pinMode(rightBackwards, OUTPUT); 
  pinMode(rightSpeed, OUTPUT);  
  pinMode(leftForwards, OUTPUT);
  pinMode(leftBackwards, OUTPUT); 
  pinMode(leftSpeed, OUTPUT); 
  stop_Stop();//stop move
  /*init HC-SR04*/
  pinMode(trig_pin, OUTPUT); 
  pinMode(echo_pin,INPUT); 
  /*init buzzer*/
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);  
  
  Timer1.initialize(100000); // set timer for 1sec(1000000)
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);
  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);

  digitalWrite(trig_pin,LOW);
  /*init servo*/
  head.attach(servo); 
  head.write(90);
  delay(2000);
  disableBuzzer();
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("--------");
}
int c = 0;
void loop() {
  

  for(int i=0; i<3; i++){
    while(getDistance() > 12){
      forwards();
      set_Motorspeed(200,210);
    }
    //turning
    stop_Stop();
    delay(2000);
    m1Counter = 0;
    if(i%2==1){
      while(m2Counter <= 80){
        set_Motorspeed(150,150);
        rotateLeft();
      }
    }else{
      while(m1Counter <= 100){
        set_Motorspeed(200,200);
        rotateRight();
      }
    }

    
    stop_Stop();
    delay(2000);
    
  }
  while(1){}
  /*
  forwards();
  set_Motorspeed(SPEED,SPEED);
  Serial.print("Dist: ");
  Serial.println(getDistance());
  for(int i=0; i < 3; i++){
    Serial.println(getDistance());
    while(getDistance() <= threshold){
      Serial.println(getDistance());
      set_Motorspeed(SPEED,SPEED);
      forwards();
    }
    stop_Stop();
    soundAlarm();
    delay(500);
    if(i % 2 == 0){
      rotateRight();
    }else{
      rotateLeft();
    }
    
    
  //turn 
  }
  */

}