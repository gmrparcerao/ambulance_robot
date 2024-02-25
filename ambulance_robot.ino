/*
Sao Paulo State Technology College of Itaquera "Professor Miguel Reale"
Sao Paulo, 2019/11/07
Associate of Science in Industrial Automation

Names of undergraduate students:
  Bruno Ryuiti Kitano
  Gabriel Liobino Sampaio
  Guilherme Matheus Rafael Parcerão
  Matheus Batista Rodrigues Carvalho

Goals: 
  Software to control a line follower robot using Assembly language

Hardware:
  Arduino MEGA
  nRF24L1 radiofrequency module
  HC-SR04 ultrassonic sensor
  QRE1113 infrared sensor

Reviews: 
  R000 - begin

This code is for didatic purposes only. No warranty of any kind is provided.
*/

#include <Wire.h>

// RF24L01
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(22, 24); // CE, CSN
const byte address[6] = "00001";
int recieved_data;
#define ledHouse1 26
#define ledHouse2 28
#define ledHouse3 30

// Pins and variables for the ultrassonic sensor
#define trig1 3 // Right
#define echo1 2
#define trig2 4 // Left
#define echo2 5
#define trig3 11 // Front
#define echo3 12
#define LDR1 A0
#define LDR2 A1

unsigned long duration;
float distance, rightSensor, leftSensor, LDR1Val, LDR2Val;
int LDRedge = 800, aux1house = 0, aux2house = 0, aux3house = 0, aux1end = 0, aux2end = 0, aux3end = 0;
bool right, left;

// Variables for motor speed
int velRight, velLeft;

// d = distance to walls
float d = 8.5;
int vA = 255, vB = 120;

// Motor pins
#define motorRight 9
#define motorLeft 13

// Led and Buzzer
int cont1 = 0, cont2 = 0;
#define led1 6
#define led2 7
#define buzzer 10

void setup() 
  {
// RF24L01
      radio.begin();
      radio.openReadingPipe(0, address);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
      pinMode(ledHouse1, OUTPUT); // House 1
      pinMode(ledHouse2, OUTPUT); // House 2
      pinMode(ledHouse3, OUTPUT); // House 3
      
// Ultrassonic sensor pins
      pinMode(trig1, OUTPUT);
      pinMode(echo1, INPUT);
      pinMode(trig2, OUTPUT);
      pinMode(echo2, INPUT);
      pinMode(trig3, OUTPUT);
      pinMode(echo3, INPUT);
  
// Motor pins
      pinMode(motorRight, OUTPUT);
      pinMode(motorLeft, OUTPUT);
      Serial.begin(9600);
  
// Led
      pinMode(led1, OUTPUT);
      pinMode(led2, OUTPUT);
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);

      pinMode(buzzer, OUTPUT);
// Timer1 configuration
      cli();//stop interrupts
      
        //set timer1 interrupt at 1Hz
        TCCR1A = 0;// set entire TCCR1A register to 0
        TCCR1B = 0;// same for TCCR1B
        TCNT1  = 0;//initialize counter value to 0
        // set compare match register for 1hz increments
// Timer1 time configuration
        OCR1A = round(16000000/(10*1024));      // = (16*10^6) / (Hz*1024) - 1 (must be < 0,2384Hz)
        // turn on CTC mode
        TCCR1B |= (1 << WGM12);
        // Set CS12 and CS10 bits for 1024 prescaler
        TCCR1B |= (1 << CS12) | (1 << CS10);  
        // enable timer compare interrupt
        TIMSK1 |= (1 << OCIE1A);
      
      sei();//allow interrupts
  }

ISR(TIMER1_COMPA_vect)                        // Function to process Timer1 interruption
  {
     cont1++;
     if(cont2 <= 10)
          {
            tone(buzzer, 1450);
            cont2++;
          }
        if(cont2 > 10)
            {
              tone(buzzer, 1890);
              cont2++;
             if(cont2 ==20) cont2 = 0;  
            }
     if(cont1 >= 5)
      {
        cont1 = 0;
        digitalWrite(led1, (!digitalRead(led1)));
        digitalWrite(led2, (!digitalRead(led2)));
        
      }
  }
  
void SonarSensor(int trig,int echo)           // Function to control the ultrassonic sensor
  {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distance = (duration/2) / 29.1;
  }

void start()
  {
// stop
    LDR1Val = analogRead(LDR1);
    LDR2Val = analogRead(LDR2);
   if(LDR1Val >= LDRedge && LDR2Val >= LDRedge)
   {
    if(aux1end == 1 || aux2end == 1 || aux3end == 1) 
    {
      stop();
    }
    }
    

// start
    SonarSensor(trig1, echo1);
    rightSensor = distance;
    SonarSensor(trig2, echo2);
    leftSensor = distance;
    if(leftSensor < d)
      {
         if(rightSensor < d)
         {
            velRight = vA;
            velLeft = vA;
          }
      }
    analogWrite(motorRight, velRight);
    analogWrite(motorLeft, velLeft);
    
// if left sensor is distant, turn right
    if(leftSensor > d)
      {
        velRight = vB;
        velLeft = vA;
      }
    analogWrite(motorRight, velRight);
    analogWrite(motorLeft, velLeft);
  
// if right sensor is distant, turn left
    if(rightSensor > d)
      {
        velRight = vA;
        velLeft = vB;
      }
    analogWrite(motorRight, velRight);
    analogWrite(motorLeft, velLeft);
  }

void stop()
  {
    digitalWrite(motorRight, LOW);
    digitalWrite(motorLeft, LOW);
    delay(10000);

    if(aux1end == 1) aux1end = 0;
    if(aux1house == 1 && recieved_data == 11 && aux1end == 0) 
      {
        aux1house = 0;
        aux1end = 1;          
      }
   
    if(aux2end == 1) aux2end = 0;
    if(aux2house == 1 && recieved_data == 21 && aux2end == 0) 
      {
        aux2house = 0;
        aux2end = 1;          
      }

    if(aux3end == 1) aux3end = 0;
    if(aux3house == 1 && recieved_data == 31 && aux3end == 0) 
      {
        aux3house = 0;
        aux3end = 1;          
      }
  }
  
void loop() 
  {
   if (radio.available()){
    radio.read( &recieved_data, sizeof(int)); 
    if(recieved_data == 1){
      digitalWrite(ledHouse1, HIGH);}
    if(recieved_data == 11){
      digitalWrite(ledHouse1, LOW);} 
    
    if(recieved_data == 2){
      digitalWrite(ledHouse2, HIGH);}
    if(recieved_data == 21){
      digitalWrite(ledHouse2, LOW);} 

    if(recieved_data == 3){
      digitalWrite(ledHouse3, HIGH);}
    if(recieved_data == 31){
      digitalWrite(ledHouse3, LOW);} 
  } 
  Serial.println(recieved_data);
  
// House 1
  if(recieved_data == 1 && aux1end == 0)
    {
      aux1house = 1;
      start();
    }
  if(aux1house == 1 && recieved_data == 11 && aux1end == 0)
    {
      stop();
    }
  if(aux1end == 1)
    {
      start();  
    }

// House 2
  if(recieved_data == 2 && aux2end == 0)
    {
      aux2house = 1;
      start();
    }
  if(aux2house == 1 && recieved_data == 21 && aux2end == 0)
    {
      stop();
    }
  if(aux2end == 1)
    {
      start();  
    }

// House 3
  if(recieved_data == 3 && aux3end == 0)
    {
      aux3house = 1;
      start();
    }
  if(aux3house == 1 && recieved_data == 31 && aux3end == 0)
    {
      stop();
    }
  if(aux3end == 1)
    {
      start();  
    }
}
