#pragma once
#include "Arduino.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SimpleDHT.h>

// clasa LCD!!!!

class LCD : public LiquidCrystal_I2C{
public:
  LCD(uint8_t, uint8_t, uint8_t);
  void printMsg(int, const char*);
  void start();
  void clearLine(int);
};


LCD::LCD(uint8_t a, uint8_t b, uint8_t c) : LiquidCrystal_I2C(a, b, c) {}

void LCD::printMsg(int line, const char *msg){
  this->clearLine(line);
  this->setCursor(0, line);
  this->print(msg);
}

void LCD::start(){
  this->init();
  this->backlight();
}

void LCD::clearLine(int line){
  for(int i = 0; i < 20; i++) {
    this->setCursor(i, line);
    this->print(" ");
  }
}


// clasa led!!!


class led{

    int intensitate = 0;
  public:
    led();
    led(int);
    ~led();

    void set_led(int);
    int get_led();
    void power_on_led(int);
    void power_off_led(int);
};

led::led()
{
  intensitate = 0;
}

led::led(int i)
{
  if((i >= 0)&&(i <= 255))
    intensitate = i;
  else
    intensitate = 0;
}

led::~led()
{
  intensitate = 0;
}

void led::set_led(int i)
{
  if((i >= 0)&&(i <= 255))
    intensitate = i;
  else
    intensitate = 0;
}

int led::get_led()
{
  return intensitate;
}

void led::power_on_led(int port_dig)
{
  // test legat de port 
  analogWrite(port_dig, intensitate);
  
}

void led::power_off_led(int port_dig)
{
  // test legat de port 
  analogWrite(port_dig, LOW);
  
}

led led1(0);

const int analogInPin = A0;
const int analogOutPin = 9;

int sensorValue = 0;
int outputValue = 0;


int calibrationTime = 10; 
long unsigned int lowIn;  
long unsigned int pause = 5000;

boolean lockLow = true;
boolean takeLowTime;  

int pirPin = 3;

LiquidCrystal_I2C lcd(0x27 , 16 , 2 );
LCD L(0X27 , 16 , 2);



  int pinDHT11 = 2;
  SimpleDHT11 dht11(pinDHT11);

void setup()
{
  Serial.begin(9600);
  L.start();
  Serial.println("LCD...");
  Wire.begin();
  Wire.beginTransmission(0x27);
  Serial.println("Check for LCD");
  int error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);
  if (error == 0) {
    Serial.println(": LCD found.");
    lcd.begin(16, 2); // initialize the lcd
    lcd.setBacklight(255);
    lcd.home(); 
    lcd.clear();
    lcd.print("Hello LCD");
    delay(1000);
  } else {
    Serial.println(": LCD not found.");
  } 
}

void loop()
{
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);

  // DHT11
  // start working...
  Serial.println("=================================");
  Serial.println("Sample DHT11...");
  
  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err="); Serial.println(err);delay(1000);
    return;
  }
  
  Serial.print("Sample OK: ");
  Serial.print((int)temperature); Serial.print(" *C, "); 
  Serial.print((int)humidity); Serial.println(" H");

  L.printMsg(0 , "Temperatura:");
  lcd.print((int)temperature);
  lcd.print((char)223);
  lcd.print("C");
  delay(1200);

  L.printMsg(1 , "Umiditate:");
  lcd.print((int)humidity);
  lcd.print("%");

//PIR

led1.set_led(outputValue);

  if(digitalRead(pirPin) == HIGH){
       led1.power_on_led(6);   //the led visualizes the sensors output pin state
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         Serial.println("---");
         Serial.print("motion detected at ");
         Serial.print(millis()/1000);
         Serial.println(" sec"); 
         delay(50);
         }         
         takeLowTime = true;
       }

     if(digitalRead(pirPin) == LOW){       
        led1.power_off_led(6);  //the led visualizes the sensors output pin state
        if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
           Serial.print("motion ended at ");      //output
           Serial.print((millis() - pause)/1000);
           Serial.println(" sec");
           delay(50);
           }
       }

}


