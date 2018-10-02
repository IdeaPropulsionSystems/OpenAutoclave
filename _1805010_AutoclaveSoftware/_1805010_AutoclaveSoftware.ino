//Include Libraries_________________________________________________________________

//Library for Thermocouple
#include "max6675.h"

//Libraries for 1602 i2c LCD
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

//Libraries for file management
#include <EEPROM.h>

//Pin definitions_________________________________________________________________

//pin definitions for speaker
int speakerPin1 = 3;
int speakerPin2 = 4;

//pin definitions for PushButton
int buttonPin1 = 5;
int buttonPin2 = 6;

//pin definitions for Thermocouple Reader
int thermoDO = 8;
int thermoCS = 9;
int thermoCLK = 10;
int vccPin = 11;
int gndPin = 12;

//pin definitions for MOSFET power transistor
int MOSFET_gndPin = A1;
int MOSFET_vccPin = A2;
int MOSFET_sigPin = A3;

//pin definitions for LCD 
#define I2C_ADDR    0x3F // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

//Declare global variables_________________________________________________________________

//serial communication variables
int incomingByte;
int streamingState=0;

//timing variables
unsigned long currentTimeRead = 0;
unsigned long logTimeStamp = 0;
unsigned long streamTimeStamp = 0;


//data logging variables
unsigned int tempRead=0; //16 bit number representing a temperature read from the thermocouple
int addr = 0;
int addrFree = 0;//the address of the first free byte of EEPROM memory, checking up from addr=0.
unsigned int recalledData=0; //variable into which the tempearature data on the EEPROM can be read




//Functions Defined_________________________________________________________________


MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);// function for thermocouple reader defined

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin); //function for the LCD defined


void findFreeMemoryFunction(){ //locates first free byte of EEPROM memory and write the address to addrFree variable
for (unsigned int i=0; EEPROM.read(i)>0; i++){
  addrFree=i;
 }

//Write start marker "13" into first byte, then sets addrFree forward one byte; ready to log data! 
EEPROM.write(addrFree,13);
addrFree=addrFree+1;  
}



void eraseMemoryFunction(){
  Serial.println("             ERASE EEPROM DATA: Are you SURE?   y / n");
      while (Serial.available()==0){}
      incomingByte = (Serial.read());
      
       if (incomingByte == 121) {
        Serial.println("                          OK... Erasing All Data...");
        for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
        }
        Serial.println("                          EEPROM has been erased.");
        
       incomingByte = 1;
       }
        if (incomingByte!=121 && incomingByte!=1) {
        Serial.println("                          Erase CANCELLED.");
        Serial.println("");
        incomingByte = 1;
        }
}

void downloadMemoryFunction(){
    addr = 0;
  for(int i = 0; i < EEPROM.length(); i++){
    recalledData=EEPROM.read(addr);
    Serial.println(recalledData);
    addr++;
    }
 
}

void displayInfoFunction(){
  Serial.println("*******************************************************");
  Serial.println("* Open Source Autoclave Controller                    *");
  Serial.println("* Build No.180505.1                                   *");
  Serial.println("* written by David Hartkop, 2018                      *");
  Serial.println("* Distributed freely for use, sale, and modification  *");
  Serial.println("* by Idea Propulsion Systems LLC                      *");
  Serial.println("*******************************************************");
}

void streamDataFunction(){
  Serial.print(thermocouple.readCelsius());
  Serial.print(" C ");
  Serial.print(thermocouple.readFahrenheit());
  Serial.println(" F");
  streamTimeStamp = currentTimeRead;
  incomingByte=0;
}

void logDataFunction(){
      tempRead =   thermocouple.readCelsius(); //read thermocouple into tempRead varialbe
       EEPROM.write(addrFree, tempRead); //store byte in the EEPROM memory
        addrFree++;
  logTimeStamp = currentTimeRead;
}

void thermostatFunction(){
  
}

void timetempIntegratorFunction(){
  
}


void LCDUpdateFunction(){
  
 lcd.setCursor(0,0);
 lcd.print(thermocouple.readCelsius());
 lcd.print("C     ");  
 //lcd.setCursor(0,1);
 lcd.print(thermocouple.readFahrenheit());  
 lcd.print("F");   
  
}


void soundsFunction(int x){
  int freq = 0;
  int tonedelay = 50;

  if (x==1){
  //Solar Exposure Good   
    freq = 30;
    for (int i=0; i < 10; i=i+1){
    freq = freq + 100;
     tone(speakerPin2,freq);
     delay(tonedelay);
     noTone(speakerPin2);
     }
  }

  if (x==2){
  //Solar Exposure Faded
    freq = 1000;
    for (int i=0; i < 10; i=i+1){
    freq = freq - 100;
    tone(speakerPin2,freq);
    delay(tonedelay);
    noTone(speakerPin2);
    }
  }

  if (x==3){
  //Steralization Cycle Complete
    tone(speakerPin2,440);
    delay(200);
    noTone(speakerPin2);
    tone(speakerPin2,659);
    delay(200);
    noTone(speakerPin2);
    tone(speakerPin2,880);
    delay(100);
    noTone(speakerPin2);
    delay(100);
    tone(speakerPin2,880);
    delay(100);
    noTone(speakerPin2);
    delay(50);
    tone(speakerPin2,880);
    delay(200);
    noTone(speakerPin2); 
  }

  if (x==4){
  //Error Warning
    for (int i=0; i < 10; i++){
    tone(speakerPin2,60);
    delay(100);
    noTone(speakerPin2);
    delay(100);
    }
  }


}


//Setup_________________________________________________________________

void setup() {

//initialize speaker pinmodes
  pinMode(speakerPin1, OUTPUT); digitalWrite(speakerPin1, LOW);
  pinMode(speakerPin2, OUTPUT); digitalWrite(speakerPin2, LOW);
  
// initialize push button pinmodes
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, OUTPUT); digitalWrite(buttonPin2, LOW);
  
//initialize thermocouple reader pinmodes
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);

//initialize pins for MOSFET module pinmodes
  pinMode(MOSFET_gndPin, OUTPUT); digitalWrite(A1, LOW); 
  pinMode(MOSFET_vccPin, OUTPUT); digitalWrite(A2, HIGH); 
  pinMode(MOSFET_sigPin, OUTPUT); digitalWrite(A3, HIGH); 

// initialzie LCD
  lcd.begin (16,2); //  <<----- My LCD was 16x2
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE); // Switch on the backlight
  lcd.setBacklight(HIGH);  

//initialize serial communication
  Serial.begin(9600);
  delay(2000); //allow Serial interface to stabilize
  Serial.println();
  Serial.println(" Enter d=download EEPROM, e=erase EEPROM, s=stream live data, i=info");

//find & mark a location to start logging data to EEPROM
findFreeMemoryFunction();

//play a happy start-up sound!
soundsFunction(1);

}

//Main Loop Begin_________________________________________________________________

void loop() {
  
  currentTimeRead = millis(); //Update the currentTimeRead variable


  incomingByte = (Serial.read()); //reads serial data in the register into a program variable

//ERASE ALL EEPROM BYTES TO ZERO
if (incomingByte==101){ //101="e"
  eraseMemoryFunction();
    }

//DOWNLOAD RECORDED TEMPERATURE DATA FROM EEPROM TO SERIAL MONITOR
 if (incomingByte == 100){ //100 = "d"
   downloadMemoryFunction();
    }

//DISPLAY INFO ABOUT BUILD
 if (incomingByte == 105){  //105 = "i"
     displayInfoFunction();
    }

//START LIVE-STREAMING TEMPEARATURE DATA TO SERIAL MONITOR
   if (incomingByte == 115 || (streamingState ==1)){  //115 = "s"
      streamingState = 1;
      if (currentTimeRead - streamTimeStamp >= 1000){ //<-This number is the data stream delay in ms
        streamDataFunction();
        }     
    }
    
//STOP LIVE-STREAMING TEMPEARATURE DATA TO SERIAL MONITOR
    if (incomingByte == 115 && streamingState ==1){  //115 = "s"
      streamingState = 0;
    }
 
//HANDLES EXCEPTIONS TO SERIAL INPUT BY DISPLAYING MENU OPTIONS
 if(incomingByte > 0){  
      Serial.println(" Enter d=download EEPROM, e=erase EEPROM, s=stream live data, i=info");
    }

//DATA LOGGER - Logs temperature readings to EEPROM
if (currentTimeRead - logTimeStamp >= 60000){ //<-This number is the delay between logged samples in ms
logDataFunction();
}

//THERMOSTAT - Keeps temperature from crossing set point and burning up autoclave tape & instrument wrappings
thermostatFunction();

//LCD UPDATE - writes relevant information to the LCD
LCDUpdateFunction();



 delay(500);//delay to keep thermocouple reads stable
}
