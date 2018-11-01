
//OpenAutoclave Software version 181031.1 by David Hartkop
//Creative Commons Attribution ShareAlike 3.0 License
//https://creativecommons.org/licenses/by-sa/3.0/

/* __________CONFIGURATION VARIABES____________*/
int setPoint_heater = 160; //DEFAULT 160 degrees Celsius. The Autoclave will reach and hold this temp. for the exposure time  
int exposureTime = 120; //DEFAULT 120 minutes for which the autoclave will be held at the SetPoint_heater temperature
int displayUnits = 0; // enter 0 for Celsius, DEFAULT 0 enter 1 for Fahrenheit display on the LCD


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

//Declare variables_________________________________________________________________

//serial communication variables
int incomingByte;


//timing variables
unsigned long currentTimeRead = 0;
unsigned long logTimeStamp = 0;
unsigned long thermostatTimeStamp = 0;
unsigned long countdownTimeStamp = 0; 

//data logging variables
unsigned int tempRead=0; //16 bit number representing a temperature read from the thermocouple
int addr = 0;
int addrFree = 0;//the address of the first free byte of EEPROM memory, checking up from addr=0.
unsigned int recalledData=0; //variable into which the tempearature data on the EEPROM can be read

//temperature control variables                      
int setPoint_upper = setPoint_heater + 3;
int setPoint_lower = setPoint_heater + 2;
int heaterState = 0;
    
//countdown variable
unsigned long countDownSec = exposureTime * 60; //converts minutes exposure time to seconds of countdown


//Functions Defined_________________________________________________________________


MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);// function for thermocouple reader defined

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin); //function for the LCD defined

void displayTempFunction(int x){

if (x == 0){
    lcd.print(thermocouple.readCelsius());
    lcd.print("C");
  }
  else{
    lcd.print(thermocouple.readFahrenheit());
    lcd.print("F");
  }
  
}

void findFreeMemoryFunction(){ //locates first free byte of EEPROM memory and write the address to addrFree variable
for (unsigned int i=0; EEPROM.read(i)>0; i++){
  addrFree=i;

  if (i == EEPROM.length()){ //If there is no free space found in the entire EEPROM, this subroutine erases all bytes to 0.
    for (unsigned int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);}
  }
 }

  

//Write start marker "13" into first byte, then sets addrFree forward one byte; ready to log data! 
EEPROM.write(addrFree,13);
addrFree=addrFree+1;  
}



void eraseMemoryFunction(){
        Serial.println("                          OK... Erasing All Data...");
        for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
        }
        Serial.println("                          EEPROM has been erased.");
        
       incomingByte = 1;
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
  Serial.println("* Build No.181031.1                                   *");
  Serial.println("* written by David Hartkop, 2018                      *");
  Serial.println("* Distributed freely for use, sale, and modification  *");
  Serial.println("* by Idea Propulsion Systems LLC                      *");
  Serial.println("*******************************************************");
}


void logDataFunction(){
if (currentTimeRead - logTimeStamp >= 60000){ //<-This number is the delay between logged samples in ms
  
      tempRead =   thermocouple.readCelsius(); //read thermocouple into tempRead varialbe
       EEPROM.write(addrFree, tempRead); //store byte in the EEPROM memory
        addrFree++;
  logTimeStamp = currentTimeRead;
  }
}

void thermostatFunction(){ //Simple hysteresis control. No frills. It is more relay-friendly than a PID controller.

if (currentTimeRead - thermostatTimeStamp >= 4000){ //<-This number is the delay between thermostat samples in ms
  tempRead = thermocouple.readCelsius(); //read thermocouple into tempRead varialbe
  if (heaterState == 1 && tempRead >= setPoint_upper){ //temp has reached upper setpoint, turn off the heater
    heaterState = 0;
  }

  if (heaterState == 0 && tempRead <= setPoint_lower){ //temp has fallen to the lower setpoint, turn heater back on
    heaterState = 1;
  }
  
  if (heaterState == 0){
   digitalWrite(MOSFET_sigPin, HIGH); //Writing HIGH means you are cutting power to the heaters.
  }

  else {
   digitalWrite(MOSFET_sigPin, LOW);//Writing LOW means you are sending power to the heaters.
  } 

  thermostatTimeStamp = currentTimeRead;
  }
}


void LCDUpdateFunction(){

    //Display the temperature
    lcd.setCursor(0,0);

displayTempFunction(displayUnits);

    //calculate min & sec values
    int minutes = (countDownSec/(60));
    int seconds = (countDownSec)-minutes*60;

    //display min & sec remaining on timer
    lcd.setCursor(0,1);
    lcd.print(minutes);
    lcd.print(" min ");
    lcd.print(seconds);
    lcd.print(" sec "); 

  //pause the timer and show "Heating" message if temp is below the set point
  if (thermocouple.readCelsius()<setPoint_heater){
    lcd.setCursor(8,0);
    lcd.print("Heating ");
    delay (500);
    lcd.setCursor(8,0);
    lcd.print("to ");
      if (displayUnits == 0){
        lcd.print(setPoint_heater);
        lcd.print("C");
        }
      else{
        lcd.print((setPoint_heater*(1.8))+32,0);
        lcd.print("F");
  }

    
    delay (250);
  }

  else{ //allows the timer to run
    lcd.setCursor(8,0);
    lcd.print("        ");
  if (currentTimeRead - countdownTimeStamp >= 1000){ 
    countDownSec = countDownSec-1;
    
    if (countDownSec < 1){
   
    while(true){ //while loops the 'done' alert sound and keeps heaters disabled.
    digitalWrite(MOSFET_sigPin, HIGH); //Writing HIGH means you are cutting power to the heaters.
    lcd.setCursor(0,0);
    displayTempFunction(displayUnits);
    delay(1000);
    lcd.setCursor(0,1);
    lcd.print("Cycle Complete!");
    soundsFunction(3);
    logDataFunction();
      }
     }
  countdownTimeStamp = currentTimeRead;
    }
  
  }
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
    delay(500);
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

void softReset(){ //Function jumps thread to start of program, restarting all functions.
asm volatile ("  jmp 0");
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
  pinMode(MOSFET_gndPin, OUTPUT); digitalWrite(MOSFET_gndPin, LOW); 
  pinMode(MOSFET_vccPin, OUTPUT); digitalWrite(MOSFET_vccPin, HIGH); 
  pinMode(MOSFET_sigPin, OUTPUT); digitalWrite(MOSFET_sigPin, LOW); //Set sigPin HIGH to cut power to heaters. MOSFET activates a relay that opens the circuit...

// initialzie LCD
  lcd.begin (16,2); //  <<----- My LCD was 16x2
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE); // Switch on the backlight
  lcd.setBacklight(HIGH);  


//initialize serial communication
  Serial.begin(9600);
  delay(2000); //allow Serial interface to stabilize
  Serial.println();
Serial.println(" Enter d=download EEPROM, *=erase EEPROM, t=current temp read, i=info");

//find & mark a location to start logging data to EEPROM
findFreeMemoryFunction();

//play a happy start-up sound!
soundsFunction(1);



}

//Main Loop Begin_________________________________________________________________

void loop() {
delay(500);//half second delay to keep thermocouple reads stable
  
currentTimeRead = millis(); //Update the currentTimeRead variable

incomingByte = (Serial.read()); //reads serial data in the register into a program variable

//LCD UPDATE - scans inputs and writes relevant information to the LCD
LCDUpdateFunction();


//ERASE ALL EEPROM BYTES TO ZERO
if (incomingByte==42){ //42="*"
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


//CURRENT TEMP READ
   if (incomingByte == 116){  //116 = "t"
      Serial.print(thermocouple.readCelsius());
  Serial.print("˚C ");
  Serial.print(thermocouple.readFahrenheit());
  Serial.print("˚F Timestamp: ");
  Serial.println(currentTimeRead);
 incomingByte=0;
    }

//HANDLES EXCEPTIONS TO SERIAL INPUT BY DISPLAYING MENU OPTIONS
 if(incomingByte > 11){  
      Serial.println(" Enter d=download EEPROM, *=erase EEPROM, t=current temp read, i=info");
    }


    
//DATA LOGGER - Logs temperature readings to EEPROM
logDataFunction();


//THERMOSTAT - Keeps temperature from crossing set point and burning up autoclave tape & instrument wrappings
thermostatFunction();


//RESET BUTTON - Hold down the button for 5 seconds and the OpenAutoclave software jumps to line 0.
while(digitalRead(buttonPin1) == LOW){ 
lcd.setCursor(0,1);
lcd.print("Hold to RESET");

  if(millis() - currentTimeRead > 5000){ 
     soundsFunction(2);
     softReset();
  }


}


}
