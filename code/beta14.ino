/*
  Project: GPS buoy for position logging from underwater.
  Function: This sketch listens to a GPS module and when a fix meets certain criteria, 10 lines of position information are stored on the SDcard.
  The operator can see the status of the logging using an RGB LED (or seperate LEDs).

  Wiring GPS module:
  GPS module -> Arduino Pro Mini 8MHz 3V3
  GND           GND
  RX pin        not used, GPS can only TX
  TX pin        Digital pin 8
  VCC pin       3V3

  Wiring SD card module:
  SD card module -> Arduino Pro Mini 8MHz 3V3
  VCC           3.3V
  CS            Digital pin 10
  D1/MOSI       Digital pin 11
  SCK/CLK       Digital pin 13
  D0/MISO       Digital pin 12
  GND           GND

 Description:  This program logs a certain amount of GPS fields in a tabular fashion on a SDcard.
 You must also enable HDOP in GPSfix_cfg.h.
     Most NeoGPS examples display *all* configured GPS fields in a CSV format
     (e.g., NMEA.ino).
  Prerequisites:
     1) NMEA.ino works with your device (correct TX/RX pins and baud rate)
     2) GPS_FIX_HDOP is defined in GPSfix_cfg.h
  'Serial' is for debug output to the Serial Monitor window.
  License:
    Copyright (C) 2014-2017, SlashDevin
    This file is part of NeoGPS
    NeoGPS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    NeoGPS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
*/
//************************************************************
#include <SPI.h>//include library code to communicate with SPI devices
#include <SD.h>//include library code for SD card
#include <NMEAGPS.h> //the GPS library
#include <GPSport.h> //The used port configuration set for NMEAGPS

//**********************************************************************
NMEAGPS     gps;
const int chipSelect = 10; //SDcard is set to pin 10.
// Check configuration
#ifndef GPS_FIX_HDOP
#error You must uncomment GPS_FIX_HDOP in GPSfix_cfg.h!
#endif
//**********************************************************************
//Button configuration

const int buttonPin = 6;     // the number of the pushbutton pin
const int btpowerpin = A0; //setting the bluetooth power pin

//LED configuration
const int blueled = 3;
const int redled = 5;
const int greenled = 4;
int ledState = LOW;
//int redState = LOW;
//int greenState = LOW;

unsigned long previousMillis = 0;  //stores the last time the LED was updated
const long interval = 200; //Interval at which to blink LED
int buttonState = 0;
int counter = 0;
boolean printheaders = false; //needed for a one time run in loop

void setup()
{
  pinMode(buttonPin, INPUT); //The button is used to start another logging cycle
  pinMode(btpowerpin, OUTPUT); //setting the power pin of the BT module.
  pinMode(blueled, OUTPUT); //Blue part of the RGB Led
  pinMode(redled, OUTPUT); //Red part of the RGB Led
  pinMode(greenled, OUTPUT); //Green part of the RGB Led
  digitalWrite(greenled, HIGH); //As I'm currently using common Anode RGB Led, this is set to High. When using common Cathode, use LOW instead.
  digitalWrite(redled, HIGH); //As I'm currently using common Anode RGB Led, this is set to High. When using common Cathode, use LOW instead.
  digitalWrite(blueled, HIGH); //As I'm currently using common Anode RGB Led, this is set to High. When using common Cathode, use LOW instead.
  //DEBUG_PORT.begin(9600);
  gpsPort.begin(9600);
  //Serial.print("Initializing SD card...");//setup for the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present!");
    digitalWrite(blueled, LOW);
    digitalWrite(redled, LOW);
    delay(5000);
    return;
  }
  //Serial.println("Card initialized.");
  //Serial.println("File opened. Start logging GPS data to SD card:");
 
  //DEBUG_PORT.println
  //(
  //  F( "Sats HDOP Latitude  Longitude  Date       Time    Speed  Heading  \n"
  //     "          (deg)     (deg)                                         \n" )
  //);

  //repeat( '-', 133 );
 
}
void loop()
{
    digitalWrite(blueled, HIGH);
   
            if ((counter == 0) && (printheaders == false))  { //making sure this runs only once
      printheaders = true;
      File dataFile = SD.open("gpsdata.csv", FILE_WRITE);
        if (dataFile) {     // if the file is available, write to it:
        dataFile.println("Date, Time , Lat , Long , Sats , HDOP , Speed");//prints the headers for our data
        }
        dataFile.close();//file closed  
    }
    if (gps.available( gpsPort ) && (counter <= 9)) {
    gps_fix fix = gps.read();
                                   
    if (fix.valid.hdop && (fix.hdop < 2000)){  
        bool  validDT         = fix.valid.date & fix.valid.time;
               
               // print(             fix.satellites       , fix.valid.satellites, 3             );
               // print(             fix.hdop / 1000.0      , fix.valid.hdop      , 6, 2          );
               // print(             fix.latitude ()      , fix.valid.location  , 10, 6         );
               // print(             fix.longitude()      , fix.valid.location  , 11, 6         );
               // DEBUG_PORT.println();
   
                   
    File dataFile = SD.open("gpsdata.csv", FILE_WRITE);
    digitalWrite(redled, HIGH);
    digitalWrite(greenled, LOW); //data is being written to the card
    if (dataFile) {     // if the file is available, write to it:
     
       
      if (fix.dateTime.date < 10)
        dataFile.print( '0' );
      dataFile.print(fix.dateTime.date);
      dataFile.print( '/' );
      if (fix.dateTime.month < 10)
        dataFile.print( '0' );
      dataFile.print(fix.dateTime.month);
      dataFile.print( '/' );
      dataFile.print(fix.dateTime.year);
      dataFile.print(',');

      if (fix.dateTime.hours < 10)
        dataFile.print( '0' );
      dataFile.print(fix.dateTime.hours);
      dataFile.print( ':' );
      if (fix.dateTime.minutes < 10)
        dataFile.print( '0' );
      dataFile.print(fix.dateTime.minutes);
      dataFile.print( ':' );
      if (fix.dateTime.seconds < 10)
        dataFile.print( '0' );
      dataFile.print(fix.dateTime.seconds);
      dataFile.print(',');
      dataFile.print(fix.latitude (), 10);
      dataFile.print(",");
      dataFile.print(fix.longitude(), 11);
      dataFile.print(",");
      dataFile.print(fix.satellites, 3);
      dataFile.print(",");
      dataFile.print(fix.hdop / 1000.0, 6);
      dataFile.print(",");
      dataFile.print(fix.heading (), 7);
      dataFile.print(",");
      dataFile.print(fix.speed_kph(), 7);
      dataFile.print("\n");
      dataFile.close();
    //Serial.println(counter, DEC);
    counter++;
    }
   }
   else {
      noFix();
    }
    }
    else {
  digitalWrite(greenled, HIGH);    //Lines written, no more write actions.
  if (counter >= 10){ //once 10 lines are written, called poslog, positive log to stop writing lines and wait for operator input
    poslog();
  }
    }
}  


void noFix() //blink the red LED to show there is no fix
{
  unsigned long currentMillis = millis();
  //digitalWrite(greenled, HIGH);
    if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == HIGH) {
      ledState = LOW;
    } else {
      ledState = HIGH;
    }
  }
    // set the LED with the ledState of the variable:
    digitalWrite(redled, ledState);
}

void poslog()
  {
   unsigned long currentMillis = millis();
   digitalWrite(blueled, HIGH);
        while (digitalRead(buttonPin) == HIGH) { //Wait for the operator to press butan
          counter = 0;
          printheaders = false;
          break;
        }
        if (currentMillis - previousMillis >= interval) { //else, just blink the led
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == HIGH) {
        ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
  }
    // set the LED with the ledState of the variable:
    digitalWrite(blueled, ledState);
    }
   

//-----------------
//  Print utilities

static void repeat( char c, int8_t len )
{
  for (int8_t i = 0; i < len; i++)
    DEBUG_PORT.write( c );
}

static void printInvalid( int8_t len )
{
  DEBUG_PORT.write( ' ' );
  repeat( '*', abs(len) - 1 );
}

static void print( float val, bool valid, int8_t len, int8_t prec )
{
  if (!valid) {
    printInvalid( len );
  } else {
    char s[16];
    dtostrf( val, len, prec, s );
    DEBUG_PORT.print( s );
  }
}

static void print( int32_t val, bool valid, int8_t len )
{
  if (!valid) {
    printInvalid( len );
  } else {
    char s[16];
    ltoa( val, s, 10 );
    repeat( ' ', len - strlen(s) );
    DEBUG_PORT.print( s );
  }
}

static void print( const __FlashStringHelper *str, bool valid, int8_t len )
{
  if (!valid) {
    printInvalid( len );
  } else {
    int slen = strlen_P( (const char *) str );
    repeat( ' ', len - slen );
    DEBUG_PORT.print( str );
  }
}

static void print( const NeoGPS::time_t & dt, bool valid, int8_t len )
{
  if (!valid) {
    printInvalid( len );
  } else {
    DEBUG_PORT.write( ' ' );
    Serial << dt; // this "streaming" operator outputs date and time
  }
}

//------------------------------------------------------------
//  This snippet is from NMEAaverage.  It keeps all the
//    compass direction strings in FLASH memory, saving RAM.

const char nCD  [] PROGMEM = "N";
const char nneCD[] PROGMEM = "NNE";
const char neCD [] PROGMEM = "NE";
const char eneCD[] PROGMEM = "ENE";
const char eCD  [] PROGMEM = "E";
const char eseCD[] PROGMEM = "ESE";
const char seCD [] PROGMEM = "SE";
const char sseCD[] PROGMEM = "SSE";
const char sCD  [] PROGMEM = "S";
const char sswCD[] PROGMEM = "SSW";
const char swCD [] PROGMEM = "SW";
const char wswCD[] PROGMEM = "WSW";
const char wCD  [] PROGMEM = "W";
const char wnwCD[] PROGMEM = "WNW";
const char nwCD [] PROGMEM = "NW";
const char nnwCD[] PROGMEM = "NNW";

const char * const dirStrings[] PROGMEM =
{ nCD, nneCD, neCD, eneCD, eCD, eseCD, seCD, sseCD,
  sCD, sswCD, swCD, wswCD, wCD, wnwCD, nwCD, nnwCD
};

const __FlashStringHelper *compassDir( uint16_t bearing ) // degrees CW from N
{
  const int16_t directions    = sizeof(dirStrings) / sizeof(dirStrings[0]);
  const int16_t degreesPerDir = 360 / directions;
  int8_t  dir           = (bearing + degreesPerDir / 2) / degreesPerDir;

  while (dir < 0)
    dir += directions;
  while (dir >= directions)
    dir -= directions;

  return (const __FlashStringHelper *) pgm_read_ptr( &dirStrings[ dir ] );

} // compassDir
