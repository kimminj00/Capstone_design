#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <stdio.h>
#include <string.h>
#include <U8g2lib.h>

#define OLED_RESET LED_BUILTIN
#define vibe_Motor 3

Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

 
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
//static const unsigned char PROGMEM logo[] = {}; 
 
#define SSD1306_LCDHEIGHT 64
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif



static const unsigned char PROGMEM battery_full [] = {
0xFF, 0xFE, 0x80, 0x06, 0x9B, 0x33, 0xBB, 0xB3, 0xBB, 0xB3, 0xBB, 0xB3, 0x80, 0x02, 0xFF, 0xFE   };

static const unsigned char PROGMEM battery_empty []= {
0xFF, 0xFE, 0x80, 0x02, 0x98, 0x03, 0xB8, 0x03, 0xB8, 0x03, 0xB8, 0x03, 0x80, 0x02, 0xFF, 0xFE } ;


 void battery_charge();
 void main_oled();
 #define patient_num 56187


  
RF24 radio(7,8);
const byte address[6] = "00002";

void setup() {
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  radio.begin();
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  pinMode(4, OUTPUT);

}

void loop() {
  main_oled();
 
 if(radio.available()){
    char text = "";
    radio.read(&text, sizeof(text));
    analogWrite(vibe_Motor,0);
    Serial.println(text);   

  switch(text){

      case 1:
          display.clearDisplay();
          battery_charge();
          analogWrite(vibe_Motor,400);     
          display.setTextSize(2);
          display.setCursor(20,10);
          display.println("TOUCHED");
          display.display();
          delay(500);
          break;

       case 2:
          display.clearDisplay();
          battery_charge();
          analogWrite(vibe_Motor,400);
          display.setTextSize(2);
          display.setCursor(20,10);
          display.print("Warning!");
          display.display();
          delay(5000);
          break; 
          }        
           
               }
  }

  void main_oled(){
   display.clearDisplay();
   display.setTextSize(2);
   display.setCursor(20,10);
   battery_charge();
   display.println(patient_num);
   display.display();
    }

  void battery_charge(){
  int val = analogRead(A0);
  float voltage = val*(3.3/1023.0);
  if(voltage >= 2.5){
      display.drawBitmap(100,0,battery_full,16,8,1);
      display.display();
    }
    else{
      display.clearDisplay();
      display.drawBitmap(100,0,battery_empty,16,8,1);}

   

    }
