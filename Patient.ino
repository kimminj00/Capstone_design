#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MPU6050.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <nRF24L01.h>
#include <RF24.h>



#define OLED_RESET LED_BUILTIN
Adafruit_SSD1306 display(OLED_RESET);

//#define SCREEN_WIDTH 128 // OLED 가로 넓이, 픽셀 사이즈  
//#define SCREEN_HEIGHT 32 // OLED 세로 넓이, 픽셀 사이즈

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
 
#define SSD1306_LCDHEIGHT 64
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif


SoftwareSerial HCSerial(5,6); // HC-06

long patient_num =  ;   

      
//SoftwareSerial btSerial (BT_RXD, BT_TXD); // nRF24
RF24 radio(7,8);
const byte address[6] = "00002";

int touchSensor = 3;
const int mpu_add = 0x68;

static const unsigned char PROGMEM battery_full [] = {
0xFF, 0xFE, 0x80, 0x06, 0x9B, 0x33, 0xBB, 0xB3, 0xBB, 0xB3, 0xBB, 0xB3, 0x80, 0x02, 0xFF, 0xFE   };

static const unsigned char PROGMEM battery_empty []= {
0xFF, 0xFE, 0x80, 0x02, 0x98, 0x03, 0xB8, 0x03, 0xB8, 0x03, 0xB8, 0x03, 0x80, 0x02, 0xFF, 0xFE } ;

void value_init();
void mpu_6050_init();
void accel_calculate();
int Normal_angle = 30;
int Crash_value = 300;
void Shock_Sensing();
void Emergency_state();
boolean Warn_state = false;
void battery_charge();
void main_oled();


char text=0;
long AcX,AcY,AcZ;
double AngleAcX = 0, AngleAcY = 0;
long normal_x, normal_y, normal_z;
long deltha_x[3], deltha_y[3], deltha_z[3],deltha ;
float Alt_c,Alt_s[2],Alt_r;

unsigned long now = 0;
unsigned long past = 0;
double dt = 0;

long mapping_value = 1000;
long event_value = 1000;

const int sum_count = 4;
const int delay_main = 2;
long dely_config = 6000;

const float RADIANS_TO_DEGREES = 180/PI;
const float DEG_PER_SEC = 32767/250;

Adafruit_BMP280 bmp; // I2C Interface


void setup(){

 if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  

  HCSerial.begin(9600);                
//  btSerial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  analogReference(EXTERNAL);
  display.begin(SSD1306_SWITCHCAPVCC, 0X3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
 
  pinMode(touchSensor,INPUT);

  Serial.begin(9600);

  mpu6050_init();
  past=millis();
  }


void loop(){   
    main_oled();
      
  int touchValue = digitalRead(touchSensor);

  if(touchValue==1){
    text=1;
    radio.write(&text, sizeof(text));
    HCSerial.println(text);
    delay(1000);
    }else { text=0; radio.write(&text,sizeof(text)); delay(1000);}
  
  value_init();
  Shock_Sensing();
  }

    void value_init(){
      normal_x=0; normal_y=0; normal_z=0; 
      for(int i=0;i<3;i++){
        deltha_x[i]=0; deltha_y[i]=0; deltha_z[i]=0; Alt_s[i]=0; Alt_r=0;
        AngleAcX=0; AngleAcY=0; 
        
        }
      }


    void accel_calculate(){
      AcX = 0 ; AcY = 0 ; AcZ = 0; Alt_c =0;
      Wire.beginTransmission(mpu_add);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(mpu_add,14,true);

      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
      Alt_c = bmp.readAltitude(100560);

      normal_x = map(int(AcX),-16384,16384,0,mapping_value);
      normal_y = map(int(AcY),-16384,16384,0,mapping_value);
      normal_z = map(int(AcZ),-16384,16384,0,mapping_value);

      float accel_xz , accel_yz;
      accel_xz = sqrt(pow(AcX,2)+pow(AcZ,2));
      AngleAcX = atan(AcY/accel_xz)*RADIANS_TO_DEGREES;
      accel_yz = sqrt(pow(AcZ,2)+pow(AcY,2));
      AngleAcY = atan(-AcX/accel_yz)*RADIANS_TO_DEGREES;
      
      }


    void Shock_Sensing(){
      getDT();
       for(int i=0;i<sum_count;i++){
        accel_calculate();
        deltha_x[1]=deltha_x[1]+(normal_x);
        deltha_y[1]=deltha_y[1]+(normal_y);
        deltha_z[1]=deltha_z[1]+(normal_z);
        Alt_s[1]=Alt_s[1]+(Alt_c);
      delay(5);
    }
        deltha_x[1]=int(deltha_x[1]/sum_count);
        deltha_y[1]=int(deltha_y[1]/sum_count);
        deltha_z[1]=int(deltha_z[1]/sum_count);
        Alt_s[1]=float(Alt_s[1]/sum_count);
      
      
        for(int i=0;i<sum_count;i++){
        deltha_x[2]=deltha_x[2]+(normal_x);
        deltha_y[2]=deltha_y[2]+(normal_y);
        deltha_z[2]=deltha_z[2]+(normal_z);
       
      delay(5);
      }
      
       deltha_x[2]=int(deltha_x[2]/sum_count);
       deltha_y[2]=int(deltha_y[2]/sum_count);
       deltha_z[2]=int(deltha_z[2]/sum_count);
       
      
   deltha_x[0]=abs(deltha_x[1]-deltha_x[2]);
   deltha_y[0]=abs(deltha_y[1]-deltha_y[2]);
   deltha_z[0]=abs(deltha_z[1]-deltha_z[2]);
   
      
   deltha=deltha_x[0]+deltha_y[0]+deltha_z[0];

//      Serial.print("deltha:");
//      Serial.println(deltha);
  
    if((deltha >= Crash_value)){
      delay(3000);
         Alt_c = bmp.readAltitude(100560);
         Alt_r = Alt_s[1]-(Alt_c);
//          Serial.print("Alt_r:");
         Serial.println(Alt_r);
        if(Alt_r >= 0.02){
//      Serial.println("Drop detection");
      display.setTextSize(2);
      display.setCursor(20,10);
      display.print("Drop");
      display.display();
      Action_config();
     
      }
      else{
        text=0;
//        Serial.println("No problem");
        return 0;}
     }
    }

  void Action_config(){
//      Serial.println("Action_start");
      long TIMER1 = (long)millis()+5000;
      int WARN = 0;

      while((TIMER1 > millis())&&(WARN == 0)){
        value_init();
        accel_calculate();
        AngleAcX = abs(AngleAcX);
        AngleAcY=abs(AngleAcY);
        
        //정상
        if((AngleAcX>Normal_angle)&&(AngleAcY>Normal_angle)){
//          Serial.println("no problem");
          WARN = 1;
          display.clearDisplay();
        }
      }     
      if(WARN == 1) { return 0; }
      else{

      //비상모드
      TIMER1 = (long)millis()+6000;
      while(TIMER1 > millis()){
        Emergency_state();
        display.clearDisplay();
        break;
        }
        }
      }

  void Emergency_state(){
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(20,10);
      display.print("Warning");
      display.display();
      text = 2;
      radio.write(&text,sizeof(text));
      HCSerial.println(text);
//      Serial.println("Warning!");
      delay(2000);
    }

  void mpu6050_init(){
    Wire.begin();
    Wire.beginTransmission(mpu_add);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    }
    
  void getDT(){
    now = millis();
    dt = (now - past)/1000.0;
    past = now;
    }

      void main_oled(){
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
