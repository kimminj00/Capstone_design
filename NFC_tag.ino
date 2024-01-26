#include <Ethernet.h>

//NFC
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

//와이파이
#include <string.h> 
#include <SoftwareSerial.h>
#include "ESP8266.h"
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>

#define PN532_IRQ   (9)
#define PN532_RESET (8)

#define vibeMotor 13

//와이파이 연결정보
#define SSID "wifi_name"
#define PASSWORD "wifi_password"
#define SERVERIP "serverIP"
SoftwareSerial mySerial(2,3);

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

void setup() {
  pinMode(vibeMotor,OUTPUT);
  
  Serial.begin(9600);
  while(!Serial) delay(10);

  mySerial.begin(9600);
  mySerial.println("AT+RST\r\n");
  delay(1000);
  mySerial.println("AT+CWMODE=3\r\n");
  
  Serial.println("ESP8266 wifi 연결");
  boolean connected=false;
  for(int i=0;i<10;i++){
    if(connectWiFi()){  
      connected = true;  
      break;
    }
  }
  if (!connected){
    while(1);
  }
  delay(5000);
  mySerial.println("AT+CIPMUX=0");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board"); 
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  
  Serial.println("Waiting for an ISO14443A Card ...");
}

void loop(void) {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint8_t uidLength;  // Length of the UID (4 or 7 bytes depending on ISO14443A card type) 

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // Display some basic information about the card

    String nfcnum = printDec(uid, 3);
    String place = "물리치료실";

    Serial.println("환자가 "+place+"에 있습니다");
    delay(3000);

    while(1)
    {
      String cmd = "AT+CIPSTART=\"TCP\",\"";
      cmd += SERVERIP;
      cmd += "\",80";
      Serial.println(cmd);
      mySerial.println(cmd);
      delay(3000);
      if(mySerial.find("Error"))
      {
        Serial.println("TCP connect error");
        return;
      }

      delay(5000);
  
      cmd = "GET /NFC.php?PatientNum="+nfcnum+"&Place="+place;
      cmd += " HTTP/1.0\r\n";
      cmd += "Host: leemina316.dothome.co.kr\r\n";
      cmd += "\r\n";
      cmd += "\r\n";
      mySerial.print("AT+CIPSEND=");
      mySerial.println(cmd.length());
  
      Serial.println(cmd);
  
      if(mySerial.find(">"))
      {
        Serial.print(">");
      }else
      {
        mySerial.println("AT+CIPCLOSE");
        Serial.println("connect timeout");
        delay(1000);
        return;
      }
  
      mySerial.print(cmd);
      delay(2000);
      //Serial.find("+IPD");
      while(Serial.available())
      {
        char c = Serial.read();
        mySerial.write(c);
        if(c=='\r') mySerial.print('\n');
      }
      Serial.println("데이터 전송완료");
      analogWrite(vibeMotor, 10000);
      delay(1000);
      analogWrite(vibeMotor, LOW);
      delay(1000);
      break;
      }
  }
}

boolean connectWiFi()
{
  String cmd = "AT+CWJAP=\"";
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASSWORD;
  cmd+="\"";
  mySerial.println(cmd);
  Serial.println(cmd);
  delay(5000);
    if(mySerial.find("OK"))
    {
      Serial.println("Wifi 연결 완료.");
      analogWrite(vibeMotor, 10000);
      delay(1000);
      analogWrite(vibeMotor, LOW);
      delay(1000);
      return true;
    }
    else
    {
      Serial.println("Wifi 연결 중");
      return false;
    }
}

String printDec(byte *buffer, int bufferSize)
{
  String ID = "";
  for(byte i=1; i<bufferSize; i++)
  {
    Serial.print(buffer[i] < 0x10 ? "0" : "");
    Serial.print(buffer[i], DEC);
    String a(buffer[i]);
    ID += buffer[i];
  }
  return (ID);
}
