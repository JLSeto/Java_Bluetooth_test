#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Memsic.h>
#include "Adafruit_SHT31.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
#define BNO055_SAMPLERATE_DELAY_MS (1)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Memsic hello;

int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

long previousMillis;
long currentMillis;
int StartCommand;
String ax, ay, az,
       t_sht31, h_sht31,
       temp, flow, dataString, t;
int sensorValue;
float voltage;
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  iSDcard();
  iBluetooth();
  iSHT();
  iMemsic();

  //Serial.println("Setup Complete");
}

void loop() {

  if(bluetooth.available()){
    StartCommand = bluetooth.read();
    }

  while(StartCommand == 49){
    currentMillis = millis();
    t = String(currentMillis);
    if(currentMillis - previousMillis >= 0){
      previousMillis = currentMillis;
      updateIMUData();
      updateMemsic(); //*flow rate
      updateSHT();  // *Pod temp
      updateData(); //
      readBattery();
      //Serial.println(dataString);
      bluetooth.print(dataString);
      //bluetooth.print(voltage);
      bluetooth.write(" ");
      bluetooth.flush();
      writeDataFile();
   if(bluetooth.available()){
    StartCommand = bluetooth.read();
    }else{StartCommand=49;}


    }
    }


}


void iSDcard(){
  while (!Serial){
    ;}
  //Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  //Serial.println("Card Initialized.");
  }

void iBluetooth(){
  bluetooth.begin(115200);
  bluetooth.print("$");
  bluetooth.print("$");
  bluetooth.print("$");
  delay(100);
  bluetooth.println("U,9600,N");
  bluetooth.begin(9600);
  bno.begin();
  //Serial.println("Bluetooth Initialized");
  }

void iSHT(){
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    //Serial.println("Couldn't find SHT31");
    while (1) delay(1);
    //Serial.println("SHT Sensor Initialized");
  }
  }

void iMemsic(){
  while(!Serial);
  Wire.begin();
  //Serial.println("Memsic Initialized");
  }

void updateIMUData(){
  sensors_event_t event;
  bno.getEvent(&event);
  ax = String(event.orientation.x, 2);
  ay = String(event.orientation.y, 2);
  az = String(event.orientation.z, 2);

  delay(BNO055_SAMPLERATE_DELAY_MS);

}

void updateMemsic(){
  flow = String(hello.read_i2cflow());
  temp = String(hello.read_i2ctemp());

  }

void updateSHT(){
  t_sht31 = String(sht31.readTemperature());
  h_sht31 = String(sht31.readHumidity());
  }

void updateData(){
//  dataString = t + ","
//                 + flow + "," + temp
//               + "," +h_sht31 + "," + t_sht31 + "," + String(voltage);

  dataString = t + "," +
               ax + "," + ay + "," + az + "," +
               flow + "," + temp + "," +h_sht31 + "," + t_sht31 + "," + String(voltage);

  }

void writeDataFile(){
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
    //Serial.println("error opening datalog.txt");
    }
  }

void readBattery(){
   sensorValue = analogRead(A0);
   voltage = sensorValue * (5.0 / 1023.0);

  }
