//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define RXD2 16
#define TXD2 17

BluetoothSerial SerialBT;
String message = "";
String response;
char incomingChar;

void respondWith(String response) {
  response = response.c_str();
  for (int i = 0; response[i] != '\0';i++) {
    SerialBT.write(response[i]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
}

void loop() {
  // if (Serial.available()) {
  //   SerialBT.write(Serial.read());
  // }
  if (SerialBT.available()){
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n'){
      message += String(incomingChar);
    }
    else{
      message = "";
    }
    Serial.write(incomingChar);  
  }
  // Check received message and control output accordingly
  if (message =="status"){
  }
  else if (message =="4"){
    response = "TURNING LEFT";
    Serial2.write(0x31);
  }
  else if (message == "2") {
    response = "TURNING RIGHT";
    Serial2.write(0x32);
  }
  else if (message == "1") {
    response = "MOVING FORWARD";
    Serial2.write(0x33);
  }
  else if (message == "3") {
    response = "MOVING BACKWARD";
    Serial2.write(0x34);
  }
  else if (message == "8") {
    response = "MOVING LEFT FORWARD";
    Serial2.write(0x35);
  }
  else if (message == "5") {
    response = "MOVING RIGHT FORWARD";
    Serial2.write(0x36);
  }
  else if (message == "7") {
    response = "MOVING LEFT BACKWARD";
    Serial2.write(0x37);
  }
  else if (message == "6") {
    response = "MOVING RIGHT BACKWARD";
    Serial2.write(0x38);
  }
  else if (message == "9") {
    response = "TURNING LEFT FULL";
    Serial2.write(0x39);
  }
  else if (message == "k") {
    response = "TURNING RIGHT FULL";
    Serial2.write(0x3a);
  }
  else if (message == "0") {
    response = "STOPPING";
    Serial2.write(0x3b);
  }
  else if (message == "f") {
    response = "FULL SPEED";
    Serial2.write(0x11);
  }
  else if (message == "h") {
    response = "HALF SPEED";
    Serial2.write(0x12);
  }
  else if (message == "e") {
    response = "PLAY END TONE";
    Serial2.write(0x99);
  }
  else if (message == "se") {
    response = "STOP TONE";
    Serial2.write(0x98);
  }
  respondWith(response);
  message = "";
  delay(20);
}
