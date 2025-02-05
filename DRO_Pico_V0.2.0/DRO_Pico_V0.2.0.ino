/*
* Diese Progran nutz den Raspberry Pico RP240
* Als Bord ist das Das "Arduino Mbed OS RP2040" auszuwählen. Die normale Rasberry Pico Libary funktionieren nicht.
*/
#include <TM1637TinyDisplay6.h>
#include <Wire.h> 
#include "mpr121.h"
#include "pio_encoder.h"
#include <NeoPixelConnect.h>
#define MAXIMUM_NUM_NEOPIXELS 12 // WS2812 for Status 
#define tasterEncode_y0 22 //Decimal-to-BCD Encoder A0 for botton 1-8
#define tasterEncode_y1 26 //Decimal-to-BCD Encoder A1 for botton 1-8
#define tasterEncode_y2 27 //Decimal-to-BCD Encoder A2 for botton 1-8
#define tasterEncode_y3 28 //Decimal-to-BCD Encoder A3 for botton 1-8

/*
Change for I2C in pins_arduino.h for Raspberry PICO (in Mac: Users/xxxxx/Library/Arduino15/packages/arduino/hardware/mbed_rp2040/x.x.x/variants/RASPBERRY_PI_PICO/pins_arduino.h)
in the wire section the I2C Pins in which you use. Default is GPIO4 and GPIO5

// Wire
#define PIN_WIRE_SDA        (12u) -> Pin GPIO12
#define PIN_WIRE_SCL        (13u) -> Pin GPIO13

*/#define MPR_ADD 0x5A // I2C Adress MPR121

int irqPin = 11;  // IRQ Pin for MPR121
boolean touchStates[12]; //to keep track of the previous touch states

NeoPixelConnect p(0, MAXIMUM_NUM_NEOPIXELS, pio1, 1);

PioEncoder encoderZ0(2); // encoder is connected to GPIO2 and GPIO3
PioEncoder encoderZ1(4); // encoder is connected to GPIO4 and GPIO5
PioEncoder encoderX(6);  // encoder is connected to GPIO6 and GPIO7
PioEncoder encoderZR(8); // encoder is connected to GPIO8 and GPIO9
// Module TM1637 connection pins (Digital Pins)
// TM1637TinyDisplay display(CLK, DIO);
TM1637TinyDisplay6 displayZ0(21,20);
TM1637TinyDisplay6 displayZ1(19,18);
TM1637TinyDisplay6 displayX(17,16);
TM1637TinyDisplay6 displayZR(15,14);

int renameKey;
boolean ready = false;
boolean setNumberZ0 = false;
boolean setNumberZ1 = false;
boolean setNumberZR = false;
boolean setNumberX = false;
boolean dotNumber = false;
boolean setIncZ0 = false;
boolean setIncZ1 = false;
boolean setIncZ0Z1 = false;
boolean setIncZR = false;
boolean setIncX = false;
boolean addZ0Z1 = false;
const int ledPin = LED_BUILTIN;
const int ledActive = 10;
int ledState = LOW;  // ledState used to set the LED
float encoderCountZ0 = 0;
float encoderCountZ1 = 0;
float encoderCountZR = 0;
float encoderCountX = 0;
float valueZ0 = 0;
float valueZ1 = 0;
float valueZ0Z1 = 0;
float valueZR = 0;
float valueX = 0;
float memEncoderCountZ0 = 0;
float memEncoderCountZ1 = 0;
float memEncoderCountZR = 0;
float memEncoderCountX = 0;
float incrementZ0 = 0;
float incrementZ1 = 0;
float incrementZ0Z1 = 0;
float incrementZR = 0;
float incrementX = 0;
long inputValue[3];
float inputDotValue[2];
unsigned long previousMillis = 0;
const long interval = 700; 
int a;
float dotValue;
uint8_t bottonState = 0;        // current state of the button
boolean bottonPressed = false;    // previous state of the button
void setup() {
  pinMode(irqPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(tasterEncode_y0, INPUT); //Decimal-to-BCD Encoder A0
  pinMode(tasterEncode_y1, INPUT); //Decimal-to-BCD Encoder A1
  pinMode(tasterEncode_y2, INPUT); //Decimal-to-BCD Encoder A2
  pinMode(tasterEncode_y3, INPUT); //Decimal-to-BCD Encoder A3
  pinMode(ledActive, OUTPUT);
  digitalWrite(ledActive, LOW); //enable pulldown resistor
//  delay(500);
  Wire.begin();
  mpr121_setup();
  Serial.begin(115200);
  encoderZ0.begin();
  encoderZ1.begin();
  encoderZR.begin();
  encoderX.begin();
  encoderX.flip();
  encoderZR.flip();
  // encoderZ0.flip();

  displayZ0.begin();
  displayZ1.begin();
  displayZR.begin();
  displayX.begin();

  pixel_Test();
   
  displayX.setBrightness(0x0f);
  displayZR.setBrightness(0x0f);
  displayZ0.setBrightness(0x0f);
  displayZ1.setBrightness(0x0f);
  delay(100);
  displayZ0.showString("V2.1.1");
  displayZ1.showString("READY?");
  displayX.showString("touch");
  displayZR.showString("1");

//  displayX.showNumberDec(0, false); // Expect: ___0
//  displayZR.showNumberDec(0, false); // Expect: ___0
//  displayZ0.showNumberDec(0, false); // Expect: ___0
  //p.neoPixelClear(true);
  p.neoPixelSetValue(3, 0, 255, 0, true);
  delay(10);
  p.neoPixelSetValue(4, 0, 255, 0, true);
  delay(10);
  p.neoPixelSetValue(5, 0, 255, 0, true);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // memEncoderCount the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
    digitalWrite(ledActive, ledState);
  }
  if (ready == false) {  
    readTouchOne();
  }
  else {
    readTaster();
    readTouchInputs();
  
    if (encoderCountX != encoderX.getCount() and setNumberX == false){
      encoderCountX = encoderX.getCount()/2;
    }
    if (encoderCountZR != encoderZR.getCount() and setNumberZR == false){
      encoderCountZR = encoderZR.getCount()/2;
    }
    if (encoderCountZ0 != encoderZ0.getCount() and setNumberZ0 == false){
      encoderCountZ0 = encoderZ0.getCount()/2;
    }
    if (encoderCountZ1 != encoderZ1.getCount() and setNumberZ1 == false){
      encoderCountZ1 = encoderZ1.getCount()/2;
    }
    if (setIncX == true) {
      valueX = encoderCountX - memEncoderCountX + (incrementX * 256);
    } else {
      valueX = encoderCountX;
    }
    if (setIncZR == true) {
      valueZR = encoderCountZR - memEncoderCountZR + (incrementZR * 256);
    } else {
      valueZR = encoderCountZR;
    }
    if (setIncZ0 == true) {
      valueZ0 = encoderCountZ0 - memEncoderCountZ0 + (incrementZ0 * 100.25);
    } else {
      valueZ0 = encoderCountZ0;
    }
    if (setIncZ1 == true) {
      valueZ1 = encoderCountZ1 - memEncoderCountZ1 + (incrementZ1 * 256);
    } else {
      valueZ1 = encoderCountZ1;
    }
    if (addZ0Z1 == true){
      displayZ0.showString("------");
      displayZ1.showNumber((valueZ0/100.25 + valueZ1/256) , 2);
    } else {
      displayZ0.showNumber(valueZ0/100.25, 2);
      displayZ1.showNumber(valueZ1/256 , 2);
    }
    displayX.showNumber(valueX/256, 2);
    displayZR.showNumber(valueZR/256, 2);
  }
}

void readTouchInputs(){
  if(!checkInterrupt()){    
    //read the touch state from the MPR121
    Wire.requestFrom(MPR_ADD,2);     
    byte LSB = Wire.read();
    byte MSB = Wire.read();    
    uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states    
    for (int i=0; i < 12; i++){  // Check what electrodes were pressed
      if(touched & (1<<i)){      
        if(touchStates[i] == 0){
          // Reset Incremet Counter or reset encoder
          if (setNumberZ0 == false and setNumberZ1 == false and setNumberZR == false and setNumberX == false){
            if(allocateNum(i) == 1) {
              if (setIncZ0 == true){
                memEncoderCountZ0 = encoderCountZ0;
                incrementZ0 = 0;
              }else{
                encoderZ0.reset();
              }
            }
            else if(allocateNum(i) == 4) {
              if (setIncZ1 == true){
                memEncoderCountZ1 = encoderCountZ1;
                incrementZ1 = 0;
              }else{
                encoderZ1.reset();
              }
            }
            else if(allocateNum(i) == 7) {
              if (setIncX == true){
                memEncoderCountX = encoderCountX;
                incrementX = 0;
              }else{
                encoderX.reset();
              }
            }
            else if(allocateNum(i) == 11) {
              if (setIncZR == true){
                memEncoderCountZR = encoderCountZR;
                incrementZR = 0;
              }else{
                encoderZR.reset();
              }
            }
            // set to Display zo increment counter
            else if(allocateNum(i) == 2) {
              setIncZ0 = !setIncZ0;
              if (setIncZ0 == true){
                p.neoPixelSetValue(4, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(0, 0, 255, 0, true);
              }else{
                p.neoPixelSetValue(0, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(4, 0, 255, 0, true);
               }
            }
            else if(allocateNum(i) == 5) {
              setIncZ1 = !setIncZ1;
              if (setIncZ1 == true){
                p.neoPixelSetValue(1, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(5, 0, 255, 0, true);
              }else{
                p.neoPixelSetValue(5, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(1, 0, 255, 0, true);
               }
            }
            else if(allocateNum(i) == 8) {
              setIncX = !setIncX;
              if (setIncX == true){
                p.neoPixelSetValue(2, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(6, 0, 255, 0, true);
              }else{
                p.neoPixelSetValue(6, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(2, 0, 255, 0, true);
              }
            }
            else if(allocateNum(i) == 0) {
              setIncZR = !setIncZR;
              if (setIncZR == true){
                p.neoPixelSetValue(3, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(7, 0, 255, 0, true);
              }else{
                p.neoPixelSetValue(7, 0, 0, 0, true);
                delay(5);
                p.neoPixelSetValue(3, 0, 255, 0, true);
              }
            }
          }
          // Number input for each axe
          if (setNumberZ0 == true){
            if((allocateNum(i) < 10) and (dotNumber == false)) {
              if (a == 0 ){
                setIncZ0 = true;
                memEncoderCountZ0 = encoderCountZ0;
                inputValue[0] = allocateNum(i);
                incrementZ0 = inputValue[0];
              }else if (a == 1){
                inputValue[1] = allocateNum(i);
                incrementZ0 = (inputValue[0]*10 + inputValue[1]);
              }else if (a == 2){
                inputValue[2] = allocateNum(i);
                incrementZ0 = (inputValue[0]*100 + inputValue[1]*10 + inputValue[2]);
              }
            }
            if((allocateNum(i) < 10) and (dotNumber == true)) {
              if (a == 0 ){
                inputDotValue[0] = allocateNum(i);
                incrementZ0 = (incrementZ0 + inputDotValue[0]/10);
              }else if (a == 1){
                inputDotValue[1] = allocateNum(i);
                incrementZ0 = (incrementZ0 + inputDotValue[1]/100);
              }
            }
            if (allocateNum(i) == 12){
              dotNumber = true;
              a = -1;
            }
            //set minus
            if (allocateNum(i) == 11){
              incrementZ0 = incrementZ0*(-1);
            }
            a++;
          }
          else if (setNumberZ1 == true){
            if((allocateNum(i) < 10) and (dotNumber == false)) {
              if (a == 0 ){
                setIncZ1 = true;
                memEncoderCountZ1 = encoderCountZ1;
                inputValue[0] = allocateNum(i);
                incrementZ1 = inputValue[0];
              }else if (a == 1){
                inputValue[1] = allocateNum(i);
                incrementZ1 = (inputValue[0]*10 + inputValue[1]);
              }else if (a == 2){
                inputValue[2] = allocateNum(i);
                incrementZ1 = (inputValue[0]*100 + inputValue[1]*10 + inputValue[2]);
              }
            }
            if((allocateNum(i) < 10) and (dotNumber == true)) {
              if (a == 0 ){
                inputDotValue[0] = allocateNum(i);
                incrementZ1 = (incrementZ1 + inputDotValue[0]/10);
              }else if (a == 1){
                inputDotValue[1] = allocateNum(i);
                incrementZ1 = (incrementZ1 + inputDotValue[1]/100);
              }
            }
            if (allocateNum(i) == 12){
              dotNumber = true;
              a = -1;
            }
            //set minus
            if (allocateNum(i) == 11){
              incrementZ1 = incrementZ1*(-1);
            }
            a++;
          }
          else if (setNumberX == true){
            if((allocateNum(i) < 10) and (dotNumber == false)) {
              if (a == 0 ){
                setIncX = true;
                memEncoderCountX = encoderCountX;
                inputValue[0] = allocateNum(i);
                incrementX = inputValue[0];
              }else if (a == 1 ){
                inputValue[1] = allocateNum(i);
                incrementX = (inputValue[0]*10 + inputValue[1]);
              }else if (a == 2 ){
                inputValue[2] = allocateNum(i);
                incrementX = (inputValue[0]*100 + inputValue[1]*10 + inputValue[2]);
              }
            }
            if((allocateNum(i) < 10) and (dotNumber == true)) {
              if (a == 0 ){
                inputDotValue[0] = allocateNum(i);
                incrementX = (incrementX + inputDotValue[0]/10);
              }else if (a == 1 ){
                inputDotValue[1] = allocateNum(i);
                incrementX = (incrementX + inputDotValue[1]/100);
              }
            }
            if (allocateNum(i) == 12){
              dotNumber = true;
              a = -1;
            }
            //set minus
            if (allocateNum(i) == 11){
              incrementX = incrementX*(-1);
            }
            a++;
          }
          else if (setNumberZR == true){
            if((allocateNum(i) < 10) and (dotNumber == false)) {
              if (a == 0 ){
                setIncZR = true;
                memEncoderCountZR = encoderCountZR;
                inputValue[0] = allocateNum(i);
                incrementZR = inputValue[0];
              }else if (a == 1 ){
                inputValue[1] = allocateNum(i);
                incrementZR = (inputValue[0]*10 + inputValue[1]);
              }else if (a == 2 ){
                inputValue[2] = allocateNum(i);
                incrementZR = (inputValue[0]*100 + inputValue[1]*10 + inputValue[2]);
              }
            }
            if((allocateNum(i) < 10) and (dotNumber == true)) {
              if (a == 0 ){
                inputDotValue[0] = allocateNum(i);
                incrementZR = (incrementZR + inputDotValue[0]/10);
              }else if (a == 1 ){
                inputDotValue[1] = allocateNum(i);
                incrementZR = (incrementZR + inputDotValue[1]/100);
              }
            }
            if (allocateNum(i) == 12){
              dotNumber = true;
              a = -1;
            }
            //set minus
            if (allocateNum(i) == 11){
              incrementZR = incrementZR*(-1);
            }
            a++;
          }
        }else if(touchStates[i] == 1){
          //pin i is still being touched
        }  
        touchStates[i] = 1;      
      }else{
        touchStates[i] = 0;
      }    
    }    
  }
}

void readTaster(){
  //buttonState = 0;
  bottonState = digitalRead(tasterEncode_y0) + (digitalRead(tasterEncode_y1)<<1) + (digitalRead(tasterEncode_y2)<<2) + (digitalRead(tasterEncode_y3)<<3);
  if (bottonState >> 0 && bottonPressed == false) {      // Taster wurde gedrückt
    bottonPressed = true;
//   Serial.print(bottonState);
    if (bottonState == 1 and setNumberX == false and setNumberZ1 == false and setNumberZR == false) {
      setNumberZ0 = !setNumberZ0;
      if (setNumberZ0 == true) {
        p.neoPixelSetValue(0, 0, 0, 255, true);
        delay(5);
        p.neoPixelSetValue(4, 0, 0, 255, true);
        setIncZ0 = true;
        incrementZ0 = 0;
        setNumberZ1 = false;
        setNumberX = false;
        setNumberZR = false;
      }
      if (setNumberZ0 == false) {
        dotNumber = false;
        setIncZ0 = true;
        p.neoPixelSetValue(0, 0, 0, 0, true);
        delay(10);
        p.neoPixelSetValue(4, 0, 255, 0, true);
        inputValue[0] = 0;
        a = 0;
        dotValue = 0;
      }
    }
    if (bottonState == 2 and setNumberX == false and setNumberZ0 == false and setNumberZR == false) {
      setNumberZ1 = !setNumberZ1;
      if (setNumberZ1 == true) {
        p.neoPixelSetValue(1, 0, 0, 255, true);
        delay(5);
        p.neoPixelSetValue(5, 0, 0, 255, true);
        setIncZ1 = true;
        incrementZ1 = 0;
        setNumberZ0 = false;
        setNumberX = false;
        setNumberZR = false;
      }
      if (setNumberZ1 == false) {
        dotNumber = false;
        setIncZ1 = true;
        p.neoPixelSetValue(1, 0, 0, 0, true);
        delay(10);
        p.neoPixelSetValue(5, 0, 255, 0, true);
        inputValue[0] = 0;
        a = 0;
        dotValue = 0;
      }
    }
    if (bottonState == 3 and setNumberZ0 == false and setNumberZ1 == false and setNumberZR == false) {
      setNumberX = !setNumberX;
      if (setNumberX == true) {
        p.neoPixelSetValue(2, 0, 0, 255, true);
        delay(5);
        p.neoPixelSetValue(6, 0, 0, 255, true);
        setIncX = true;
        incrementX = 0;
        setNumberZ0 = false;
        setNumberZ1 = false;
        setNumberZR = false;
      }
      if (setNumberX == false) {
        setIncX = true;
        dotNumber = false;
        p.neoPixelSetValue(2, 0, 0, 0, true);
        delay(5);
        p.neoPixelSetValue(6, 0, 255, 0, true);
        inputValue[0] = 0;
        a = 0;
        dotValue = 0;
      }
    }
    if (bottonState == 4 and setNumberZ0 == false and setNumberZ1 == false and setNumberX == false) {
      setNumberZR = !setNumberZR;
      if (setNumberZR == true) {
        p.neoPixelSetValue(3, 0, 0, 255, true);
        delay(5);
        p.neoPixelSetValue(7, 0, 0, 255, true);
        setIncZR = true;
        incrementZR = 0;
        setNumberZ0 = false;
        setNumberZ1 = false;
        setNumberX = false;
      }
      if (setNumberZR == false) {
        setIncZR = true;
        dotNumber = false;
        p.neoPixelSetValue(3, 0, 0, 0, true);
        delay(5);
        p.neoPixelSetValue(7, 0, 255, 0, true);
        inputValue[0] = 0;
        a = 0;
        dotValue = 0;
      }
    }
    if (bottonState == 5 and setNumberZ0 == false and setNumberZ1 == false and setNumberX == false  and setNumberZR == false){
      addZ0Z1 = !addZ0Z1;
      if (addZ0Z1 == true){
        p.neoPixelSetValue(8, 0, 0, 255, true);
        delay(5);
      } else {
        p.neoPixelSetValue(8, 0, 0, 0, true);
        delay(5);
      }
    }
  }
  if (bottonState == 0 && bottonPressed == true) {
     bottonPressed = false;
  }
}

void readTouchOne(){
  if(!checkInterrupt()){    
    //read the touch state from the MPR121
    Wire.requestFrom(MPR_ADD,2);     
    byte LSB = Wire.read();
    byte MSB = Wire.read();    
    uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states    
    for (int i=0; i < 12; i++){  // Check what electrodes were pressed
      if(touched & (1<<i)){      
        if(touchStates[i] == 0){
          if (setNumberZ0 == false and setNumberZ1 == false and setNumberZR == false and setNumberX == false){
            if(allocateNum(i) == 1) {
              digitalWrite(ledActive, HIGH); //enable pullup resistor
              ready = !ready;
            }
          }
        }else if(touchStates[i] == 1){
          //pin i is still being touched
        }  
        touchStates[i] = 1;      
      }else{      
        touchStates[i] = 0;
      }    
    }    
  }
}
int allocateNum(int touchedKey){
  if(touchedKey == 3) {
    renameKey = 1;
  }else if (touchedKey == 7){
    renameKey = 2;
  }else if (touchedKey == 11){
    renameKey = 3;
  }else if (touchedKey == 2){
    renameKey = 4;
  }else if (touchedKey == 6){
    renameKey = 5;
  }else if (touchedKey == 10){
    renameKey = 6;
  }else if (touchedKey == 1){
    renameKey = 7;
  }else if (touchedKey == 5){
    renameKey = 8;
  }else if (touchedKey == 9){
    renameKey = 9;
  }else if (touchedKey == 4){
    renameKey = 0;
  }else if (touchedKey == 0){
    renameKey = 11;
  }else if (touchedKey == 8){
    renameKey = 12;
  }
  return renameKey;
}
void mpr121_setup(void){

  set_register(MPR_ADD, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  set_register(MPR_ADD, MHD_R, 0x01);
  set_register(MPR_ADD, NHD_R, 0x01);
  set_register(MPR_ADD, NCL_R, 0x00);
  set_register(MPR_ADD, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(MPR_ADD, MHD_F, 0x01);
  set_register(MPR_ADD, NHD_F, 0x01);
  set_register(MPR_ADD, NCL_F, 0xFF);
  set_register(MPR_ADD, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  set_register(MPR_ADD, ELE0_T, TOU_THRESH);
  set_register(MPR_ADD, ELE0_R, REL_THRESH);
 
  set_register(MPR_ADD, ELE1_T, TOU_THRESH);
  set_register(MPR_ADD, ELE1_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE2_T, TOU_THRESH);
  set_register(MPR_ADD, ELE2_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE3_T, TOU_THRESH);
  set_register(MPR_ADD, ELE3_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE4_T, TOU_THRESH);
  set_register(MPR_ADD, ELE4_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE5_T, TOU_THRESH);
  set_register(MPR_ADD, ELE5_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE6_T, TOU_THRESH);
  set_register(MPR_ADD, ELE6_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE7_T, TOU_THRESH);
  set_register(MPR_ADD, ELE7_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE8_T, TOU_THRESH);
  set_register(MPR_ADD, ELE8_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE9_T, TOU_THRESH);
  set_register(MPR_ADD, ELE9_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE10_T, TOU_THRESH);
  set_register(MPR_ADD, ELE10_R, REL_THRESH);
  
  set_register(MPR_ADD, ELE11_T, TOU_THRESH);
  set_register(MPR_ADD, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  set_register(MPR_ADD, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(MPR_ADD, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  
  // Section F
  // Enable Auto Config and auto Reconfig
  /*set_register(MPR_ADD, ATO_CFG0, 0x0B);
  set_register(MPR_ADD, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   
  set_register(MPR_ADD, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  set_register(MPR_ADD, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
  
  set_register(MPR_ADD, ELE_CFG, 0x0C);
  
}

boolean checkInterrupt(void){
  return digitalRead(irqPin);
}

void set_register(int address, unsigned char r, unsigned char v){
    Wire.beginTransmission(address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}

void pixel_Test(){
  for (int i=0; i < 6; i++){  
    p.neoPixelSetValue(i, 255, 0, 0, true);
    delay(100);
    p.neoPixelClear(true);
    delay(100);
  }
  for (int i=0; i < 6; i++){  
    p.neoPixelSetValue(i, 0, 255, 0, true);
    delay(100);
    p.neoPixelClear(true);
    delay(100);
  }
  for (int i=0; i < 6; i++){  
    p.neoPixelSetValue(i, 0, 0, 255, true);
    delay(100);
    p.neoPixelClear(true);
    delay(100);
  }
}


