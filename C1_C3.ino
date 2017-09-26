/*
Built by Tom Greenwood Spring 2017 
For Completion of Masters Degree 
in Intermedia Music Technology
at the University of Oregon

The following code is used for Cube1 and Cube3.

I left all of the Debugging code in the file 
so that if you experiece a problem you can activate 
it to assess the problem.

I recommend purchasing Building 
Wireless Sensor Networks Using Arduino 
by Matthijs Koojiman.
for more information on XBee networks
*/

//  Libraries to interact with the XBee module 
//  I used the S2C model
#include <XBee.h>

//  This is used to debug XBee packet information
//#include <Printers.h>

//  This library packages the data to send over the XBee network
#include "binary.h"
XBeeWithCallbacks xbee;

//  Define the serial port usages
#define DebugSerial Serial
#define XBeeSerial Serial1

//  Set up for communication to the SparkFun 9DOF sensor stick
//  It uses I2C communication
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu;

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


//  Change the Declination to callibrate 
//  the magnometer to your location
#define DECLINATION -15.8 // Declination (degrees) in Eugene, OR.

//  Call the Neopixel library to set up communication to the LEDs
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif


//  Sets the pin that will talk to the Neopixels
#define PIN 6

//  Sets the amount of neopixles in the chain
//  I kept this at 20 since Cube4 has 20 LEDs.
#define NUMPIXELS 20
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);

//  Creat a counting variable to call sendPacket
//  function at a specific interval of time
unsigned long last_tx_time = 0;

//  Create arrray to store 9DOF data
int Sensors [9]; 

//  Create array to store color and brightness data
uint8_t ColorsIn [25]; 

//  Varaibles for logic gates to map colors to sides of the cube
float accxf;
float accyf;
float acczf;
  
void setup() {
//   Initialize communication with Neopixels  
   pixels.begin();

//   Setup debug serial output
  DebugSerial.begin(115200);
  
//   Setup XBee serial communication
  XBeeSerial.begin(38400);
  xbee.begin(XBeeSerial);

//   Setup 9DOF communication
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()){
    DebugSerial.println("Failed to communicate with LSM9DS1.");
    DebugSerial.println("Double-check wiring.");
    DebugSerial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
  }

  delay(1);

//  Setup callbacks
//  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&DebugSerial);
//  xbee.onResponse(printErrorCb, (uintptr_t)(Print*)&DebugSerial);
  xbee.onZBRxResponse(processRxPacket);
}

void processRxPacket(ZBRxResponse& rx, uintptr_t) {
// Access buffer and get length
  Buffer c(rx.getData(), rx.getDataLength());

// Remove first byte of data  
  uint8_t type1 = c.remove<uint8_t>();

//  If the byte == 1 and there are still 25 more bytes in
//  the packet, iterate through packet and deposit data in
//  an array        
    if (type1 == 1 && c.len() == 25) {
//      DebugSerial.print(F("9DoF packet received from "));
//      printHex(DebugSerial, rx.getRemoteAddress64());
//      DebugSerial.println();
      for(uint8_t i = 0; i < 25; i++){
      ColorsIn[i] = (c.remove<uint8_t>());
      }
    }
//    DebugSerial.println(F("Unknown or invalid packet"));
//    printResponse(rx, DebugSerial);
}   
 
void getGyro(){
// Read Gyro data from 9DOF sensor
  imu.readGyro();
  Sensors[0] = (imu.gx);
  Sensors[1] = (imu.gy);
  Sensors[2] = (imu.gz);
}

void getAccel(){
// Read Accel data from 9DOF sensor
  imu.readAccel();
  accxf = (imu.calcAccel(imu.ax));
  accyf = (imu.calcAccel(imu.ay));
  acczf = (imu.calcAccel(imu.az));
  Sensors[3] = (imu.ax);
  Sensors[4] = (imu.ay);
  Sensors[5] = (imu.az);
}

void getMag(){
// Read Mag data from 9DOF sensor
  imu.readMag(); 
  Sensors[6] = (imu.mx);
  Sensors[7] = (imu.my);
  Sensors[8] = (imu.mz);
}

void mapData(){
 //convert Gyro data to 8bit data to easily transferable over the network
 Sensors[0] = map(Sensors[0], -32768, 32768, 0, 254);
 Sensors[1] = map(Sensors[1], -32768, 32768, 0, 254);
 Sensors[2] = map(Sensors[2], -32768, 32768, 0, 254);
 //convert Accel data to 8bit data to easily transferable over the network
 Sensors[3] = map(Sensors[3], -32767, 32767, 0, 254);
 Sensors[4] = map(Sensors[4], -32767, 32767, 0, 254);
 Sensors[5] = map(Sensors[5], -32767, 32767, 0, 254);
 //convert Mag data to 8bit data to easily transferable over the network
 Sensors[6] = map(Sensors[6], -6000, 6000, 0, 254);
 Sensors[7] = map(Sensors[7], -6000, 6000, 0, 254);
 Sensors[8] = map(Sensors[8], -4096, 4096, 0, 254);

}

void sendPacket(){
    // Prepare the Zigbee Transmit Request API packet
    ZBTxRequest txRequest;

// specific adress of XBee coordinator     
//    txRequest.setAddress64(0x0013A200410548A7);

// Adress 0x0000000000000000 tell Xbee to send data to coordinator
    txRequest.setAddress64(0x0000000000000000);

    // Allocate 38 payload bytes: 1 type byte plus two floats of 4 bytes each
    AllocBuffer<11> packetA;

    // Packet type, cube identifer, 9 degrees of freedom data
    packetA.append<uint8_t>(1);
    
    //This line is the determining factor to the basestation
    //to know what cube it is communicating with.
    packetA.append<uint8_t>(1);
    
    for(uint8_t i = 0; i < 9; i++){
      packetA.append<uint8_t>(Sensors[i]);
    }
    txRequest.setPayload(packetA.head, packetA.len());
    // And send it
    xbee.send(txRequest);
}

void changeColor(){
//  Logic gates the microprocessor performes to 
//  know what side is face up.
//  Each side has its own color assosiated with it

 if((accxf > -0.33 && accxf <= 0.33) && (accyf > -0.33 && accyf <= 0.33) && (acczf > 0.33)){
    for(int m=0;m<NUMPIXELS;m++){
    pixels.setPixelColor(m, ColorsIn[1], ColorsIn[2], ColorsIn[3], ColorsIn[4]);
    pixels.setBrightness(ColorsIn[0]);    
    pixels.show();
    }
  }

 else if((accxf > -0.33 && accxf <= 0.33) && (accyf > -0.33 && accyf <= 0.33) && (acczf <= -0.33)){
    for(int m=0;m<NUMPIXELS;m++){
    pixels.setPixelColor(m, ColorsIn[5], ColorsIn[6], ColorsIn[7], ColorsIn[8]);
    pixels.setBrightness(ColorsIn[0]);    
    pixels.show();
    }
  }
 
 else if((accxf > 0.33) && (accyf <= 0.33 && accyf > -0.33) && (acczf <= 0.33 && acczf > -0.33)){
 
    for(int m=0;m<NUMPIXELS;m++){
    pixels.setPixelColor(m, ColorsIn[9], ColorsIn[10], ColorsIn[11], ColorsIn[12]);
    pixels.setBrightness(ColorsIn[0]);    
    pixels.show();
    }
  }

 else if((accxf <= -0.33) && (accyf <= 0.33 && accyf > -0.33) && (acczf <= 0.33 && acczf > -0.33)){
    for(int m=0;m<NUMPIXELS;m++){
    pixels.setPixelColor(m, ColorsIn[13], ColorsIn[14], ColorsIn[15], ColorsIn[16]);
    pixels.setBrightness(ColorsIn[0]);
    pixels.show();
    }
  }
    
 else if((accxf > -0.33 && accxf <= 0.33) && (accyf <= -0.33) && (acczf <= 0.3 && acczf >= -0.33)){
    for(int m=0;m<NUMPIXELS;m++){
    pixels.setPixelColor(m, ColorsIn[17], ColorsIn[18], ColorsIn[19], ColorsIn[20]);
    pixels.setBrightness(ColorsIn[0]);
    pixels.show();
    }
  }

 else if((accxf > -0.33 && accxf <= 0.33) && (accyf > 0.33) && (acczf <= 0.33 && acczf > -0.33)){
    for(int m=0;m<NUMPIXELS;m++){
    pixels.setPixelColor(m, ColorsIn[21], ColorsIn[22], ColorsIn[23], ColorsIn[24]);
    pixels.setBrightness(ColorsIn[0]);
    pixels.show();
    }
  }
}
   
void loop() {
  // Check the serial port to see if there is a new packet available
  xbee.loop(); //process incoming XBee frames
  getGyro();  //  gets Gyro data
  getAccel(); //  gets Accel data 
  getMag();   //  gets Magnam data
  mapData();  //condenses data down to 8 bit information
  changeColor(); //changes neopixels based on positioning 
//   Send a packet every 30 milliseconds
  if(millis() - last_tx_time > 30) { 
    sendPacket(); //send data to basestation
    last_tx_time = millis();
  }
}
