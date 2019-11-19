/*
  Saving camera stills over UART on SparkFun Edge board
  By: Owen Lyke
  SparkFun Electronics
  Date: November 18th 2019
  This example code is in the public domain

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15170

  This example shows how to use the HM01B0 camera. It will stream frames captured by the 
  camera over the UART. (You can choose the port and baud rate - Serial and 921600 by default)

  Record the stream of data (capture as many frames as you like) and then use the 'raw2bmp.py'
  tool to convert the stream to a set of still images
*/

#include "hm01b0.h"

#define SERIAL_PORT Serial
#define BAUD_RATE   115200

HM01B0 myCamera;

void setup() {
  // put your setup code here, to run once:

  // Start up serial monitor
  SERIAL_PORT.begin(BAUD_RATE);
  do {
    delay(500);
  }while(!SERIAL_PORT);

  // tests1
  SERIAL_PORT.println("Hello World");
  SERIAL_PORT.printf("myCamera.cfg.interface: 0x%08X\n", (uint32_t)myCamera.cfg.interface);
  SERIAL_PORT.printf("myCamera.cfg.interface->init(&myCamera.cfg, myCamera.cfg.interface->arg): %d\n", (uint32_t)myCamera.cfg.interface->init(&myCamera.cfg, myCamera.cfg.interface->arg));


  // Turn on camera regulator if using Edge board
#if defined (ARDUINO_SFE_EDGE)
  pinMode(AM_BSP_GPIO_CAMERA_HM01B0_DVDDEN, OUTPUT);
  digitalWrite(AM_BSP_GPIO_CAMERA_HM01B0_DVDDEN, HIGH);
  SERIAL_PORT.println("Turned on Edge camera regulator");
#endif


  // Start the camera
  myCamera.begin();
  

}

void loop() {
  // put your main code here, to run repeatedly:

  myCamera.capture();
//  SERIAL_PORT.printf("First byte of data: 0x%02X\n", myCamera.frameBuffer[0]);

  delay(1000);

}
