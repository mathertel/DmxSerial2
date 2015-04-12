// - - - - -
// DmxSerial2 - A hardware supported interface to DMX and RDM.
// RDMSerialRecv.ino: Sample RDM application.
//
// Copyright (c) 2011-2013 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// 
// This Arduino project is a sample application for the DMXSerial2 library that shows
// how a 3 channel receiving RDM client can be implemented.
// The 3 channels are used for PWM Output:
// address (startAddress) + 0 (red) -> PWM Port 9
// address (startAddress) + 1 (green) -> PWM Port 6
// address (startAddress) + 2 (blue) -> PWM Port 5
//
// Ths sample shows how Device specific RDM Commands are handled in the processCommand function.
// The following RDM commands are implemented here:
// E120_LAMP_HOURS
// E120_DEVICE_HOURS
//
// More documentation and samples are available at http://www.mathertel.de/Arduino
// 06.12.2012 created from DMXSerialRecv sample.
// 09.12.2012 added first RDM response.
// 22.01.2013 first published version to support RDM
// 03.03.2013 Using DMXSerial2 as a library
// 15.05.2013 Arduino Leonard and Arduino MEGA compatibility
// 15.12.2013 ADD: output information on a LEONARDO board by using the #define SERIAL_DEBUG definition
//            If you have to save pgm space you can delete the inner lines of this "#if" blocks
// 24.01.2014 Peter Newman/Sean Sill: Get device specific PIDs returning properly in supportedParameters
// 24.01.2014 Peter Newman: Make the device specific PIDs compliant with the OLA RDM Tests. Add device model ID option
// 12.04.2015 change of using datatype boolean to bool8.
// - - - - -

#include <EEPROM.h>
#include <DMXSerial2.h>

// uncomment this line for enabling information on a LEONARD board. 
// #define SERIAL_DEBUG

// Constants for demo program

const int RedPin =    9;  // PWM output pin for Red Light.
const int GreenPin =  6;  // PWM output pin for Green Light.
const int BluePin =   5;  // PWM output pin for Blue Light.

// color: #203050 * 2
#define RedDefaultLevel   0x20 * 2
#define GreenDefaultLevel 0x30 * 2
#define BlueDefaultLevel  0x50 * 2

// define the RGB output color 
void rgb(byte r, byte g, byte b)
{
  analogWrite(RedPin,   r);
  analogWrite(GreenPin, g);
  analogWrite(BluePin,  b);
} // rgb()

// see DMXSerial2.h for the definition of the fields of this structure
const uint16_t my_pids[] = {E120_DEVICE_HOURS, E120_LAMP_HOURS};
struct RDMINIT rdmInit = {
  "mathertel.de", // Manufacturer Label
  1, // Device Model ID
  "Arduino RDM Device", // Device Model Label
  3, // footprint
  (sizeof(my_pids)/sizeof(uint16_t)), my_pids
};


void setup () {

#if defined(SERIAL_DEBUG)
  // The Serial port can be used on Arduino Leonard Boards for debugging purpose
  // because it is not mapped to the real serial port of the ATmega32U4 chip but to the USB port.
  // Dont't use that on Arduino Uno, 2009,... boards based on ATmega328 or ATmega168 chips.
  Serial.begin(9600);       
  while (!Serial) ;
  Serial.println("starting...");
#endif

  // initialize the Serial interface to be used as an RDM Device Node.
  // There are several constants that have to be passed to the library so it can reposonse to the
  // corresponding commands for itself.
  DMXSerial2.init(&rdmInit, processCommand);

  uint16_t start = DMXSerial2.getStartAddress();
#if defined(SERIAL_DEBUG)
  Serial.print("Listening on DMX address #"); Serial.println(start);
#endif

  // set default values to dark red
  // this color will be shown when no signal is present for the first 5 seconds.
  DMXSerial2.write(start + 0, 30);
  DMXSerial2.write(start + 1,  0);
  DMXSerial2.write(start + 2,  0);
  
  // enable pwm outputs
  pinMode(RedPin,   OUTPUT); // sets the digital pin as output
  pinMode(GreenPin, OUTPUT);
  pinMode(BluePin,  OUTPUT);

#if defined(SERIAL_DEBUG)
  // output the current DeviceID

  DEVICEID thisDevice;
  DMXSerial2.getDeviceID(thisDevice);

  Serial.print("This Device is: ");
  if (thisDevice[0] < 0x10) Serial.print('0'); Serial.print(thisDevice[0], HEX);
  if (thisDevice[1] < 0x10) Serial.print('0'); Serial.print(thisDevice[1], HEX);
  Serial.print(":");
  if (thisDevice[2] < 0x10) Serial.print('0'); Serial.print(thisDevice[2], HEX);
  if (thisDevice[3] < 0x10) Serial.print('0'); Serial.print(thisDevice[3], HEX);
  if (thisDevice[4] < 0x10) Serial.print('0'); Serial.print(thisDevice[4], HEX);
  if (thisDevice[5] < 0x10) Serial.print('0'); Serial.print(thisDevice[5], HEX); 
  Serial.println();
#endif

} // setup()


void loop() {
  // Calculate how long no data backet was received
  unsigned long lastPacket = DMXSerial2.noDataSince();

  if (DMXSerial2.isIdentifyMode()) {
    // RDM Command for Indentification was sent.
    // Blink the device.
    unsigned long now = millis();
    if (now % 1000 < 500) {
      rgb(200, 200, 200);
    } else {
      rgb(0, 0, 0);
    } // if
    
  } else if (lastPacket < 30000) {
    // read recent DMX values and set pwm levels 
    analogWrite(RedPin,   DMXSerial2.readRelative(0));
    analogWrite(GreenPin, DMXSerial2.readRelative(1));
    analogWrite(BluePin,  DMXSerial2.readRelative(2));

  } else {
#if defined(SERIAL_DEBUG)
  Serial.println("no signal since 30 secs.");
#endif
    // Show default color, when no data was received since 30 seconds or more.
    analogWrite(RedPin,   RedDefaultLevel);
    analogWrite(GreenPin, GreenDefaultLevel);
    analogWrite(BluePin,  BlueDefaultLevel);
  } // if
  
  // check for unhandled RDM commands
  DMXSerial2.tick();
} // loop()


// This function was registered to the DMXSerial2 library in the initRDM call.
// Here device specific RDM Commands are implemented.
bool8 processCommand(struct RDMDATA *rdm, uint16_t *nackReason)
{
  byte CmdClass       = rdm->CmdClass;     // command class
  uint16_t Parameter  = rdm->Parameter;	   // parameter ID
  bool8 handled = false;

// This is a sample of how to return some device specific data
  if (Parameter == SWAPINT(E120_DEVICE_HOURS)) { // 0x0400
    if (CmdClass == E120_GET_COMMAND) {
      if (rdm->DataLength > 0) {
        // Unexpected data
        *nackReason = E120_NR_FORMAT_ERROR;
      } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        *nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        rdm->DataLength = 4;
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->Data[2] = 2;
        rdm->Data[3] = 0;
        handled = true;
      }
    } else if (CmdClass == E120_SET_COMMAND) {
      // This device doesn't support set
      *nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
    }

  } else if (Parameter == SWAPINT(E120_LAMP_HOURS)) { // 0x0401
    if (CmdClass == E120_GET_COMMAND) {
      if (rdm->DataLength > 0) {
        // Unexpected data
        *nackReason = E120_NR_FORMAT_ERROR;
      } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        *nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        rdm->DataLength = 4;
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->Data[2] = 0;
        rdm->Data[3] = 1;
        handled = true;
      }
    } else if (CmdClass == E120_SET_COMMAND) {
      // This device doesn't support set
      *nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
    }
  } // if
  
  return handled;
} // processCommand


// End.

