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
// This sample shows how Device specific RDM Commands are handled in the processCommand function.
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
// 25.05.2017 Stefan Krupop: Add support for sensors
// 15.08.2018 Stefan Krupop: Add example for sensors
// 21.08.2018 improvements and typo by Peter Newman
// 31.10.2018 Remove unnecessary #include <EEPROM.h> by Graham Hanson
// 04.06.2023 Tim Nijssen: Add support for device-specific parameters
// 05.06.2023 Tim Nijssen: integrate sensors example
// 05.06.2023 Tim Nijssen: Add example for device-specific parameters
// - - - - -

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
const RDMSENSOR my_sensors[] = {
  {E120_SENS_TEMPERATURE, E120_UNITS_CENTIGRADE, E120_PREFIX_NONE, 0, 80, 10, 30, true, false, "Temperature"}
};
struct RDMINIT rdmInit = {
  "mathertel.de", // Manufacturer Label
  1, // Device Model ID
  "Arduino RDM Device", // Device Model Label
  3, // footprint
  (sizeof(my_pids)/sizeof(uint16_t)), my_pids,
  (sizeof(my_sensors)/sizeof(RDMSENSOR)), my_sensors
  //0, NULL
};

unsigned long previousMillis = 0;
double temperature = 0.0;
int16_t _lowestValue = 100;
int16_t _highestValue = 0;

void setup () {

#if defined(SERIAL_DEBUG)
  // The Serial port can be used on Arduino Leonard Boards for debugging purpose
  // because it is not mapped to the real serial port of the ATmega32U4 chip but to the USB port.
  // Don't use that on Arduino Uno, 2009,... boards based on ATmega328 or ATmega168 chips.
  Serial.begin(9600);       
  while (!Serial) ;
  Serial.println("starting...");
#endif

  // initialize the Serial interface to be used as an RDM Device Node.
  // There are several constants that have to be passed to the library so it can respond to the
  // corresponding commands for itself.
  DMXSerial2.init(&rdmInit, processCommand, getSensorValue);

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
    // RDM command for identification was sent.
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

  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();
    temperature = getTemp();
    if (!isnan(temperature)) {
      if (temperature < _lowestValue) {
        _lowestValue = temperature;
      }
      if (temperature > _highestValue) {
        _highestValue = temperature;
      }
    }
  }
  
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

// This function was registered to the DMXSerial2 library in the initRDM call.
// Here retrieval of sensor values is implemented.
bool8 getSensorValue(uint8_t sensorNr, int16_t *value, int16_t *lowestValue, int16_t *highestValue, int16_t *recordedValue) {
  if (sensorNr == 0) {
    if (isnan(temperature)) {
      return false;
    }
    *value = temperature;
    *lowestValue = _lowestValue;
    *highestValue = _highestValue;
    return true;
  }
  return false;
} // getSensorValue

// Read the ATmega328 internal temperature sensor
// https://theorycircuit.com/arduino-internal-temperature-sensor/
double getTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
} // getTemp

// End.

