// - - - - -
// DMXSerial2 - A hardware supported interface to DMX and RDM.
// DMXSerial2.cpp: Library implementation file
//
// Copyright (c) 2011-2013 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
//
// Documentation and samples are available at http://www.mathertel.de/Arduino
//
// IMPORTANT NOTE: Even when I name by implementation RDM, it is not guaranteed as a fully
// compliant RDM implementation. Right now it is only a experimental version with regards
// to the published information about RDM available on the internet. (see links inside the
// code)
//
// History:
// see DMXSerial2.h
// - - - - -

#include "Arduino.h"
#include <EEPROM.h>
#include "DMXSerial2.h"

#include <avr/interrupt.h>

// ----- Debugging -----

// to debug on an oscilloscope, enable this
#undef SCOPEDEBUG
#ifdef SCOPEDEBUG
#define DmxTriggerPin 4 // low spike at beginning of start byte
#define DmxISRPin 3 // low during interrupt service routines
#endif

// ----- Timing, Testing, and Debugging helpers -----

// Helper function for simply sending an RGB signal out to indicate a debug or testing condition.
void rgbDebug(byte r, byte g, byte b)
{
  analogWrite(9, r);
  analogWrite(6, g);
  analogWrite(5, b);
} // rgbDebug()

// TIMING:
unsigned long _timingReceiveEnd; // when the last incoming byte was received

// ----- Constants -----

// Define port & bit values for Hardware Serial Port.
// The library works unchanged with the Arduino 2009, UNO and Arduino MGEA 2560 boards,
// using the serial port 0 and the Arduino Leonardo using serial port 1 (on the 32u4 boards the first USART is USART1)

// For using the serial port 1 on a Arduino MEGA 2560 board, enable the DMX_USE_PORT1 definition.
// #define DMX_USE_PORT1

#if !defined(DMX_USE_PORT1) && defined(USART_RX_vect)
// These definitions are for using serial port 0
#define UCSRnA UCSR0A  // Control and Status Register A
#define TXCn   TXC0

#define UCSRnB UCSR0B  // USART Control and Status Register B

#define RXCIEn RXCIE0  // Enable Receive Complete Interrupt
#define TXCIEn TXCIE0  // Enable Transmission Complete Interrupt
#define UDRIEn UDRIE0  // Enable Data Register Empty Interrupt
#define RXENn  RXEN0   // Enable Receiving
#define TXENn  TXEN0   // Enable Sending

#define UCSRnC UCSR0C  // Control and Status Register C
#define USBSn  USBS0   // Stop bit select 0=1bit, 1=2bits
#define UCSZn0 UCSZ00  // Character size 00=5, 01=6, 10=7, 11=8 bits
#define UPMn0  UPM00   // Parity setting 00=N, 10=E, 11=O

#define UBRRnH UBRR0H  // USART Baud Rate Register High
#define UBRRnL UBRR0L  // USART Baud Rate Register Low

#define UDRn   UDR0    // USART Data Register
#define UDREn  UDRE0   // USART Data Ready
#define FEn    FE0     // Frame Error

#define USARTn_RX_vect   USART_RX_vect
#define USARTn_TX_vect   USART_TX_vect
#define USARTn_UDRE_vect USART_UDRE_vect

#elif !defined(DMX_USE_PORT1) && defined(USART0_RX_vect)
// These definitions are for using serial port 0
#define UCSRnA UCSR0A  // Control and Status Register A
#define TXCn   TXC0

#define UCSRnB UCSR0B  // USART Control and Status Register B

#define RXCIEn RXCIE0  // Enable Receive Complete Interrupt
#define TXCIEn TXCIE0  // Enable Transmission Complete Interrupt
#define UDRIEn UDRIE0  // Enable Data Register Empty Interrupt
#define RXENn  RXEN0   // Enable Receiving
#define TXENn  TXEN0   // Enable Sending

#define UCSRnC UCSR0C  // Control and Status Register C
#define USBSn  USBS0   // Stop bit select 0=1bit, 1=2bits
#define UCSZn0 UCSZ00  // Character size 00=5, 01=6, 10=7, 11=8 bits
#define UPMn0  UPM00   // Parity setting 00=N, 10=E, 11=O

#define UBRRnH UBRR0H  // USART Baud Rate Register High
#define UBRRnL UBRR0L  // USART Baud Rate Register Low

#define UDRn   UDR0    // USART Data Register
#define UDREn  UDRE0   // USART Data Ready
#define FEn    FE0     // Frame Error

#define USARTn_RX_vect   USART0_RX_vect
#define USARTn_TX_vect   USART0_TX_vect
#define USARTn_UDRE_vect USART0_UDRE_vect

#elif defined(DMX_USE_PORT1) || defined(USART1_RX_vect)
// These definitions are for using serial port 1
#define UCSRnA UCSR1A  // Control and Status Register A
#define TXCn   TXC1

#define UCSRnB UCSR1B  // USART Control and Status Register B

#define RXCIEn RXCIE1  // Enable Receive Complete Interrupt
#define TXCIEn TXCIE1  // Enable Transmission Complete Interrupt
#define UDRIEn UDRIE1  // Enable Data Register Empty Interrupt
#define RXENn  RXEN1   // Enable Receiving
#define TXENn  TXEN1   // Enable Sending

#define UCSRnC UCSR1C  // Control and Status Register C
#define USBSn  USBS1   // Stop bit select 0=1bit, 1=2bits
#define UCSZn0 UCSZ10  // Character size 00=5, 01=6, 10=7, 11=8 bits
#define UPMn0  UPM10   // Parity setting 00=N, 10=E, 11=O

#define UBRRnH UBRR1H  // USART Baud Rate Register High
#define UBRRnL UBRR1L  // USART Baud Rate Register Low

#define UDRn   UDR1    // USART Data Register
#define UDREn  UDRE1   // USART Data Ready
#define FEn    FE1     // Frame Error

#define USARTn_RX_vect   USART1_RX_vect
#define USARTn_TX_vect   USART1_TX_vect
#define USARTn_UDRE_vect USART1_UDRE_vect

#endif


// The formats for serial transmission, already defined in `Arduino.h` -> `HardwareSerial.h`
// We do not include `HardwareSerial.h` but home it's still there in future version.
// If not, here are the required values:

#ifndef SERIAL_8N2
#define SERIAL_8N2  ((1<<USBSn) | (0<<UPMn0) | (3<<UCSZn0))
#endif

#ifndef SERIAL_8E1
#define SERIAL_8E1  ((0<<USBSn) | (2<<UPMn0) | (3<<UCSZn0))
#endif

// not needed by DMXSerial2:
// #define SERIAL_8N1  ((0<<USBSn) | (0<<UPMn0) | (3<<UCSZn0))
// #define SERIAL_8E2  ((1<<USBSn) | (2<<UPMn0) | (3<<UCSZn0))


// the break timing is 10 bits (start + 8 data + parity) of this speed
// the mark-after-break is 1 bit of this speed plus approx 6 usec
// 100000 bit/sec is good for DMX: gives aprox 10 usec per bit.
// That gives 100 usec break and 10+ usec MAB
// 1990 spec says transmitter must send >= 92 usec break and >= 12 usec MAB
// receiver must accept 88 us break and 8 us MAB
// #define BREAKSPEED     100000

// 45500 bit/sec is for RDM: gives aprox 22 usec per bit.
// That gives 220 usec break and 22+ usec MAB
#define BREAKSPEED     45500
#define BREAKFORMAT    SERIAL_8E1

#define DMXSPEED       250000
#define DMXFORMAT      SERIAL_8N2

// ----- Enumerations -----

// current state of receiving or sending DMX/RDM Bytes
typedef enum {
  IDLE,      // ignoring everything and wait for the next BREAK
             // or a valid RDM packet arrived, need for processing !
  BREAK,     // received a BREAK: now a new packet will start
  DMXDATA,   // receiving DMX data into the _dmxData buffer
  RDMDATA,   // receiving RDM data into the _rdm.buffer
  CHECKSUMH, // received the High byte of the _rdm.buffer checksum
  CHECKSUML  // received the Low byte of the _rdm.buffer checksum
} DMXReceivingState;

// ----- Structs -----

// the special discovery response message
struct DISCOVERYMSG {
  byte headerFE[7];
  byte headerAA;
  byte maskedDevID[12];
  byte checksum[4];
}; // struct DISCOVERYMSG


// The DEVICEINFO structure (length = 19) has to be responded for E120_DEVICE_INFO
// See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=96
struct DEVICEINFO {
  byte protocolMajor;
  byte protocolMinor;
  uint16_t deviceModel;
  uint16_t productCategory;
  uint32_t softwareVersion;
  uint16_t footprint;
  byte currentPersonality;
  byte personalityCount;
  uint16_t startAddress;
  uint16_t subDeviceCount;
  byte sensorCount;
}; // struct DEVICEINFO


// This structure is defined for mapping the values into the EEPROM
struct EEPROMVALUES {
  byte sig1; // 0x6D  signature 1, EPROM values are valid of both signatures match.
  byte sig2; // 0x68  signature 2
  uint16_t startAddress; // the DMX start address can be changed by a RDM command.
  char deviceLabel[DMXSERIAL_MAX_RDM_STRING_LENGTH]; // the device Label can be changed by a RDM command.
  DEVICEID deviceID;    // store the device ID to allow easy software updates.
}; // struct EEPROMVALUES


// ----- non class variables -----
// variables that are needed inside the interrupt routines so they are not declared in the class definition.

// The Device ID must be a unique number for each individual device.
// The first 2 bytes are specific for a manufacturer.
// The list of codes is available at http://tsp.plasa.org/tsp/working_groups/CP/mfctrIDs.php
// I use the number 0x0987 that is registered with myself.
// For the other 4 bytes I use the date of creation: 0x2012 0x11 0x02
// When the EEPROM values are valid, the _devID is taken from these values.
// This allows software updates without losing a specific device ID.

// It was an easy job to register a manufacturer id to myself as explained
// on http://tsp.plasa.org/tsp/working_groups/CP/mfctrIDs.php.
// Feel free to use my manufacturer id yourself if you promise only to use it
// for experiments and never to put a real device.
// If no valid EEPROM parameter block was found the following device ID is used and the last 2 bytes are randomized.
// If you plan for more please request your own manufacturer id
// and adjust the next line and the first two values in the array below that to use it:
DEVICEID _devID = { 0x09, 0x87, 0x20, 0x12, 0x00, 0x00 };

// The Device ID for addressing all devices of a manufacturer.
DEVICEID _devIDGroup = { 0x09, 0x87, 0xFF, 0xFF, 0xFF, 0xFF };

// The Device ID for addressing all devices: 6 times 0xFF.
DEVICEID _devIDAll = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


// This is the buffer for RDM packets being received and sent.
// this structure is needed to RDM data separate from DMX data.
union RDMMEM {
  // the most common RDM packet layout for commands
  struct RDMDATA packet;

  // the layout of the RDM packet when returning a discovery message
  struct DISCOVERYMSG discovery;

  // the byte array used while receiving and sending.
  byte buffer[60];
} _rdm; // union RDMMEM


// This flag will be set when a full RDM packet was received.
bool8 _rdmAvailable;

// This is the current 16 bit checksum for RDM commands, used by the interrupt routines.
uint16_t _rdmCheckSum;

// static data that is not needed externally so it is not put into the class definition.
bool8 _isMute;    // is set to true when RDM discovery command muted this device.

uint8_t _dmxModePin = 2;
uint8_t _dmxModeOut = HIGH;
uint8_t _dmxModeIn = LOW;

// ----- Macros -----

// calculate prescaler from baud rate and cpu clock rate at compile time
// nb implements rounding of ((clock / 16) / baud) - 1 per atmega datasheet
#define Calcprescale(B)     ( ( (((F_CPU)/8)/(B)) - 1 ) / 2 )

// compare 2 DeviceIDs
#define DeviceIDCmp(id1, id2) memcmp(id1, id2, sizeof(DEVICEID))

// copy an DeviceID id2 to id1
#define DeviceIDCpy(id1, id2) memcpy(id1, id2, sizeof(DEVICEID))


// ----- DMXSerial Private variables -----
// These variables are not class members because they have to be reached by the interrupt implementations.
// don't use these variable from outside, use the appropriate methods.

volatile uint8_t  _dmxState;          // Current State of receiving DMX Bytes
volatile int      _dmxPos;            // the current read or write position in a DMX/RDM packet transmission.
unsigned long     _gotLastPacket = 0; // the last time (using the millis function) a packet was received.

volatile byte *_dmxSendBuffer;
volatile int _dmxSendLen;

// Array of DMX values (raw).
// Entry 0 will never be used for DMX data but will store the startbyte (0 for DMX mode).
volatile byte _dmxData[DMXSERIAL_MAX+1];

// Create a single class instance. Multiple class instances (multiple simultaneous DMX ports) are not supported.
DMXSerialClass2 DMXSerial2;


// ----- forwards -----

void _DMXSerialBaud(uint16_t baud_setting, uint8_t format);
void _DMXSerialWriteByte(uint8_t data);

void respondMessage(bool8 isHandled, uint16_t nackReason = E120_NR_UNKNOWN_PID);
int random255();

// ----- Class implementation -----

// Initialize or reinitialize the DMX RDM mode.
// The other values are stored for later use with the specific commands.
void DMXSerialClass2::init(struct RDMINIT *initData, RDMCallbackFunction func, RDMGetSensorValue sensorFunc, uint8_t modePin, uint8_t modeIn, uint8_t modeOut)
{
  // This structure is defined for mapping the values in the EEPROM
  struct EEPROMVALUES eeprom;

  // save the given initData for later use.
  _initData = initData;
  _rdmFunc = func;
  _sensorFunc = sensorFunc;

  _dmxModePin = modePin;
  _dmxModeIn = modeIn;
  _dmxModeOut = modeOut;

  _baseInit();

  // now initialize RDM specific elements
  _isMute = false;
  _rdmAvailable = false;
  _identifyMode = false;
  _softwareLabel = "Arduino RDM 1.0";

  // read from EEPROM or set defaults
  for (unsigned int i = 0; i < sizeof(eeprom); i++)
    ((byte *)(&eeprom))[i] = EEPROM.read(i);

  // check if the EEPROM values are from the RDM library
  if ((eeprom.sig1 == 0x6D) && (eeprom.sig2 == 0x68)) {
    _startAddress = eeprom.startAddress;
    strcpy (deviceLabel, eeprom.deviceLabel);
    DeviceIDCpy(_devID, eeprom.deviceID);

    // setup the manufacturer addressing device-ID
    _devIDGroup[0] = _devID[0];
    _devIDGroup[1] = _devID[1];

  } else {
    // set default values
    _startAddress = 1;
    strcpy (deviceLabel, "new");
    _devID[4] = random255(); // random(255);
    _devID[5] = random255(); // random(255);
  } // if
  _saveEEPRom();

  // now start
  digitalWrite(_dmxModePin, _dmxModeIn); // data in direction

  _dmxSendBuffer = _rdm.buffer;
  // _dmxSendLen = ... will be set individually

  // Setup Hardware
  // Enable receiver and transmitter and interrupts
  UCSRnB = (1<<RXENn) | (1<<RXCIEn);
  _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT); // Enable serial reception with a 250k rate
} // initRDM()


// Read the current value of a channel.
uint8_t DMXSerialClass2::read(int channel)
{
  // adjust parameter
  if (channel < 1) channel = 1;
  if (channel > DMXSERIAL_MAX) channel = DMXSERIAL_MAX;
  // read value from buffer
  return(_dmxData[channel]);
} // read()


// get the deviceID
void DMXSerialClass2::getDeviceID (DEVICEID id) {
  DeviceIDCpy(id, _devID);
} // getDeviceID()


// Read the current value of a channel, relative to the startAddress.
uint8_t DMXSerialClass2::readRelative(unsigned int channel)
{
  uint8_t val = 0;
  if (channel < _initData->footprint) {
    // channel is in a valid range! (channel >= 0) is assumed by (unsigned int) type.
    val = _dmxData[_startAddress + channel];
  } // if
  return(val);
} // readRelative()


// Write the value into the channel.
// The value is just stored in the sending buffer and will be picked up
// by the DMX sending interrupt routine.
void DMXSerialClass2::write(int channel, uint8_t value)
{
  // adjust parameters
  if (channel < 1) channel = 1;
  if (channel > DMXSERIAL_MAX) channel = DMXSERIAL_MAX;

  // The next 2 comparisons have no effect because uint8_t covers exact this range. -> removed
  // When changing DMXSERIAL_MAX_SLOT_VALUE or _dmxData space allocation, check again.
  // if (value < DMXSERIAL_MIN_SLOT_VALUE) value = DMXSERIAL_MIN_SLOT_VALUE;
  // if (value > DMXSERIAL_MAX_SLOT_VALUE) value = DMXSERIAL_MAX_SLOT_VALUE;

  // store value for later sending
  _dmxData[channel] = value;
} // write()


// Register a self implemented function for RDM callbacks
void DMXSerialClass2::attachRDMCallback(RDMCallbackFunction newFunction)
{
  _rdmFunc = newFunction;
} // attachRDMCallback

// Register a self implemented function to get sensor values
void DMXSerialClass2::attachSensorCallback(RDMGetSensorValue newFunction)
{
  _sensorFunc = newFunction;
} // attachSensorCallback

// some functions to hide the internal variables from being changed

unsigned long DMXSerialClass2::noDataSince() {
  /* Make sure we don't load partial updates of this multi-byte value */
  noInterrupts();
  unsigned long lastPacket = _gotLastPacket;
  interrupts();
  return(millis() - lastPacket);
}

bool8 DMXSerialClass2::isIdentifyMode() { return(_identifyMode); }
uint16_t DMXSerialClass2::getStartAddress() { return(_startAddress); }
uint16_t DMXSerialClass2::getFootprint() { return(_initData->footprint); }


// Handle RDM Requests and send response
// see http://www.opendmx.net/index.php/RDM_Discovery
// see http://www.enttec.com/docs/sniffer_manual.pdf
void DMXSerialClass2::tick(void)
{
  if (((_dmxState == IDLE) || (_dmxState == BREAK)) && (_rdmAvailable)) { // 15.06.2015
    // never process twice.
    _rdmAvailable = false;

    // respond to RDM commands now.
    bool8 packetIsForMe = false;
    bool8 packetIsForGroup = false;
    bool8 packetIsForAll = false;
    bool8 isHandled = false;

    struct RDMDATA *rdm = &_rdm.packet;

    byte     CmdClass  = rdm->CmdClass;  // command class
    uint16_t Parameter = rdm->Parameter; // parameter ID

    // in the ISR only some global conditions are checked: DestID
    if (DeviceIDCmp(rdm->DestID, _devIDAll) == 0) {
      packetIsForAll = true;
    } else if (DeviceIDCmp(rdm->DestID, _devIDGroup) == 0) {
      packetIsForGroup = true;
    } else if (DeviceIDCmp(rdm->DestID, _devID) == 0) {
      packetIsForMe = true;
    } // if

      if ((! packetIsForMe) && (! packetIsForGroup) && (! packetIsForAll)) {
        // ignore this packet

      } else if (CmdClass == E120_DISCOVERY_COMMAND) { // 0x10
        // handle all Discovery commands locally
        if (Parameter == SWAPINT(E120_DISC_UNIQUE_BRANCH)) { // 0x0001
          // not tested here for pgm space reasons: rdm->Length must be 24+6+6 = 36
          // not tested here for pgm space reasons: rdm->_DataLength must be 6+6 = 12

          if (! _isMute) {
            // check if my _devID is in the discovery range
            if ((DeviceIDCmp(rdm->Data, _devID) <= 0) && (DeviceIDCmp(_devID, rdm->Data+6) <= 0)) {

              // respond a special discovery message !
              struct DISCOVERYMSG *disc = &_rdm.discovery;
              _rdmCheckSum = 6 * 0xFF;

              // fill in the _rdm.discovery response structure
              for (byte i = 0; i < 7; i++)
                disc->headerFE[i] = 0xFE;
              disc->headerAA = 0xAA;
              for (byte i = 0; i < 6; i++) {
                disc->maskedDevID[i+i]   = _devID[i] | 0xAA;
                disc->maskedDevID[i+i+1] = _devID[i] | 0x55;
                _rdmCheckSum += _devID[i];
              }
              disc->checksum[0] = (_rdmCheckSum >> 8)   | 0xAA;
              disc->checksum[1] = (_rdmCheckSum >> 8)   | 0x55;
              disc->checksum[2] = (_rdmCheckSum & 0xFF) | 0xAA;
              disc->checksum[3] = (_rdmCheckSum & 0xFF) | 0x55;

              // disable all interrupt routines and send the _rdm.discovery packet
              // now send out the _rdm.buffer without a starting BREAK.
              _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT);
              UCSRnB = (1<<TXENn); // no interrupts !

              // delayMicroseconds(50);  // ??? 180
              digitalWrite(_dmxModePin, _dmxModeOut); // data Out direction

              unsigned int _rdmBufferLen = sizeof(_rdm.discovery);
              for (unsigned int i = 0; i < _rdmBufferLen; i++) {
                UDRn = _rdm.buffer[i];
                UCSRnA= (1<<TXCn);
                loop_until_bit_is_set(UCSRnA, TXCn);
              } // for

              digitalWrite(_dmxModePin, _dmxModeIn); // data Out direction

              // Re-enable receiver and Receive interrupt
              _dmxState= IDLE; // initial state
              UCSRnB = (1<<RXENn) | (1<<RXCIEn);
              _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT); // Enable serial reception with a 250k rate
            } // if
          } // if

        } else if (Parameter == SWAPINT(E120_DISC_UN_MUTE)) { // 0x0003
          isHandled = true;
          if (_rdm.packet.DataLength > 0) {
            // Unexpected data
            // Do nothing
          } else {
            _isMute = false;
            if (packetIsForMe) { // Only actually respond if it's sent direct to us
              // Control field
              _rdm.packet.Data[0] = 0b00000000;
              _rdm.packet.Data[1] = 0b00000000;
              _rdm.packet.DataLength = 2;
              respondMessage(true); // 21.11.2013
            }
          }

        } else if (Parameter == SWAPINT(E120_DISC_MUTE)) { // 0x0002
          isHandled = true;
          if (_rdm.packet.DataLength > 0) {
            // Unexpected data
            // Do nothing
          } else {
            _isMute = true;
            if (packetIsForMe) { // Only actually respond if it's sent direct to us
              // Control field
              _rdm.packet.Data[0] = 0b00000000;
              _rdm.packet.Data[1] = 0b00000000;
              _rdm.packet.DataLength = 2;
              respondMessage(true); // 21.11.2013
            }
          }
        } // if

      } else {
        // don't ignore packets not sent directly but via broadcasts.
        // Only send an answer on directly sent packages.
        DMXSerial2._processRDMMessage(CmdClass, Parameter, isHandled, packetIsForMe);

      } // if
  } // if
} // tick()


// Terminale operation
void DMXSerialClass2::term(void)
{
  // Disable all USART Features, including Interrupts
  UCSRnB = 0;
} // term()


// Process the RDM Command Message by changing the _rdm buffer and returning (true).
// if returning (false) a NAK will be sent.
// This method processes the commands/parameters regarding mute, DEviceInfo, devicelabel,
// manufacturer label, DMX Start address.
// When parameters are changed by a SET command they are persisted into EEPROM.
// When doRespond is true, send an answer back to the controller node.
void DMXSerialClass2::_processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 handled, bool8 doRespond)
{
  uint16_t nackReason = E120_NR_UNKNOWN_PID;

  // call the device specific method
  if ((! handled) && (_rdmFunc)) {
    handled = _rdmFunc(&_rdm.packet, &nackReason);
  } // if

  // if not already handled the command: handle it using this implementation
  if (! handled ) {

    if (Parameter == SWAPINT(E120_IDENTIFY_DEVICE)) { // 0x1000
      if (CmdClass == E120_SET_COMMAND) { // 0x30
        if (_rdm.packet.DataLength != 1) {
          // Oversized data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if ((_rdm.packet.Data[0] != 0) && (_rdm.packet.Data[0] != 1)) {
          // Out of range data
          nackReason = E120_NR_DATA_OUT_OF_RANGE;
        } else {
          _identifyMode = _rdm.packet.Data[0] != 0;
          _rdm.packet.DataLength = 0;
          handled = true;
        }
      } else if (CmdClass == E120_GET_COMMAND) { // 0x20
        if (_rdm.packet.DataLength > 0) {
          // Unexpected data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if (_rdm.packet.SubDev != 0) {
          // No sub-devices supported
          nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
        } else {
          _rdm.packet.Data[0] = _identifyMode;
          _rdm.packet.DataLength = 1;
          handled = true;
        }
      } // if

    } else if ((CmdClass == E120_GET_COMMAND) && (Parameter == SWAPINT(E120_DEVICE_INFO))) { // 0x0060
      if (_rdm.packet.DataLength > 0) {
        // Unexpected data
        nackReason = E120_NR_FORMAT_ERROR;
      } else if (_rdm.packet.SubDev != 0) {
        // No sub-devices supported
        nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        // return all device info data
        DEVICEINFO *devInfo = (DEVICEINFO *)(_rdm.packet.Data); // The data has to be responded in the Data buffer.

        devInfo->protocolMajor = 1;
        devInfo->protocolMinor = 0;
        devInfo->deviceModel = SWAPINT(_initData->deviceModelId);
        devInfo->productCategory = SWAPINT(E120_PRODUCT_CATEGORY_DIMMER_CS_LED);
        devInfo->softwareVersion = SWAPINT32(0x01000000);// 0x04020900;
        devInfo->footprint = SWAPINT(_initData->footprint);
        devInfo->currentPersonality = 1;
        devInfo->personalityCount = 1;
        devInfo->startAddress = SWAPINT(_startAddress);
        devInfo->subDeviceCount = 0;
        devInfo->sensorCount = _initData->sensorsLength;

         _rdm.packet.DataLength = sizeof(DEVICEINFO);
        handled = true;
      }

    } else if ((CmdClass == E120_GET_COMMAND) && (Parameter == SWAPINT(E120_MANUFACTURER_LABEL))) { // 0x0081
      if (_rdm.packet.DataLength > 0) {
        // Unexpected data
        nackReason = E120_NR_FORMAT_ERROR;
      } else if (_rdm.packet.SubDev != 0) {
        // No sub-devices supported
        nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        // return the manufacturer label
        _rdm.packet.DataLength = strlen(_initData->manufacturerLabel);
        memcpy(_rdm.packet.Data, _initData->manufacturerLabel, _rdm.packet.DataLength);
        handled = true;
      }

    } else if ((CmdClass == E120_GET_COMMAND) && (Parameter == SWAPINT(E120_DEVICE_MODEL_DESCRIPTION))) { // 0x0080
      if (_rdm.packet.DataLength > 0) {
        // Unexpected data
        nackReason = E120_NR_FORMAT_ERROR;
      } else if (_rdm.packet.SubDev != 0) {
        // No sub-devices supported
        nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        // return the DEVICE MODEL DESCRIPTION
        _rdm.packet.DataLength = strlen(_initData->deviceModel);
        memcpy(_rdm.packet.Data, _initData->deviceModel, _rdm.packet.DataLength);
        handled = true;
      }

    } else if (Parameter == SWAPINT(E120_DEVICE_LABEL)) { // 0x0082
      if (CmdClass == E120_SET_COMMAND) {
        if (_rdm.packet.DataLength > DMXSERIAL_MAX_RDM_STRING_LENGTH) {
          // Oversized data
          nackReason = E120_NR_FORMAT_ERROR;
        } else {
          memcpy(deviceLabel, _rdm.packet.Data, _rdm.packet.DataLength);
          deviceLabel[_rdm.packet.DataLength] = '\0';
          _rdm.packet.DataLength = 0;
          // persist in EEPROM
          _saveEEPRom();
          handled = true;
        }
      } else if (CmdClass == E120_GET_COMMAND) {
        if (_rdm.packet.DataLength > 0) {
          // Unexpected data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if (_rdm.packet.SubDev != 0) {
          // No sub-devices supported
          nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
        } else {
          _rdm.packet.DataLength = strlen(deviceLabel);
          memcpy(_rdm.packet.Data, deviceLabel, _rdm.packet.DataLength);
          handled = true;
        }
      } // if

    } else if ((CmdClass == E120_GET_COMMAND) && (Parameter == SWAPINT(E120_SOFTWARE_VERSION_LABEL))) { // 0x00C0
      if (_rdm.packet.DataLength > 0) {
        // Unexpected data
        nackReason = E120_NR_FORMAT_ERROR;
      } else if (_rdm.packet.SubDev != 0) {
        // No sub-devices supported
        nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        // return the SOFTWARE_VERSION_LABEL
        _rdm.packet.DataLength = strlen(_softwareLabel);
        memcpy(_rdm.packet.Data, _softwareLabel, _rdm.packet.DataLength);
        handled = true;
      }

    } else if (Parameter == SWAPINT(E120_DMX_START_ADDRESS)) { // 0x00F0
      if (CmdClass == E120_SET_COMMAND) {
        if (_rdm.packet.DataLength != 2) {
          // Oversized data
          nackReason = E120_NR_FORMAT_ERROR;
        } else {
          uint16_t newStartAddress = READINT(_rdm.packet.Data);
          if ((newStartAddress <= 0) || (newStartAddress > DMXSERIAL_MAX)) {
            // Out of range start address
            // TODO(Peter): Should it be newStartAddress less footprint?
            nackReason = E120_NR_DATA_OUT_OF_RANGE;
          } else {
            _startAddress = newStartAddress;
            _rdm.packet.DataLength = 0;
            // persist in EEPROM
            _saveEEPRom();
            handled = true;
          }
        }
      } else if (CmdClass == E120_GET_COMMAND) {
        if (_rdm.packet.DataLength > 0) {
          // Unexpected data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if (_rdm.packet.SubDev != 0) {
          // No sub-devices supported
          nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
        } else {
          WRITEINT(_rdm.packet.Data, _startAddress);
          _rdm.packet.DataLength = 2;
          handled = true;
        }
      } // if

    } else if (Parameter == SWAPINT(E120_SUPPORTED_PARAMETERS)) { // 0x0050
      if (CmdClass == E120_GET_COMMAND) {
        if (_rdm.packet.DataLength > 0) {
          // Unexpected data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if (_rdm.packet.SubDev != 0) {
          // No sub-devices supported
          nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
        } else {
          // Some supported PIDs shouldn't be returned as per the standard, these are:
          // E120_DISC_UNIQUE_BRANCH
          // E120_DISC_MUTE
          // E120_DISC_UN_MUTE
          // E120_SUPPORTED_PARAMETERS
          // E120_IDENTIFY_DEVICE
          // E120_DEVICE_INFO
          // E120_DMX_START_ADDRESS
          // E120_SOFTWARE_VERSION_LABEL
          _rdm.packet.DataLength = 2 * (3 + _initData->additionalCommandsLength);
          WRITEINT(_rdm.packet.Data   , E120_MANUFACTURER_LABEL);
          WRITEINT(_rdm.packet.Data+ 2, E120_DEVICE_MODEL_DESCRIPTION);
          WRITEINT(_rdm.packet.Data+ 4, E120_DEVICE_LABEL);
          uint8_t offset = 6;
          if (_initData->sensorsLength > 0) {
            _rdm.packet.DataLength += 2 * 2;
            offset += 2 * 2;
            WRITEINT(_rdm.packet.Data+ 6, E120_SENSOR_DEFINITION);
            WRITEINT(_rdm.packet.Data+ 8, E120_SENSOR_VALUE);
          }
          for (uint16_t n = 0; n < _initData->additionalCommandsLength; n++) {
            WRITEINT(_rdm.packet.Data+offset+n+n, _initData->additionalCommands[n]);
          }
          handled = true;
        }
      } else if (CmdClass == E120_SET_COMMAND) {
        // Unexpected set
        nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
      }

// ADD: PARAMETER_DESCRIPTION

    } else if (Parameter == SWAPINT(E120_SENSOR_DEFINITION) && _initData->sensorsLength > 0) { // 0x0200
      if (CmdClass == E120_GET_COMMAND) {
        if (_rdm.packet.DataLength != 1) {
          // Unexpected data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if (_rdm.packet.SubDev != 0) {
          // No sub-devices supported
          nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
        } else {
          uint8_t sensorNr = _rdm.packet.Data[0];
          if (sensorNr >= _initData->sensorsLength) {
            // Out of range sensor
            nackReason = E120_NR_DATA_OUT_OF_RANGE;
          } else {
            _rdm.packet.DataLength = 13 + strlen(_initData->sensors[sensorNr].description);
            _rdm.packet.Data[0] = sensorNr;
            _rdm.packet.Data[1] = _initData->sensors[sensorNr].type;
            _rdm.packet.Data[2] = _initData->sensors[sensorNr].unit;
            _rdm.packet.Data[3] = _initData->sensors[sensorNr].prefix;
            WRITEINT(_rdm.packet.Data +  4, _initData->sensors[sensorNr].rangeMin);
            WRITEINT(_rdm.packet.Data +  6, _initData->sensors[sensorNr].rangeMax);
            WRITEINT(_rdm.packet.Data +  8, _initData->sensors[sensorNr].normalMin);
            WRITEINT(_rdm.packet.Data + 10, _initData->sensors[sensorNr].normalMax);
            _rdm.packet.Data[12] = (_initData->sensors[sensorNr].lowHighSupported ? 2 : 0) | (_initData->sensors[sensorNr].recordedSupported ? 1 : 0);
            memcpy(_rdm.packet.Data + 13, _initData->sensors[sensorNr].description, _rdm.packet.DataLength - 13);
            handled = true;
          }
        }
      } else if (CmdClass == E120_SET_COMMAND) {
        // Unexpected set
        nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
      }
    } else if (Parameter == SWAPINT(E120_SENSOR_VALUE) && _initData->sensorsLength > 0) { // 0x0201
      if (CmdClass == E120_GET_COMMAND) {
        if (_rdm.packet.DataLength != 1) {
          // Unexpected data
          nackReason = E120_NR_FORMAT_ERROR;
        } else if (_rdm.packet.SubDev != 0) {
          // No sub-devices supported
          nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
        } else {
          uint8_t sensorNr = _rdm.packet.Data[0];
          if (sensorNr >= _initData->sensorsLength) {
            // Out of range sensor
            nackReason = E120_NR_DATA_OUT_OF_RANGE;
          } else {
            int16_t sensorValue = 0;
            int16_t lowestValue = 0;
            int16_t highestValue = 0;
            int16_t recordedValue = 0;
            bool8 res = false;
            if (_sensorFunc) {
              res = _sensorFunc(sensorNr, &sensorValue, &lowestValue, &highestValue, &recordedValue);
            }
            if (res) {
              _rdm.packet.DataLength = 9;
              _rdm.packet.Data[0] = sensorNr;
              WRITEINT(_rdm.packet.Data +  1, sensorValue);
              WRITEINT(_rdm.packet.Data +  3, lowestValue);
              WRITEINT(_rdm.packet.Data +  5, highestValue);
              WRITEINT(_rdm.packet.Data +  7, recordedValue);
              handled = true;
            } else {
              nackReason = E120_NR_HARDWARE_FAULT;
            }
          }
        }
      } else if (CmdClass == E120_SET_COMMAND) {
        // Unhandled set. Set on a sensor is used to reset stats.
        // User should process it in own handler when sensor supports high/low or recorded value.
        nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
      }

    } else {
      handled = false;

    }  // if
  }  // if

  if (doRespond)
    respondMessage(handled, nackReason);
} // _processRDMMessage


// ----- internal functions and interrupt implementations -----

// internal init function for all static initializations.
void DMXSerialClass2::_baseInit() {
  _dmxPos = 0;

  _dmxState= IDLE; // initial state
  _gotLastPacket = millis(); // remember current (relative) time in msecs.

  // initialize the DMX buffer
  for (int n = 0; n < DMXSERIAL_MAX+1; n++)
    _dmxData[n] = 0;

  pinMode(_dmxModePin, OUTPUT); // enables pin 2 for output to control data direction
} // _baseInit


// save all data to EEPROM
void DMXSerialClass2::_saveEEPRom()
{
  // This structure is defined for mapping the values in the EEPROM
  struct EEPROMVALUES eeprom;

  // initialize
  for (unsigned int i = 0; i < sizeof(eeprom); i++)
    ((byte *)(&eeprom))[i] = 0x00;

  eeprom.sig1 = 0x6D;
  eeprom.sig2 = 0x68;
  eeprom.startAddress = _startAddress;
  strcpy (eeprom.deviceLabel, deviceLabel);
  DeviceIDCpy(eeprom.deviceID, _devID);

  for (unsigned int i = 0; i < sizeof(eeprom); i++) {
    if (((byte *)(&eeprom))[i] != EEPROM.read(i))
      EEPROM.write(i, ((byte *)(&eeprom))[i]);
  } // for
} // _saveEEPRom


// ----- internal non-class functions and interrupt implementations -----

// Initialize the Hardware serial port with the given baud rate
// using 8 data bits, no parity, 2 stop bits for data
// and 8 data bits, even parity, 1 stop bit for the break
void _DMXSerialBaud(uint16_t baud_setting, uint8_t format)
{
  // assign the baud_setting to the USART Baud Rate Register
  UCSRnA = 0;                 // 04.06.2012: use normal speed operation
  UBRRnH = baud_setting >> 8;
  UBRRnL = baud_setting;

  // 2 stop bits and 8 bit character size, no parity
  UCSRnC = format;
} // _DMXSerialBaud


// send the next byte after current byte was sent completely.
void _DMXSerialWriteByte(uint8_t data)
{
  // putting data into buffer sends the data
  UDRn = data;
} // _DMXSerialWrite


// Interrupt Service Routine, called when a byte or frame error was received.
ISR(USARTn_RX_vect)
{
  //  digitalWrite(rxStatusPin, HIGH);
  uint8_t  USARTstate= UCSRnA;    //get state before data!
  uint8_t  DmxByte   = UDRn;      //get data
  uint8_t  DmxState  = _dmxState; //just load once from SRAM to increase speed

  if (USARTstate & (1<<FEn)) { //check for break
    _dmxState = BREAK; // break condition detected.
    _dmxPos= 0;        // The next data byte is the start byte

  } else if (DmxState == IDLE) {
    // wait on...

  } else if (DmxState == BREAK) {
    if (DmxByte == 0) {
      _dmxState = DMXDATA; // DMX data start code detected
      // _dmxData[_dmxPos++] = DmxByte;  // store in DMX buffer
      _dmxPos = 1;
      _gotLastPacket = millis(); // remember current (relative) time in msecs.

    } else if (DmxByte == E120_SC_RDM) {
      _dmxState = RDMDATA;  // RDM command start code
      _rdm.buffer[_dmxPos++] = DmxByte;  // store in RDM buffer (in StartCode)
      _rdmCheckSum = DmxByte;

    } else {
      // This might be a non RDM command -> not implemented so wait for next BREAK !
      _dmxState= IDLE;
    } // if

  } else if (DmxState == DMXDATA) {
    // another DMX byte
    _dmxData[_dmxPos++]= DmxByte;  // store received data into DMX data buffer.

    if (_dmxPos > DMXSERIAL_MAX) { // all channels done.
      _dmxState = IDLE; // wait for next break
    } // if

  } else if (DmxState == RDMDATA) {
    // another RDM byte
    if (_dmxPos >= (int)sizeof(_rdm.buffer)) {
      // too much data ...
      _dmxState = IDLE; // wait for next break
    } else {
      _rdm.buffer[_dmxPos++] = DmxByte;
      _rdmCheckSum += DmxByte;

      if (_dmxPos == _rdm.packet.Length) {
        // all data received. Now getting checksum !
        _dmxState = CHECKSUMH; // wait for checksum High Byte
      } // if
    } // if

  } else if (DmxState == CHECKSUMH) {
    // High byte of RDM checksum -> subtract from checksum
    _rdmCheckSum -= DmxByte << 8;
    _dmxState = CHECKSUML;

  } else if (DmxState == CHECKSUML) {
    // Low byte of RDM checksum -> subtract from checksum
    _rdmCheckSum -= DmxByte;

    // now check some error conditions and addressing issues
    if ((_rdmCheckSum == 0) && (_rdm.packet.SubStartCode == E120_SC_SUB_MESSAGE)) { // 0x01
      // prepare for answering when tick() is called
      _rdmAvailable = true;
      _gotLastPacket = millis(); // remember current (relative) time in msecs.
      // TIMING: remember when the last byte was sent
      _timingReceiveEnd = micros();
    } // if

    _dmxState = IDLE; // wait for next break or RDM package processing.
  } // if

} // ISR(USART_RX_vect)


// Interrupt service routines that are called when the actual byte was sent.
// When changing speed (for sending break and sending start code) we use TX finished interrupt
// which occurs shortly after the last stop bit is sent
// When staying at the same speed (sending data bytes) we use data register empty interrupt
// which occurs shortly after the start bit of the *previous* byte
// When sending a DMX sequence it just takes the next channel byte and sends it out.
// In DMXController mode when the buffer was sent completely the DMX sequence will resent, starting with a BREAK pattern.
// In DMXReceiver mode this interrupt is disabled and will not occur.
// In RDM mode this interrupt acts like in DMXController mode but the packet is not resent automatically.
ISR(USARTn_TX_vect)
{
  if (_dmxPos == 0) {
    // this interrupt occurs after the stop bits of the break byte
    // now back to DMX speed: 250000baud
    _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT);
    // take next interrupt when data register empty (early) and handle sending the next byte in USART_UDRE_vect
    UCSRnB = (1<<TXENn) | (1<<UDRIEn);

    // write start code
    _DMXSerialWriteByte((uint8_t)0);
    _dmxPos = 1;
  } // if
} // ISR(USART_TX_vect)


// send back original Message including changed data in some cases
void respondMessage(bool8 isHandled, uint16_t nackReason)
{
  int bufferLen;
  uint16_t checkSum = 0;

  // no need to set these data fields:
  // StartCode, SubStartCode
  if (isHandled) {
    _rdm.packet.ResponseType = E120_RESPONSE_TYPE_ACK; // 0x00
  } else {
    _rdm.packet.ResponseType = E120_RESPONSE_TYPE_NACK_REASON; // 0x00
    _rdm.packet.DataLength = 2;
    _rdm.packet.Data[0] = (nackReason >> 8) & 0xFF;
    _rdm.packet.Data[1] = nackReason & 0xFF;
  } // if
  _rdm.packet.Length = _rdm.packet.DataLength + 24; // total packet length

  // swap SrcID into DestID for sending back.
  DeviceIDCpy(_rdm.packet.DestID, _rdm.packet.SourceID);
  DeviceIDCpy(_rdm.packet.SourceID, _devID);

  _rdm.packet.CmdClass++;
  // Parameter

  // prepare buffer and Checksum
  _dmxSendLen = _rdm.packet.Length;
  for (byte i = 0; i < _dmxSendLen; i++) {
    checkSum += _dmxSendBuffer[i];
  } // for

  // TIMING: don't send too fast, min: 176 microseconds
  unsigned long d = micros() - _timingReceiveEnd;
  if (d < 190)
    delayMicroseconds(190 - d);

  // send package by starting with a BREAK
  UCSRnB = (1<<TXENn); // send without no interrupts !

  _DMXSerialBaud(Calcprescale(BREAKSPEED), BREAKFORMAT);
  digitalWrite(_dmxModePin, _dmxModeOut); // data Out direction
  UDRn = 0;
  UCSRnA= (1<<TXCn);
  loop_until_bit_is_set(UCSRnA, TXCn);

  _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT);

  bufferLen = _rdm.packet.Length;

  for (byte i = 0; i < bufferLen; i++) {
    UDRn = _rdm.buffer[i];
    UCSRnA= (1<<TXCn);
    loop_until_bit_is_set(UCSRnA, TXCn);
  } // for

  // send High Byte
  UDRn = checkSum >> 8;
  UCSRnA= (1<<TXCn);
  loop_until_bit_is_set(UCSRnA, TXCn);

  // send Low Byte
  UDRn = checkSum & 0xFF;
  UCSRnA= (1<<TXCn);
  loop_until_bit_is_set(UCSRnA, TXCn);

  digitalWrite(_dmxModePin, _dmxModeIn); // switch back to data In direction

  // Re-enable receiver and Receive interrupt
  _dmxState= IDLE; // initial state
  UCSRnB = (1<<RXENn) | (1<<RXCIEn);
  _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT); // Enable serial reception with a 250k rate
} // respondMessage


// generate a random number 0..255
int random255() {
  int num = 0;
  for (int i = 0; i<8; i++) {
     num |= (analogRead(A0) & 0x01) << i;
    delay(5);
  }
  return(num);
} // random255()

// --- End of File ---
