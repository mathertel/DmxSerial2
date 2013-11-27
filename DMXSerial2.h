// - - - - -
// DMXSerial2 - A hardware supported interface to DMX and RDM.
// DMXSerial2.h: Library header file
// 
// Copyright (c) 2011-2013 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// 
// Documentation and samples are available at http://www.mathertel.de/Arduino
// 25.07.2011 creation of the DMXSerial library.
// 01.12.2011 include file changed to work with the Arduino 1.0 environment
// 10.05.2012 added method noDataSince to check how long no packet was received
// 22.01.2013 first published version to support RDM
// 01.03.2013 finished some "TIMING" topics
// 08.03.2013 finished as a library
// - - - - -

#ifndef DmxSerial_h
#define DmxSerial_h

#include "rdm.h"

// ----- Constants -----

#define DMXSERIAL_MAX 512 // max. number of supported DMX data channels

#define DmxModePin 2     // Arduino pin 2 for controlling the data direction
#define DmxModeOut HIGH  // set the level to HIGH for outgoing data direction
#define DmxModeIn  LOW   // set the level to LOW  for incomming data direction

// ----- Enumerations -----

// ----- Types -----

typedef uint8_t boolean;
typedef uint8_t byte;

// ----- structures -----

// The RDMDATA structure (length = 24+data) is used by all GET/SET RDM commands.
// The maximum data length I found through my searches was 32. I give 2 extra bytes for security reason.
struct RDMDATA {
  byte     StartCode;    // Start Code 0xCC for RDM
  byte     SubStartCode; // Start Code 0x01 for RDM
  byte     Length;       // packet length
  byte     DestID[6];
  byte     SourceID[6];

  byte     _TransNo;     // transaction number, not checked
  byte     ResponseType;    // ResponseType
  byte     _unknown;     // I don't know, ignore this
  uint16_t _SubDev;      // sub device number (root = 0) 
  byte     CmdClass;     // command class
  uint16_t Parameter;	   // parameter ID
  byte     DataLength;   // parameter data length in bytes
  byte     Data[32+2];   // data byte field
}; // struct RDMDATA


// ----- macros -----

// 16-bit integers in the RDM protocol are transmitted highbyte - lowbyte.
// but the ATMEGA processors store 16-bit data in highbyte - lowbyte order.
// Use SWAPINT to swap the 2 bytes of an 16-bit int to match the byte order on the DMX Protocol.
// avoid using this macro on variables but use it on the constant definitions.
#define SWAPINT(i) (((i&0x00FF)<<8) | ((i&0xFF00)>>8))

// read a 16 bit number from a data buffer location
#define READINT(p) ((p[0]<<8) | (p[1]))

// write a 16 bit number to a data buffer location
#define WRITEINT(p, d) (p)[0] = (d&0xFF00)>>8; (p)[1] = (d&0x00FF);

// ----- Callback function types -----

extern "C" {
  typedef boolean (*RDMCallbackFunction)(struct RDMDATA *buffer);
}

// ----- Library Class -----

// These types are used to pass all the data into the initRDM function.
// The library needs this data to reposonse at the corresponding commands for itself.

struct RDMPERSONALITY {
  uint16_t footprint;
  // maybe more here... when supporting more personalitites.
}; // struct RDMPERSONALITY


struct RDMINIT {
  char          *manufacturerLabel; //
  char          *deviceModel;       //
  uint16_t footprint;
  // uint16_t personalityCount;
  // RDMPERSONALITY *personalities;
  uint16_t        additionalCommandsLength;
  uint16_t       *additionalCommands;
}; // struct RDMINIT



class DMXSerialClass2
{
  public:
    // Initialize for RDM mode.
    void    init (struct RDMINIT *initData, RDMCallbackFunction func);
    
    // Read the last known value of a channel.
    uint8_t read       (int channel);

    // Read the last known value of a channel by using the startAddress and footprint range.
    uint8_t readRelative(unsigned int channel);

    // Write a new value of a channel.
    void    write      (int channel, uint8_t value);

    // Calculate how long no data backet was received
    unsigned long noDataSince();

    // ----- RDM specific members -----
    
    // Return true when identify mode was set on by controller.
    boolean isIdentifyMode();

    // Return the current DMX start address that is the first dmx address used by the device.
    uint16_t getStartAddress();

    // Return the current DMX footprint, that is the number of dmx addresses used by the device.
    uint16_t getFootprint();

    // Register a device-specific implemented function for RDM callbacks
    void    attachRDMCallback (RDMCallbackFunction newFunction);

    // check for unprocessed RDM Command.
    void    tick(void);

    // Terminate operation.
    void    term(void);
    
    // A short custom label given to the device. 
    char deviceLabel[32];

    // don't use that method from extern.
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, boolean isHandled);
  
  private:
    // common internal initialization function.
    void _baseInit();

    // save all data to EEPROM
    void _saveEEPRom();
    
    // callback function to device specific code
    RDMCallbackFunction _rdmFunc;

    // remember the given manufacturer label and device model strings during init
    struct RDMINIT *_initData;

    // intern parameter settings
    const char *_softwareLabel;
    boolean  _identifyMode;
    uint16_t _startAddress;
}; // class DMXSerialClass2


// Use the DMXSerial2 library through the DMXSerial2 object.
extern DMXSerialClass2 DMXSerial2;

#endif

// --- End of File ---