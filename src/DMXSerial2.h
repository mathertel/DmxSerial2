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
// 12.04.2015 making library Arduino 1.6.x compatible
// 12.04.2015 change of using datatype boolean to bool8.
// - - - - -

#ifndef DmxSerial_h
#define DmxSerial_h

#include "rdm.h"

// ----- Constants -----

#define DMXSERIAL_MAX 512 // max. number of supported DMX data channels

#define DMXSERIAL_MIN_SLOT_VALUE 0 // min. value a DMX512 slot can take

#define DMXSERIAL_MAX_SLOT_VALUE 255 // max. value a DMX512 slot can take

#define DMXSERIAL_MAX_RDM_STRING_LENGTH 32 // max. length of a string in RDM

// ----- Enumerations -----

// ----- Types -----

typedef uint8_t bool8;
typedef uint8_t byte;


// This is the definition for a unique DEVICE ID.
// DEVICEID[0..1] ESTA Manufacturer ID
// DEVICEID[2..5] unique number
typedef byte DEVICEID[6];

// ----- structures -----

// The RDMDATA structure (length = 24+data) is used by all GET/SET RDM commands.
// The maximum permitted data length according to the spec is 231 bytes.
struct RDMDATA {
  byte     StartCode;    // Start Code 0xCC for RDM
  byte     SubStartCode; // Start Code 0x01 for RDM
  byte     Length;       // packet length
  byte     DestID[6];
  byte     SourceID[6];

  byte     _TransNo;     // transaction number, not checked
  byte     ResponseType;    // ResponseType
  byte     _unknown;     // I don't know, ignore this
  uint16_t SubDev;      // sub device number (root = 0) 
  byte     CmdClass;     // command class
  uint16_t Parameter;	   // parameter ID
  byte     DataLength;   // parameter data length in bytes
  byte     Data[231];   // data byte field
}; // struct RDMDATA


// ----- macros -----

// 16-bit and 32-bit integers in the RDM protocol are transmitted highbyte - lowbyte.
// but the ATMEGA processors store them in highbyte - lowbyte order.
// Use SWAPINT to swap the 2 bytes of an 16-bit int to match the byte order on the DMX Protocol.
// avoid using this macro on variables but use it on the constant definitions.
#define SWAPINT(i) (((i&0x00FF)<<8) | ((i&0xFF00)>>8))
// Use SWAPINT32 to swap the 4 bytes of a 32-bit int to match the byte order on the DMX Protocol.
#define SWAPINT32(i) ((i&0x000000ff)<<24) | ((i&0x0000ff00)<<8) | ((i&0x00ff0000)>>8) | ((i&0xff000000)>>24)

// read a 16 bit number from a data buffer location
#define READINT(p) ((p[0]<<8) | (p[1]))

// write a 16 bit number to a data buffer location
#define WRITEINT(p, d) (p)[0] = (d&0xFF00)>>8; (p)[1] = (d&0x00FF);

// ----- Callback function types -----

extern "C" {
  typedef bool8 (*RDMCallbackFunction)(struct RDMDATA *buffer, uint16_t *nackReason);
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
  const uint16_t          deviceModelId;       //
  char          *deviceModel;       //
  uint16_t footprint;
  // uint16_t personalityCount;
  // RDMPERSONALITY *personalities;
  const uint16_t        additionalCommandsLength;
  const uint16_t       *additionalCommands;
}; // struct RDMINIT



class DMXSerialClass2
{
  public:
    // Initialize for RDM mode.
    void    init (struct RDMINIT *initData, RDMCallbackFunction func, uint8_t modePin = 2, uint8_t modeIn = 0, uint8_t modeOut = 1);
    
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
    bool8 isIdentifyMode();

    // Returns the Device ID. Copies the UID to the buffer passed through the uid argument.
    void getDeviceID(DEVICEID id);

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
    char deviceLabel[DMXSERIAL_MAX_RDM_STRING_LENGTH];

    // don't use that method from extern.
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 isHandled);

    // save all data to EEPROM
	void _saveEEPRom();
  private:
    // process a relevant message
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 isHandled, bool8 doRespond);
  
    // common internal initialization function.
    void _baseInit();

    // callback function to device specific code
    RDMCallbackFunction _rdmFunc;

    // remember the given manufacturer label and device model strings during init
    struct RDMINIT *_initData;

    // intern parameter settings
    const char *_softwareLabel;
    bool8  _identifyMode;
    uint16_t _startAddress;
}; // class DMXSerialClass2


// Use the DMXSerial2 library through the DMXSerial2 object.
extern DMXSerialClass2 DMXSerial2;

#endif

// --- End of File ---
