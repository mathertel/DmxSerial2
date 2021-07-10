// - - - - -
// DMXSerial2 - A hardware supported interface to DMX and RDM.
// DMXSerial2.h: Library header file
//
// Copyright (c) 2011-2013 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
//
// Documentation and samples are available at http://www.mathertel.de/Arduino
// 25.07.2011 creation of the DMXSerial library.
// 10.09.2011 fully control the serial hardware register
//            without using the Arduino Serial (HardwareSerial) class to avoid ISR implementation conflicts.
// 01.12.2011 include file changed to work with the Arduino 1.0 environment
// 28.12.2011 unused variable DmxCount removed
// 10.05.2012 added method noDataSince to check how long no packet was received
// 04.06.2012: set UCSRnA = 0 to use normal speed operation
// 30.07.2012 corrected TX timings with UDRE and TX interrupts
//            fixed bug in 512-channel RX
// 02.11.2012 starting RDM related experimental version.
// 22.01.2013 first published version to support RDM
// 01.03.2013 finished some "TIMING" topics
// 08.03.2013 finished as a library
// 12.05.2013 added the defines to support Arduino MEGA 2560 (port 0 and 1) and Arduino Leonardo (port 1)
// 15.05.2013 Arduino Leonard and Arduino MEGA compatibility
// 16.05.2013 using #0987 as manufacurer id, that was registered to myself (mathertel.de).
// 18.06.2013 implementing random device IDs
// 01.09.2013 implemented all minimal required RDM parameters (+SOFTWARE_VERSION_LABEL, +SUPPORTED_PARAMETERS)
// 06.09.2013 simplifications, removing pure DMX mode code and memory optimizations.
// 21.11.2013 response to E120_DISC_MUTE and E120_DISC_UN_MUTE messages as required by the spec.
// 03.12.2013 Code merged from raumzeitlabor
// 04.12.2013 Allow manufacturer broadcasts
// 05.12.2013 FIX: response only to direct commands as required by the spec.
// 13.12.2013 ADD: getDeviceID() function added
// 15.12.2013 introducing the type DEVICEID and copy by using memcpy to save pgm space.
// 12.01.2014 Peter Newman: make the responder more compliant with the OLA RDM Tests
// 24.01.2014 Peter Newman: More compliance with the OLA RDM Tests around sub devices and mute messages
// 24.01.2014 Peter Newman/Sean Sill: Get device specific PIDs returning properly in supportedParameters
// 24.01.2014 Peter Newman: Make the device specific PIDs compliant with the OLA RDM Tests. Add device model ID option
// 12.04.2015 making library Arduino 1.6.x compatible
// 12.04.2015 change of using datatype boolean to bool8.
// 15.06.2015 On DMX lines sometimes a BREAK condition occurs in between RDM packets from the controller
//            and the device response. Ignore that when no data has arrived.
// 25.05.2017 Stefan Krupop: Add support for sensors
// 21.08.2018 improvements and typo by Peter Newman
// - - - - -

#ifndef DmxSerial_h
#define DmxSerial_h

#include "rdm.h"

// ----- Constants -----

#define DMXSERIAL_MAX 512                  ///< The max. number of supported DMX data channels
#define DMXSERIAL_MIN_SLOT_VALUE 0         ///< The min. value a DMX512 slot can take
#define DMXSERIAL_MAX_SLOT_VALUE 255       ///< The max. value a DMX512 slot can take
#define DMXSERIAL_MAX_RDM_STRING_LENGTH 32 ///< The max. length of a string in RDM

// ----- Enumerations -----

// ----- Types -----

typedef uint8_t bool8;
typedef uint8_t byte;


/**
 * @brief Type definition for a unique DEVICE ID.
 *
 * DEVICEID[0..1] contains a ESTA Manufacturer ID.
 * DEVICEID[2..5] contains a unique number per device.
 */
typedef byte DEVICEID[6];

// ----- structures -----


/**
 * @brief The RDMDATA structure defines the RDM network packages.
 *
 * This structure has the size of (24+data) and is used by all GET/SET RDM commands.
 * The maximum permitted data length according to the spec is 231 bytes.
 */

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

/// @brief Use SWAPINT to swap the 2 bytes of an 16-bit int to match the byte order on the DMX Protocol.
///
/// 16-bit and 32-bit integers in the RDM protocol are transmitted highbyte - lowbyte.
/// but the ATMEGA processors store them in highbyte - lowbyte order.
/// avoid using this macro on variables but use it on the constant definitions.
#define SWAPINT(i) (((i&0x00FF)<<8) | ((i&0xFF00)>>8))

/// Use SWAPINT32 to swap the 4 bytes of a 32-bit int to match the byte order on the DMX Protocol.
#define SWAPINT32(i) ((i&0x000000ff)<<24) | ((i&0x0000ff00)<<8) | ((i&0x00ff0000)>>8) | ((i&0xff000000)>>24)

/// read a 16 bit number from a data buffer location
#define READINT(p) ((p[0]<<8) | (p[1]))

/// write a 16 bit number to a data buffer location
#define WRITEINT(p, d) (p)[0] = (d&0xFF00)>>8; (p)[1] = (d&0x00FF);


// ----- Callback function types -----

extern "C" {
  /**
   * @brief Callback function for RDM functions.
   *
   * @param [in,out] buffer Buffer containing the RDM network package.
   * @param [in,out] nackReason on error a RDM Response NACK Reason Code.
   */
  typedef bool8 (*RDMCallbackFunction)(struct RDMDATA *buffer, uint16_t *nackReason);

  /**
   * @brief Callback function for RDM sensors.
   */
  typedef bool8 (*RDMGetSensorValue)(uint8_t sensorNr, int16_t *value, int16_t *lowestValue, int16_t *highestValue, int16_t *recordedValue);
}

// ----- Library Class -----

// These types are used to pass all the data into the initRDM function.
// The library needs this data to reposonse at the corresponding commands for itself.

struct RDMPERSONALITY {
  uint16_t footprint;
  // maybe more here... when supporting more personalities.
}; // struct RDMPERSONALITY

struct RDMSENSOR {
  uint8_t type;
  uint8_t unit;
  uint8_t prefix;
  int16_t rangeMin;
  int16_t rangeMax;
  int16_t normalMin;
  int16_t normalMax;
  bool8 lowHighSupported;
  bool8 recordedSupported;
  char *description;
}; // struct RDMSENSOR

struct RDMINIT {
  const char          *manufacturerLabel; //
  const uint16_t          deviceModelId;       //
  const char          *deviceModel;       //
  uint16_t footprint;
  // uint16_t personalityCount;
  // RDMPERSONALITY *personalities;
  const uint16_t        additionalCommandsLength;
  const uint16_t       *additionalCommands;
  const uint8_t sensorsLength;
  const RDMSENSOR *sensors;
}; // struct RDMINIT



class DMXSerialClass2
{
  public:
    /**
     * @brief Initialize for RDM mode.
     * @param [in] initData Startup parameters.
     * @param [in] func Callback function for answering on device specific features.
     * @param [in] modePin The pin used to switch the communication direction. This parameter is optiona and defaults to 2.
     * @param [in] modeIn  The level for inbound communication. This parameter is optiona and defaults to 0 = LOW.
     * @param [in] modeOut The level for outbound communication. This parameter is optiona and defaults to 1 = HIGH.
     */
    void    init (struct RDMINIT *initData, RDMCallbackFunction func, uint8_t modePin = 2, uint8_t modeIn = 0, uint8_t modeOut = 1) {
      init(initData, func, NULL, modePin, modeIn, modeOut);
    }

    /**
     * @brief Initialize for RDM mode with sensor.
     * @param [in] initData Startup parameters.
     * @param [in] func Callback function for answering on device specific features.
     * @param [in] sensorFunc Callback function for retrieving a sensor value.
     * @param [in] modePin The pin used to switch the communication direction. This parameter is optiona and defaults to 2.
     * @param [in] modeIn  The level for inbound communication. This parameter is optiona and defaults to 0 = LOW.
     * @param [in] modeOut The level for outbound communication. This parameter is optiona and defaults to 1 = HIGH.
     */
    void    init (struct RDMINIT *initData, RDMCallbackFunction func, RDMGetSensorValue sensorFunc, uint8_t modePin = 2, uint8_t modeIn = 0, uint8_t modeOut = 1);

    /**
     * @brief Read the current value of a channel.
     * @param [in] channel The channel number.
     * @return uint8_t The current value.
     */
    uint8_t read       (int channel);

    // Read the last known value of a channel by using the startAddress and footprint range.
    uint8_t readRelative(unsigned int channel);

    /**
     * @brief Write a new value to a channel.
     * This function also can be called in DMXReceiver mode to set a channel value even when no data is received.
     * It will be overwritten by the next received package.
     * @param [in] channel The channel number.
     * @param [in] value The current value.
     * @return void
     */
    void    write      (int channel, uint8_t value);

    /**
     * @brief Return the duration since data was received.
     * On startup the internal timer is reset too.
     * @return long milliseconds since last pdata package.
     */
    unsigned long noDataSince();

    // ----- RDM specific members -----

    /// Return true when identify mode was set on by controller.
    bool8 isIdentifyMode();

    /// Returns the Device ID. Copies the UID to the buffer passed through the uid argument.
    void getDeviceID(DEVICEID id);

    /// Return the current DMX start address that is the first dmx address used by the device.
    uint16_t getStartAddress();

    /// Return the current DMX footprint, that is the number of dmx addresses used by the device.
    uint16_t getFootprint();

    /// Register a device-specific implemented function for RDM callbacks
    void    attachRDMCallback (RDMCallbackFunction newFunction);

    /// Register a device-specific implemented function for getting sensor values
    void    attachSensorCallback (RDMGetSensorValue newFunction);

    /// check for unprocessed RDM Command.
    void    tick(void);

    /// Terminate operation.
    void    term(void);

    /// A short custom label given to the device.
    char deviceLabel[DMXSERIAL_MAX_RDM_STRING_LENGTH];

    /// don't use that method from extern.
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 isHandled);

    /// save all data to EEPROM
    void _saveEEPRom();

  private:
    /// process a relevant message
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 isHandled, bool8 doRespond);

    /// common internal initialization function.
    void _baseInit();

    /// callback function to device specific code
    RDMCallbackFunction _rdmFunc;

    /// callback function to get sensor value
    RDMGetSensorValue _sensorFunc;

    /// remember the given manufacturer label and device model strings during init
    struct RDMINIT *_initData;

    /// intern parameter settings
    const char *_softwareLabel;
    bool8  _identifyMode;
    uint16_t _startAddress;
}; // class DMXSerialClass2


// Use the DMXSerial2 library through the DMXSerial2 object.
extern DMXSerialClass2 DMXSerial2;

#endif

// --- End of File ---
