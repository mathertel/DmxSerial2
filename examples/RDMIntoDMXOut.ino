// - - - - -
// DmxSerial2 - A hardware supported interface to DMX and RDM.
// RDMIntoDMXOut.ino: Receive DMX at an RDM set address and output DMX commands on channel 1, to make any non-RDM DMX device RDM compliant.
//
// Copyright (c) 2011-2013 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
//
// More documentation and samples are available at http://www.mathertel.de/Arduino
// - - - - -

#include <DmxSimple.h>
#include <EEPROM.h>
#include <DMXSerial2.h>

// see DMXSerial2.h for the definition of the fields of this structure
const uint16_t my_pids[] = {};
struct RDMINIT rdmInit = {
  "Company Name", // Manufacturer Label
  1, // Device Model ID
  "RDM Module", // Device Model Label
  3, // footprint
  (sizeof(my_pids)/sizeof(uint16_t)), my_pids
};

uint16_t startAddress = 0; 
int maxSize = 3;

void setup () {
  DMXSerial2.init(&rdmInit, processCommand);
  
  pinMode(9, OUTPUT); // defined isIdentifyMode pin for output
  digitalWrite(9, LOW);
  DmxSimple.usePin(3); // Use Digital pin 3 for DMXSimple output
  DmxSimple.maxChannel(maxSize); // set the maxChannel for DMXSimple to the same as the RDM module's max size

} // setup()


void loop() {
    if (DMXSerial2.isIdentifyMode()) {
      
      digitalWrite(9, HIGH); // indicator light for the Identify mode
      
    } else {
      startAddress = DMXSerial2.getStartAddress(); // retrieve current RDM start address
      
      for (int i = 0; i < maxSize; i++){ // for all DMX packets sent to addresses up to maxSize, forward to DMX out
        
        DmxSimple.write(i + 1, DMXSerial2.readRelative(i)); // Grab all DMX packets sent to RDM address and forward to DMX out
      }
      
      digitalWrite(9, LOW);
    }
    
  DMXSerial2.tick();
} // loop()

// This function was registered to the DMXSerial2 library in the initRDM call.
// Here device specific RDM Commands are implemented.
bool8 processCommand(struct RDMDATA *rdm, uint16_t *nackReason)
{
  byte CmdClass       = rdm->CmdClass;     
  uint16_t Parameter  = rdm->Parameter;     
  bool8 handled = false;

  if (CmdClass == E120_SET_COMMAND) {
      *nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
  }
  
  return handled;
} 
