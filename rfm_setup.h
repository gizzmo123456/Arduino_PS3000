// rfm_setup.h
// basic setup for the RFM radio on the feather board
// this is shared between 2 devices, which is cool cuz
// the keys need to be the same

#ifndef RFM_SETUP
#define RFM_SETUP

#include <RH_RF69.h>

void rfm_setup(RH_RF69 rfm)
{
	
	// set freq
  if( !rfm.setFrequency(RADIO_FREQ) )
  {
    Serial.println(" Failed to set frequency ");
  }

  // give it some juice.
  rfm.setTxPower(20, true); // give it all the jucie (range 14 to 20)

  // Let's keep this between me and you.
  uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rfm.setEncryptionKey(key);
  
}

#endif