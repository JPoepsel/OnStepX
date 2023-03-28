// ICHaus TW8 1Vss to SPI converter/interpolator
// jp  03.2023
#pragma once

#include "../Encoder.h"

#if AXIS1_ENCODER == TW8_SPI || AXIS2_ENCODER == TW8_SPI || AXIS3_ENCODER == TW8_SPI || \
    AXIS4_ENCODER == TW8_SPI || AXIS5_ENCODER == TW8_SPI || AXIS6_ENCODER == TW8_SPI || \
    AXIS7_ENCODER == TW8_SPI || AXIS8_ENCODER == TW8_SPI || AXIS9_ENCODER == TW8_SPI

  // Designed according SPI protocol description found in TW8_datasheet_D3en.pdf and TW8_PR_datasheet_B5en.pdf

  // Uses std. SPI pins except CS which use passed "ChipSelectPin"
  // This may changed for general usage and transfered in the constructor as it is done e.g. for the TmcSPI-Drivers....
  // See IcTw8::init() in IC_TW8.cpp
 
  class IcTw8 : public Encoder {
    public:
      IcTw8(int16_t ChipSelectPin);
      void init();      
      int32_t read();
      void write(int32_t count);

    private:
    
      int32_t offsetPosition = 0;
      int16_t TheChipSelectPin=0;
      word inter;  // interpolation factor read from TW8 
  };

#endif
