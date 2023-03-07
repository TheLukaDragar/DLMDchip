
#include <MyGenericSPI.h>
#include <SPI.h>




// Declare a single default instance of the hardware SPI interface class
MyGenericSPI hardware_spi;




MyGenericSPI::MyGenericSPI(Frequency frequency, BitOrder bitOrder, DataMode dataMode)
    :

    RHGenericSPI(frequency, bitOrder, dataMode)
{
}

uint8_t MyGenericSPI::transfer(uint8_t data) 
{
    return SPI.transfer(data);
}



void MyGenericSPI::attachInterrupt() 
{
    SPI.attachInterrupt();

}

void MyGenericSPI::detachInterrupt() 
{
    SPI.detachInterrupt();
}
    
void MyGenericSPI::begin() 
{
#if defined(SPI_HAS_TRANSACTION)
    // Perhaps this is a uniform interface for SPI?
    // Currently Teensy and ESP32 only
   uint32_t frequency;
   if (_frequency == Frequency16MHz)
       frequency = 16000000;
   else if (_frequency == Frequency8MHz)
       frequency = 8000000;
   else if (_frequency == Frequency4MHz)
       frequency = 4000000;
   else if (_frequency == Frequency2MHz)
       frequency = 2000000;
   else
       frequency = 1000000;

#if ((RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_SAM_DUE) || defined(ARDUINO_ARCH_SAMD))) || defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32L0) || defined(NRF52) || defined (ARDUINO_ARCH_RP2040)
    // Arduino Due in 1.5.5 has its own BitOrder :-(
    // So too does Arduino Zero
    // So too does rogerclarkmelbourne/Arduino_STM32
    // So too does GrumpyOldPizza/ArduinoCore-stm32l0
    // So too does RPI Pico
    ::BitOrder bitOrder;
// This no longer relevant: new versions is uint8_t
//#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
//   ::BitOrder bitOrder;
#else
    uint8_t bitOrder;
#endif



   if (_bitOrder == BitOrderLSBFirst)
       bitOrder = LSBFIRST;
   else
       bitOrder = MSBFIRST;
   
    uint8_t dataMode;
    if (_dataMode == DataMode0)
	dataMode = SPI_MODE0;
    else if (_dataMode == DataMode1)
	dataMode = SPI_MODE1;
    else if (_dataMode == DataMode2)
	dataMode = SPI_MODE2;
    else if (_dataMode == DataMode3)
	dataMode = SPI_MODE3;
    else
	dataMode = SPI_MODE0;

    // Save the settings for use in transactions
    //SPISettings dont mach any constructor error fix it

    _settings = SPISettings(frequency, LSBFIRST, dataMode);
   

    //import the SPI SETTINGS from SPI.h
    if(bitOrder == LSBFIRST)
    {
        _settings = SPISettings(frequency, LSBFIRST, dataMode);
    }
    else
    {
        _settings = SPISettings(frequency, MSBFIRST, dataMode);
    }

   SPI.begin();
    
    

#endif

}

void MyGenericSPI::end() 
{
    return SPI.end();
}

void MyGenericSPI::beginTransaction()
{
#if defined(SPI_HAS_TRANSACTION)
    SPI.beginTransaction(_settings);
#endif
}

void MyGenericSPI::endTransaction()
{
#if defined(SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}

void MyGenericSPI::usingInterrupt(uint8_t interrupt)
{
#if defined(SPI_HAS_TRANSACTION) && !defined(RH_MISSING_SPIUSINGINTERRUPT)
    SPI.usingInterrupt(interrupt);
#endif
    (void)interrupt;
}


