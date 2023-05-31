#ifndef ISRMX_H
#define ISRMX_H

#include <RHGenericSPI.h>
#include <RHSPIDriver.h>

class ISRMX : public RHSPIDriver
{
public:
    ISRMX(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI &spi);
    // ... other public member functions ...

    // init
    bool init();

    bool change_modulation(uint8_t modulation);

    // reset
    bool reset();

    void default_cfg();

void fsk_config();

void sleep_config();


    uint8_t read(uint8_t reg);

bool available();

    // recv
    bool send(const uint8_t *data, uint8_t len);

    bool recv(uint8_t *buf, uint8_t *len);

    uint8_t maxMessageLength();


private:
    uint8_t modulation;
    uint8_t receive_mode;
    // ... other private member variables ...

protected:
};


#endif // ISRMX_H