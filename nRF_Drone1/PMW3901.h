#ifndef __PMW3901_H__
#define __PMW3901_H__

#include <stdint.h>

class optical_PMW3901
{
    public:
    optical_PMW3901(uint8_t cspin);

    bool begin(void);

    void readMotion(int16_t *deltaX, int16_t *deltaY);
    
    private:
    uint8_t _cs;
    uint8_t registerRead(uint8_t reg);
    void registerWrite(uint8_t reg, uint8_t value);
    void initRegisters(void);
};

#endif