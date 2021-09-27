 #include "mbed.h"
#include "PMW3901.h"

#define PMW3901_MISO D11
#define PMW3901_MOSI D12
#define PMW3901_SCK D13
#define PMW3901_CS D6
#define SPI_READ 0x80

DigitalOut PMW_cs(PMW3901_CS);
SPI PMW3901(PMW3901_MISO, PMW3901_MOSI, PMW3901_SCK);
optical_PMW3901::optical_PMW3901(uint8_t cspin)
    : _cs(cspin)
    {}

bool optical_PMW3901::begin(void)
{
    // SPI PMW3901(D11, D12, D13);
    PMW_cs = 1;
    wait_us(1000);
    PMW_cs = 0;
    wait_us(1000);
    PMW_cs = 1;
    wait_us(1000);
    registerWrite(0x3A, 0x5A);
    wait_us(5000);

    uint8_t chipId = registerRead(0x00);
    uint8_t dIpihc = registerRead(0x5F);

    if(chipId != 0x49 && dIpihc != 0xB8) return false;

    registerRead(0x02);
    registerRead(0x03);
    registerRead(0x04);
    registerRead(0x05);
    registerRead(0x06);

    // wait_us(1000);

    initRegisters();
    registerWrite(0x7f, 0x14);
    registerWrite(0x6f, 0x1c);
    registerWrite(0x7f, 0x00);

    return true;
}

void optical_PMW3901::readMotion(int16_t *deltaX, int16_t *deltaY)
{
    registerRead(0x02);
    // wait_us(50);
    // if(registerRead(0x02) & 0x80)
    // {
        *deltaX = ((int16_t)registerRead(0x04) << 8) | registerRead(0x03);
        *deltaY = ((int16_t)registerRead(0x06) << 8) | registerRead(0x05);
    // }
}

void optical_PMW3901::registerWrite(uint8_t reg, uint8_t value) 
{
    // DigitalOut PMW_cs(PMW3901_CS);
    // SPI PMW3901(D11, D12, D13);

    reg |= 0x80u;

    PMW3901.format(8,3);
    PMW3901.frequency(1000000);

    PMW_cs = 0;
    // wait_us(50);
    PMW3901.write(reg);
    PMW3901.write(value);
    // wait_us(50);
    PMW_cs = 1;

    // wait_us(200);
}

uint8_t optical_PMW3901::registerRead(uint8_t reg)
{
    reg &= ~0x80u;

    PMW_cs = 0;
    // wait_us(50);
    PMW3901.write(reg);
    // wait_us(50);
    uint8_t value = PMW3901.write(0);
    // wait_us(100);
    PMW_cs = 1;
    return value;    
}

void optical_PMW3901::initRegisters()
{
    registerWrite(0x7F, 0x00);
    registerWrite(0x61, 0xAD);
    registerWrite(0x7F, 0x03);
    registerWrite(0x40, 0x00);
    registerWrite(0x7F, 0x05);
    registerWrite(0x41, 0xB3);
    registerWrite(0x43, 0xF1);
    registerWrite(0x45, 0x14);
    registerWrite(0x5B, 0x32);
    registerWrite(0x5F, 0x34);
    registerWrite(0x7B, 0x08);
    registerWrite(0x7F, 0x06);
    registerWrite(0x44, 0x1B);
    registerWrite(0x40, 0xBF);
    registerWrite(0x4E, 0x3F);
    registerWrite(0x7F, 0x08);
    registerWrite(0x65, 0x20);
    registerWrite(0x6A, 0x18);
    registerWrite(0x7F, 0x09);
    registerWrite(0x4F, 0xAF);
    registerWrite(0x5F, 0x40);
    registerWrite(0x48, 0x80);
    registerWrite(0x49, 0x80);
    registerWrite(0x57, 0x77);
    registerWrite(0x60, 0x78);
    registerWrite(0x61, 0x78);
    registerWrite(0x62, 0x08);
    registerWrite(0x63, 0x50);
    registerWrite(0x7F, 0x0A);
    registerWrite(0x45, 0x60);
    registerWrite(0x7F, 0x00);
    registerWrite(0x4D, 0x11);
    registerWrite(0x55, 0x80);
    registerWrite(0x74, 0x1F);
    registerWrite(0x75, 0x1F);
    registerWrite(0x4A, 0x78);
    registerWrite(0x4B, 0x78);
    registerWrite(0x44, 0x08);
    registerWrite(0x45, 0x50);
    registerWrite(0x64, 0xFF);
    registerWrite(0x65, 0x1F);
    registerWrite(0x7F, 0x14);
    registerWrite(0x65, 0x60);
    registerWrite(0x66, 0x08);
    registerWrite(0x63, 0x78);
    registerWrite(0x7F, 0x15);
    registerWrite(0x48, 0x58);
    registerWrite(0x7F, 0x07);
    registerWrite(0x41, 0x0D);
    registerWrite(0x43, 0x14);
    registerWrite(0x4B, 0x0E);
    registerWrite(0x45, 0x0F);
    registerWrite(0x44, 0x42);
    registerWrite(0x4C, 0x80);
    registerWrite(0x7F, 0x10);
    registerWrite(0x5B, 0x02);
    registerWrite(0x7F, 0x07);
    registerWrite(0x40, 0x41);
    registerWrite(0x70, 0x00);

    // wait_us(100 * 1000);
    registerWrite(0x32, 0x44);
    registerWrite(0x7F, 0x07);
    registerWrite(0x40, 0x40);
    registerWrite(0x7F, 0x06);
    registerWrite(0x62, 0xf0);
    registerWrite(0x63, 0x00);
    registerWrite(0x7F, 0x0D);
    registerWrite(0x48, 0xC0);
    registerWrite(0x6F, 0xd5);
    registerWrite(0x7F, 0x00);
    registerWrite(0x5B, 0xa0);
    registerWrite(0x4E, 0xA8);
    registerWrite(0x5A, 0x50);
    registerWrite(0x40, 0x80);
}