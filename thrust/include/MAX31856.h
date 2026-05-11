#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SPIDevice.h>

#define MAX31856_CR0_REG    0x00
#define MAX31856_CR1_REG    0x01
#define MAX31856_MASK_REG   0x02
#define MAX31856_CJHF_REG   0x03
#define MAX31856_CJLF_REG   0x04
#define MAX31856_LTHFTH_REG 0x05
#define MAX31856_LTHFTL_REG 0x06
#define MAX31856_LTLFTH_REG 0x07
#define MAX31856_LTLFTL_REG 0x08
#define MAX31856_CJTO_REG   0x09
#define MAX31856_CJTH_REG   0x0A
#define MAX31856_CJTL_REG   0x0B
#define MAX31856_LTCBH_REG  0x0C
#define MAX31856_LTCBM_REG  0x0D
#define MAX31856_LTCBL_REG  0x0E
#define MAX31856_SR_REG     0x0F

#define MAX31856_CR0_AUTOCONVERT 0x80
#define MAX31856_CR0_1SHOT       0x40
#define MAX31856_CR0_OCFAULT1    0x20
#define MAX31856_CR0_OCFAULT0    0x10
#define MAX31856_CR0_CJ          0x08
#define MAX31856_CR0_FAULT       0x04
#define MAX31856_CR0_FAULTCLR    0x02
#define MAX31856_CR0_FILTER50HZ  0x01

#define MAX31856_TC_TEMP_LSB 0.0078125f

typedef enum {
    MAX31856_TCTYPE_B  = 0x00,
    MAX31856_TCTYPE_E  = 0x01,
    MAX31856_TCTYPE_J  = 0x02,
    MAX31856_TCTYPE_K  = 0x03,
    MAX31856_TCTYPE_N  = 0x04,
    MAX31856_TCTYPE_R  = 0x05,
    MAX31856_TCTYPE_S  = 0x06,
    MAX31856_TCTYPE_T  = 0x07,
    MAX31856_VMODE_G8  = 0x08,
    MAX31856_VMODE_G32 = 0x0C,
} max31856_thermocoupletype_t;

typedef enum {
    MAX31856_ONESHOT    = 0,
    MAX31856_CONTINUOUS = 1,
} max31856_conversion_mode_t;

typedef enum {
    MAX31856_NOISE_FILTER_60HZ = 0,
    MAX31856_NOISE_FILTER_50HZ = 1,
} max31856_noise_filter_t;

class MAX31856 {
public:
    MAX31856(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk);
    MAX31856(int8_t spi_cs, SPIClass *_spi = &SPI, uint32_t spi_freq = 1000000);

    bool begin();

    void setConversionMode(max31856_conversion_mode_t mode);
    max31856_conversion_mode_t getConversionMode();

    void setThermocoupleType(max31856_thermocoupletype_t type);
    max31856_thermocoupletype_t getThermocoupleType();

    uint8_t readFault();
    void    clearFault();

    void setColdJunctionFaultThresholds(int8_t low, int8_t high);
    void setTempFaultThresholds(float flow, float fhigh);

    void setColdJunctionFaultThreshholds(int8_t low, int8_t high);
    void setTempFaultThreshholds(float flow, float fhigh);

    void setColdJunctionOffset(int8_t offset);

    void setNoiseFilter(max31856_noise_filter_t noiseFilter);

    void triggerOneShot();
    bool conversionComplete();

    float readCJTemperature();
    float readThermocoupleTemperature();

private:
    uint8_t  readRegister8(uint8_t addr);
    uint16_t readRegister16(uint8_t addr);
    uint32_t readRegister24(uint8_t addr);
    void     readRegisterN(uint8_t addr, uint8_t buf[], uint8_t n);
    void     writeRegister8(uint8_t addr, uint8_t data);

    Adafruit_SPIDevice         spi_dev;
    bool                       initialized;
    max31856_conversion_mode_t conversionMode;
};
