#include "MAX31856.h"
#include <math.h>

MAX31856::MAX31856(int8_t spi_cs, int8_t spi_mosi,
                                     int8_t spi_miso, int8_t spi_clk)
    : spi_dev(spi_cs, spi_clk, spi_miso, spi_mosi, 1000000,
              SPI_BITORDER_MSBFIRST, SPI_MODE1),
      initialized(false), conversionMode(MAX31856_ONESHOT) {}

MAX31856::MAX31856(int8_t spi_cs, SPIClass *_spi, uint32_t spi_freq)
    : spi_dev(spi_cs, spi_freq, SPI_BITORDER_MSBFIRST, SPI_MODE1, _spi),
      initialized(false), conversionMode(MAX31856_ONESHOT) {}

bool MAX31856::begin(void) {
  initialized = spi_dev.begin();

  if (!initialized)
    return false;

  writeRegister8(MAX31856_MASK_REG, 0x00);
  writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);
  writeRegister8(MAX31856_CJTO_REG, 0x00);

  setThermocoupleType(MAX31856_TCTYPE_K);
  setConversionMode(MAX31856_ONESHOT);

  return true;
}

void MAX31856::setConversionMode(max31856_conversion_mode_t mode) {
  conversionMode = mode;

  uint8_t t = readRegister8(MAX31856_CR0_REG);

  if (conversionMode == MAX31856_CONTINUOUS) {
    t |= MAX31856_CR0_AUTOCONVERT;
    t &= ~MAX31856_CR0_1SHOT;
  } else {
    t &= ~MAX31856_CR0_AUTOCONVERT;
    t |= MAX31856_CR0_1SHOT;
  }

  writeRegister8(MAX31856_CR0_REG, t);
}

max31856_conversion_mode_t MAX31856::getConversionMode(void) {
  return conversionMode;
}

void MAX31856::setThermocoupleType(max31856_thermocoupletype_t type) {
  uint8_t t = readRegister8(MAX31856_CR1_REG);

  t &= 0xF0;
  t |= (uint8_t)type & 0x0F;

  writeRegister8(MAX31856_CR1_REG, t);
}

max31856_thermocoupletype_t MAX31856::getThermocoupleType(void) {
  uint8_t t = readRegister8(MAX31856_CR1_REG);

  t &= 0x0F;

  return (max31856_thermocoupletype_t)t;
}

uint8_t MAX31856::readFault(void) {
  return readRegister8(MAX31856_SR_REG);
}

void MAX31856::clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CR0_REG);
  t |= MAX31856_CR0_FAULTCLR;
  writeRegister8(MAX31856_CR0_REG, t);
}

void MAX31856::setColdJunctionFaultThresholds(int8_t low, int8_t high) {
  writeRegister8(MAX31856_CJLF_REG, low);
  writeRegister8(MAX31856_CJHF_REG, high);
}

void MAX31856::setColdJunctionFaultThreshholds(int8_t low, int8_t high) {
  setColdJunctionFaultThresholds(low, high);
}

void MAX31856::setColdJunctionOffset(int8_t offset) {
  writeRegister8(MAX31856_CJTO_REG, offset);
}

void MAX31856::setNoiseFilter(max31856_noise_filter_t noiseFilter) {
  uint8_t t = readRegister8(MAX31856_CR0_REG);

  if (noiseFilter == MAX31856_NOISE_FILTER_50HZ) {
    t |= MAX31856_CR0_FILTER50HZ;
  } else {
    t &= ~MAX31856_CR0_FILTER50HZ;
  }

  writeRegister8(MAX31856_CR0_REG, t);
}

void MAX31856::setTempFaultThresholds(float flow, float fhigh) {
  int16_t low;
  int16_t high;

  flow *= 16;
  low = flow;

  fhigh *= 16;
  high = fhigh;

  writeRegister8(MAX31856_LTHFTH_REG, high >> 8);
  writeRegister8(MAX31856_LTHFTL_REG, high);

  writeRegister8(MAX31856_LTLFTH_REG, low >> 8);
  writeRegister8(MAX31856_LTLFTL_REG, low);
}

void MAX31856::setTempFaultThreshholds(float flow, float fhigh) {
  setTempFaultThresholds(flow, fhigh);
}

void MAX31856::triggerOneShot(void) {
  if (conversionMode == MAX31856_CONTINUOUS)
    return;

  clearFault();

  uint8_t t = readRegister8(MAX31856_CR0_REG);

  t &= ~MAX31856_CR0_AUTOCONVERT;
  t |= MAX31856_CR0_1SHOT;

  writeRegister8(MAX31856_CR0_REG, t);
}

bool MAX31856::conversionComplete(void) {
  if (conversionMode == MAX31856_CONTINUOUS)
    return true;

  return !(readRegister8(MAX31856_CR0_REG) & MAX31856_CR0_1SHOT);
}

float MAX31856::readCJTemperature(void) {
  int16_t temp = readRegister16(MAX31856_CJTH_REG);
  return temp / 256.0;
}

float MAX31856::readThermocoupleTemperature(void) {
  if (conversionMode == MAX31856_ONESHOT) {
    triggerOneShot();

    uint32_t start = millis();

    while (!conversionComplete()) {
      if (millis() - start > 250)
        return NAN;

      delay(10);
    }
  }

  uint8_t fault = readFault();
  if (fault) return NAN;

  int32_t temp24 = readRegister24(MAX31856_LTCBH_REG);

  if (temp24 & 0x800000) {
    temp24 |= 0xFF000000;
  }

  temp24 >>= 5;

  return temp24 * MAX31856_TC_TEMP_LSB;
}

uint8_t MAX31856::readRegister8(uint8_t addr) {
  uint8_t ret = 0;

  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t MAX31856::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};

  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];

  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

uint32_t MAX31856::readRegister24(uint8_t addr) {
  uint8_t buffer[3] = {0, 0, 0};

  readRegisterN(addr, buffer, 3);

  uint32_t ret = buffer[0];

  ret <<= 8;
  ret |= buffer[1];
  ret <<= 8;
  ret |= buffer[2];

  return ret;
}

void MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F;

  spi_dev.write_then_read(&addr, 1, buffer, n);
}

void MAX31856::writeRegister8(uint8_t addr, uint8_t data) {
  addr |= 0x80;

  uint8_t buffer[2] = {addr, data};

  spi_dev.write(buffer, 2);
}
