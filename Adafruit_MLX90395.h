#ifndef ADAFRUIT_MLX90395_H
#define ADAFRUIT_MLX90395_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

/** Status Byte values */
#define MLX90395_STATUS_OK 0x00     /**< OK value for status response. */
#define MLX90395_STATUS_RESET 0x02
#define MLX90395_STATUS_SMMODE 0x20
#define MLX90395_STATUS_DRDY 0x01
#define MLX90395_STATUS_WOCMODE 0x40
#define MLX90395_STATUS_BURSTMODE 
#define MLX90395_STATUS_BURSTMODE_MEASUREMENT_COUNTER_MASK 0x70
#define BURSTMODE_MEASUREMENT_COUNT(status) (status>>4 & 0x07)  /* get measurement count from status byte for burst mode */
#define MLX90395_STATUS_COMMUNICATION_ERROR 0x08
#define MLX90395_STATUS_OVF_ERROR 0x04

#define MLX90395_LIB_ERROR 0xFF /* transfer function return value when I2C communication fails */

/** Register map. */
#define MLX90395_AXIS_ALL (0x0E)      /**< X+Y+Z axis bits for commands. Drops T */
#define MLX90395_REG_0 (0x00)         /**< Gain */
#define MLX90395_REG_1 (0x01)         /**< Burst, comm mode */
#define MLX90395_REG_2 (0x02)         /**< Oversampling, filter, res. */
//#define MLX90395_CONF4 (0x03)         /**< Sensitivty drift. */
#define MLX90395_GAIN_SHIFT (4)       /**< Left-shift for gain bits. */
#define MLX90395_HALL_CONF (0x0C)     /**< Hall plate spinning rate adj. */

enum MLX90395_COMMAND_REGISTERS {
  MLX90395_REG_SB = (0x10),  /**< Start burst mode. */
  MLX90395_REG_SWOC = (0x20),  /**< Start wakeup on change mode. */
  MLX90395_REG_SM = (0x30),  /**> Start single-meas mode. */
  MLX90395_REG_RM = (0x40),  /**> Read measurement. */
  MLX90395_REG_RR = (0x50),  /**< Read register. */
  MLX90395_REG_WR = (0x60),  /**< Write register. */
  MLX90395_REG_EX = (0x80),  /**> Exit mode. */
  MLX90395_REG_RV = (0xC0),  /**< Read voltage. */
  MLX90395_REG_HR = (0xD0),  /**< Memory recall. */
  MLX90395_REG_HS = (0xE0),  /**< Memory store. */
  MLX90395_REG_RT = (0xF0),  /**< Reset. */
};

typedef enum mlx90393_osr {
  MLX90395_OSR_1,
  MLX90395_OSR_2,
  MLX90395_OSR_4,
  MLX90395_OSR_8,
} mlx90393_osr_t;

typedef enum mlx90393_res {
  MLX90395_RES_16,
  MLX90395_RES_17,
  MLX90395_RES_18,
  MLX90395_RES_19,
} mlx90393_res_t;

static const float gainMultipliers[16] = {
    0.2, 0.25,  0.3333, 0.4, 0.5,  0.6, 0.75,  1,
    0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.375, 0.5};

#define MLX90395_DEFAULT_ADDR (0x0C) /* Can also be 0x18, depending on IC */

/** Class for interfacing with MLX90395 magnetometer */
class Adafruit_MLX90395 : public Adafruit_Sensor {
public:
  Adafruit_MLX90395();
  bool begin_I2C(uint8_t i2c_addr = MLX90395_DEFAULT_ADDR,
                 TwoWire *wire = &Wire);
  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI);

  void reset(void);
  bool exitMode(void);
  bool startBurstMode(void);
  bool startWOCMode(void);
  bool startSingleMeasurementMode(void);
  bool readMeasurement(float *x, float *y, float *z);
  bool readData(float *x, float *y, float *z);

  mlx90393_osr_t getOSR(void);
  bool setOSR(mlx90393_osr_t osrval);
  mlx90393_res_t getResolution(void);
  bool setResolution(mlx90393_res_t resval);
  uint8_t getGain(void);
  bool setGain(uint8_t gainval);

  void getSensor(sensor_t *sensor);
  bool getEvent(sensors_event_t *event);

  uint16_t uniqueID[3]; ///< 48 bits of unique identifier, read during init

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
  Adafruit_SPIDevice *spi_dev = NULL;

  bool _init(void);
  uint8_t command(uint8_t cmd);
  bool readRegister(uint8_t reg, uint16_t *data);
  bool writeRegister(uint8_t reg, uint16_t *data);

  mlx90393_res_t _resolution = MLX90395_RES_17;
  uint8_t _gain = 0;
  float _uTLSB = 0;
  int32_t _sensorID = 90395;
  int _cspin;
};

#endif
