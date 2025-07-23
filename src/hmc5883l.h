#ifndef HMC5883L_H
#define HMC5883L_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cstdint>

class HMC5883L {
private:
    // QMC5883L I2C address (0x0D for QMC5883L clones)
    uint8_t device_address;
    
    // QMC5883L Register addresses (different from HMC5883L!)
    static constexpr uint8_t REG_DATA_X_LSB = 0x00;
    static constexpr uint8_t REG_DATA_X_MSB = 0x01;
    static constexpr uint8_t REG_DATA_Y_LSB = 0x02;
    static constexpr uint8_t REG_DATA_Y_MSB = 0x03;
    static constexpr uint8_t REG_DATA_Z_LSB = 0x04;
    static constexpr uint8_t REG_DATA_Z_MSB = 0x05;
    static constexpr uint8_t REG_STATUS = 0x06;
    static constexpr uint8_t REG_TEMP_LSB = 0x07;
    static constexpr uint8_t REG_TEMP_MSB = 0x08;
    static constexpr uint8_t REG_CONTROL_1 = 0x09;
    static constexpr uint8_t REG_CONTROL_2 = 0x0A;
    static constexpr uint8_t REG_SET_RESET_PERIOD = 0x0B;
    static constexpr uint8_t REG_CHIP_ID = 0x0D;
    
    // QMC5883L Configuration values
    static constexpr uint8_t MODE_STANDBY = 0x00;
    static constexpr uint8_t MODE_CONTINUOUS = 0x01;
    static constexpr uint8_t ODR_10HZ = 0x00;
    static constexpr uint8_t ODR_50HZ = 0x04;
    static constexpr uint8_t ODR_100HZ = 0x08;
    static constexpr uint8_t ODR_200HZ = 0x0C;
    static constexpr uint8_t RNG_2G = 0x00;
    static constexpr uint8_t RNG_8G = 0x10;
    static constexpr uint8_t OSR_512 = 0x00;
    static constexpr uint8_t OSR_256 = 0x40;
    static constexpr uint8_t OSR_128 = 0x80;
    static constexpr uint8_t OSR_64 = 0xC0;
    
    // QMC5883L Status bits
    static constexpr uint8_t STATUS_DRDY = 0x01;
    static constexpr uint8_t STATUS_OVL = 0x02;
    static constexpr uint8_t STATUS_DOR = 0x04;
    
    // Expected Chip ID for QMC5883L
    static constexpr uint8_t CHIP_ID_VALUE = 0xFF;

    i2c_inst_t* i2c_port;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t baudrate;
    float scale_factor;
    bool initialized;

    // Private helper methods
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t* value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
    int16_t combineBytes(uint8_t lsb, uint8_t msb); // Note: QMC5883L is little-endian
    void setScaleFactor(uint8_t range);

public:
    // Data structure for magnetometer readings
    struct MagnetometerData {
        int16_t x_raw;
        int16_t y_raw;
        int16_t z_raw;
        float x_gauss;
        float y_gauss;
        float z_gauss;
        float heading_degrees;
        int16_t temperature_raw;
        float temperature_celsius;
    };

    // Range settings enum (QMC5883L specific)
    enum class Gain : uint8_t {
        GAIN_2G = RNG_2G,  // ±2 Gauss (default)
        GAIN_8G = RNG_8G   // ±8 Gauss
    };

    // Data rate enum (QMC5883L specific)
    enum class DataRate : uint8_t {
        ODR_10HZ = HMC5883L::ODR_10HZ,   // 10 Hz
        ODR_50HZ = HMC5883L::ODR_50HZ,   // 50 Hz (default)
        ODR_100HZ = HMC5883L::ODR_100HZ, // 100 Hz
        ODR_200HZ = HMC5883L::ODR_200HZ  // 200 Hz
    };

    // Measurement mode enum
    enum class Mode : uint8_t {
        CONTINUOUS = MODE_CONTINUOUS,
        STANDBY = MODE_STANDBY
    };

    // Constructor
    HMC5883L(i2c_inst_t* i2c, uint8_t sda, uint8_t scl, uint32_t baud = 400000, uint8_t address = 0x0D);
    
    // Destructor
    ~HMC5883L();

    // Initialization and configuration
    bool begin();
    bool isConnected();
    bool reset();
    bool setGain(Gain gain);
    bool setMode(Mode mode);
    bool setDataRate(DataRate rate);
    bool calibrate(uint16_t samples = 100);

    // Data reading methods
    bool readRawData(int16_t& x, int16_t& y, int16_t& z);
    bool readData(MagnetometerData& data);
    bool isDataReady();
    bool readTemperature(int16_t& temp_raw, float& temp_celsius);
    
    // Utility methods
    float calculateHeading(float x, float y);
    float getScaleFactor() const { return scale_factor; }
    bool isInitialized() const { return initialized; }
    bool getChipId(uint8_t& chip_id);
    
    // Register access (for advanced users)
    bool readRegisterPublic(uint8_t reg, uint8_t* value);
    bool writeRegisterPublic(uint8_t reg, uint8_t value);
};

#endif // HMC5883L_H