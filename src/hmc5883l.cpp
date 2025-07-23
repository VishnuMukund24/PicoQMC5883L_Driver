#include "hmc5883l.h"
#include <cmath>
#include <stdio.h>

// Constructor
HMC5883L::HMC5883L(i2c_inst_t* i2c, uint8_t sda, uint8_t scl, uint32_t baud, uint8_t address)
    : i2c_port(i2c), sda_pin(sda), scl_pin(scl), baudrate(baud), device_address(address),
      scale_factor(1.0f/12000.0f), initialized(false) {
}

// Destructor
HMC5883L::~HMC5883L() {
    // I2C cleanup is handled by the Pico SDK
}

// Initialize the sensor
bool HMC5883L::begin() {
    // Initialize I2C
    i2c_init(i2c_port, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    sleep_ms(10); // Allow sensor to stabilize
    
    // Check if sensor is connected
    if (!isConnected()) {
        return false;
    }
    
    // Soft reset the QMC5883L
    if (!writeRegister(REG_CONTROL_2, 0x80)) {
        return false;
    }
    sleep_ms(10);
    
    // Configure QMC5883L with default settings:
    // Continuous mode, 50Hz, 2G range, 512 OSR
    uint8_t config = MODE_CONTINUOUS | ODR_50HZ | RNG_2G | OSR_512;
    if (!writeRegister(REG_CONTROL_1, config)) {
        return false;
    }
    
    // Set SET/RESET period register
    if (!writeRegister(REG_SET_RESET_PERIOD, 0x01)) {
        return false;
    }
    
    // Set scale factor for 2G range (default)
    setScaleFactor(RNG_2G);
    
    sleep_ms(10);
    initialized = true;
    return true;
}

// Check if sensor is connected by reading chip ID
bool HMC5883L::isConnected() {
    uint8_t chip_id;
    
    // Try to read the chip ID register
    if (getChipId(chip_id)) {
        if (chip_id == CHIP_ID_VALUE) {
            printf("QMC5883L detected (Chip ID: 0x%02X)\n", chip_id);
            return true;
        } else {
            printf("Unknown chip ID: 0x%02X (expected: 0x%02X)\n", chip_id, CHIP_ID_VALUE);
            // Some clones might have different chip IDs, try status register
        }
    }
    
    // Alternative check: try to read the status register
    uint8_t status;
    if (readRegister(REG_STATUS, &status)) {
        printf("Device found at 0x%02X (status: 0x%02X)\n", device_address, status);
        return true;
    }
    
    return false;
}

// Get chip ID
bool HMC5883L::getChipId(uint8_t& chip_id) {
    return readRegister(REG_CHIP_ID, &chip_id);
}

// Reset the sensor
bool HMC5883L::reset() {
    initialized = false;
    return begin();
}

// Set measurement range
bool HMC5883L::setGain(Gain gain) {
    uint8_t current_config;
    if (!readRegister(REG_CONTROL_1, &current_config)) {
        return false;
    }
    
    // Clear range bits and set new range
    current_config &= ~0x30; // Clear RNG bits
    current_config |= static_cast<uint8_t>(gain);
    
    if (!writeRegister(REG_CONTROL_1, current_config)) {
        return false;
    }
    
    setScaleFactor(static_cast<uint8_t>(gain));
    return true;
}

// Set data rate
bool HMC5883L::setDataRate(DataRate rate) {
    uint8_t current_config;
    if (!readRegister(REG_CONTROL_1, &current_config)) {
        return false;
    }
    
    // Clear ODR bits and set new rate
    current_config &= ~0x0C; // Clear ODR bits
    current_config |= static_cast<uint8_t>(rate);
    
    return writeRegister(REG_CONTROL_1, current_config);
}

// Set measurement mode
bool HMC5883L::setMode(Mode mode) {
    uint8_t current_config;
    if (!readRegister(REG_CONTROL_1, &current_config)) {
        return false;
    }
    
    // Clear mode bits and set new mode
    current_config &= ~0x03; // Clear mode bits
    current_config |= static_cast<uint8_t>(mode);
    
    return writeRegister(REG_CONTROL_1, current_config);
}

// Read raw magnetometer data
bool HMC5883L::readRawData(int16_t& x, int16_t& y, int16_t& z) {
    uint8_t buffer[6];
    
    if (!readRegisters(REG_DATA_X_LSB, buffer, 6)) {
        return false;
    }
    
    // QMC5883L data order: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB (little-endian)
    x = combineBytes(buffer[0], buffer[1]);
    y = combineBytes(buffer[2], buffer[3]);
    z = combineBytes(buffer[4], buffer[5]);
    
    return true;
}

// Read temperature data
bool HMC5883L::readTemperature(int16_t& temp_raw, float& temp_celsius) {
    uint8_t buffer[2];
    if (!readRegisters(REG_TEMP_LSB, buffer, 2)) {
        return false;
    }
    
    temp_raw = combineBytes(buffer[0], buffer[1]);
    temp_celsius = temp_raw / 100.0f; // QMC5883L temperature in 0.01°C units
    
    return true;
}

// Read magnetometer data with conversion to Gauss and heading calculation
bool HMC5883L::readData(MagnetometerData& data) {
    if (!readRawData(data.x_raw, data.y_raw, data.z_raw)) {
        return false;
    }
    
    // Convert to Gauss
    data.x_gauss = data.x_raw * scale_factor;
    data.y_gauss = data.y_raw * scale_factor;
    data.z_gauss = data.z_raw * scale_factor;
    
    // Calculate heading
    data.heading_degrees = calculateHeading(data.x_gauss, data.y_gauss);
    
    // Read temperature
    readTemperature(data.temperature_raw, data.temperature_celsius);
    
    return true;
}

// Check if new data is ready
bool HMC5883L::isDataReady() {
    uint8_t status;
    if (!readRegister(REG_STATUS, &status)) {
        return false;
    }
    return (status & STATUS_DRDY) != 0; // DRDY bit
}

// Calculate heading in degrees
float HMC5883L::calculateHeading(float x, float y) {
    float heading = atan2(y, x) * 180.0f / M_PI;
    
    // Normalize to 0-360 degrees
    if (heading < 0) {
        heading += 360.0f;
    }
    
    return heading;
}

// Calibrate the sensor (simple offset calibration)
bool HMC5883L::calibrate(uint16_t samples) {
    // This is a basic calibration - in practice, you might want
    // a more sophisticated calibration routine
    printf("Starting calibration with %d samples...\n", samples);
    printf("Rotate the sensor in all directions during calibration.\n");
    
    // Simple calibration routine - collect min/max values
    int16_t min_x = 32767, max_x = -32768;
    int16_t min_y = 32767, max_y = -32768;
    int16_t min_z = 32767, max_z = -32768;
    
    for (uint16_t i = 0; i < samples; i++) {
        int16_t x, y, z;
        if (readRawData(x, y, z)) {
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
            if (z < min_z) min_z = z;
            if (z > max_z) max_z = z;
        }
        sleep_ms(10);
    }
    
    printf("Calibration complete:\n");
    printf("X range: %d to %d (center: %d)\n", min_x, max_x, (min_x + max_x) / 2);
    printf("Y range: %d to %d (center: %d)\n", min_y, max_y, (min_y + max_y) / 2);
    printf("Z range: %d to %d (center: %d)\n", min_z, max_z, (min_z + max_z) / 2);
    
    return true;
}

// Private helper methods
bool HMC5883L::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    int result = i2c_write_blocking(i2c_port, device_address, buffer, 2, false);
    return result == 2;
}

bool HMC5883L::readRegister(uint8_t reg, uint8_t* value) {
    int result = i2c_write_blocking(i2c_port, device_address, &reg, 1, true);
    if (result != 1) return false;
    
    result = i2c_read_blocking(i2c_port, device_address, value, 1, false);
    return result == 1;
}

bool HMC5883L::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    int result = i2c_write_blocking(i2c_port, device_address, &reg, 1, true);
    if (result != 1) return false;
    
    result = i2c_read_blocking(i2c_port, device_address, buffer, length, false);
    return result == (int)length;
}

// QMC5883L uses little-endian format (LSB first)
int16_t HMC5883L::combineBytes(uint8_t lsb, uint8_t msb) {
    return (int16_t)((msb << 8) | lsb);
}

void HMC5883L::setScaleFactor(uint8_t range) {
    switch (range) {
        case RNG_2G:  scale_factor = 1.0f / 12000.0f; break; // ±2 Gauss
        case RNG_8G:  scale_factor = 1.0f / 3000.0f;  break; // ±8 Gauss
        default:      scale_factor = 1.0f / 12000.0f; break; // Default to 2G
    }
}

// Public register access methods
bool HMC5883L::readRegisterPublic(uint8_t reg, uint8_t* value) {
    return readRegister(reg, value);
}

bool HMC5883L::writeRegisterPublic(uint8_t reg, uint8_t value) {
    return writeRegister(reg, value);
}