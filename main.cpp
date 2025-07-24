#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hmc5883l.h"
#include <cmath>

int main() {
    // Initialize stdio for USB output
    stdio_init_all();
    
    // Wait for USB connection
    sleep_ms(3000);
    printf("QMC5883L Magnetometer Test (using HMC5883L interface)\n");
    printf("=====================================================\n");
    
    // Create HMC5883L instance with address 0x0D (QMC5883L address)
    // Using I2C0, SDA on GP4, SCL on GP5, 400kHz, address 0x0D
    HMC5883L compass(i2c0, 4, 5, 400000, 0x0D);
    printf("Attempting to initialize QMC5883L at address 0x0D...\n");
    
    // Initialize the sensor
    if (!compass.begin()) {
        printf("Failed to initialize QMC5883L!\n");
        printf("Check wiring and I2C connections.\n");
        
        // Try to scan for I2C devices
        printf("\nScanning I2C bus for devices...\n");
        for (int addr = 0x08; addr < 0x78; addr++) {
            uint8_t rxdata;
            if (i2c_read_blocking(i2c0, addr, &rxdata, 1, false) >= 0) {
                printf("Found I2C device at address 0x%02X\n", addr);
            }
        }
        
        // Try to read chip ID for debugging
        uint8_t chip_id;
        if (compass.getChipId(chip_id)) {
            printf("Chip ID: 0x%02X (expected: 0xFF for QMC5883L)\n", chip_id);
        }
        
        while (1) {
            sleep_ms(1000);
        }
    }
    
    printf("QMC5883L initialized successfully!\n");
    
    // Optional: Configure sensor settings
    printf("Configuring sensor settings...\n");
    compass.setGain(HMC5883L::Gain::GAIN_2G);        // ±2 Gauss range
    compass.setDataRate(HMC5883L::DataRate::ODR_50HZ); // 50Hz data rate
    compass.setMode(HMC5883L::Mode::CONTINUOUS);       // Continuous measurement
    
    printf("Sensor configuration:\n");
    printf("- Range: ±2 Gauss\n");
    printf("- Data Rate: 50 Hz\n");
    printf("- Mode: Continuous\n");
    printf("- Scale Factor: %.6f Gauss/LSB\n", compass.getScaleFactor());
    printf("Starting measurement loop...\n");
    printf("Press Ctrl+C to exit\n\n");
    
    // Optional: Run calibration
    // Uncomment the next lines to run a calibration sequence
    /*
    printf("Starting calibration in 3 seconds - get ready to rotate the sensor!\n");
    sleep_ms(3000);
    compass.calibrate(200); // Collect 200 samples for calibration
    printf("Calibration complete!\n\n");
    */
    
    int loop_count = 0;
    int successful_reads = 0;
    int failed_reads = 0;
    int no_data_count = 0;
    
    // Main loop with enhanced debugging
    while (1) {
        loop_count++;
        printf("Loop #%d: ", loop_count);
        
        // Check if new data is available
        if (compass.isDataReady()) {
            printf("Data ready - ");
            
            HMC5883L::MagnetometerData data;
            if (compass.readData(data)) {
                successful_reads++;
                printf("SUCCESS\n");
                printf("  Raw values: X=%d, Y=%d, Z=%d\n", 
                       data.x_raw, data.y_raw, data.z_raw);
                printf("  Gauss values: X=%.3f, Y=%.3f, Z=%.3f\n", 
                       data.x_gauss, data.y_gauss, data.z_gauss);
                printf("  Heading of : %.2f degrees\n", data.heading_degrees);
                printf("  Temperature: %.1f°C\n", data.temperature_celsius);
                
                // Calculate magnetic field strength
                float field_strength = sqrt(data.x_gauss * data.x_gauss + 
                                          data.y_gauss * data.y_gauss + 
                                          data.z_gauss * data.z_gauss);
                printf("  Field strength: %.3f Gauss\n", field_strength);
                
                // Convert heading to cardinal direction
                const char* direction;
                if (data.heading_degrees >= 337.5 || data.heading_degrees < 22.5) {
                    direction = "North";
                } else if (data.heading_degrees < 67.5) {
                    direction = "Northeast";
                } else if (data.heading_degrees < 112.5) {
                    direction = "East";
                } else if (data.heading_degrees < 157.5) {
                    direction = "Southeast";
                } else if (data.heading_degrees < 202.5) {
                    direction = "South";
                } else if (data.heading_degrees < 247.5) {
                    direction = "Southwest";
                } else if (data.heading_degrees < 292.5) {
                    direction = "West";
                } else {
                    direction = "Northwest";
                }
                printf("  Cardinal direction: %s\n", direction);
                
            } else {
                failed_reads++;
                printf("FAILED to read data\n");
            }
            
        } else {
            no_data_count++;
            printf("No data ready\n");
        }
        
        // Print statistics every 20 loops
        if (loop_count % 20 == 0) {
            printf("\n--- Statistics after %d loops ---\n", loop_count);
            printf("Successful reads: %d\n", successful_reads);
            printf("Failed reads: %d\n", failed_reads);
            printf("No data ready: %d\n", no_data_count);
            if (loop_count > 0) {
                printf("Success rate: %.1f%%\n", 
                       (successful_reads * 100.0f) / loop_count);
                printf("Data ready rate: %.1f%%\n",
                       ((successful_reads + failed_reads) * 100.0f) / loop_count);
            }
            printf("--------------------------------\n\n");
        }
        
        // Adjust delay based on data availability
        if (compass.isDataReady()) {
            sleep_ms(500); // Longer delay after successful read
        } else {
            sleep_ms(50);  // Shorter delay when waiting for data
        }
    }
    
    return 0;
}