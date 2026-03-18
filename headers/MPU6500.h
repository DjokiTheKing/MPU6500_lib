#ifndef MPU6500_H
#define MPU6500_H

#include "pico/stdlib.h"
#include "pico/stdio.h"

#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "MPU6500_regmap.h"

/// @brief Class for interfacing with the MPU6500 module. Must call initialize() before using the sensor.
class MPU6500{
    public:
        enum ACCEL_SCALE{
            ACCEL_SCALE_2G =0b00,
            ACCEL_SCALE_4G =0b01,
            ACCEL_SCALE_8G =0b10,
            ACCEL_SCALE_16G=0b11
        };

        enum ACCEL_BANDWIDTH{
            ACCEL_BANDWIDTH_1130HZ_NO_DLPF=0b1000,
            ACCEL_BANDWIDTH_460HZ_DLPF=0b0000,
            ACCEL_BANDWIDTH_184HZ_DLPF=0b0001,
            ACCEL_BANDWIDTH_92HZ_DLPF=0b0010,
            ACCEL_BANDWIDTH_41HZ_DLPF=0b0011,
            ACCEL_BANDWIDTH_20HZ_DLPF=0b0100,
            ACCEL_BANDWIDTH_10HZ_DLPF=0b0101,
            ACCEL_BANDWIDTH_5HZ_DLPF=0b0110,
            ACCEL_BANDWIDTH_460HZ_DLPF_1=0b0111,
        };

        enum GRYO_SCALE{
            GYRO_SCALE_250DPS =0b00,
            GYRO_SCALE_500DPS =0b01,
            GYRO_SCALE_1000DPS=0b10,
            GYRO_SCALE_2000DPS=0b11
        };

        enum GRYO_BANDWIDTH{
            GYRO_BANDWIDTH_8800HZ_NO_DLPF=0b00011,
            GYRO_BANDWIDTH_3600HZ_NO_DLPF=0b00010,
            GYRO_BANDWIDTH_250HZ_DLPF=0b00000,
            GYRO_BANDWIDTH_184HZ_DLPF=0b00100,
            GYRO_BANDWIDTH_92HZ_DLPF=0b01000,
            GYRO_BANDWIDTH_41HZ_DLPF=0b01100,
            GYRO_BANDWIDTH_20HZ_DLPF=0b10000,
            GYRO_BANDWIDTH_10HZ_DLPF=0b10100,
            GYRO_BANDWIDTH_5HZ_DLPF=0b11000,
            GYRO_BANDWIDTH_3600HZ_DLPF=0b11100,
        };

    public:
        /// @brief Create an instance using the i2c as the selected interface.
        /// @param sda SDA pin number.
        /// @param scl SCL pin number.
        /// @param i2c i2c instance(default is i2c0).
        /// @param ad0 value of ad0 pin on the module(default is 0);
        MPU6500(uint sda, uint scl, i2c_inst_t* i2c, uint ad0=0);

        /// @brief Create an instance using the spi as the selected interface.
        /// @param miso MISO pin number.
        /// @param mosi MOSI pin number.
        /// @param sck SCK pin number.
        /// @param cs CS pin number.
        /// @param spi spi instance(default is spi0).
        /// @param ad0 value of ad0 pin on the module(default is 0);
        MPU6500(uint sck, uint mosi, uint miso, uint cs, spi_inst_t* spi);

        /// @brief Initialize the sensor for use.
        bool initialize();

        /// @brief Gets the temperature data from the local buffer.
        /// @param temperature The temperature in Celsius.
        void temperature(float &temperature);

        /// @brief Gets the acceleration data from the local buffer.
        /// @param acceleration The x,y,z acceleration.
        void acceleration(float acceleration[3]);

        /// @brief Gets the gyroscope data from the local buffer.
        /// @param gyroscope The x,y,z gyroscope data.
        void gyroscope(float gyroscope[3]);

        /// @brief Configures the accelerometer with the given scale and bandwidth.
        void configure_accelerometer(ACCEL_SCALE scale, ACCEL_BANDWIDTH bandwidth);

        /// @brief Configures the gyroscope with the given scale and bandwidth.
        void configure_gyroscope(GRYO_SCALE scale, GRYO_BANDWIDTH bandwidth);

        /// @brief Calibrate the sensor, must be in a flat position and not moving.
        void calibrate();

        /// @brief Calibrate the sensor, must be in a flat position and not moving. Will save the calibration data to the passed save buffer.
        /// @param save Save buffer where the calibration data will be stored. 
        void calibrate_and_save(uint8_t save[12]);

        /// @brief Write the calibration data previously saved using calibrate_and_save() to the sensor.
        /// @param save Save buffer where the calibration data is stored.
        void write_calibration_from_save(uint8_t save[12]);

        void temperature_raw(int16_t &temperature);
        void acceleration_raw(int16_t acceleration[3]);
        void gyroscope_raw(int16_t gyroscope[3]);

        /// @brief Check if sensor is responding.
        /// @return If false sensor is okay.
        bool status();

        /// @brief Perform the self test of the sensors.
        /// @return If true sensor passed the selftest.
        bool self_test();
    
    private:
        i2c_inst_t* i2c = nullptr;
        int sda=-1, scl=-1; 

        spi_inst_t* spi = nullptr;
        int miso=-1, mosi=-1, sck=-1, cs=-1;

        float acc_scale = 1.f, gyro_scale = 1.f;
        uint8_t MPU6500_ADDRESS = 0x68;
    private:
        void write_register(uint8_t addr, uint8_t data);
        void read_registers(uint8_t addr, uint8_t data[], uint length);
};

#include "MPU6500.hpp"

#endif //MPU6500_H