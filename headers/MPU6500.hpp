#ifndef MPU6500_IMPL_H
#define MPU6500_IMPL_H

#include "MPU6500.h"

inline MPU6500::MPU6500(uint sda, uint scl, i2c_inst_t *i2c, uint ad0)
{
    MPU6500_ADDRESS += ad0;
}

inline MPU6500::MPU6500(uint miso, uint mosi, uint sck, uint cs, spi_inst_t *spi, uint ad0)
{
    MPU6500_ADDRESS += ad0;
}

inline void MPU6500::initialize()
{
}

inline void MPU6500::temperature(float &temperature)
{
}

inline void MPU6500::acceleration(float acceleration[3])
{
}

inline void MPU6500::gyroscope(float gyroscope[3])
{
}

inline void MPU6500::configure_accelerometer(ACCEL_SCALE scale, ACCEL_BANDWIDTH bandwidth)
{
}

inline void MPU6500::configure_gyroscope(GRYO_SCALE scale, GRYO_BANDWIDTH bandwidth)
{
}

inline void MPU6500::calibrate()
{
}

inline void MPU6500::calibrate_and_save(uint8_t save[12])
{
}

inline void MPU6500::write_calibration_from_save(uint8_t save[12])
{
}

inline void MPU6500::temperature_raw(int16_t &temperature)
{
}

inline void MPU6500::acceleration_raw(int16_t acceleration[3])
{
}

inline void MPU6500::gyroscope_raw(int16_t gyroscope[3])
{
}

inline bool MPU6500::status()
{
    return false;
}

inline bool MPU6500::self_test()
{
    return false;
}

inline void MPU6500::write_register(uint8_t addr, uint8_t data)
{
}

inline void MPU6500::read_registers(uint8_t addr, uint8_t data[], uint length)
{
}

#endif // MPU6500_IMPL_H