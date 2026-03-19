#ifndef MPU6500_IMPL_H
#define MPU6500_IMPL_H

#include "MPU6500.h"

inline MPU6500::MPU6500(uint sda, uint scl, i2c_inst_t *i2c, uint ad0)
{
    MPU6500_ADDRESS += ad0;
    this->i2c = i2c;
    this->sda = sda;
    this->scl = scl;
}

inline MPU6500::MPU6500(uint sck, uint mosi, uint miso, uint cs, spi_inst_t *spi)
{
    this->spi = spi;
    this->miso = miso;
    this->mosi = mosi;
    this->sck = sck;
    this->cs = cs;
}

inline bool MPU6500::initialize()
{
    if(i2c != nullptr){
        i2c_init(i2c, 400000);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);

        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }else if(spi != nullptr){
        spi_init(spi, 20000000);
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(sck, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);

        gpio_init(cs);
        gpio_set_dir(cs, GPIO_OUT);
        gpio_put(cs, 1);
    }else{
        return true;
    }
    if(status()) return true;

    write_register(MPU6500_PWR_MGMT_1, 0x80);
    busy_wait_ms(100);

    write_register(MPU6500_PWR_MGMT_1, 0x01);
    write_register(MPU6500_PWR_MGMT_2, 0x00);

    write_register(MPU6500_INT_ENABLE, 0x00);
    write_register(MPU6500_INT_PIN_CFG, 0x02);

    write_register(MPU6500_FIFO_EN, 0x00);

    write_register(MPU6500_I2C_MST_CTRL, 0x00);
    write_register(MPU6500_USER_CTRL, 0x00);
    write_register(MPU6500_USER_CTRL, 0x0C);

    configure_accelerometer(ACCEL_SCALE_2G, ACCEL_BANDWIDTH_184HZ_DLPF);
    configure_gyroscope(GYRO_SCALE_250DPS, GYRO_BANDWIDTH_184HZ_DLPF);
    write_register(MPU6500_SMPLRT_DIV, 0);

    if(status()) return true;
    else return false;
}

inline void MPU6500::temperature(float &temperature)
{
    int16_t temp;
    temperature_raw(temp);

    temperature = float(temp) / 333.87f + 21.0f;
}

inline void MPU6500::acceleration(float acceleration[3])
{
    int16_t accel_raw[3];
    acceleration_raw(accel_raw);

    for (uint8_t item = 0; item < 3; item++) acceleration[item] = float(accel_raw[item]) * acc_scale;
}

inline void MPU6500::gyroscope(float gyroscope[3])
{
    int16_t gyro_raw[3];
    gyroscope_raw(gyro_raw);

    for (uint8_t item = 0; item < 3; item++) gyroscope[item] = float(gyro_raw[item]) * gyro_scale;
}

inline void MPU6500::motion(float acceleration[3], float gyroscope[3])
{
    
    uint8_t buffer[14];

    read_registers(MPU6500_DATA_REGS, buffer, 14);

    acceleration[0] = float(int16_t(buffer[0] << 8) | buffer[1]) * acc_scale;
    acceleration[1] = float(int16_t(buffer[2] << 8) | buffer[3]) * acc_scale;
    acceleration[2] = float(int16_t(buffer[4] << 8) | buffer[5]) * acc_scale;

    gyroscope[0] = float(int16_t(buffer[8]  << 8) | buffer[9] ) * gyro_scale;
    gyroscope[1] = float(int16_t(buffer[10] << 8) | buffer[11]) * gyro_scale;
    gyroscope[2] = float(int16_t(buffer[12] << 8) | buffer[13]) * gyro_scale;
}

inline void MPU6500::configure_accelerometer(ACCEL_SCALE scale, ACCEL_BANDWIDTH bandwidth)
{
    write_register(MPU6500_ACCEL_CONFIG, scale << 3);
    switch (scale)
    {
    case ACCEL_SCALE_2G:
        acc_scale = 2.f / 32767.5f;
        break;
    case ACCEL_SCALE_4G:
        acc_scale = 4.f / 32767.5f;
        break;
    case ACCEL_SCALE_8G:
        acc_scale = 8.f / 32767.5f;
        break;
    case ACCEL_SCALE_16G:
        acc_scale = 16.f / 32767.5f;
        break;
    default:
        acc_scale = 1.f;
        break;
    }
    write_register(MPU6500_ACCEL_CONFIG_2, bandwidth);
}

inline void MPU6500::configure_gyroscope(GRYO_SCALE scale, GRYO_BANDWIDTH bandwidth)
{
    write_register(MPU6500_GYRO_CONFIG, (scale << 3) | (bandwidth & 0x03));
    switch (scale)
    {
    case GYRO_SCALE_250DPS:
        gyro_scale = 250.0f / 32767.5f;
        break;
    case GYRO_SCALE_500DPS:
        gyro_scale = 500.0f / 32767.5f;
        break;
    case GYRO_SCALE_1000DPS:
        gyro_scale = 1000.0f / 32767.5f;
        break;
    case GYRO_SCALE_2000DPS:
        gyro_scale = 2000.0f / 32767.5f;
        break;
    default:
        gyro_scale = 1.f;
        break;
    }
    write_register(MPU6500_CONFIG, (bandwidth & 0x1C) >> 2);
}

inline void MPU6500::calibrate()
{
    int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };
    uint8_t buffer[12] = { 0 };

    configure_gyroscope(GYRO_SCALE_250DPS, GYRO_BANDWIDTH_184HZ_DLPF);
    configure_accelerometer(ACCEL_SCALE_2G, ACCEL_BANDWIDTH_184HZ_DLPF);

    write_register(MPU6500_USER_CTRL, 0x40);
    write_register(MPU6500_FIFO_EN, 0x78);

    busy_wait_ms(40);

    write_register(MPU6500_FIFO_EN, 0x00);

    spi_set_baudrate(spi, 1000000);
    read_registers(MPU6500_FIFO_COUNT_H, buffer, 2);
    
    uint16_t read_count = ((uint16_t(buffer[0]) << 8) | buffer[1]);
    
    read_count /= 12;
    for(int i = 0; i < read_count; ++i){
        read_registers(MPU6500_FIFO_R_W, buffer, 12);
        accel_bias[0] += int32_t((int16_t(int8_t(buffer[0]))  << 8) | buffer[1] );
		accel_bias[1] += int32_t((int16_t(int8_t(buffer[2]))  << 8) | buffer[3] );
		accel_bias[2] += int32_t((int16_t(int8_t(buffer[4]))  << 8) | buffer[5] );

		gyro_bias[0]  += int32_t((int16_t(int8_t(buffer[6]))  << 8) | buffer[7] );
		gyro_bias[1]  += int32_t((int16_t(int8_t(buffer[8]))  << 8) | buffer[9] );
		gyro_bias[2]  += int32_t((int16_t(int8_t(buffer[10])) << 8) | buffer[11]);
    }

    write_register(MPU6500_USER_CTRL, 0x00);
    write_register(MPU6500_USER_CTRL, 0x04);

    accel_bias[0] /= int32_t(read_count);
	accel_bias[1] /= int32_t(read_count);
	accel_bias[2] /= int32_t(read_count);
    
	gyro_bias[0]  /= int32_t(read_count);
	gyro_bias[1]  /= int32_t(read_count);
	gyro_bias[2]  /= int32_t(read_count);

    if (accel_bias[2] > 0L) {
		accel_bias[2] -= 16384;
	}
	else {
		accel_bias[2] += 16384;
	}

    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};    
                    
    spi_set_baudrate(spi, 1000000);

    read_registers(MPU6500_XA_OFFSET_H, read_data, 2); 
    acc_bias_reg[0] = (int16_t(read_data[0]) << 8) | read_data[1];

    read_registers(MPU6500_YA_OFFSET_H, read_data, 2);
    acc_bias_reg[1] = (int16_t(read_data[0]) << 8) | read_data[1];

    read_registers(MPU6500_ZA_OFFSET_H, read_data, 2);
    acc_bias_reg[2] = (int16_t(read_data[0]) << 8) | read_data[1];

    for (int i = 0; i < 3; i++) {
        uint8_t save_bit = acc_bias_reg[i] % 2;
        acc_bias_reg[i] = (((acc_bias_reg[i] >> 1) - (int16_t(accel_bias[i] >> 4))) << 1) | save_bit;
    }
    
    uint8_t accel_offset_data[6] = {0};
    accel_offset_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    accel_offset_data[1] = (acc_bias_reg[0]) & 0xFF;
    accel_offset_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    accel_offset_data[3] = (acc_bias_reg[1]) & 0xFF;
    accel_offset_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    accel_offset_data[5] = (acc_bias_reg[2]) & 0xFF;

    write_register(MPU6500_XA_OFFSET_H, accel_offset_data[0]);
    write_register(MPU6500_XA_OFFSET_L, accel_offset_data[1]);
    write_register(MPU6500_YA_OFFSET_H, accel_offset_data[2]);
    write_register(MPU6500_YA_OFFSET_L, accel_offset_data[3]);
    write_register(MPU6500_ZA_OFFSET_H, accel_offset_data[4]);
    write_register(MPU6500_ZA_OFFSET_L, accel_offset_data[5]);


    uint8_t gyro_offset_data[6] = {0};
    gyro_offset_data[0] = ((-int16_t(gyro_bias[0] / 4)) >> 8) & 0xFF;
    gyro_offset_data[1] = ( -int16_t(gyro_bias[0] / 4)) & 0xFF;
    gyro_offset_data[2] = ((-int16_t(gyro_bias[1] / 4)) >> 8) & 0xFF;
    gyro_offset_data[3] = ( -int16_t(gyro_bias[1] / 4)) & 0xFF;
    gyro_offset_data[4] = ((-int16_t(gyro_bias[2] / 4)) >> 8) & 0xFF;
    gyro_offset_data[5] = ( -int16_t(gyro_bias[2] / 4)) & 0xFF;

    write_register(MPU6500_XG_OFFSET_H, gyro_offset_data[0]);
    write_register(MPU6500_XG_OFFSET_L, gyro_offset_data[1]);
    write_register(MPU6500_YG_OFFSET_H, gyro_offset_data[2]);
    write_register(MPU6500_YG_OFFSET_L, gyro_offset_data[3]);
    write_register(MPU6500_ZG_OFFSET_H, gyro_offset_data[4]);
    write_register(MPU6500_ZG_OFFSET_L, gyro_offset_data[5]);
}

inline void MPU6500::calibrate(int32_t accel_bias_save[3], int32_t gyro_bias_save[3])
{
    int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };
    uint8_t buffer[12] = { 0 };

    configure_gyroscope(GYRO_SCALE_250DPS, GYRO_BANDWIDTH_184HZ_DLPF);
    configure_accelerometer(ACCEL_SCALE_2G, ACCEL_BANDWIDTH_184HZ_DLPF);

    write_register(MPU6500_USER_CTRL, 0x40);
    write_register(MPU6500_FIFO_EN, 0x78);

    busy_wait_ms(40);

    write_register(MPU6500_FIFO_EN, 0x00);

    spi_set_baudrate(spi, 1000000);
    read_registers(MPU6500_FIFO_COUNT_H, buffer, 2);
    
    uint16_t read_count = ((uint16_t(buffer[0]) << 8) | buffer[1]);
    
    read_count /= 12;
    for(int i = 0; i < read_count; ++i){
        read_registers(MPU6500_FIFO_R_W, buffer, 12);
        accel_bias[0] += int32_t((int16_t(int8_t(buffer[0]))  << 8) | buffer[1] );
		accel_bias[1] += int32_t((int16_t(int8_t(buffer[2]))  << 8) | buffer[3] );
		accel_bias[2] += int32_t((int16_t(int8_t(buffer[4]))  << 8) | buffer[5] );

		gyro_bias[0]  += int32_t((int16_t(int8_t(buffer[6]))  << 8) | buffer[7] );
		gyro_bias[1]  += int32_t((int16_t(int8_t(buffer[8]))  << 8) | buffer[9] );
		gyro_bias[2]  += int32_t((int16_t(int8_t(buffer[10])) << 8) | buffer[11]);
    }

    write_register(MPU6500_USER_CTRL, 0x00);
    write_register(MPU6500_USER_CTRL, 0x04);

    accel_bias[0] /= int32_t(read_count);
	accel_bias[1] /= int32_t(read_count);
	accel_bias[2] /= int32_t(read_count);
    
	gyro_bias[0]  /= int32_t(read_count);
	gyro_bias[1]  /= int32_t(read_count);
	gyro_bias[2]  /= int32_t(read_count);

    if (accel_bias[2] > 0L) {
		accel_bias[2] -= 16384;
	}
	else {
		accel_bias[2] += 16384;
	}

    accel_bias_save[0] = accel_bias[0];
    accel_bias_save[1] = accel_bias[1];
    accel_bias_save[2] = accel_bias[2];

    gyro_bias_save[0] = gyro_bias[0];
    gyro_bias_save[1] = gyro_bias[1];
    gyro_bias_save[2] = gyro_bias[2];

    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};    
                    
    spi_set_baudrate(spi, 1000000);

    read_registers(MPU6500_XA_OFFSET_H, read_data, 2); 
    acc_bias_reg[0] = (int16_t(read_data[0]) << 8) | read_data[1];

    read_registers(MPU6500_YA_OFFSET_H, read_data, 2);
    acc_bias_reg[1] = (int16_t(read_data[0]) << 8) | read_data[1];

    read_registers(MPU6500_ZA_OFFSET_H, read_data, 2);
    acc_bias_reg[2] = (int16_t(read_data[0]) << 8) | read_data[1];

    for (int i = 0; i < 3; i++) {
        uint8_t save_bit = acc_bias_reg[i] % 2;
        acc_bias_reg[i] = (((acc_bias_reg[i] >> 1) - (int16_t(accel_bias[i] >> 4))) << 1) | save_bit;
    }
    
    uint8_t accel_offset_data[6] = {0};
    accel_offset_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    accel_offset_data[1] = (acc_bias_reg[0]) & 0xFF;
    accel_offset_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    accel_offset_data[3] = (acc_bias_reg[1]) & 0xFF;
    accel_offset_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    accel_offset_data[5] = (acc_bias_reg[2]) & 0xFF;

    write_register(MPU6500_XA_OFFSET_H, accel_offset_data[0]);
    write_register(MPU6500_XA_OFFSET_L, accel_offset_data[1]);
    write_register(MPU6500_YA_OFFSET_H, accel_offset_data[2]);
    write_register(MPU6500_YA_OFFSET_L, accel_offset_data[3]);
    write_register(MPU6500_ZA_OFFSET_H, accel_offset_data[4]);
    write_register(MPU6500_ZA_OFFSET_L, accel_offset_data[5]);


    uint8_t gyro_offset_data[6] = {0};
    gyro_offset_data[0] = ((-int16_t(gyro_bias[0] / 4)) >> 8) & 0xFF;
    gyro_offset_data[1] = ( -int16_t(gyro_bias[0] / 4)) & 0xFF;
    gyro_offset_data[2] = ((-int16_t(gyro_bias[1] / 4)) >> 8) & 0xFF;
    gyro_offset_data[3] = ( -int16_t(gyro_bias[1] / 4)) & 0xFF;
    gyro_offset_data[4] = ((-int16_t(gyro_bias[2] / 4)) >> 8) & 0xFF;
    gyro_offset_data[5] = ( -int16_t(gyro_bias[2] / 4)) & 0xFF;

    write_register(MPU6500_XG_OFFSET_H, gyro_offset_data[0]);
    write_register(MPU6500_XG_OFFSET_L, gyro_offset_data[1]);
    write_register(MPU6500_YG_OFFSET_H, gyro_offset_data[2]);
    write_register(MPU6500_YG_OFFSET_L, gyro_offset_data[3]);
    write_register(MPU6500_ZG_OFFSET_H, gyro_offset_data[4]);
    write_register(MPU6500_ZG_OFFSET_L, gyro_offset_data[5]);
}

inline void MPU6500::write_calibration_from_save(int32_t accel_bias[3], int32_t gyro_bias[3])
{
    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};    
                    
    spi_set_baudrate(spi, 1000000);

    read_registers(MPU6500_XA_OFFSET_H, read_data, 2); 
    acc_bias_reg[0] = (int16_t(read_data[0]) << 8) | read_data[1];

    read_registers(MPU6500_YA_OFFSET_H, read_data, 2);
    acc_bias_reg[1] = (int16_t(read_data[0]) << 8) | read_data[1];

    read_registers(MPU6500_ZA_OFFSET_H, read_data, 2);
    acc_bias_reg[2] = (int16_t(read_data[0]) << 8) | read_data[1];

    for (int i = 0; i < 3; i++) {
        uint8_t save_bit = acc_bias_reg[i] % 2;
        acc_bias_reg[i] = (((acc_bias_reg[i] >> 1) - (int16_t(accel_bias[i] >> 4))) << 1) | save_bit;
    }
    
    uint8_t accel_offset_data[6] = {0};
    accel_offset_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    accel_offset_data[1] = (acc_bias_reg[0]) & 0xFF;
    accel_offset_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    accel_offset_data[3] = (acc_bias_reg[1]) & 0xFF;
    accel_offset_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    accel_offset_data[5] = (acc_bias_reg[2]) & 0xFF;

    write_register(MPU6500_XA_OFFSET_H, accel_offset_data[0]);
    write_register(MPU6500_XA_OFFSET_L, accel_offset_data[1]);
    write_register(MPU6500_YA_OFFSET_H, accel_offset_data[2]);
    write_register(MPU6500_YA_OFFSET_L, accel_offset_data[3]);
    write_register(MPU6500_ZA_OFFSET_H, accel_offset_data[4]);
    write_register(MPU6500_ZA_OFFSET_L, accel_offset_data[5]);


    uint8_t gyro_offset_data[6] = {0};
    gyro_offset_data[0] = ((-int16_t(gyro_bias[0] / 4)) >> 8) & 0xFF;
    gyro_offset_data[1] = ( -int16_t(gyro_bias[0] / 4)) & 0xFF;
    gyro_offset_data[2] = ((-int16_t(gyro_bias[1] / 4)) >> 8) & 0xFF;
    gyro_offset_data[3] = ( -int16_t(gyro_bias[1] / 4)) & 0xFF;
    gyro_offset_data[4] = ((-int16_t(gyro_bias[2] / 4)) >> 8) & 0xFF;
    gyro_offset_data[5] = ( -int16_t(gyro_bias[2] / 4)) & 0xFF;

    write_register(MPU6500_XG_OFFSET_H, gyro_offset_data[0]);
    write_register(MPU6500_XG_OFFSET_L, gyro_offset_data[1]);
    write_register(MPU6500_YG_OFFSET_H, gyro_offset_data[2]);
    write_register(MPU6500_YG_OFFSET_L, gyro_offset_data[3]);
    write_register(MPU6500_ZG_OFFSET_H, gyro_offset_data[4]);
    write_register(MPU6500_ZG_OFFSET_L, gyro_offset_data[5]);
}

inline void MPU6500::temperature_raw(int16_t &temperature)
{
    uint8_t buffer[2];

    read_registers(MPU6500_TEMP_REGS, buffer, 2);

    temperature = int16_t(buffer[0] << 8) | buffer[1];
}

inline void MPU6500::acceleration_raw(int16_t acceleration[3])
{
    uint8_t buffer[6];

    read_registers(MPU6500_ACCEL_REGS, buffer, 6);

    acceleration[0] = int16_t(buffer[0] << 8) | buffer[1];
    acceleration[1] = int16_t(buffer[2] << 8) | buffer[3];
    acceleration[2] = int16_t(buffer[4] << 8) | buffer[5];
}

inline void MPU6500::gyroscope_raw(int16_t gyroscope[3])
{
    uint8_t buffer[6];

    read_registers(MPU6500_GYRO_REGS, buffer, 6);

    gyroscope[0] = int16_t(buffer[0] << 8) | buffer[1];
    gyroscope[1] = int16_t(buffer[2] << 8) | buffer[3];
    gyroscope[2] = int16_t(buffer[4] << 8) | buffer[5];
}

inline bool MPU6500::status()
{
    uint8_t buf;
    read_registers(MPU6500_WHO_AM_I, &buf, 1);
    return buf == 0x70 ? false : true;
}

inline bool MPU6500::self_test()
{
    return true;
}

inline void MPU6500::write_register(uint8_t addr, uint8_t data)
{
    uint8_t buf[] = {addr, data};
    if(i2c != nullptr){
        i2c_write_blocking(i2c, MPU6500_ADDRESS, buf, 2, false);
    }else if(spi != nullptr){
        spi_set_baudrate(spi, 1000000);
        gpio_put(cs, 0);
        spi_write_blocking(spi, buf, 2);
        gpio_put(cs, 1);
        spi_set_baudrate(spi, 20000000);
        busy_wait_us(100);
    }
}

inline void MPU6500::read_registers(uint8_t addr, uint8_t data[], uint length)
{
    if(i2c != nullptr){
        i2c_write_blocking(i2c, MPU6500_ADDRESS, &addr, 1, true);
        i2c_read_blocking(i2c, MPU6500_ADDRESS, data, length, false);
    }else if(spi != nullptr){
        addr |= 0x80;
        gpio_put(cs, 0);
        spi_write_blocking(spi, &addr, 1);
        spi_read_blocking(spi, 0, data, length);
        gpio_put(cs, 1);
        busy_wait_us(1);
    }
}

#endif // MPU6500_IMPL_H