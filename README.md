@[toc](【Renesas RA6M4开发板之I2C读取mpu6050】)

# 1.0 mpu6050
![在这里插入图片描述](https://img-blog.csdnimg.cn/bbcbe57ca0fc4a8f8ab935b9ec6f02a5.png)

**此图转载钦源盛数码专营店**

*本篇通过Renesas RA6M4开发板采用I2C读取mpu6050传感器的角加速度，角速度和温度示例程序演示。*
## 1.1 mpu6050介绍
MPU6050是一种非常流行的空间运动传感器芯片，可以获取器件当前的三个加速度分量和三个旋转角速度。由于其体积小巧，功能强大，精度较高，不仅被广泛应用于工业，同时也是航模爱好者的神器，被安装在各类飞行器上驰骋蓝天。
## 1.2 mpu6050特点
使用芯片:MPU-6050
供电电源:3-5V(内部低压差稳压)
通信方式:标准lIC通信协议
芯片内置16BITAD转换器,16位数据输出
陀螺仪范围:±250 500 1000 2000 °/s
加速度范围:±2±4±8±16G
温度范围：-20℃~60℃
采用沉金PCB,机器焊接工艺保证质量
引脚间距2.54MM
需在气体环境中工作，不可测量液体和反接电源😅😅😅
![在这里插入图片描述](https://img-blog.csdnimg.cn/c89e20e581814f8dab99943044fd48b6.png)




尺寸大小如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/4d8cc58fe7f24eb18555806f6a5735ba.png)

## 1.3 mpu6050应用
运动感测游戏
现实增强
行人导航器
“零触控”手势用户接口
姿势快捷方式
认证
电子稳像（EIS: Electronic lmage Stabilization )
光学稳像（Ols: Optical lmage Stabilization )

![在这里插入图片描述](https://img-blog.csdnimg.cn/c24508f9e2764007841f8523f1aa7ace.png)



# 2. RT-theard配置
## 2.1 硬件需求
1、需要mpu6050采集气体环境下的气压和温度，I2C通讯接线**SDA---p504;SCL---p506**，不需要关注地址后面库自带配置了，与[ssd1306](https://blog.csdn.net/VOR234/article/details/125742886)不同

> 实现功能：
> 采用I2C读取mpu6050传感器的角加速度，角速度和温度示例


2、RA6M4开发板
![在这里插入图片描述](https://img-blog.csdnimg.cn/4c5dcda23c6d4afaacb393dc46a7ae51.png)
3、USB下载线，ch340串口和附带6根母母线，**rx---p613;tx---p614**        
![在这里插入图片描述](https://img-blog.csdnimg.cn/5145949611cf4d889a315946a478d75f.png)
## 2.2 软件配置
Renesas RA6M4开发板环境配置参照：[【基于 RT-Thread Studio的CPK-RA6M4 开发板环境搭建】](https://blog.csdn.net/vor234/article/details/125634313)
1、新建项目RA6M4-mpu6050工程
![在这里插入图片描述](https://img-blog.csdnimg.cn/72d6cccae46a41a7bf0d068421edb268.png)
2、点击RT-theard Setting，在软件包下添加软件包，然后搜索mpu相关软件支持包，点击添加即可,然后出现对应包。
![在这里插入图片描述](https://img-blog.csdnimg.cn/759e31fb34784b9e91cc4eb8aaa7fa96.png)


3、配置ssd306，右键选择配置项
![在这里插入图片描述](https://img-blog.csdnimg.cn/7209c6e06c19446d9caff6feb8a1bd2d.png)

4、在软件包中开启示例程序。
![在这里插入图片描述](https://img-blog.csdnimg.cn/634b62b3669a4c2397b4780026a423cc.png)


5、在硬件中，启动I2C，设置端口SDA---p505;SCL---p506
![在这里插入图片描述](https://img-blog.csdnimg.cn/89fb03f69484445e92a1a3d2618c38df.png)

6、全部保存刚刚的配置，更新当前配置文件

**保存完是灰色，没有保存是蓝色。**
# 3. 代码分析
1、刚刚加载软件包在packages文件夹下，
![在这里插入图片描述](https://img-blog.csdnimg.cn/bb46a4c7058b469a8bf001f1cb130298.png)

mpu6xxx.c代码更改为如下
（或者头文件添加`#include "bsp_api.h"`,否则会报错`unitx_t`，根据提示全部改为`rt_unitx_t`也OK，下面是第二种方法，增加了手动校准）😅😅😅
*mpu6xxx.c*

```cpp
/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-23     flybreak     the first version
 * 2021-09-09     scratch-er   added setting and getting sensor offsets
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "mpu6xxx"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "mpu6xxx.h"
#include "mpu6xxx_reg.h"

#ifdef PKG_USING_MPU6XXX_MAG
#include "ak8963_reg.h"
#endif

#define MPU6XXX_ACCEL_SEN     (16384)
#define MPU6XXX_GYRO_SEN      (1310)

#define MPU60X0_SPI_MAX_SPEED (1000 * 1000)
#define MPU60X0_TEMP_SEN      (340)
#define MPU60X0_TEMP_OFFSET   (36.5)

#define MPU6500_TEMP_SEN      (333.87)
#define MPU6500_TEMP_OFFSET   (21)

// MAG
#define AK8963_RANGE          (4912)
#define AK8963_FULLSCALE      (32760)

/**
 * This function writes the value of the register for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t mpu6xxx_write_reg(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t data)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = 2;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &reg, 1, &data, 1);
#endif
    }
    return res;
}

/**
 * This function reads the value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param len number of register
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t mpu6xxx_read_regs(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif
#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;              /* Read data pointer */
        msgs[1].len   = len;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf, len);
#endif
    }
    return res;
}

/**
 * This function writes a bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param bit the position of the register
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing a bit value of registers successfully.
 */
static rt_err_t mpu6xxx_write_bit(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t bit, rt_uint8_t data)
{
    rt_uint8_t byte;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    byte = (data != 0) ? (byte | (1 << bit)) : (byte & ~(1 << bit));

    return mpu6xxx_write_reg(dev, reg, byte);
}

/**
 * This function reads a bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param bit the position of the register
 * @param data read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading a bit value of registers successfully.
 */
static rt_err_t mpu6xxx_read_bit(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t bit, rt_uint8_t *data)
{
    rt_uint8_t byte;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    *data = byte & (1 << bit);

    return RT_EOK;
}

/**
 * This function writes multi-bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param start_bit the start position of the register
 * @param len number of bits to write
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing multi-bit value of registers successfully.
 */
static rt_err_t mpu6xxx_write_bits(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t start_bit, rt_uint8_t len, rt_uint8_t data)
{
    rt_uint8_t byte, mask;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    mask = ((1 << len) - 1) << (start_bit - len + 1);
    data <<= (start_bit - len + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    byte &= ~(mask); // zero all important bits in existing byte
    byte |= data; // combine data with existing byte

    return mpu6xxx_write_reg(dev, reg, byte);
}

/**
 * This function reads multi-bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param start_bit the start position of the register
 * @param len number of bits to write
 * @param data read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading multi-bit value of registers successfully.
 */
static rt_err_t mpu6xxx_read_bits(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t start_bit, rt_uint8_t len, rt_uint8_t *data)
{
    rt_uint8_t byte, mask;
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, reg, 1, &byte);
    if (res != RT_EOK)
    {
        return res;
    }

    mask = ((1 << len) - 1) << (start_bit - len + 1);
    byte &= mask;
    byte >>= (start_bit - len + 1);
    *data = byte;

    return RT_EOK;
}

// MAG
#ifdef PKG_USING_MPU6XXX_MAG

#define MAG_READ_DELAY_TIME         50

static void mpu92_mag_write_reg(struct mpu6xxx_device *dev, rt_uint8_t addr, rt_uint8_t data)
{
    rt_uint8_t  status = 0;
    rt_uint32_t timeout = MAG_READ_DELAY_TIME;

    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_REG, addr);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_DO, data);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);

    do
    {
        mpu6xxx_read_regs(dev, MPU6XXX_RA_I2C_MST_STATUS, 1, &status);
        rt_thread_mdelay(1);
    } while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

}

#endif // PKG_USING_MPU6XXX_MAG

/**
 * This function gets the raw data of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param accel the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_accel_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_ACCEL_XOUT_H, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    accel->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    accel->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    accel->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

    return RT_EOK;
}

/**
 * This function gets the raw data of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_gyro_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_GYRO_XOUT_H, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    gyro->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    gyro->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    gyro->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

    return RT_EOK;
}

#ifdef PKG_USING_MPU6XXX_MAG
/**
 * This function gets the raw data of the magnetometer
 *
 * @param dev the pointer of device driver structure
 * @param mag the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_mag_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *mag)
{
    rt_uint8_t buffer[8];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_EXT_SENS_DATA_00, 8, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    mag->x = ((rt_uint16_t)buffer[2] << 8) + buffer[1];
    mag->y = ((rt_uint16_t)buffer[4] << 8) + buffer[3];
    mag->z = ((rt_uint16_t)buffer[6] << 8) + buffer[5];

    return RT_EOK;
}
#endif

/**
 * This function gets the raw data of the temperature
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_temp_raw(struct mpu6xxx_device *dev, rt_int16_t *temp)
{
    rt_uint8_t buffer[2];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_TEMP_OUT_H, 2, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    *temp = ((rt_uint16_t)buffer[0] << 8) + buffer[1];

    return RT_EOK;
}

/**
 * This function gets mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t *param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    switch (cmd)
    {
    case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_SAMPLE_RATE: /* Sample Rate */
        /* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        if (res != RT_EOK)
        {
            break;
        }

        if (data == 0 || data == 7) /* dlpf is disable */
        {
            res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
            *param = 8000 / (data + 1);
        }
        else /* dlpf is enable */
        {
            res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
            *param = 1000 / (data + 1);
        }
        break;
    case MPU6XXX_SLEEP: /* sleep mode */
        res = mpu6xxx_read_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, &data);
        *param = data;
        break;
    }

    return res;
}

/**
 * This function set mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK reprensents  setting the parameter successfully.
 */
rt_err_t mpu6xxx_set_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    switch (cmd)
    {
    case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
        res = mpu6xxx_write_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, param);
        dev->config.gyro_range = param;
        break;
    case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
        res = mpu6xxx_write_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, param);
        dev->config.accel_range = param;
        break;
    case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
        res = mpu6xxx_write_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, param);
        break;
    case MPU6XXX_SAMPLE_RATE: /* Sample Rate = 16-bit unsigned value.
                                 Sample Rate = [1000 -  4]HZ when dlpf is enable
                                 Sample Rate = [8000 - 32]HZ when dlpf is disable */

        //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        if (res != RT_EOK)
        {
            break;
        }

        if (data == 0 || data == 7) /* dlpf is disable */
        {
            if (param > 8000)
                data = 0;
            else if (param < 32)
                data = 0xFF;
            else
                data = 8000 / param - 1;
        }
        else /* dlpf is enable */
        {
            if (param > 1000)
                data = 0;
            else if (param < 4)
                data = 0xFF;
            else
                data = 1000 / param - 1;
        }
        res = mpu6xxx_write_reg(dev, MPU6XXX_RA_SMPLRT_DIV, data);
        break;
    case MPU6XXX_SLEEP: /* Configure sleep mode */
        res = mpu6xxx_write_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, param);
        break;
    }

    return res;
}

/**
 * This function gets the data of the accelerometer, unit: mg(mm/s^2)
 *
 * @param dev the pointer of device driver structure
 * @param accel the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_accel(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel)
{
    struct mpu6xxx_3axes tmp;
    rt_uint16_t sen;
    rt_err_t res;

    res = mpu6xxx_get_accel_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    sen = MPU6XXX_ACCEL_SEN >> dev->config.accel_range;

    accel->x = (rt_int32_t)tmp.x * 1000 / sen;
    accel->y = (rt_int32_t)tmp.y * 1000 / sen;
    accel->z = (rt_int32_t)tmp.z * 1000 / sen;

    return RT_EOK;
}

/**
 * This function gets the data of the gyroscope, unit: deg/10s
 * Here deg/10s means 10 times higher precision than deg/s.
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_gyro(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro)
{
    struct mpu6xxx_3axes tmp;
    rt_uint16_t sen;
    rt_err_t res;

    res = mpu6xxx_get_gyro_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    sen = MPU6XXX_GYRO_SEN >> dev->config.gyro_range;

    gyro->x = (rt_int32_t)tmp.x * 100 / sen;
    gyro->y = (rt_int32_t)tmp.y * 100 / sen;
    gyro->z = (rt_int32_t)tmp.z * 100 / sen;

    return RT_EOK;
}

#ifdef PKG_USING_MPU6XXX_MAG

/**
 * This function gets the data of the magnetometer, unit: uT
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_mag(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *mag)
{
    struct mpu6xxx_3axes tmp;
    rt_err_t res;

    res = mpu6xxx_get_mag_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    mag->x = ((rt_int32_t)tmp.x * AK8963_RANGE) / AK8963_FULLSCALE;
    mag->y = ((rt_int32_t)tmp.y * AK8963_RANGE) / AK8963_FULLSCALE;
    mag->z = ((rt_int32_t)tmp.z * AK8963_RANGE) / AK8963_FULLSCALE;

    return RT_EOK;
}

#endif

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_temp(struct mpu6xxx_device *dev, float *temp)
{
    rt_int16_t tmp;
    rt_err_t res;

    res = mpu6xxx_get_temp_raw(dev, &tmp);
    if (res != RT_EOK)
    {
        return res;
    }

    if (dev->id == MPU6050_WHO_AM_I)
    {
        /* mpu60x0: Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53 */
        *temp = (double)tmp / MPU60X0_TEMP_SEN + MPU60X0_TEMP_OFFSET;
    }
    else
    {
        /* mpu6500:  ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity)+ 21degC */
        *temp = (double)tmp / MPU6500_TEMP_SEN + MPU6500_TEMP_OFFSET;
    }

    return RT_EOK;
}

/**
* This function sets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  setting the offsets successfully.
 */
rt_err_t mpu6xxx_set_accel_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_err_t res=0;
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XA_OFFS_H, (offset->x)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XA_OFFS_L_TC, (offset->x)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YA_OFFS_H, (offset->y)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YA_OFFS_L_TC, (offset->y)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZA_OFFS_H, (offset->z)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZA_OFFS_L_TC, (offset->z)&0x00ff);
    return res;
}

/**
* This function gets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  reading the offsets successfully.
 */
rt_err_t mpu6xxx_get_accel_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_XA_OFFS_H, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    offset->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    offset->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    offset->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

return RT_EOK;
}

/**
* This function sets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  setting the offsets successfully.
 */
rt_err_t mpu6xxx_set_gyro_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_err_t res=0;
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XG_OFFS_USRH, (offset->x)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_XG_OFFS_USRL, (offset->x)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YG_OFFS_USRH, (offset->y)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_YG_OFFS_USRL, (offset->y)&0x00ff);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZG_OFFS_USRH, (offset->z)>>8);
    res |= mpu6xxx_write_reg(dev, MPU6XXX_RA_ZG_OFFS_USRL, (offset->z)&0x00ff);
    return res;
}

/**
* This function gets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents  reading the offsets successfully.
 */
rt_err_t mpu6xxx_get_gyro_offset(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *offset)
{
    rt_uint8_t buffer[6];
    rt_err_t res;

    res = mpu6xxx_read_regs(dev, MPU6XXX_RA_XG_OFFS_USRH, 6, buffer);
    if (res != RT_EOK)
    {
        return res;
    }

    offset->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    offset->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    offset->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

return RT_EOK;
}

/**
 * This function initialize the mpu6xxx device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mpu6xxx_device *mpu6xxx_init(const char *dev_name, rt_uint8_t param)
{
    struct mpu6xxx_device *dev = RT_NULL;
    rt_uint8_t reg = 0xFF, res = RT_EOK;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mpu6xxx_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for mpu6xxx device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* find mpu6xxx device at address: 0x68 */
            dev->i2c_addr = MPU6XXX_ADDRESS_AD0_LOW;
            if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
            {
                /* find mpu6xxx device at address 0x69 */
                dev->i2c_addr = MPU6XXX_ADDRESS_AD0_HIGH;
                if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
                {
                    LOG_E("Can't find device at '%s'!", dev_name);
                    goto __exit;
                }
            }
            LOG_D("Device i2c address is:'0x%x'!", dev->i2c_addr);
        }
    }
    else if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
#ifdef RT_USING_SPI
        struct rt_spi_configuration cfg;

        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = MPU60X0_SPI_MAX_SPEED; /* Set spi max speed */

        rt_spi_configure((struct rt_spi_device *)dev->bus, &cfg);
#endif
    }
    else
    {
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
    {
        LOG_E("Failed to read device id!");
        goto __exit;
    }

    dev->id = reg;

    switch (dev->id)
    {
    case MPU6050_WHO_AM_I:
        LOG_I("Find device: mpu6050!");
        break;
    case MPU6500_WHO_AM_I:
        LOG_I("Find device: mpu6500!");
        break;
    case MPU9250_WHO_AM_I:
        LOG_I("Find device: mpu9250!");
        break;
    case ICM20608G_WHO_AM_I:
    case ICM20608D_WHO_AM_I:
        LOG_I("Find device: icm20608!");
        break;
    case 0xFF:
        LOG_E("No device connection!");
        goto __exit;
    default:
        LOG_W("Unknown device id: 0x%x!", reg);
    }

    res += mpu6xxx_get_param(dev, MPU6XXX_ACCEL_RANGE, &dev->config.accel_range);
    res += mpu6xxx_get_param(dev, MPU6XXX_GYRO_RANGE, &dev->config.gyro_range);

    res += mpu6xxx_write_bits(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_CLKSEL_BIT, MPU6XXX_PWR1_CLKSEL_LENGTH, MPU6XXX_CLOCK_PLL_XGYRO);
    res += mpu6xxx_set_param(dev, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_250DPS);
    res += mpu6xxx_set_param(dev, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);
    res += mpu6xxx_set_param(dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE);

#ifdef PKG_USING_MPU6XXX_MAG
    mpu6xxx_write_reg(dev, MPU6XXX_RA_USER_CTRL, 0x20);
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL2, 0x01);      /* [0]  Reset Device                  */
    rt_thread_mdelay(1);
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x00);      /* [1]  Power-down mode               */
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x0F);      /* [2]  Fuse ROM access mode          */
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x00);      /* [3]  Power-down mode               */
    rt_thread_mdelay(1);    // 100us
    mpu92_mag_write_reg(dev, AK8963_REG_CNTL1, 0x16);      /* [4]  16bits and Continuous measurement mode 2 */

    /* config mpu9250 i2c */
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_MST_CTRL, 0x5D);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV0_REG, AK8963_REG_ST1);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_SLV4_CTRL, 0x09);
    rt_thread_mdelay(2);
    mpu6xxx_write_reg(dev, MPU6XXX_RA_I2C_MST_DELAY_CTRL, 0x81);
#endif

    if (res == RT_EOK)
    {
        LOG_I("Device init succeed!");
    }
    else
    {
        LOG_W("Error in device initialization!");
    }
    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mpu6xxx_deinit(struct mpu6xxx_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mpu6xxx(int argc, char **argv)
{
    static struct mpu6xxx_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mpu6xxx [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mpu6xxx by given name，dev_name such as i2c1 \n");
        rt_kprintf("         sr <var>              Set sample rate to var\n");
        rt_kprintf("                               var = [1000 -  4] when dlpf is enable\n");
        rt_kprintf("                               var = [8000 - 32] when dlpf is disable\n");
        rt_kprintf("         gr <var>              Set gyro range to var\n");
        rt_kprintf("                               var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("         ar <var>              Set accel range to var\n");
        rt_kprintf("                               var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("         sleep <var>           Set sleep status\n");
        rt_kprintf("                               var = 0 means disable, = 1 means enable\n");
        rt_kprintf("         read [num]            read [num] times mpu6xxx\n");
        rt_kprintf("                               num default 5\n");
        return ;
    }
    else if (!strcmp(argv[1], "read"))
    {
        struct mpu6xxx_3axes accel, gyro, mag;
        float temp;
        rt_uint16_t num = 5;

        if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mpu6xxx first!\n");
            return ;
        }
        if (argc == 3)
        {
            num = atoi(argv[2]);
        }

        while (num --)
        {
            mpu6xxx_get_accel(dev, &accel);
            mpu6xxx_get_gyro(dev, &gyro);
            mpu6xxx_get_mag(dev, &mag);
            mpu6xxx_get_temp(dev, &temp);

            rt_kprintf("accel.x = %4d mg, accel.y = %4d mg, accel.z = %4d mg, ", accel.x+50, accel.y, accel.z-800);
            rt_kprintf("gyro.x = %4d deg/10s, gyro.y = %4d deg/10s, gyro.z = %4d deg/10s, ", gyro.x-70, gyro.y+22, gyro.z-9);
            rt_kprintf("mag.x = %4d uT, mag.y = %4d uT, mag.z = %4d uT", mag.x, mag.y, mag.z);
            rt_kprintf("temp = %d.%d ℃\n", (int)(temp * 100) / 100, (int)(temp * 100) % 100);

            rt_thread_mdelay(100);
        }
    }
    else if (argc == 3)
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mpu6xxx_deinit(dev);
            }
            dev = mpu6xxx_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mpu6xxx first!\n");
            return ;
        }
        else if (!strcmp(argv[1], "sr"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_SAMPLE_RATE, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "sleep"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_SLEEP, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "gr"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_GYRO_RANGE, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "ar"))
        {
            mpu6xxx_set_param(dev, MPU6XXX_ACCEL_RANGE, atoi(argv[2]));
        }
        else
        {
            rt_kprintf("Unknown command, please enter 'mpu6xxx' get help information!\n");
        }
    }
    else
    {
        rt_kprintf("Unknown command, please enter 'mpu6xxx' get help information!\n");
    }
}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mpu6xxx, mpu6xxx sensor function);
#endif

```

mpu6050.c

```cpp
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-11     Asus       the first version
 */

#include "sensor_inven_mpu6xxx.h"

int rt_hw_mpu6xxx_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)MPU6XXX_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_mpu6xxx_init("mpu", &cfg);
    return 0;
}
INIT_APP_EXPORT(rt_hw_mpu6xxx_port);


```

3、main.c文件在re_gen文件夹下，主程序围绕“hal_entry();”函数（在src文件夹），这些默认不变


# 4. 下载验证
1、编译重构
![在这里插入图片描述](https://img-blog.csdnimg.cn/7d739cda4f4644a3bd9c40545e17466f.png)
编译成功

2、下载程序
![在这里插入图片描述](https://img-blog.csdnimg.cn/cf303dd07ab248af91c34aa2a995f9cc.png)

下载成功


3、CMD串口调试

![在这里插入图片描述](https://img-blog.csdnimg.cn/181227ee2ed64ef2801477ece50cf41c.png)
然后板载复位，开始串口打印显示！🎉🎉🎉
![在这里插入图片描述](https://img-blog.csdnimg.cn/36582b538e9845c4921418ffa989e127.png)
开始测试，输入`mpu6xxx`
![在这里插入图片描述](https://img-blog.csdnimg.cn/06bf0f87a3b1424ebec8a458c4a7baf1.png)
接着校准mpu6050，输入`mpu6xxx  probe i2c1`
![在这里插入图片描述](https://img-blog.csdnimg.cn/7ab40c7dcfee41cf85dadbf210b176f1.png)

晃动传感器，读取角加速度，角速度和温度，输入`mpu6xxx read 20`
效果如下

![请添加图片描述](https://img-blog.csdnimg.cn/a2998d51c81d4ffd8a653e07b988530c.gif)
打印日志

```cpp

 \ | /
- RT -     Thread Operating System
 / | \     4.1.0 build Jul 13 2022 21:35:51
 2006 - 2022 Copyright by RT-Thread team
[I/mpu6xxx] Find device: mpu6050!
[I/mpu6xxx] Device init succeed!
[I/sensor] rt_sensor[acce_mpu] init success
[I/sensor] rt_sensor[gyro_mpu] init success
[I/sensor] rt_sensor[mag_mpu] init success
[I/sensor.inven.mpu6xxx] sensor init success

Hello RT-Thread!
msh >mpu6xxx

mpu6xxx [OPTION] [PARAM]
         probe <dev_name>      Probe mpu6xxx by given name such as i2c1
         sr <var>              Set sample rate to var
                               var = [1000 -  4] when dlpf is enable
                               var = [8000 - 32] when dlpf is disable
         gr <var>              Set gyro range to var
                               var = [0 - 3] means [250 - 2000DPS]
         ar <var>              Set accel range to var
                               var = [0 - 3] means [2 - 16G]
         sleep <var>           Set sleep status
                               var = 0 means disable, = 1 means enable
         read [num]            read [num] times mpu6xxx
                               num default 5
msh >mpu6xxx probe i2c1
[I/mpu6xxx] Find device: mpu6050!
[I/mpu6xxx] Device init succeed!
msh >mpu6xxx read 20
accel.x =  200 mg, accel.y =   -4 mg, accel.z =    3 mg, gyro.x =    0 deg/10s, gyro.y =   -1 deg/10s, gyro.z =   -2 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.65 ℃℃
accel.x =  203 mg, accel.y =   -4 mg, accel.z =   19 mg, gyro.x =   -1 deg/10s, gyro.y =   -1 deg/10s, gyro.z =    0 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.65 ℃℃
accel.x =  194 mg, accel.y =   -9 mg, accel.z =    6 mg, gyro.x =   -1 deg/10s, gyro.y =    1 deg/10s, gyro.z =   -2 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.60 ℃℃
accel.x =  210 mg, accel.y =   11 mg, accel.z =   15 mg, gyro.x =   -6 deg/10s, gyro.y =    0 deg/10s, gyro.z =   -1 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.60 ℃℃
accel.x =  389 mg, accel.y =  408 mg, accel.z =  499 mg, gyro.x =   25 deg/10s, gyro.y = -127 deg/10s, gyro.z =  353 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.65 ℃℃
accel.x =  187 mg, accel.y = -218 mg, accel.z =  -70 mg, gyro.x =   37 deg/10s, gyro.y =  -15 deg/10s, gyro.z =  478 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.74 ℃℃
accel.x = -149 mg, accel.y = -561 mg, accel.z =  246 mg, gyro.x = -2571 deg/10s, gyro.y =  479 deg/10s, gyro.z =  121 deg/10s, mag.x =    0 uT, mag.y =
    0 uT, mag.z =    0 uTtemp = 31.60 ℃℃
accel.x =  104 mg, accel.y = -109 mg, accel.z =  -42 mg, gyro.x = -1431 deg/10s, gyro.y = -1333 deg/10s, gyro.z = -1885 deg/10s, mag.x =    0 uT, mag.y
 =    0 uT, mag.z =    0 uTtemp = 31.70 ℃℃
accel.x =  677 mg, accel.y = -592 mg, accel.z =  330 mg, gyro.x = -313 deg/10s, gyro.y = -2479 deg/10s, gyro.z = -1443 deg/10s, mag.x =    0 uT, mag.y
=    0 uT, mag.z =    0 uTtemp = 31.55 ℃℃
accel.x =  749 mg, accel.y =  -57 mg, accel.z = -410 mg, gyro.x = -1377 deg/10s, gyro.y = -2479 deg/10s, gyro.z = -611 deg/10s, mag.x =    0 uT, mag.y
=    0 uT, mag.z =    0 uTtemp = 31.65 ℃℃
accel.x =  512 mg, accel.y = -146 mg, accel.z = -1845 mg, gyro.x = -799 deg/10s, gyro.y = -2479 deg/10s, gyro.z = -149 deg/10s, mag.x =    0 uT, mag.y
=    0 uT, mag.z =    0 uTtemp = 31.55 ℃℃
accel.x = -180 mg, accel.y =  420 mg, accel.z = -2800 mg, gyro.x =  519 deg/10s, gyro.y = 1497 deg/10s, gyro.z =  140 deg/10s, mag.x =    0 uT, mag.y =
    0 uT, mag.z =    0 uTtemp = 31.60 ℃℃
accel.x =  237 mg, accel.y =  243 mg, accel.z = -1148 mg, gyro.x = 1585 deg/10s, gyro.y = 2523 deg/10s, gyro.z = 1265 deg/10s, mag.x =    0 uT, mag.y =
    0 uT, mag.z =    0 uTtemp = 31.60 ℃℃
accel.x =   77 mg, accel.y = -667 mg, accel.z = -257 mg, gyro.x =  907 deg/10s, gyro.y = 2523 deg/10s, gyro.z = 1608 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.51 ℃℃
accel.x = -239 mg, accel.y = -726 mg, accel.z =  644 mg, gyro.x =  557 deg/10s, gyro.y = 2523 deg/10s, gyro.z =  651 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.51 ℃℃
accel.x = -230 mg, accel.y = -556 mg, accel.z =  181 mg, gyro.x =  215 deg/10s, gyro.y =  294 deg/10s, gyro.z =  -54 deg/10s, mag.x =    0 uT, mag.y =
   0 uT, mag.z =    0 uTtemp = 31.65 ℃℃
accel.x = -385 mg, accel.y = -600 mg, accel.z =  570 mg, gyro.x =   -1 deg/10s, gyro.y = -2045 deg/10s, gyro.z = -128 deg/10s, mag.x =    0 uT, mag.y =
    0 uT, mag.z =    0 uTtemp = 31.55 ℃℃
accel.x = -335 mg, accel.y = -419 mg, accel.z = -310 mg, gyro.x = -1128 deg/10s, gyro.y = -2479 deg/10s, gyro.z = -1016 deg/10s, mag.x =    0 uT, mag.y
 =    0 uT, mag.z =    0 uTtemp = 31.55 ℃℃
accel.x =  534 mg, accel.y =  100 mg, accel.z = -1428 mg, gyro.x = -1368 deg/10s, gyro.y = -2479 deg/10s, gyro.z = -1052 deg/10s, mag.x =    0 uT, mag.
y =    0 uT, mag.z =    0 uTtemp = 31.55 ℃℃
accel.x =  469 mg, accel.y =  265 mg, accel.z = -1841 mg, gyro.x = -1220 deg/10s, gyro.y = -2479 deg/10s, gyro.z = -656 deg/10s, mag.x =    0 uT, mag.y
 =    0 uT, mag.z =    0 uTtemp = 31.46 ℃℃
```

数据显示一开始没有怎么变化，后面就会变化加快（我手抖了😁），mpu6050不支持磁力所以全部为零。

这样我们就可以天马行空啦!
![请添加图片描述](https://img-blog.csdnimg.cn/38fa4dc8ddf24799990c57cb0ba0a414.gif)


参考文献；
[【基于 RT-Thread Studio的CPK-RA6M4 开发板环境搭建】](https://blog.csdn.net/vor234/article/details/125634313)
