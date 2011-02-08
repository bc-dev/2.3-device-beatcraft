#ifndef __LINUX_I2C_AMI602_H
#define __LINUX_I2C_AMI602_H

/* linux/i2c/ami602.h */

struct ami602_platform_data {
	unsigned pin_rst;
	unsigned pin_bsy;
	unsigned pin_trg;
};

#define AMI602_SENSOR_DATA_MAX	4095
#define AMI602_DELTA_ACCEL	800

struct ami602_position {
	int accel_x;
	int accel_y;
	int accel_z;
	int mag_x;
	int mag_y;
	int mag_z;
};

#define AMI602_IOC_MAGIC	'B'
#define AMI602_IOCPOSITION	_IOR(AMI602_IOC_MAGIC, 0, struct ami602_position)

#endif
