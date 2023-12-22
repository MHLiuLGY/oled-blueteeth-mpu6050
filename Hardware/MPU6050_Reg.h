#ifndef __MPU6050_REG_H__
#define __MPU6050_REG_H__

#define MPU6050_SMPLRT_DIV		0x19 //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define MPU6050_CONFIG			0x1A //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define MPU6050_GYRO_CONFIG		0x1B //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define MPU6050_ACCEL_CONFIG	0x1C //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define MPU6050_ACCEL_XOUT_H	0x3B
#define MPU6050_ACCEL_XOUT_L	0x3C
#define MPU6050_ACCEL_YOUT_H	0x3D
#define MPU6050_ACCEL_YOUT_L	0x3E
#define MPU6050_ACCEL_ZOUT_H	0x3F
#define MPU6050_ACCEL_ZOUT_L	0x40
#define MPU6050_TEMP_OUT_H		0x41
#define MPU6050_TEMP_OUT_L		0x42
#define MPU6050_GYRO_XOUT_H		0x43
#define MPU6050_GYRO_XOUT_L		0x44
#define MPU6050_GYRO_YOUT_H		0x45
#define MPU6050_GYRO_YOUT_L		0x46
#define MPU6050_GYRO_ZOUT_H		0x47
#define MPU6050_GYRO_ZOUT_L		0x48

#define MPU6050_PWR_MGMT_1		0x6B
#define MPU6050_PWR_MGMT_2		0x6C
#define MPU6050_WHO_AM_I		0x75

#define SlaveAddress			0x68 //IIC��ַ�Ĵ���0x68

#endif