#ifndef _MPU6050_DMP_H_
#define _MPU6050_DMP_H_

struct MPU6050_DMP_Data{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
};//或者用typedef struct，这样在定义时不用struct

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);//返回值 0：读成功  -1：读失败
uint8_t mpu6050_read (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);//返回值 0：读成功  -1：读失败
void mpu6050_write_reg(uint8_t reg, uint8_t dat);
uint8_t mpu6050_read_reg (uint8_t reg);

void MPU6050_Init(void);
void MPU6050_DMP_GetData(struct MPU6050_DMP_Data *Data);

void get_ms(unsigned long *time);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
#define q30  1073741824.0f

extern float Pitch,Roll,Yaw;
extern int16_t dpwm;
extern float target;
extern float Itotal;
extern short gyro[3], accel[3];
extern uint8_t mpu6050_flag;

uint8_t MPU6050_DMP_Init(void);
void MPU6050_DMP_SetTarget(float target0);
void DEFAULT_MPU_HZ_GET(void);

#endif
