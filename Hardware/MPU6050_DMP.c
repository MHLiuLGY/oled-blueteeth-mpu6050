#include "stm32f10x.h"                  // Device header
#include "MPU6050I2C.h"
#include "MPU6050_Reg.h"
#include "Serial.h"
#include "Delay.h"
#include <math.h>

#include "MPU6050_DMP.h"
#include "inv_mpu.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu_dmp_motion_driver.h"

void get_ms(unsigned long *time){}

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf){ 
    uint8_t i;
    addr=addr<<1;
    MPU6050I2C_Start();
    MPU6050I2C_SendByte(addr);
    MPU6050I2C_ReceiveACK();//�˴������ж�һ�´ӻ��Ƿ�Ӧ��
    MPU6050I2C_SendByte(reg);
    MPU6050I2C_ReceiveACK();//�˴������ж�һ�´ӻ��Ƿ�Ӧ��

    for(i=0;i<len;i++){
        MPU6050I2C_SendByte(*buf++);
        MPU6050I2C_ReceiveACK();//�˴������ж�һ�´ӻ��Ƿ�Ӧ��
    }
    MPU6050I2C_Stop();

    return 0;
}

void mpu6050_write_reg(uint8_t reg, uint8_t dat){
    mpu6050_write(SlaveAddress,reg,1,&dat);
}

uint8_t mpu6050_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf){
    uint8_t i;
    addr=addr<<1;
    MPU6050I2C_Start();
    MPU6050I2C_SendByte(addr);
    MPU6050I2C_ReceiveACK();//�˴������ж�һ�´ӻ��Ƿ�Ӧ��
    MPU6050I2C_SendByte(reg);
    MPU6050I2C_ReceiveACK();//�˴������ж�һ�´ӻ��Ƿ�Ӧ��

    MPU6050I2C_Start();
    MPU6050I2C_SendByte(addr|0x01);
    MPU6050I2C_ReceiveACK();//�˴������ж�һ�´ӻ��Ƿ�Ӧ��
    for(i=0;i<len-1;i++){
        *buf++=MPU6050I2C_ReceiveByte();
        MPU6050I2C_SendACK(0);
    }
    *buf=MPU6050I2C_ReceiveByte();
    MPU6050I2C_SendACK(1);
    MPU6050I2C_Stop();

    return 0;
}

uint8_t mpu6050_read_reg (uint8_t reg){
    uint8_t dat;
    mpu6050_read(SlaveAddress,reg,1,&dat);
    return dat;
}

void MPU6050_Init(void)
{
    MPU6050I2C_Init(); 
 
    //Serial_Printf("MPU6050_ID=%x\r\n",mpu6050_read_reg(MPU6050_WHO_AM_I));

    mpu6050_write_reg(MPU6050_PWR_MGMT_1,0x00);
    mpu6050_write_reg(MPU6050_PWR_MGMT_2,0x00);

    mpu6050_write_reg(MPU6050_SMPLRT_DIV,0x07);
    mpu6050_write_reg(MPU6050_CONFIG,0x06);
    mpu6050_write_reg(MPU6050_GYRO_CONFIG,0x18);
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG,0x01);
}

void MPU6050_DMP_GetData(struct MPU6050_DMP_Data *Data){
    Data->AccX=mpu6050_read_reg(MPU6050_ACCEL_XOUT_H)<<8 | mpu6050_read_reg(MPU6050_ACCEL_XOUT_L);
    Data->AccY=mpu6050_read_reg(MPU6050_ACCEL_YOUT_H)<<8 | mpu6050_read_reg(MPU6050_ACCEL_YOUT_L);
    Data->AccZ=mpu6050_read_reg(MPU6050_ACCEL_ZOUT_H)<<8 | mpu6050_read_reg(MPU6050_ACCEL_ZOUT_L);
    Data->GyroX=mpu6050_read_reg(MPU6050_GYRO_XOUT_H)<<8 | mpu6050_read_reg(MPU6050_GYRO_XOUT_L);
    Data->GyroY=mpu6050_read_reg(MPU6050_GYRO_YOUT_H)<<8 | mpu6050_read_reg(MPU6050_GYRO_YOUT_L);
    Data->GyroZ=mpu6050_read_reg(MPU6050_GYRO_ZOUT_H)<<8 | mpu6050_read_reg(MPU6050_GYRO_ZOUT_L);
}

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

uint8_t MPU6050_DMP_Init(void)
{
    uint16_t flag=0;
    int32_t result=0;
    struct int_param_s int_param;
    Serial_Printf("MPU6050_IO_Init...\r\n");
    MPU6050I2C_Init();
    result=mpu_init(&int_param);
    if(!result){                                                                    //mpu��ʼ��
        Serial_Printf("������Ҫ������...\r\n");
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))                          //������Ҫ�Ĵ�����
            Serial_Printf("Ϊ��Ҫ�����ݿ���fifo...\r\n");
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))                       //����fifo
            Serial_Printf("���ò���Ƶ��=%3dHz...\r\n",DEFAULT_MPU_HZ);
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))                                    //���òɼ�����,��ø���fifo��������int�ж�
            Serial_Printf("����dmp�̼�...\r\n");
        if(!dmp_load_motion_driver_firmware())                                      //����dmp�̼�
            Serial_Printf("���������Ƿ���...\r\n");
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))//���������Ƿ���
            Serial_Printf("ʹ��dma����...\r\n");
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))                                                  //dmp_enable_feature
            Serial_Printf("���ò���Ƶ��=%3dHz...\r\n",DEFAULT_MPU_HZ);
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))                                      //���òɼ�����,��ø���fifo��������int�ж�
            Serial_Printf("�Լ�...\r\n");
        if(!run_self_test())                                                            //�Լ죬���Լ�飬�ɹ�����0
            Serial_Printf("�Լ�OK...\r\n");
        Serial_Printf("ʹ��...\r\n");
        while(mpu_set_dmp_state(1)!=0){                                             //ʹ��
            flag++;
            if(flag>50)
                break;
            Delay_ms(100);
        }
        if(flag>=50){
            Serial_Printf("ʹ��ʧ��,����while(1)...\r\n");
        while(1){}
            return 1;
        }
        else
            Serial_Printf("ʹ��OK...\r\n");
    }
    else
        return result;
    Serial_Printf("MPU6050_Init OK...\r\n\r\n");
    
    return 0;
}

float Pitch,Roll,Yaw;

void DEFAULT_MPU_HZ_GET(void)//��5MS  200HZ����һ�£��� DEFAULT_MPU_HZ Ƶ�ʱ���һ�£�����int�ж����Ŷ�ȡ���ⲿ�ж��޷�ʵ�֣���
{
    //uint8_t mpu6050_flag=0;
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more); 
    if(sensors & INV_WXYZ_QUAT){
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;

        Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;                                 // pitch
        Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
        Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;               //yaw
        //mpu6050_flag=1;
    }
}

float yaw_acc_error;//���Իع����MPU6050���ƫ�����ֵ
uint32_t ms_5;//5ms����

int16_t dpwm;      //PID���Ƶõ�������pwm������
float prviousYaw;  //��¼��һ��ƫ����
float arr;         //Ԥ��ֵ��ʵ��ֵ�Ĳ�
float prvious_arr; //��¼��һ��Ԥ��ֵ��ʵ��ֵ�Ĳ�
float Itotal;      //I���ۼ�ֵ
float P=0.5;       //����ϵ��
float I=0.3;       //����ϵ��
float D=3;         //΢��ϵ��

float target;      //����Ŀ��

void TIM1_UP_IRQHandler(){
    if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET){
        prviousYaw=Yaw;
        
        DEFAULT_MPU_HZ_GET();
        ms_5++;
        yaw_acc_error=ms_5*0.0000201645-0.48903838;//���Իع����MPU6050���ƫ��
        Yaw-=yaw_acc_error;
        
        /*pid���ջ�����ֱ����ʻ*/
        prvious_arr=prviousYaw-target;
        arr=Yaw-target;
        
        Itotal+=arr;
        Itotal=(Itotal>=20||Itotal<=-20) ? 0 : Itotal;
        
        dpwm=P*arr+I*Itotal+D*(arr-prvious_arr);
        
        TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
    }
}
