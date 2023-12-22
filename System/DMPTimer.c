#include "stm32f10x.h"                  // Device header
#include "DMPTimer.h"

void DMPTimer_Init(){
    /*RCC TIM1*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

    /*ʱ��Դ*/
    TIM_InternalClockConfig(TIM1);

    /*ʱ����Ԫ*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//�˲�����Ƶ��
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
    TIM_TimeBaseInitStructure.TIM_Period=5000-1;//�Զ���װ���Ĵ�������ARR��0-65535
    TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//Ԥ��Ƶ���Ĵ�������PSC��0-65535
    //CK_CNT_OV=CK_PSC/(PSC+1)/(ARR+1)    CK_PSC=72MHz    T=1/CK_CNT_OV
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//���޸߼�������
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);//��ʼ���жϱ�־λ��λ
    TIM_ClearFlag(TIM1,TIM_FLAG_Update);//�ڿ����ж�ǰ���ֶ�����жϱ�־λ

    /*�ж��������*/
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);//ÿ�θ��¿��������ж�

    /*NVIC*/
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//������һ��������ֻ��һ��

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
    NVIC_Init(&NVIC_InitStructure);

    /*ʹ�ܶ�ʱ��*/
    TIM_Cmd(TIM1,ENABLE);
}
