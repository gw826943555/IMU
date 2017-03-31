#include "Cled.h"

Cled::Cled()
{
	Init();
}

void Cled::Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
}

void Cled::Toggle(void)
{
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==SET)
	{
		GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
	}else{
		GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
	}
}

void Cled::ON(void)
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
}

void Cled::OFF(void)
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
}

