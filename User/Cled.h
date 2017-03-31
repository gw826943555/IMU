#ifndef __CLED__H
#define __CLED__H
#include "stm32f4xx.h"
#include "Singleton.h"

class Cled
{
public:
	Cled();
	void Init();
	void Toggle();
	void ON();
	void OFF();
};

typedef NormalSingleton<Cled>	LED;

#endif

