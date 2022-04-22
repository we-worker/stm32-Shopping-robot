#include "delay.h"
static __IO uint32_t TimingDelay; //SysTick计数变量
__IO uint8_t b10msFlag; //每隔1ms由滴答中断置1，调用后请及时清零。

#if defined MPU6050    //如果使用MPU6050，则需要下列代码
__IO uint32_t TimeStamp_ms=0; //毫秒绝对值
uint8_t TimeStamp_ms_counter=0;
#endif

/*全局变量TimingDelay减一函数，直到0为止*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
	
#if defined MPU6050    //如果使用MPU6050，则需要下列代码
	TimeStamp_ms_counter++;
	if(TimeStamp_ms_counter > 99)
	{
		TimeStamp_ms++;
		TimeStamp_ms_counter = 0;
	}
#endif
	
}

/*延时nTime*10微秒*/
void Delay_10us(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  while(TimingDelay != 0);   //一直等到TimingDelay减到0为止
}
/*延时nTime毫秒*/
void Delay_ms(__IO uint32_t nTime)
{ 
	uint32_t i;
	i=nTime;
	while(i--)            //一直等到i减到0为止，i每Delay_10us(100)即1毫秒减一
		Delay_10us(100);
}
#if defined MPU6050    //如果使用MPU6050，则需要下列代码
int stm32_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = TimeStamp_ms;
    return 0;
}
#endif
