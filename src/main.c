#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

void RCC_Config(void)
{
	// 8Mhz -> 168Mhz
	RCC->CR &= ~(1 << 0);
	RCC->CR |= 1 << 16;
	while(!(RCC->CR & (1 << 17)));
	RCC->CR |= 1<<19;
	RCC->PLLCFGR = 0x00000000;
	RCC->PLLCFGR |= (1 << 22);
	RCC->PLLCFGR |= (4<<0);
	RCC->PLLCFGR |= (168<<6);
	RCC->PLLCFGR &= ~(1 << 16);
	RCC->PLLCFGR &= ~(1 << 17);
	RCC->CR |= (1 << 24);
	while(!(RCC->CR & (1 << 25)));
	RCC->CFGR &= ~(1 << 0);
	RCC->CFGR |= (1 << 1);
	while(!(RCC->CFGR & (1 << 1)));
}
void GPIO_Config(void)
{
	RCC->AHB1ENR = 0x9; 				//->GPIOA and GPIOD clock's enabled.

	//Settings of GPIOD
	GPIOD->MODER |= (0x55 << 24);   	//->D12, D13, D14, D15 = Output.
	GPIOD->OTYPER |= (0xff << 3);       //->D12, D13, D14, D15 = Open-drain.
	GPIOD->OSPEEDR |= (0xff << 24);   	//->D12, D13, D14, D15 = Very High speed.
}
void EXTI_Config()
{
	RCC->APB2ENR |= 1 << 14;           	//->SYSCFG(EXTI) = Enabled.
	SYSCFG->EXTICR[0] = 0x000;          //->EXTI = GPIOA.

	NVIC_EnableIRQ(EXTI0_IRQn);         //->Interrupt for EXTI0 = Enable.
	NVIC_EnableIRQ(EXTI1_IRQn);         //->Interrupt for EXTI1 = Enable.
	NVIC_EnableIRQ(EXTI2_IRQn);         //->Interrupt for EXTI2 = Enable.

	NVIC_SetPriority(EXTI0_IRQn, 0);    //->Priority of EXTIO = 0.
	NVIC_SetPriority(EXTI1_IRQn, 1);    //->Priority of EXTIO = 1.
	NVIC_SetPriority(EXTI2_IRQn, 2);    //->Priority of EXTIO = 2.

	//@brief The priority of EXTI is EXTI0.

	EXTI->IMR = 0x7;                    //->0111 = These became interrupts.
	EXTI->RTSR = 0x7;                   //->0111 = These became rising trigger.
}
void delay(uint32_t time)
{
	while(--time);
}
///--> EXTI0 Interrupt Function <--///
void EXTI0_IRQHandler()
{
	if(EXTI->PR & 1)    					//->Check the flag of EXTI0
	{
		GPIOD->ODR = (1 << 12);
		delay(33600000);
		EXTI->PR = 1; 						//->Clear flag
	}
}
///--> EXTI1 Interrupt Function <--///
void EXTI1_IRQHandler()
{
	if(EXTI->PR & (1 << 1))    				//->Check the flag of EXTI1
	{
		GPIOD->ODR = (1 << 13);
		delay(33600000);
		EXTI->PR = (1 << 1); 				//->Clear flag
	}
}
///--> EXTI2 Interrupt Function <--///
void EXTI2_IRQHandler()
{
	if(EXTI->PR & (1 << 2))    				//->Check the flag of EXTI2
	{
		GPIOD->ODR = (1 << 15);
		delay(33600000);
		EXTI->PR = (1 << 2); 				//->Clear flag
	}
}
int main(void)
{
	//Settings of Clock
	RCC_Config();
	SystemCoreClockUpdate();

	//Settings of GPIO
	GPIO_Config();

	//Settings of External Interrupt
	EXTI_Config();

	while (1)
	{
		GPIOD->ODR = (0xf << 12);           //->All bits became SET.
	}
}












void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
  /* TODO, implement your code here */
  return;
}
uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
  /* TODO, implement your code here */
  return -1;
}
