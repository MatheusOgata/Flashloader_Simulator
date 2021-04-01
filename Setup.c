#include "Setup.h"


const uint32_t SystemCoreClock = 160000000 ;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

const uint8_t aTxBuffer[] = " Teste Flashloader \r\n";

/*int main(void){

	uint8_t ucCount = 9;

	HAL_Init();
	SystemClock_Config();
	SystemCoreClockUpdate();

	GPIO_Init();
	UART4_Init();
	

	while(1){

		GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		if(ucCount++ >= 9){
			UART_Transmit(aTxBuffer, 21);
			ucCount = 0;
		}
		HAL_Delay(500);
	}


}*/

void SystemInit(void){
	*(uint32_t *)SCB_CPACR |= ((3UL << 10*2)|(3UL << 11*2));
        
}

void HAL_Init(void){
	*(uint32_t *)FLASH_ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}

void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);           

  reg_value  =  *(uint32_t *)SCB_AIRCR;                                                  
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << 8U)                      );             
  *(uint32_t *)SCB_AIRCR =  reg_value;
}

void SystemClock_Config(void){
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR &= ~PWR_CR_VOS;
	PWR->CR |= PWR_REGULATOR_VOLTAGE_SCALE1;
	
	RCC->CR |= RCC_CR_HSEON;
	
	*(__IO uint32_t *) RCC_CR_PLLON_BB = 0x00UL;
	
	RCC->PLLCFGR = (PLLSource | PLLM | (PLLN << RCC_PLLCFGR_PLLN_Pos) | (((PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) | (PLLQ << RCC_PLLCFGR_PLLQ_Pos));
	
	*(__IO uint32_t *) RCC_CR_PLLON_BB = 0x01UL;

	*(__IO uint8_t *)ACR_BYTE0_ADDRESS = (uint8_t)5;
	
	RCC->CFGR &= ~RCC_CFGR_PPRE1; 
	RCC->CFGR |= RCC_HCLK_DIV16;
	
	RCC->CFGR &=  ~RCC_CFGR_PPRE2; 
	RCC->CFGR |= (RCC_HCLK_DIV16 << 3);
	
	RCC->CFGR &= ~RCC_CFGR_HPRE; 
	RCC->CFGR |= RCC_SYSCLK_DIV1;
	
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_SYSCLKSOURCE_PLLCLK;
	
	RCC->CFGR &= ~RCC_CFGR_PPRE1; 
	RCC->CFGR |= RCC_HCLK_DIV4;
	
	RCC->CFGR &= ~RCC_CFGR_PPRE2; 
	RCC->CFGR |= ((RCC_HCLK_DIV2) << 3U);
		
}

void GPIO_Init(void){
	uint32_t temp;
	
	GPIO_TypeDef * GPIOx = GPIOD;
	uint16_t position = 12;
	uint32_t Speed = GPIO_SPEED_FREQ_LOW;
	uint32_t Mode = GPIO_MODE_OUTPUT_PP;
	uint32_t Pull = GPIO_NOPULL;
	uint32_t Alternate;
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
    temp = GPIOx->OSPEEDR; 
    temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
    temp |= (Speed << (position * 2U));
    GPIOx->OSPEEDR = temp;

    temp = GPIOx->OTYPER;
    temp &= ~(GPIO_OTYPER_OT_0 << position) ;
    temp |= (((Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
    GPIOx->OTYPER = temp;
		
	temp = GPIOx->PUPDR;
    temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
    temp |= ((Pull) << (position * 2U));
    GPIOx->PUPDR = temp;
	  
	temp = GPIOx->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
    temp |= ((Mode & GPIO_MODE) << (position * 2U));
    GPIOx->MODER = temp;	
	
	
	GPIOx = GPIOA;
	position = 1;
	Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	Mode = GPIO_MODE_AF_PP;
	Pull = GPIO_PULLUP;
	Alternate = GPIO_AF8_UART4;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	    temp = GPIOx->OSPEEDR; 
    temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
    temp |= (Speed << (position * 2U));
    GPIOx->OSPEEDR = temp;

    temp = GPIOx->OTYPER;
    temp &= ~(GPIO_OTYPER_OT_0 << position) ;
    temp |= (((Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
    GPIOx->OTYPER = temp;
		
	temp = GPIOx->PUPDR;
    temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
    temp |= ((Pull) << (position * 2U));
    GPIOx->PUPDR = temp;
	  
	temp = GPIOx->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
    temp |= ((Mode & GPIO_MODE) << (position * 2U));
    GPIOx->MODER = temp;	
	
	temp = GPIOx->AFR[position >> 3U];
    temp &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;
    temp |= ((uint32_t)(Alternate) << (((uint32_t)position & 0x07U) * 4U));
    GPIOx->AFR[position >> 3U] = temp;
	
	
	GPIOx = GPIOC;
	position = 10;
	Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	Mode = GPIO_MODE_AF_PP;
	Pull = GPIO_PULLUP;	
	Alternate = GPIO_AF8_UART4;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	temp = GPIOx->OSPEEDR; 
    temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
    temp |= (Speed << (position * 2U));
    GPIOx->OSPEEDR = temp;

    temp = GPIOx->OTYPER;
    temp &= ~(GPIO_OTYPER_OT_0 << position) ;
    temp |= (((Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
    GPIOx->OTYPER = temp;
		
	temp = GPIOx->PUPDR;
    temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
    temp |= ((Pull) << (position * 2U));
    GPIOx->PUPDR = temp;
	  
	temp = GPIOx->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
    temp |= ((Mode & GPIO_MODE) << (position * 2U));
    GPIOx->MODER = temp;	
	
	temp = GPIOx->AFR[position >> 3U];
    temp &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;
    temp |= ((uint32_t)(Alternate) << (((uint32_t)position & 0x07U) * 4U));
    GPIOx->AFR[position >> 3U] = temp;
}

void UART4_Init(void){
 uint32_t pclk;
	
 uint32_t BaudRate = 9600;
 uint32_t WordLength = UART_WORDLENGTH_8B;
 uint32_t StopBits = UART_STOPBITS_1;
 uint32_t Parity = UART_PARITY_NONE;
 uint32_t Mode = UART_MODE_TX_RX;
 uint32_t HwFlowCtl = UART_HWCONTROL_NONE;
 uint32_t OverSampling = UART_OVERSAMPLING_16;
	
 RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
 
 USART4->CR1 &=  ~USART_CR1_UE;
 
 USART4->CR2 &= USART_CR2_STOP;
 USART4->CR2 |= StopBits;
 
 USART4->CR1 &= (uint32_t)~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8);
 USART4->CR1 |= (WordLength | Parity | Mode | OverSampling);
 
 USART4->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
 USART4->CR3 |= HwFlowCtl;
	
 pclk = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);

 USART4->BRR = 4167; //4167
 
 USART4->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
 USART4->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
 
 USART4->CR3 |= USART_CR3_HDSEL;
 
 USART4->CR1 |=  USART_CR1_UE;


}

void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
  if ((GPIOx->ODR & GPIO_Pin) == GPIO_Pin)
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << GPIO_NUMBER;
  }
  else
  {
    GPIOx->BSRR = GPIO_Pin;
  }
}

void UART_Transmit(uint8_t *pData, uint32_t size){
	
	for(uint32_t i = 0; i < size; i++){
		while(!(USART4->SR & UART_FLAG_TC));
		USART4->DR = (*pData++ & (uint8_t)0xFF);
	}
}

/*void SystemCoreClockUpdate(void) {
	
    uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
    uint32_t sysclockfreq = 0U;	

	
    pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    
    pllvco = (uint32_t) ((((uint64_t) HSE_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
    
    pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) *2U);

    sysclockfreq = (pllvco/pllp);
	
    SystemCoreClock = ( (sysclockfreq) >> (AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_Pos]));

}*/

uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}


void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(HAL_TICK_FREQ_DEFAULT);
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
    __NOP();
  }
}


/*HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  return HAL_OK;
}*/
