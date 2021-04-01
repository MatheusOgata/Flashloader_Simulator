
#ifndef __MAIN_H
#define __MAIN_H

#include "stdint.h"

#define __ASM                                  __asm
#define __NOP()                             __ASM volatile ("nop")
#define     __IO    volatile 
#define HSE_VALUE    ((uint32_t)8000000U)
#define PERIPH_BB_BASE        0x42000000UL

#define HAL_MAX_DELAY      0xFFFFFFFFU
#define HAL_TICK_FREQ_DEFAULT     1U

/******************************************** SCB ******************************************************************/
#define SCB_CPACR 0xE000ED88
#define SCB_AIRCR 0xE000ED0C

#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) 
#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)   
/*******************************************************************************************************************/

/*********************************************** NVIC **************************************************************/

#define NVIC_PRIORITYGROUP_0         0x00000007U /*!< 0 bits for pre-emption priority
                                                      4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         0x00000006U /*!< 1 bits for pre-emption priority
                                                      3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         0x00000005U /*!< 2 bits for pre-emption priority
                                                      2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         0x00000004U /*!< 3 bits for pre-emption priority
                                                      1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         0x00000003U /*!< 4 bits for pre-emption priority
                                                      0 bits for subpriority */

/*******************************************************************************************************************/

/*************************************************** FLASH *********************************************************/

#define FLASH_ACR 			0x40023C00
#define	FLASH_ACR_PRFTEN	(0x1UL << 8)
#define FLASH_ACR_ICEN  	(0x1UL << 9)
#define	FLASH_ACR_DCEN		(0x1UL << 10)
/*******************************************************************************************************************/

/******************************************************** RCC ******************************************************/
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)                               
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk  
#define RCC_PLLSOURCE_HSE                RCC_PLLCFGR_PLLSRC_HSE

#define RCC_PLLCFGR_PLLN_Pos               (6U)   
#define RCC_PLLCFGR_PLLP_Pos               (16U)        
#define RCC_PLLCFGR_PLLQ_Pos               (24U)  

#define ACR_BYTE0_ADDRESS           0x40023C00U 


#define PLLSource  RCC_PLLSOURCE_HSE
#define PLLM 4
#define PLLN 160
#define PLLP 0x00000002U
#define PLLQ 7

#define RCC_CFGR_HPRE_DIV1                 0x00000000U 
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1

#define RCC_CFGR_SW_PLL                    0x00000002U
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL


typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;

#define RCC ((RCC_TypeDef*)0x40023800)

#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)                               
#define RCC_APB2ENR_SYSCFGEN_Msk           (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */
#define RCC_APB2ENR_SYSCFGEN               RCC_APB2ENR_SYSCFGEN_Msk 

#define RCC_APB1ENR_PWREN_Pos              (28U)                               
#define RCC_APB1ENR_PWREN_Msk              (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk 

#define RCC_CR_HSEON_Pos                   (16U)                               
#define RCC_CR_HSEON_Msk                   (0x1UL << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSEON                       RCC_CR_HSEON_Msk 

#define RCC_CR_OFFSET  0x23800
#define RCC_PLLON_BIT_NUMBER       0x18U
#define RCC_CR_PLLON_BB            (PERIPH_BB_BASE + (RCC_CR_OFFSET * 32U) + (RCC_PLLON_BIT_NUMBER * 4U))

#define RCC_CFGR_PPRE1_Pos                 (10U)                               
#define RCC_CFGR_PPRE1_Msk                 (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk 

#define RCC_CFGR_PPRE2_Pos                 (13U)                               
#define RCC_CFGR_PPRE2_Msk                 (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk       

#define RCC_CFGR_HPRE_Pos                  (4U)                                
#define RCC_CFGR_HPRE_Msk                  (0xFUL << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                      RCC_CFGR_HPRE_Msk  

#define RCC_CFGR_SW_Pos                    (0U)                                
#define RCC_CFGR_SW_Msk                    (0x3UL << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */
#define RCC_CFGR_SW                        RCC_CFGR_SW_Msk 

#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U 
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U  

#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16

#define RCC_PLLCFGR_PLLM_Pos               (0U)                                
#define RCC_PLLCFGR_PLLM_Msk               (0x3FUL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM                   RCC_PLLCFGR_PLLM_Msk
                              
#define RCC_PLLCFGR_PLLN_Msk               (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN                   RCC_PLLCFGR_PLLN_Msk    
                          
#define RCC_PLLCFGR_PLLP_Msk               (0x3UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP                   RCC_PLLCFGR_PLLP_Msk      

/*******************************************************************************************************************/

/******************************************************* PWR *******************************************************/

#define PWR_CR_VOS_Pos         (14U)                                           
#define PWR_CR_VOS_Msk         (0x1UL << PWR_CR_VOS_Pos)                        /*!< 0x00004000 */
#define PWR_CR_VOS             PWR_CR_VOS_Msk 

#define PWR_REGULATOR_VOLTAGE_SCALE1         PWR_CR_VOS       

typedef struct
{
  __IO uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __IO uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

#define PWR                 ((PWR_TypeDef *) 0x40007000)
/*******************************************************************************************************************/

/******************************************************* GPIO *******************************************************/

#define  GPIO_SPEED_FREQ_LOW         0x00000000U
#define  GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U
#define  GPIO_NOPULL        0x00000000U
#define  GPIO_PULLUP        0x00000001U 
#define  GPIO_MODE_OUTPUT_PP                    0x00000001U  
#define  GPIO_MODE_AF_PP                        0x00000002U
#define GPIO_AF8_UART4         ((uint8_t)0x08)

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */

#define GPIO_NUMBER           16U
#define GPIO_OUTPUT_TYPE      0x00000010U
#define GPIO_MODE             0x00000003U

#define RCC_AHB1ENR_GPIODEN_Pos            (3U)                                
#define RCC_AHB1ENR_GPIODEN_Msk            (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIODEN                RCC_AHB1ENR_GPIODEN_Msk 

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

#define GPIOD               ((GPIO_TypeDef *) 0x40020C00)
#define GPIOA 				((GPIO_TypeDef *) 0x40020000)
#define GPIOC 				((GPIO_TypeDef *) 0x40020800)

#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)                                  
#define GPIO_OSPEEDR_OSPEED0_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0             GPIO_OSPEEDR_OSPEED0_Msk 
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDR_OSPEED0

#define GPIO_OTYPER_OT0_Pos              (0U)                                  
#define GPIO_OTYPER_OT0_Msk              (0x1UL << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */
#define GPIO_OTYPER_OT0      			 GPIO_OTYPER_OT0_Msk
#define GPIO_OTYPER_OT_0                 GPIO_OTYPER_OT0

#define GPIO_PUPDR_PUPD0_Pos             (0U)                                  
#define GPIO_PUPDR_PUPD0_Msk             (0x3UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0                 GPIO_PUPDR_PUPD0_Msk  
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPD0

#define GPIO_MODER_MODER0_Pos            (0U)                                  
#define GPIO_MODER_MODER0_Msk            (0x3UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk 

/*******************************************************************************************************************/

/******************************************************* UART *******************************************************/
#define USART_CR1_TE_Pos              (3U)                                     
#define USART_CR1_TE_Msk              (0x1UL << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_TE                  USART_CR1_TE_Msk   

#define USART_CR1_RE_Pos              (2U)                                     
#define USART_CR1_RE_Msk              (0x1UL << USART_CR1_RE_Pos)               /*!< 0x00000004 */
#define USART_CR1_RE 				  USART_CR1_RE_Msk

#define UART_WORDLENGTH_8B                  0x00000000U
#define UART_STOPBITS_1                     0x00000000U
#define UART_PARITY_NONE                    0x00000000U
#define UART_MODE_TX_RX                     ((uint32_t)(USART_CR1_TE | USART_CR1_RE))
#define UART_HWCONTROL_NONE                  0x00000000U
#define UART_OVERSAMPLING_16                    0x00000000U

#define RCC_APB1ENR_UART4EN_Pos            (19U)                               
#define RCC_APB1ENR_UART4EN_Msk            (0x1UL << RCC_APB1ENR_UART4EN_Pos)   /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN    				RCC_APB1ENR_UART4EN_Msk

#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)                                
#define RCC_AHB1ENR_GPIOAEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN                RCC_AHB1ENR_GPIOAEN_Msk     

#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)                                
#define RCC_AHB1ENR_GPIOCEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIOCEN                RCC_AHB1ENR_GPIOCEN_Msk

typedef struct
{
  __IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

#define USART4   ((USART_TypeDef *) 0x40004C00)

#define USART_CR1_UE_Pos              (13U)                                    
#define USART_CR1_UE_Msk              (0x1UL << USART_CR1_UE_Pos)               /*!< 0x00002000 */
#define USART_CR1_UE                  USART_CR1_UE_Msk 

#define USART_CR1_M_Pos               (12U)                                    
#define USART_CR1_M_Msk               (0x1UL << USART_CR1_M_Pos)                /*!< 0x00001000 */
#define USART_CR1_M    				  USART_CR1_M_Msk

#define USART_CR1_PS_Pos              (9U)                                     
#define USART_CR1_PS_Msk              (0x1UL << USART_CR1_PS_Pos)               /*!< 0x00000200 */
#define USART_CR1_PS                  USART_CR1_PS_Msk                         /*!<Parity Selection                       */

#define USART_CR1_PCE_Pos             (10U)                                    
#define USART_CR1_PCE_Msk             (0x1UL << USART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define USART_CR1_PCE                 USART_CR1_PCE_Msk  

#define USART_CR1_OVER8_Pos           (15U)                                    
#define USART_CR1_OVER8_Msk           (0x1UL << USART_CR1_OVER8_Pos)            /*!< 0x00008000 */
#define USART_CR1_OVER8               USART_CR1_OVER8_Msk 

#define USART_CR3_RTSE_Pos            (8U)                                     
#define USART_CR3_RTSE_Msk            (0x1UL << USART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define USART_CR3_RTSE                USART_CR3_RTSE_Msk                       /*!<RTS Enable                  */

#define USART_CR3_CTSE_Pos            (9U)                                     
#define USART_CR3_CTSE_Msk            (0x1UL << USART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define USART_CR3_CTSE                USART_CR3_CTSE_Msk   

#define USART_CR3_HDSEL_Pos           (3U)                                     
#define USART_CR3_HDSEL_Msk           (0x1UL << USART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define USART_CR3_HDSEL               USART_CR3_HDSEL_Msk   

#define USART_CR3_IREN_Pos            (1U)                                     
#define USART_CR3_IREN_Msk            (0x1UL << USART_CR3_IREN_Pos)             /*!< 0x00000002 */
#define USART_CR3_IREN                USART_CR3_IREN_Msk

#define USART_CR3_SCEN_Pos            (5U)                                     
#define USART_CR3_SCEN_Msk            (0x1UL << USART_CR3_SCEN_Pos)             /*!< 0x00000020 */
#define USART_CR3_SCEN                USART_CR3_SCEN_Msk 

#define USART_CR2_LINEN_Pos           (14U)                                    
#define USART_CR2_LINEN_Msk           (0x1UL << USART_CR2_LINEN_Pos)            /*!< 0x00004000 */
#define USART_CR2_LINEN               USART_CR2_LINEN_Msk  

#define USART_CR2_CLKEN_Pos           (11U)                                    
#define USART_CR2_CLKEN_Msk           (0x1UL << USART_CR2_CLKEN_Pos)            /*!< 0x00000800 */
#define USART_CR2_CLKEN               USART_CR2_CLKEN_Msk 

#define USART_CR2_STOP_Pos            (12U)
#define USART_CR2_STOP_Msk            (0x3UL << USART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define USART_CR2_STOP                USART_CR2_STOP_Msk

#define USART_SR_TC_Pos               (6U)                                     
#define USART_SR_TC_Msk               (0x1UL << USART_SR_TC_Pos)                /*!< 0x00000040 */
#define USART_SR_TC                   USART_SR_TC_Msk 
#define UART_FLAG_TC                        ((uint32_t)USART_SR_TC)


#define UART_DIV_SAMPLING16(_PCLK_, _BAUD_)            ((uint32_t)((((_PCLK_))*25U)/(4U*((_BAUD_)))))
#define UART_DIVMANT_SAMPLING16(_PCLK_, _BAUD_)        (UART_DIV_SAMPLING16((_PCLK_), (_BAUD_))/100U)
#define UART_DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_)        ((((UART_DIV_SAMPLING16((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100U)) * 16U) + 50U) / 100U)

#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_)            ((UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4U) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0xF0U) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0FU))  
/*******************************************************************************************************************/

void SystemInit(void);
void HAL_Init(void);
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void SystemClock_Config(void);
void GPIO_Init(void);
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void SystemCoreClockUpdate(void) ;
void UART4_Init(void);
void UART_Transmit(uint8_t *pData, uint32_t size);
uint32_t HAL_GetTick (void);
void HAL_Delay(uint32_t Delay);

#endif
