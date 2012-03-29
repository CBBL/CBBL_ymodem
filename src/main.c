/**
  ******************************************************************************
  * @file    IAP/src/main.c 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    10/15/2010
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/** @addtogroup IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
static void IAP_Init(void);
//static void IO_Init(void);
static void clock_init(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*Clock initialization. */
  clock_init();

  /* Flash unlock */
  FLASH_Unlock();


  /*GPIO Initialization. */
  /*GPIOA, GPIOB on APB2 bus clock enable. */
  	  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

  	  /*PA0, PA1, PA2, PA3 in output mode general purpose push-pull. */
  	  GPIOA->CRL =(0x03 << (3 * 4)) | (0x03 << (2 * 4)) | (0x03 << (1 * 4)) | (0x03 << (0 * 4));

  	  /*PB0, PB1 in floating input mode. */
  	  GPIOB->CRL = (0x08 << (1 * 4)) | (0x08 << (0 * 4));

  	  /*why do we do this?. */
  	  GPIOB->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1;

  	  /*Light up all leds by resetting the output bits. */
  	  GPIOA->BSRR |= GPIO_BSRR_BR0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR2 | GPIO_BSRR_BR3;

  /* Initialize Key Button mounted on STM3210X-EVAL board */       
  //STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Test if button on the board is pressed during reset. */
  if ((GPIOB->IDR & GPIO_IDR_IDR1) == 0x00)
  { 
    /* If Key is pressed */
    /* Execute the IAP driver in order to re-program the Flash */
    IAP_Init();
    SerialPutString("\r\n======================================================================");
    SerialPutString("\r\n=              (C) COPYRIGHT 2010 STMicroelectronics                 =");
    SerialPutString("\r\n=                                                                    =");
    SerialPutString("\r\n=     In-Application Programming Application  (Version 3.3.0)        =");
    SerialPutString("\r\n=                                                                    =");
    SerialPutString("\r\n=                                   By MCD Application Team          =");
    SerialPutString("\r\n======================================================================");
    SerialPutString("\r\n\r\n");
    Main_Menu ();
  }
  /* Keep the user application running */
  else
  {
    /* Test if user code is programmed starting from address "ApplicationAddress" */
	uint32_t pippo1, pippo2;
	pippo1 = (*(__IO uint32_t*)ApplicationAddress); 0x08000000
	pippo2 = pippo1 & 0x2FFE0000;
    if (pippo2 == 0x20000000)
    { 
      /* Jump to user application */
      /* JumpAddress is not a pointer type. */
      /* ApplicationAddress+4 casted to a pointer to uin32_t. Jump address is the content
       * of the cell pointed by ApplicationAddress
       * Though wouldn't this be correct: JumpAddress = ApplicationAddress+4;
       * or! why does it dereferences it?
       */
      JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);

      /*Jump_To_Application now points to the same address = JumpAddress through casting. */
      Jump_To_Application = (pFunction) JumpAddress;

      /* Initialize user application's Stack Pointer */
      /* __set_MSP implemented in CMSIS (as all the functions __* like). */
      __set_MSP(*(__IO uint32_t*) ApplicationAddress);
      Jump_To_Application();
    }
  }

  while (1)
  {}
}



/**
  * @brief  Initialize the IAP: Configure RCC, USART and GPIOs.
  * @param  None
  * @retval None
  */
void IAP_Init(void)
{
  //USART_InitTypeDef USART_InitStructure;

  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
  /* USART configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */

  //USART_InitStructure.USART_BaudRate = 115200;
  //USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  //USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //USART_InitStructure.USART_Parity = USART_Parity_No;
  //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  //USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  //USART_Init(USART1, USART_InitStructure);

  /*Baud Rate at 115200. */
  uint32_t BRR=0x00000271;

  /*GPIOA, GPIOB on APB2 bus clock enable. */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

  /*USART1 on APB2 bus clock enable. */
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  /*Ensure no remap, keep PA9,PA10. */
  AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;

  /*UART Enable. */
  USART1->CR1 |= 0x00002000;

  /*Use 1 stop bit. */
  USART1->CR2 &= ~0x00003000;

  /*Set BaudRate=115.2Ksymb/sec @72MHz CLK, 1 Symb=1 Bit. */
  USART1->BRR = BRR;

  /*Set word length to 8 bits. */
  USART1->CR1 &= ~0x00001000;

  /*Configure GPIOA output mode for UART. */
  GPIOA->CRH = 0x444444B4;

  /* what about parity?
   * what about hardware flow control?
   */

  /*Set TE RE (transmission and receive enable) bits. */
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

  //STM_EVAL_COMInit(COM1, &USART_InitStructure);
}

//void IO_Init(void) {

//}

void clock_init(void) {
	/* Activate HSE. */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait until HSE is stable. */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* PLL setup. */
	RCC->CFGR |= RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLXTPRE_HSE | RCC_CFGR_PLLSRC_HSE;

	/* Activate PLL. */
	RCC->CR   |= RCC_CR_PLLON;

	/* Wait until PLL is stable. */
	while (!(RCC->CR & RCC_CR_PLLRDY));

	/* AHB, APB1 and APB2 prescaler setup. */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

	/* FLASH wait states setup. */
	FLASH->ACR |= FLASH_ACR_LATENCY_2;

	/* Set PLL as system clock */
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait until PLL is the system clock. */
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL ));
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
