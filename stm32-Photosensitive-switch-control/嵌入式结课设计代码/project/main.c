/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "platform_config.h"

/* Private typedef -----------------------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  ErrorStatus HSEStartUpStatus=SUCCESS;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
  u16 KEYPAD_STATE;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************

*******************************************************************************/
int main(void)
{ 
#ifdef DEBUG
  debug();
#endif
/************************************  Configuration  system  *********************************/
  
  /* System Clocks Configuration */
  RCC_Configuration();   

  /* NVIC Configuration */
  NVIC_Configuration();

  /* Configure the GPIO ports */
  GPIO_Configuration();  
  
  /* Configure the USART */  
  USART_Configuration();
  
  /* EXTI Configure */
  EXTI_Configuration();

  
  
/************************************  program  begin  *********************************/ 
  /*先熄灭所有的LED灯*/
  GPIO_ResetBits(GPIOC, GPIO_Pin_4);   
  GPIO_ResetBits(GPIOC, GPIO_Pin_5);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  GPIO_ResetBits(GPIOF, GPIO_Pin_11); 

  while (1)
  {
    /***** 根据按键，判断状态 *****/
    if(KEYPAD_STATE==keypad_0)
    {
      GPIO_ResetBits(GPIOB, GPIO_Pin_14);//关闭电机
      Light_up_LED(GPIOC, GPIO_Pin_4);//自动状态点亮LED 
    }
     if(KEYPAD_STATE==keypad_0&&GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))   //自动状态下，判断光敏电阻的检测结果
      {
        GPIO_SetBits(GPIOB, GPIO_Pin_14); //外界过亮开启电机关闭窗帘
        printf("Auto light  \r");//在终端处显示亮
      }
    if(KEYPAD_STATE==keypad_0&&!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))
      {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14); //外界过暗
        printf("Auto dick   \r");
      }
    if(KEYPAD_STATE==0x04)//手动状态下无命令
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_4);   //手动状态无指令
      printf("Manual        \r");
    }
      if(KEYPAD_STATE==keypad_1)//手动状态开启电机
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_4);  
      GPIO_SetBits(GPIOB, GPIO_Pin_14);
      printf("Manual open   \r");
    }
      if(KEYPAD_STATE==keypad_2)//手动状态关闭电机
    {
      GPIO_SetBits(GPIOC, GPIO_Pin_4); 
      GPIO_ResetBits(GPIOB, GPIO_Pin_14);
      printf("Manual close  \r");
    }

  }
  
}

/*******************************************************************************
      系统时钟设置
*******************************************************************************/
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

  /* Enable GPIO and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
                         RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO , ENABLE);
  
  /* Enable USART1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  

 
}

/*******************************************************************************
      中断向量控制器
*******************************************************************************/
void NVIC_Configuration(void)//嵌套中断向量控制器
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//设置优先级分组：先占优先级和从优先级，先占优先级1位，从优先级3位
  
  /* Enable the EXTI15_10 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;//IRQ 通道外部中断线 15-10 中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//该参数设置了成员 NVIC_IRQChannel 中的先占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//该参数设置了成员 NVIC_IRQChannel 中的从优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//开
  NVIC_Init(&NVIC_InitStructure); 
  
  /* Enable the EXTI9_5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;//外部中断线 9-5 中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}
/*******************************************************************************
      外部中断设置
*******************************************************************************/

void EXTI_Configuration(void)//外部中断/事件控制器
{
  EXTI_InitTypeDef EXTI_InitStructure;
  /* Connect Key Button EXTI Line to Key Button GPIO Pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);//选择GPIO管脚用作外部中断线路；PC13引脚的复用功能 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource15);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource7);
  
  /* Configure Key Button0 EXTI Line to generate an interrupt on falling edge */  
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;//选择了待使能或者失能的外部线路外部中断线 13
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//设置了被使能线路的模式，设置 EXTI 线路为中断请求
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//设置了被使能线路的触发边沿，设置输入线路下降沿为中断请求
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Configure Key Button1 EXTI Line to generate an interrupt on falling edge */  
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Configure Key Button2 EXTI Line to generate an interrupt on falling edge */  
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
}

void Light_up_LED(GPIO_TypeDef* GPIOx , u16 GPIO_Pin)
{
  GPIO_SetBits(GPIO_LED1,GPIO_PIN_LED1); //熄灭LED1
  GPIO_SetBits(GPIO_LED2,GPIO_PIN_LED2);
  GPIO_SetBits(GPIO_LED3,GPIO_PIN_LED3);
  GPIO_SetBits(GPIO_LED4,GPIO_PIN_LED4);
  GPIO_SetBits(GPIO_LED5,GPIO_PIN_LED5);
  GPIO_ResetBits(GPIOx,GPIO_Pin);        //点亮选中的LED
}


/*******************************************************************************
      GPIO口设置
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

/*--------------------- USART ----------------------*/  

  /* Configure USART1 Tx PA.9as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART1 Rx PA.10 as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

/* Configure LED1 (PC.4)as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    /*GPIOB.14 Configuration:电机 as OutPut push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
       /* Configure 光敏电阻 Pin (PC.2) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Configure Key Button0 GPIO Pin(PC.13) as input floating (Key Button EXTI Line) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* Configure Key Button1 GPIO Pin(PG.15) as input floating (Key Button EXTI Line) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  /* Configure Key Button2 GPIO Pin(PF.7) as input floating (Key Button EXTI Line) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  /* Configure LED2 (PC.5)as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure LED3 (PB.0)as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure LED4 (PB.1)as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure LED5 (PF.11)as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/*******************************************************************************
      延时程序
*******************************************************************************/
void Delaytime(u32 nCount)
{
  for(; nCount != 0; nCount--);
}



/*******************************************************************************
      USART串口设置
*******************************************************************************/
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;//波特率115200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1停止位
  USART_InitStructure.USART_Parity = USART_Parity_No ;//无奇偶检验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//打开接收发送功能

  /* Configure the USART2 */
  USART_Init(USART1 ,&USART_InitStructure);
  
  /* Enable the USART2 */
  USART_Cmd(USART1, ENABLE);
}

/*******************************************************************************
      PUTCHAR_PROTOTYPE
*******************************************************************************/
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {
  }

  return ch;
}




#ifdef  DEBUG
/*******************************************************************************
        assert_failed
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
