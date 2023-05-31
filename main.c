#include "stm32f4xx.h"
#include "math.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
//
uint8_t bSTX[]= { 0x02 };
uint8_t bADC[] = {0x41, 0x44, 0x43};
uint8_t bDAC[] = {0x44, 0x41, 0x43};
uint8_t bDI[] = {0x47, 0x44, 0x49};
uint8_t bDO[] = {0x47, 0x44, 0x4F};
uint8_t bACK[] = { 0x06 };
uint8_t bETX[] = { 0x03 };
uint8_t bProtocolDataBuffer[17] = {};	// tuy kich thuoc mang 17
uint8_t bProtocolData[8] = {};

uint8_t strCommand[3];
uint8_t strOpt[3];
uint8_t strData[8];
bool bDataAvailable = false;
	
uint8_t *subString(uint8_t *s, int pos, int index); //substring(): return extracts the characters in string “*s” at position “pos” and take “index” characters
bool ReadComm(uint8_t *pBuff, uint16_t nSize);
bool StrCompare(uint8_t *pBuff, uint8_t *Sample, uint16_t nSize);
uint8_t *t;
bool serial_Process(void);

//UART_RX
#define		BUFF_SIZE_RX		17
uint8_t data_Rx[BUFF_SIZE_RX];
uint8_t data_Tx[BUFF_SIZE_RX+1]; // MANG CHUA 1 KI TU * O PHIA CUOI
void TIM7_Config(uint16_t Frequency, uint16_t Ns);
void DAC_Ch2_WaveConfig(uint16_t* value, uint16_t BUFF);

#define		BUFF_SIZE_DAC_SV			201		// SO LAN LAU MAU SONG VUONG	
//void get_svvalue_Ch2( float voltage);
void get_svvalue_Ch2(int duty_cycle);
uint16_t svvalue_Ch2[BUFF_SIZE_DAC_SV]; 	
uint16_t wavevalue_Ch2[2];
void get_wavevalue_Ch2( float voltage);

// UART and DAC
void UART_TX_Config(void);	// cau hinh chung cho MODE UART_TX
void Display1( uint8_t* Addr_txbuff, uint16_t BUFF   );
void UART_Rx_Config(void);			//Cau hinh cho MODE UART_RX
void DAC_Pin_Config();
DMA_InitTypeDef  	DMA_InitStructure;

// DI
void DI_Config();	// ngat tren chan PA0
int pressed_count;
bool Pressed = false;
int released_count;

//void EXTI0_IRQHandler(void);

// ADC
#define nb_ADCValue 20
#define G_kd 	(float)21.4
#define R_1k 987.0
#define R0_value 100.0
#define A_value (float)0.00439285  
#define B_value (float)-0.00000642
#define V_dd 2.97
void ADC_Config(void); 
uint16_t ADCValue[nb_ADCValue]={0}; 
uint16_t average(); // ham tinh trung binh gia tri ADC
float adc_value;
float adc_value_filter;
uint16_t	adc_average = 0; // gia tri chinh thuc
uint8_t String_ADC_value[4];
#define		BUFF_SIZE_TX_ADC	4
uint8_t ADC_Value[BUFF_SIZE_TX_ADC];
uint8_t ADC_Tx_Value[BUFF_SIZE_TX_ADC +1] = {};
void TIM5_Config(uint16_t ms);
	
float ADCvalue_to_Resistance(float denta_vol);// chuyen dien ap thanh dien tro
float PT100_Calib(float resistance_value);

void delay_us(uint16_t period);
void DO_Config(void);	//Cai hinh led hoat dong
int ATD(uint8_t A[], uint8_t i); //Chuyen mang ASCII sang so
void IntToStr4 (uint16_t u, uint8_t *y);

	//DAC
uint8_t Rx_Hz[4]="0000";
uint8_t Rx_V[2]="00";
uint16_t Hz1=0;	//tan so kenh 1
float V1=0;		// bien do kenh 1
uint16_t Hz2=0;
float V2=0;
uint16_t duty;
uint8_t Rx_duty[3]="000";
	
typedef struct
{
	float Last_P;
	float Now_P;
	float out;
	float Kg;
	float Q;
	float R;
}Kalman;

Kalman kfp_instance;
void Kalman_Init(Kalman* kfp)
{
	kfp->Last_P = 1;
	kfp->Now_P = 0;
	kfp->out = 0;
	kfp->Kg = 0;
	kfp->Q = 0.0000001;
	kfp->R = 0.001;
}

float KalmanFilter(Kalman *kfp, float input)
{
	kfp->Now_P = kfp->Last_P + kfp->Q;
	kfp->Kg = kfp->Now_P/( kfp->Now_P+ kfp->R);
	kfp->out = kfp->out + kfp->Kg*(input - kfp->out);
	kfp->Last_P = (1-kfp->Kg)*kfp->Now_P;
	return kfp->out;
}
	
	
int main(void)
{
	
	DO_Config();
	DI_Config();
	DAC_Pin_Config();
	UART_TX_Config();
	UART_Rx_Config();
	Kalman_Init(&kfp_instance);
	//ADC_Config();
	
	while(1){

					}
}

void UART_Rx_Config(void)	// Cau hinh chan nhu h�m
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef   DMA_InitStructure_Rx;
  NVIC_InitTypeDef  NVIC_InitStructure;	
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);// DMA CO DOI KHONG??
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4); //
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;//
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
  /* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure_Rx.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure_Rx.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure_Rx.DMA_Memory0BaseAddr = (uint32_t)data_Rx;
  DMA_InitStructure_Rx.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure_Rx.DMA_BufferSize = BUFF_SIZE_RX; //
  DMA_InitStructure_Rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_Rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_Rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_Rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_Rx.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure_Rx.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure_Rx.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure_Rx.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure_Rx.DMA_MemoryBurst = DMA_MemoryBurst_INC8;
  DMA_InitStructure_Rx.DMA_PeripheralBurst = DMA_PeripheralBurst_INC8;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure_Rx);
  DMA_Cmd(DMA1_Stream2, ENABLE);
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	/*
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	*/
}
/*
void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) 
	{
		GPIO_ToggleBits(GPIOD,GPIO_PinSource15);
    // Đọc dữ liệu từ UART4 Rx
    //uint16_t data = USART_ReceiveData(UART4);
    
    // Xử lý dữ liệu nhận được ở đây
    
    // Xóa cờ ngắt UART4 Rx
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
  }
}
*/
void DMA1_Stream2_IRQHandler(void)	//NHAN du 17 BYTE se chay CT NGAT, 17 BYTE do luu trong MANG data_Rx
{

	bool process;
	ReadComm(data_Rx, BUFF_SIZE_RX);
	process = serial_Process();

  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	/* Kiem tra khi nao xay ra ngat */
if(process)
{
		if(StrCompare(bADC, strCommand, 3))	/***************************///ADC
		{
			ADC_Config();
			TIM5_Config(10);
			
			DAC_DeInit();
			TIM_DeInit(TIM7);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
			NVIC_DisableIRQ(EXTI0_IRQn);
		}//ADC
		else if(StrCompare(bDAC, strCommand, 3))/************************///DAC
		{
			if(!(strData[7] - 0x31))
			{	// TAN SO
				for(int i =3; i<7 ;i++)
				{
				Rx_Hz[i-3]=strData[i];
				}
				Hz1=ATD(Rx_Hz,4);
				for(int i=0;i<4;i++)
				{
				Rx_duty[i]=strData[i];
				}
				duty=ATD(Rx_duty,3);
				// CAU HINH XUNG
				get_svvalue_Ch2(duty);
				TIM7_Config(Hz1, BUFF_SIZE_DAC_SV);
				DAC_Ch2_WaveConfig(svvalue_Ch2,BUFF_SIZE_DAC_SV);
		
				// CT CHUNG	
				DAC_DMACmd(DAC_Channel_1, ENABLE);
				DMA_Cmd(DMA1_Stream5, ENABLE);		
				DAC_DMACmd(DAC_Channel_2, ENABLE);
				DMA_Cmd(DMA1_Stream6, ENABLE);
			}
			else
			{
				DAC_DeInit();
				TIM_DeInit(TIM7);
			}
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
			TIM_DeInit(TIM5);
			ADC_DeInit();
			NVIC_DisableIRQ(EXTI0_IRQn);
		}//DAC
		else if(StrCompare(bDO, strCommand, 3))	//DO
		{	
				if(!(strData[4]-0x31))
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
				}
				else
					GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				if(!(strData[5]-0x31))
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_13);
				}
				else
					GPIO_ResetBits(GPIOD, GPIO_Pin_13);
				if(!(strData[6]-0x31))
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_14);
				}
				else
					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				if(!(strData[7]-0x31))
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_15);
				}
				else
					GPIO_ResetBits(GPIOD, GPIO_Pin_15);
				
				TIM_DeInit(TIM7);//DAC
				DAC_DeInit();
				TIM_DeInit(TIM5);//ADC
				ADC_DeInit();
				NVIC_DisableIRQ(EXTI0_IRQn);

		}//DO
			else if(StrCompare(bDI, strCommand, 3)) /***************************///DI
			{
				NVIC_EnableIRQ(EXTI0_IRQn);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//DO
				ADC_DeInit();
				TIM_DeInit(TIM5);//ADC
				DAC_DeInit();
				TIM_DeInit(TIM7);//DAC
			}
			else
			{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//DO
			TIM_DeInit(TIM7);//DAC
			DAC_DeInit();
			TIM_DeInit(TIM5);//ADC
			NVIC_DisableIRQ(EXTI0_IRQn);
			ADC_DeInit();
			}
}
else
	{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//DO
			TIM_DeInit(TIM7);//DAC
			DAC_DeInit();
			TIM_DeInit(TIM5);//ADC
			NVIC_DisableIRQ(EXTI0_IRQn);
			ADC_DeInit();
	}
}
void Display1( uint8_t* Addr_txbuff, uint16_t BUFF )
{
		/*Tach 1 phan tu ham UART_TX_Config ra*/
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Addr_txbuff;
		DMA_InitStructure.DMA_BufferSize = BUFF;//BUFF_SIZE_UART;
	  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	  DMA_Cmd(DMA1_Stream4, ENABLE);		// phai cho phep truyen lai
		while(DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4)  == RESET );	
}

void UART_TX_Config(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4); //
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;//
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;//
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
	/* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
		/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}

void TIM7_Config(uint16_t Frequency, uint16_t Ns)
{
// Cau h�nh TIMER 7
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = 1000000/(Frequency*Ns)-1;     
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM7, TIM_TRGOSource_Update);
	TIM_Cmd(TIM7, ENABLE);
}

void DAC_Ch2_WaveConfig(uint16_t* value, uint16_t BUFF)
{
	DAC->CR |= (6<<16)|(1<<20);		// Mode kich hoat ngoai TIM7; co bo dem
	//Enable DMA1 clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);// cho phep DMA1-CA 2 KENH DAC
	// cau h�nh DMA1 cho DAC1 channel 7 stream 6
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->DHR12R2);	//khong phan biet DAC1 v� DAC2
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = BUFF;//BUFF_SIZE_DAC_SIN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//do phan giai DAC 12bit nen DMA chon HALFWORD
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	DAC_Cmd (DAC_Channel_2, ENABLE);
}

/*CAC HAM TAO MANG SONG */

/***** Xung Vuong ******/

void get_svvalue_Ch2(int duty_cycle) // TAO MANG SONG VUONG CHANNEL 2
{
	for(int i =0;i<duty_cycle + 1;i++)
	{
		svvalue_Ch2[i] = 0xFFF;
	}
	for(int i =duty_cycle + 1;i<BUFF_SIZE_DAC_SV;i++)
	{
		svvalue_Ch2[i] = 0;
	}
}

/***** Tao xung kenh 2 (1/2 bien do) cho mach tru ******/
void get_wavevalue_Ch2( float voltage)
{
	for(int i =0;i<2;i++)
	{
		wavevalue_Ch2[i] = 0xFFF;
	}
}

void DAC_Pin_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure; 	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); 
	// 2 kenh DAC PA4,PA5
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5; //GPIO_Pin_4 |
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //Chon mode analog
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/********** DO Config *********/
void DO_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/* End DO Config */


/********** DI Config *********/
void DI_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure; 

	/* bat clock GPIOA */ 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	/*bat clock SYSCFG */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	/* Configure PA0 pin as input floating */ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
/* Connect EXTI Line0 to PA0 pin */ 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0); 
	/* Configure EXTI Line0 */ 
	/* PD0 is connected to EXTI_Line0 */ 
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	/* Interrupt mode */ 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	/* Triggers on rising or falling edge or both */ 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	/* Enable interrupt */ 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStructure); 
	/* Enable and set EXTI Line0 Interrupt to the lowest priority */ 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 
	/* Set priority */ 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 
	NVIC_DisableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void) 
{
	memcpy(data_Tx, data_Rx, BUFF_SIZE_RX);
	data_Tx[BUFF_SIZE_RX]='*';
	Display1(data_Tx, BUFF_SIZE_RX + 1);
	delay_us(50000);
	EXTI->PR = EXTI_Line0; //XOA CO NGAT
}

/* End DI Config */


void ADC_Config(void) // debug xem can chinh gia tri trong thanh ghi data
{
	//trung voi timer DAC
	/*
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
TIM_TimeBaseStructure.TIM_Period = 8400 - 1; // T?n s? 10 KHz
TIM_TimeBaseStructure.TIM_Prescaler = 10;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	*/
DMA_InitTypeDef DMA_InitStructure_ADC; 
ADC_InitTypeDef ADC_InitStructure; 
ADC_CommonInitTypeDef ADC_CommonInitStructure; 
GPIO_InitTypeDef GPIO_InitStructure; 
	
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; /*Chon mode analog*/ 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
GPIO_Init(GPIOC, &GPIO_InitStructure);
	//////////////////
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; /*Chon mode analog*/ 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
GPIO_Init(GPIOA, &GPIO_InitStructure);
	

DMA_InitStructure_ADC.DMA_Channel = DMA_Channel_0; /*chanel duoc ho tro la chanel 0-do bang*/ 
DMA_InitStructure_ADC.DMA_Memory0BaseAddr = (uint32_t)&ADCValue; /*ghep bien DMA_Memory0BaseAddr chua dia chi va cung kieu voi bien ADCValue*/ 
DMA_InitStructure_ADC.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR)); /*gan dia chi thanh ghi chua gia tri chuyen doi ADC vao bien DMA_PeripheralBaseAddr 
cua DMA*/ 
DMA_InitStructure_ADC.DMA_DIR = DMA_DIR_PeripheralToMemory; /*chon huong chuyen du lieu*/

DMA_InitStructure_ADC.DMA_BufferSize = nb_ADCValue; /*chon kich thuoc mang du lieu*/ 
DMA_InitStructure_ADC.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /*moi lan chuyen du lieu, dia chi ngoai vi se ko tang dan*/ 
DMA_InitStructure_ADC.DMA_MemoryInc = DMA_MemoryInc_Enable; /*moi khi chuyen du lieu can tang dia chi bo nho*/ 
DMA_InitStructure_ADC.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* kich thuoc thanh ghi chua du lieu ngoai vi la 16bit*/ 
DMA_InitStructure_ADC.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; /*kich thuoc mang du lieu ADCValue là 16bit*/ 
DMA_InitStructure_ADC.DMA_Mode = DMA_Mode_Circular; /*chon mode DMA vong tron, viec chuyen doi lien tuc lap lao*/ 
DMA_InitStructure_ADC.DMA_Priority = DMA_Priority_High; /*thiet lap che do uu tien cao*/ 
DMA_InitStructure_ADC.DMA_FIFOMode = DMA_FIFOMode_Enable; 
DMA_InitStructure_ADC.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
DMA_InitStructure_ADC.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
DMA_InitStructure_ADC.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
DMA_Init(DMA2_Stream0, &DMA_InitStructure_ADC); 
/* DMA2_Stream0 enable */ 
DMA_Cmd(DMA2_Stream0, ENABLE); 

NVIC_InitTypeDef  NVIC_InitStructure_0;	
NVIC_InitStructure_0.NVIC_IRQChannel = DMA2_Stream0_IRQn;
NVIC_InitStructure_0.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure_0.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure_0.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure_0);
  /* Transfer complete interrupt mask */
 DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);


/* ADC Common Init 
**********************************************************/ 
ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; /*chon mode Independent, Dual, Triple cho ADC*/ 
ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; /*thiet lap bo chia 2, cho ADC lay mau o tan so cao nhat*/ 
ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; /*Configures the Direct memory access mode for multi ADC mode */
ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles; /*thoi gia tre giua 2 lan lay mau (5-20 chu ky)*/ 
ADC_CommonInit(&ADC_CommonInitStructure); 

ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; /*che do phan giai ADC la 12 bit*/

ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ENABLE; 
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //neu DISABLE thì chi nhan gia tri ADC, 1 lan duy dat khong thay doi

ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; //ADC_ExternalTrigConv_T2_TRGO;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//ADC1->CR2 &= ~(1<<11);
ADC_InitStructure.ADC_NbrOfConversion = 1; /*so kenh ADC chuyen doi*/ 
ADC_Init(ADC1, &ADC_InitStructure); 
/* ADC1 regular channels configuration */ 
//ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles); 
//ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_3Cycles); 
//ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_112Cycles); 
ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_480Cycles); 
/* Enable ADC1 DMA */ 
ADC_DMACmd(ADC1, ENABLE); 
/* Enable DMA request after last transfer (Single-ADC mode) */ 
ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
/* Enable ADC1 */ 
ADC_Cmd(ADC1, ENABLE); 
/* Start ADC1 Software Conversion */
ADC_SoftwareStartConv(ADC1);

}


void DMA2_Stream0_IRQHandler(void)//DMA_ADC
{
	GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
	adc_average = (uint16_t)(0xfff &average());
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
}


/********** Timer 5 cho ADC *********/
void TIM5_Config(uint16_t ms)
{
	NVIC_InitTypeDef  NVIC_InitStructure;	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	/*TIMER Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = 100000* ms;          
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	/* TIMER TRGO selection */
	TIM_SelectOutputTrigger(TIM5, TIM_TRGOSource_Update); 
	/* TIMER enable counter */
	//TIM_Cmd(TIM3, ENABLE);

	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);/*thiet lap ngat khi tran bo nho co thong so TIM_IT_Update*/ 
	TIM_Cmd(TIM5, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}
void TIM5_IRQHandler(void) 
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 
	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
		adc_value = V_dd *((float)adc_average)/4095;
		for(int i =0; i<100;i++)
		{
		adc_value_filter = KalmanFilter(&kfp_instance, adc_value);
		}
		float temp = ADCvalue_to_Resistance(adc_value_filter);
		temp = PT100_Calib(temp);
		uint16_t temp2 = (uint16_t)(temp*100);
		
		IntToStr4(temp2, ADC_Value);
		//memcpy(ADC_Tx_Value, ADC_Value, BUFF_SIZE_TX_ADC);
		memcpy(strData, ADC_Value, BUFF_SIZE_TX_ADC);
		//ADC_Tx_Value[BUFF_SIZE_TX_ADC]='*';
		memcpy(data_Tx + 7, strData, 8 );
		for(int i =4; i< 8 ; i++)
		{
			data_Tx[i + 7] = 0x30;
		}
		data_Tx[BUFF_SIZE_RX]='*';
		Display1(data_Tx, BUFF_SIZE_RX + 1);
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 
}



/********** Nhung hàm xu ly *********/
int ATD(uint8_t A[], uint8_t i)//Chuyen chuoi sang ma ASCII
{
	uint16_t D=0;
	for(int j=0;j<i;j++)
	{
		D=D+(A[j]-0x30)*pow(10,i-1-j);
	}
	return D;
}

void IntToStr4(uint16_t u, uint8_t *y)
{
	uint16_t a;
	a = u;
	y[3] = a % 10 + 0x30;
	a = a/10;
	y[2] = a % 10 + 0x30;
	a = a/10;
	y[1] = a % 10 + 0x30;
	a = a/10;
	y[0] = a + 0x30;
}

void sub_String(uint8_t *s, uint8_t *d , int pos, int index)
{
	for(int i = pos; i < pos + index ; i++)
	{
		d[i - pos] = s[i];
	}
}

bool StrCompare(uint8_t *pBuff, uint8_t *Sample, uint16_t nSize)//
{
	for(int i = 0; i < nSize; i++)
	{
		if(pBuff[i] != Sample[i])
		{
			return false;
		}
	}
	return true;
}

bool ReadComm(uint8_t *pBuff, uint16_t nSize)// neu so luong handshake thay doi thi nho thay doi vi tri mang
{
	if((pBuff[0] == bSTX[0]) && (pBuff[16] == bETX[0]))
	{
		sub_String(pBuff, strCommand, 1, 3);
		sub_String(pBuff, strOpt, 4, 3);
		sub_String(pBuff, strData, 7, 8);

		bDataAvailable = true;
	}
	else
	{
		bDataAvailable = false;
	}
	return bDataAvailable;
}
bool serial_Process(void)
{
	uint16_t nIndex = 0;
	if(bDataAvailable == true)
	{
		if(StrCompare(strCommand, (uint8_t *)"DAC", 3))
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, strCommand, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		else if(StrCompare(strCommand, (uint8_t *)"ADC", 3))
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, strCommand, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		else if(StrCompare(strCommand, (uint8_t *)"GDO", 3))
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, strCommand, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		else if(StrCompare(strCommand, (uint8_t *)"GDI", 3))
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, strCommand, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		else if(StrCompare(strCommand, (uint8_t *)"STP", 3))
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, strCommand, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		else if(StrCompare(strCommand, (uint8_t *)"TIM", 3))
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, strCommand, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		else
		{
			memcpy(data_Tx + nIndex, bSTX, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, (uint8_t *)"NUL", 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strOpt, 3);
			nIndex += 3;
			memcpy(data_Tx + nIndex, strData, 8);
			nIndex += 8;
			memcpy(data_Tx + nIndex, bACK, 1);
			nIndex += 1;
			memcpy(data_Tx + nIndex, bETX, 1);
			data_Tx[BUFF_SIZE_RX]='*';
			Display1(data_Tx, BUFF_SIZE_RX + 1);
		}
		bDataAvailable = false;
	}
	return true;
}



/********** Tinh gia tri ADC trung binh tu mang khi doc bang DMA *********/
uint16_t average() 
{
    int sum = 0;
    for (int i = 0; i < nb_ADCValue; i++) {
        sum += ADCValue[i];
    }
    return sum / nb_ADCValue;
}

/********** Tao thoi gian Delay bang Timer 6 *********/
void delay_us(uint16_t period){
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;
  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6
  	while (!TIM6->SR);
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

float ADCvalue_to_Resistance(float denta_vol) 
{
	float vol = denta_vol/G_kd + (5.0*101.0/(987 +101.0));
	float resistance = (R_1k*vol)/(5.0 - vol);
	return resistance;
}
float PT100_Calib(float resistance_value)
{
		float Temperture = -(2*(R0_value - resistance_value))/(R0_value*A_value +sqrt(R0_value*R0_value*A_value*A_value - 4*R0_value*B_value*(R0_value - resistance_value)));
	  return Temperture;
}
	