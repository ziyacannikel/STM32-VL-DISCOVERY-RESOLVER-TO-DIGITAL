#include "stdio.h"
#include "delay.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

    #define I2Cx_RCC        RCC_APB1Periph_I2C2
    #define I2Cx            I2C2
    #define I2C_GPIO_RCC    RCC_APB2Periph_GPIOB
    #define I2C_GPIO        GPIOB
    #define I2C_PIN_SDA     GPIO_Pin_11
    #define I2C_PIN_SCL     GPIO_Pin_10


void i2c_ini(){
    // Initialization struct
    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
 
    // Step 1: Initialize I2C
    RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2Cx, &I2C_InitStruct);
    I2C_Cmd(I2Cx, ENABLE);
 
    // Step 2: Initialize GPIO as open drain alternate function
    RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
    GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
}
void i2c_start(){
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
 
    // Generate start condition
    I2C_GenerateSTART(I2Cx, ENABLE);
 
    // Wait for I2C EV5. 
    // It means that the start condition has been correctly released 
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop(){
    // Generate I2C stop condition
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
	 
}
void i2c_address_direction(uint8_t address, uint8_t direction){
    // Send slave address
    I2C_Send7bitAddress(I2Cx, address, direction);
  
    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(I2Cx,
            I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if (direction == I2C_Direction_Receiver)
    { 
        while (!I2C_CheckEvent(I2Cx,
            I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
 
}
 
void i2c_transmit(uint8_t byte){
    // Send data byte
    I2C_SendData(I2Cx, byte);
    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and 
    // output on the bus)
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	 
}
 
uint8_t i2c_receive_ack(){
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
 
    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}
 
uint8_t i2c_receive_nack(){
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
 
    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}
 
void i2c_write(uint8_t address, uint8_t data ,_Bool  secim){
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Transmitter);
	/*COMMAND BYTE FOR POT*/
	  if(secim==0)
		{
			//pot0 a yazar
		  i2c_transmit((uint8_t)0xA9);//10101001
		}
		else
		{
			//pot1 e yazar
		  i2c_transmit((uint8_t)0xAA);//10101010
		}

    i2c_transmit(data);
    i2c_stop();
	 
}
 



void USART_Puts(USART_TypeDef* USARTx , volatile char *s){
	while(*s){
		while(!(USARTx -> SR & 0x00000040));
		USART_SendData(USARTx, *s);
		*s++;		
	}
}


char asc_convert(char value){
		switch(value){
			case 48  : return 0; 
			case 49  : return 1;
			case 50  : return 2;
			case 51  : return 3;
			case 52  : return 4;
			case 53  : return 5;
			case 54  : return 6;
			case 55  : return 7;
			case 56  : return 8;
			case 57  : return 9;
			case 118 : return 88; //v
			case 102 : return 99; //f
		  default  : return 77; //lüzumsuz
		}
  
}
void usart_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9;// usart1 tx
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_10;// usart1 rx
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate            = 460800;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx  ;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//////////////////kesme burda açildi
	USART_Cmd(USART1,ENABLE);
	
	/*KESME AYARLARI*/
	NVIC_InitTypeDef    NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel          = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd       = ENABLE;
	NVIC_SetPriority(USART1_IRQn,0);
  NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);


}






void init_adc1(void){
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	ADC_InitTypeDef  ADC_InitStructure;

  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
  ADC_DeInit(ADC1);

	
  ADC_InitStructure.ADC_ScanConvMode        = ENABLE;          // tarama modu açik
  ADC_InitStructure.ADC_ContinuousConvMode  = DISABLE;         // sürekli çevirim yapacagiz  /*ENABlE*/
  ADC_InitStructure.ADC_Mode                = ADC_Mode_Independent; 
  ADC_InitStructure.ADC_ExternalTrigConv    = ADC_ExternalTrigConv_None; /*ADC_ExternalTrigConv_T3_TRGO;*/
	ADC_InitStructure.ADC_DataAlign           = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel        = 3;              // 3 kanal tarayacagiz.
  ADC_Init(ADC1, &ADC_InitStructure);
	
	
  ADC_Cmd(ADC1, ENABLE); 
	
	//Check the end of ADC1 reset calibration register
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
	
	//Start ADC1 calibration
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	
}


void timer_init(void){
	
	/*TIMER3 CONFIGURATION*/
  //75uS AYARLI
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 1;                // Set prescaler 
	TIM3->ARR = 9799;             // Auto reload value 
	TIM3->DIER = TIM_DIER_UIE;    // Enable update interrupt (timer level)
	TIM3->CR1 = TIM_CR1_CEN;      // Enable timer

	NVIC_EnableIRQ(TIM3_IRQn);    // Enable interrupt from TIM3 (NVIC level)
	

	 
}
char *convert(char name, double value){//basina kodunu ekleyip düzgün string haline getiriyor
	static char hazir_veri[10];
	char str[9];
	if(name=='v'){
	sprintf(str,"%lf", value); 
		int i=0;
		hazir_veri[0]='\n';
		hazir_veri[1]='v';
		for(i=2;i<10;i++){
			hazir_veri[i]=str[i-2];
		}
	}
	else if(name=='c'){
	sprintf(str,"%lf", value); 
		int i=0;
		hazir_veri[0]='\n';
		hazir_veri[1]='c';
		for(i=2;i<10;i++){
			hazir_veri[i]=str[i-2];
		}
	}
	
	else if(name=='s'){
	sprintf(str,"%lf", value); 
		int i=0;
		hazir_veri[0]='\n';
		hazir_veri[1]='s';
		for(i=2;i<10;i++){
			hazir_veri[i]=str[i-2];
		}
	}
	
	else if(name=='a'){
	sprintf(str,"%lf", value); 
		int i=0;
		hazir_veri[0]='\n';
		hazir_veri[1]='a';
		for(i=2;i<10;i++){
			hazir_veri[i]=str[i-2];
		}
		
	}
	else if(name=='f'){
	sprintf(str, "%d", (int)value);
		int i=0;
		hazir_veri[0]='\n';
		hazir_veri[1]='f';
		for(i=2;i<10;i++){
			hazir_veri[i]=str[i-2];
		}
		
	}
	
	
	
	return hazir_veri;

} 
