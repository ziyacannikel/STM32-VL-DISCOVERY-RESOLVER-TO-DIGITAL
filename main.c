#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "delay.h"
#include "stdio.h"
#include "functions.h"
#include "math.h"
    #define PI 3.14159265358979323846
		#define oran 8.056640625/10000  // (3.3 v / 4096) sonucu
		#define kucuktme 7.3            // fiziksel olarak adc ye gelen sinyalin küçülme miktari 24/3.3
		#define a1 -(1.9112)            // filtre katsayilari 
		#define a2 0.9150
		#define b1 9.4469/10000
		#define b2 0.0019
		#define SLAVE_ADDRESS    0x28   // dijital pot adresi
		

uint16_t ADC_ValArray[3][3];    // ADC degerleri bu diziye atilacak (DMA DEN GELCEK)
double filtrelenmis_veri[3][3]; // Filtrelenmis adc degerleri
double angle;
char str[10];                   //usart verisini yazmak için

/* ASAGISI USART RECEIVE IÇIN*/
int frekans=0;                  //kullanici emri hesaplanmasi için
int frekans_gercek;             //tam olarak istenen frekans 
int v_ref=0;                    //kullanici emri hesaplanmasi için
int v_ref_gercek;               //tam olarak istenen voltaj
_Bool vstart_control=0;
_Bool fstart_control=0;
/* ---------------------------*/


/* ASAGISI FREKANS VE TEPE DEGER TESPITI IÇIN */
int frekansSayaci;
int dummy=0;
int frekans_adc=0; //EN SON GERCEK FREKANS
double tepe_deger[3];           //v_ref,v_sin,v_cos
/* ---------------------------*/

const uint16_t sineWave12bit[64] = {
	2048,2248,2447,2642,2831,3013,3185,3346,
	3495,3630,3750,3853,3939,4007,4056,4085,
	4095,4085,4056,4007,3939,3853,3750,3630,
	3495,3346,3185,3013,2831,2642,2447,2248,
	2048,1847,1648,1453,1264,1082, 910, 749,
	600 ,465 ,345 ,242 ,156 ,88  ,  39,  10,
	0   ,10  ,39  ,88  ,156 ,242 , 345, 465,
  600 ,749 ,910 ,1082,1264,1453,1648,1847};	


char bolgeTespit (void){
  

	if(filtrelenmis_veri[0][1]>2048)    //sinüs sargisi sinayli  pozitif alternans konumunda
  {
	   if(filtrelenmis_veri[0][2]>2048) //cosinüs sargisi sinayli  pozitif alternans konumunda
		 {
		    return 1;                      //sin +  cos +  1.bölge
		 }
		 else //cosinüs sargisi sinayli  negatif alternans konumunda
		 {
		    return 2;                      //sin +  cos -  1.bölge
		 }
		 
	}
  else//sinüs sargisi sinayli  negatif alternans konumunda
  {
	   if(filtrelenmis_veri[0][2]>2048) //cosinüs sargisi sinayli  pozitif alternans konumunda
		 {
		    return 4;                      //sin -  cos +  4.bölge
		 }
		 else //cosinüs sargisi sinayli  negatif alternans konumunda
		 {
		    return 3;                      //sin -  cos -  3.bölge
		 }
	}		
	

	
}	
double angleDetect(void){
  char bolge;
	 
	bolge = bolgeTespit();
	if(filtrelenmis_veri[0][0]>2048)    // referans sinyali pozitif alternans konumunda
	{
		angle=atan((filtrelenmis_veri[0][1]-2048)/(filtrelenmis_veri[0][2]-2048))*(180.0/PI);
	}
	else
  {
	  angle=atan((-filtrelenmis_veri[0][1]-2048)/(-filtrelenmis_veri[0][2]-2048))*(180.0/PI);
		if(bolge==1)
		{
		  bolge = 3;
		}
		else if(bolge==2)
		{
		  bolge = 4;
		}
		else if(bolge==3)
		{
		  bolge = 1;
		}
		else
    {
		  bolge = 2;
		}
	}
	
	
	
	

 return 5;
}
void ADC1_DMAConfig(void){
	DMA_InitTypeDef DMA_InitStructure;

  DMA_Cmd(DMA1_Channel1,DISABLE);
  DMA_DeInit ( DMA1_Channel1);

  DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)0x4001244C;            // ADC->DR Adresi
  DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t) ADC_ValArray;         // hedef adresimiz
  DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralSRC;           // ADC kaynak. Veri yönü ADC -> Hafiza 
  DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;       // ADC adresi sabit kalacak
  DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;            // Her deger alindigina memory adresi 1 artirilacak
  DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_HalfWord; // Kaynaktan alinacak veri 16 bit
  DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_HalfWord;     // Hedef büyüklügü 16 bit
  DMA_InitStructure.DMA_Mode                  = DMA_Mode_Circular;               // 3 veri alindiktan sonra basa dönülecek.
  DMA_InitStructure.DMA_Priority              = DMA_Priority_High ;              // Kanal Önceligi yüksek
  DMA_InitStructure.DMA_M2M                   = DMA_M2M_Disable;                 // hafizadan hafizaya transfer kapali. 
  DMA_InitStructure.DMA_BufferSize            = 3;                               // Alacagimiz verisayisi 3*3=9 ( 3 kanal adc okuyacagiz) 
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1,ENABLE);
	
	
}

void dizi_doldur(void){
	int i,j=0;
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			filtrelenmis_veri[i][j]=0;
			ADC_ValArray[i][j]=0;
		}
	}
	
}



void led_init(void){
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

}
void gpioinit(void){
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

}

char sayac=0; //USART IÇIN GEREKLI SILME
void USART1_IRQHandler(void){
  
	
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
		   
        if(fstart_control==1 & sayac<4)
				{
					
					  frekans=frekans+(pow(10,(3-sayac))*asc_convert(USART_ReceiveData(USART1)));
						sayac++;
				}  
				if(vstart_control==1)
				{
					
					if(sayac==0) // voltaj ilk gelisi en yuksek degerlikli 
					{
						v_ref=v_ref+(10*asc_convert(USART_ReceiveData(USART1)));
						sayac++;
						
					}
					else if (sayac==1)// voltaj ikinci gelisi
          {
					  sayac++;
					  v_ref=v_ref+asc_convert(USART_ReceiveData(USART1));
					}
				}  
			if(USART_ReceiveData(USART1) ==35) //bitis isareti
        { 
					GPIO_SetBits(GPIOC,GPIO_Pin_9); 
					GPIO_ResetBits(GPIOC,GPIO_Pin_8); 
					vstart_control=0; 
					fstart_control=0; 
					sayac=0;
					if(200 <= frekans & frekans <= 2000)
          {
						frekans_gercek = frekans;
					}
					if(4 <= v_ref & v_ref <= 24)
					{
						v_ref_gercek = v_ref;
					}
				}
				
			if(USART_ReceiveData(USART1) ==118) //v_ref baslangiç isareti
        { 
			
					vstart_control=1; 
					v_ref=0;
				}
      if(USART_ReceiveData(USART1) ==102)   //frekans baslangiç isareti
        { 
					fstart_control=1; 
					frekans=0;					
				}
        
    }
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}



			 

void tepe_frekans(void){//tepe deger ve frekans bulur
	
	  int i=0;
		for(i=0;i<3;i++)
		{
			if(filtrelenmis_veri[1][i]>=filtrelenmis_veri[0][i]  &  filtrelenmis_veri[1][i]>=filtrelenmis_veri[2][i])
			{
				tepe_deger[i]=filtrelenmis_veri[1][i];
			}
		}
	
	 frekans_adc=(int)(6667/frekansSayaci);
		 
	
}



void TIM3_IRQHandler(void){//frequency trigger 10uS
if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
	TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
	ADC1->CR2 |= ((uint32_t)0x00500000);  //adc software start
	
		/* ALTTAKI  PARÇA 5 uS DEN KISA SÜRMELI*/
		/* 24Mhz -> 41 nS bir cycle */
	
    
    filtrelenmis_veri[2][0]=filtrelenmis_veri[1][0];	 //vr2->vr3
		filtrelenmis_veri[2][1]=filtrelenmis_veri[1][1];	 //vs2->vs3
		filtrelenmis_veri[2][2]=filtrelenmis_veri[1][2];   //vc2->vc3
		filtrelenmis_veri[1][0]=filtrelenmis_veri[0][0];   //vr1->vr2
		filtrelenmis_veri[1][1]=filtrelenmis_veri[0][1];   //vs1->vs2
		filtrelenmis_veri[1][2]=filtrelenmis_veri[0][2];   //vc1->vc2
		
		/*FILTRE IMPLEMENTASYON*/
		filtrelenmis_veri[0][0]=(b1*ADC_ValArray[2][0])+(b2*ADC_ValArray[1][0])+(b1*ADC_ValArray[0][0])-(a1*filtrelenmis_veri[1][0])-(a2*filtrelenmis_veri[2][0]);
		filtrelenmis_veri[0][1]=(b1*ADC_ValArray[2][1])+(b2*ADC_ValArray[1][1])+(b1*ADC_ValArray[0][1])-(a1*filtrelenmis_veri[1][1])-(a2*filtrelenmis_veri[2][1]);	
		filtrelenmis_veri[0][2]=(b1*ADC_ValArray[2][2])+(b2*ADC_ValArray[1][2])+(b1*ADC_ValArray[0][2])-(a1*filtrelenmis_veri[1][2])-(a2*filtrelenmis_veri[2][2]);

		ADC_ValArray[2][0]=ADC_ValArray[1][0];  //vr2->vr3
		ADC_ValArray[2][1]=ADC_ValArray[1][1];  //vs2->vs3
		ADC_ValArray[2][2]=ADC_ValArray[1][2];  //vc2->vc3
		ADC_ValArray[1][0]=ADC_ValArray[0][0];  //vr1->vr2
		ADC_ValArray[1][1]=ADC_ValArray[0][1];  //vs1->vs2
	  ADC_ValArray[1][2]=ADC_ValArray[0][2];  //vc1->vc2
	    
		
			
			
  }
}

void sinus(void){	
	int i=0;
   for(i=0;i<64;i++)
	  {
      DAC_SetChannel1Data(DAC_Align_12b_R,sineWave12bit[i]);
	      //delay_ms(3);
			
	 	  tepe_frekans();
			
	     sprintf(str,"\nf%d",frekans_adc);
		   USART_Puts(USART1,str);		
      
	   // USART_Puts(USART1,convert('v',tepe_deger[0]));			
    }
 
}
int main(void){
	
		dizi_doldur();//filtrelenmis veri dizisi ve adc 0 ile dolduruldu
	  //INITLERIN SIRASI ÖNEMLII DEGISTIRME 
	  led_init();
		delay_init(); 
		usart_init();
	  init_adc1();
		ADC1_DMAConfig();
    ADC_DMACmd(ADC1,ENABLE);
		timer_init();
	  i2c_ini();
	  gpioinit();
		while (1)
		{ 
		
			tepe_frekans();
			USART_Puts(USART1,convert('v',tepe_deger[0]));
			USART_Puts(USART1,convert('c',tepe_deger[1]));
			USART_Puts(USART1,convert('s',tepe_deger[2]));    
      USART_Puts(USART1,convert('a',angle));
			USART_Puts(USART1,convert('f',frekans_adc));

			
		}



}
