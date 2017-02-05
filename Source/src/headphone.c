/*
NOTE:
SPI1 -> LIS302DL: 3-axis accelerometer
PE3  -> LIS302DL: 3-axis accelerometer
PA5  -> LIS302DL: af
PA6  -> LIS302DL: af
PA7  -> LIS302DL: af 

PA2  -> USART2
PA3  -> USART2

I2S  -> Audio

LED3, LED4, LED5, LED6
*/

#include <main.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stm32f4xx.h>
#include <misc.h>
#include <stm32f4xx_usart.h>

static uint8_t volume = 100;
extern volatile int user_mode;

// UART Declaration
#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
char print_buffer[MAX_STRLEN];
void init_USART2(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USART2_IRQHandler(void);

/*
 * We have three buffers: two output buffers used in a ping-pong arrangement, and an input
 * (microphone) circular buffer. Because the output buffers are written directly to the I2S
 * interface with DMA, they must be stereo. The microphone buffer is mono and its duration
 * is exactly 3 times the length of one of the ping-pong buffers. The idea is that during
 * normal operation, the microphone buffer will vary between about 1/3 full and 2/3 full
 * (with a 1/3 buffer margin on either side).
 */
 
#define SAMPLE_RATE 16000        			// sampling rate
#define OUT_BUFFER_SAMPLES 1024  			// number of samples per output ping-pong buffer
#define PI 							3.14159265

// define for calibrate purpose
#define X_RIGHT 230
#define X_LEFT  152
#define Y_FRONT 107
#define Y_BACK  19

// assign frequency for each note
double C_FREQUENCY 		= 261.626; // C4
double D_FREQUENCY 		= 293.665;
double E_FREQUENCY 		= 329.628;
double F_FREQUENCY 		= 349.228; // F 349.228 
double G_FREQUENCY 		= 391.995;
double A_FREQUENCY 		= 440;
double B_FREQUENCY		=	493.883;
double C5_FREQUENCY		= 523.251;

int enableSetting = 0;

static int16_t buff0 [OUT_BUFFER_SAMPLES], buff1 [OUT_BUFFER_SAMPLES];
static volatile uint8_t next_buff;              // next output buffer to write
// These functions will have different instances depending on the global function selected below
static void fill_buffer (int16_t *buffer, int num_samples);

// 3-axis accelerometer
uint8_t x,y,z;
void mySPI_Init(void);
void mySPI_SendData(uint8_t adress, uint8_t data);
uint8_t mySPI_GetData(uint8_t adress);

// dat received audio from mic
uint16_t dat; 

int frame; 
int current_sample;
int current_note;
int stop_note;
int total_frame 	= 10; // play key for 0.3 second
int total_sample 	= 2 * SAMPLE_RATE;
double samples_per_cycle;

void changeFrame(int increase){
	if(increase == 0){
		total_frame = total_frame/2;
	}else{
		total_frame = total_frame*2;
	}
}

void changePitch(int higher){
	if(higher == 0){
		C_FREQUENCY 		= C_FREQUENCY/2;
		D_FREQUENCY 		= D_FREQUENCY/2;
		E_FREQUENCY 		= E_FREQUENCY/2;
		F_FREQUENCY 		= F_FREQUENCY/2;
		G_FREQUENCY 		= G_FREQUENCY/2;
		A_FREQUENCY 		= A_FREQUENCY/2;
		B_FREQUENCY		  =	B_FREQUENCY/2;
		C5_FREQUENCY		= C5_FREQUENCY/2;
	}else{
		C_FREQUENCY 		= C_FREQUENCY*2;
		D_FREQUENCY 		= D_FREQUENCY*2;
		E_FREQUENCY 		= E_FREQUENCY*2;
		F_FREQUENCY 		= F_FREQUENCY*2;
		G_FREQUENCY 		= G_FREQUENCY*2;
		A_FREQUENCY 		= A_FREQUENCY*2;
		B_FREQUENCY		  =	B_FREQUENCY*2;
		C5_FREQUENCY		= C5_FREQUENCY*2;
	}
}


void WaveRecorderCallback (int16_t *buffer, int num_samples)
{
    int i;
		// start callback green led3 on
		//STM_EVAL_LEDOn(LED4);
    for (i = 0; i < num_samples; ++i) {
        int16_t sample = *buffer++;
        if (sample <= 32700 || sample >= -32700){
						if(sample < 0) { // make all sample to be positive
							sample = -sample;
						}
						dat = sample;
				}
        //micbuff [mic_head + i] = sample;
    }
		
    //mic_head = (mic_head + num_samples >= MIC_BUFFER_SAMPLES) ? 0 : mic_head + num_samples;
    

}
/****************************************************************************/
/****************************************************************************/
/*******//* ALL THREE PROBLEM ARE BEING INITIALISED AND CONFIGURED*/ /*******/
/****************************************************************************/
/****************************************************************************/
int kx_r=0; int ky_r=0;
int kx_l=0; int ky_l=0;
int kData=0;
void checkXY(int x,int y,int data){

//------------------------Check X-------------------------//
	// if right increase kx
		if(x > X_RIGHT-20 && x < X_RIGHT+20){
			kx_r++;
//			sprintf(sendTemp, "%d ", kx_r);
//			USART_puts(USART2, sendTemp);
			if(kx_r > 30) kx_r = 30;
			if(kx_r == 30){
				changePitch(0);
				kx_r = 0;
				USART_puts(USART2, "\n\rDECREASE the pitch ya!\n\r");
			}
		}		
		// if left increase kx
		if(x > X_LEFT-20 && x < X_LEFT+20){
			kx_l++;
			if(kx_l > 30) kx_l = 30;
			if(kx_l == 30){
				changePitch(1);
				kx_l = 0;
				USART_puts(USART2, "\n\rINCREASE the pitch yo!\n\r");
			}
		}
		// if in normal reset kx
		if(x < 3){
			kx_r = 0;
			kx_l = 0;
		}
		
//------------------------Check Y-------------------------//
		
		// if front increase ky
		if(y > Y_FRONT-20 && y < Y_FRONT+20){
			ky_r++;
			if(ky_r > 22) kx_r = 22;
			if(ky_r == 22){
				changeFrame(0);
				ky_r = 0;
				USART_puts(USART2, "\n\rDECREASE the frame hoo!\n\r");
			}
		}		
		// if back increase ky
		if(y > Y_BACK-2 && y < Y_BACK+2){
			ky_l++;
			if(ky_l > 22) ky_l = 22;
			if(ky_l == 22){
				changeFrame(1);
				ky_l = 0;
				USART_puts(USART2, "\n\rINCREASE the frame haa!\n\r");
			}
		}
		// if in normal reset ky
		if(y < 3){
			ky_r = 0;
			ky_l = 0;
		}
//------------------------Check data-------------------------//	
			if(data == 32700){
				STM_EVAL_LEDToggle(LED4);
				kData++;
				data =0;
				//reset to default
				if(kData == 10){
					kData =0;
					USART_puts(USART2, "\n\rAll values are reset!\n\r");
					C_FREQUENCY 		= 261.626;
					D_FREQUENCY 		= 293.665;
					E_FREQUENCY 		= 329.628;
					F_FREQUENCY 		= 349.228;
					G_FREQUENCY 		= 391.995;
					A_FREQUENCY 		= 440;
					B_FREQUENCY		=	493.883;
					C5_FREQUENCY		= 523.251;
					total_frame = 10;
				}
			}
		
}

void WavePlayBack(uint32_t AudioFreq) 
{ 
	// audio can be played Red LED5 on
	STM_EVAL_LEDOn(LED5);
	
	/* First, we start sampling internal microphone */
  WaveRecorderBeginSampling ();
	
  /* Initialize wave player (Codec, DMA, I2C) */
  WavePlayerInit(SAMPLE_RATE);
  
	/* Initialize USART2 with 9600 baud rate */
	init_USART2(9600); 
	
	/* SPI Declaration and GPIO configuration */
	SPI_DeInit(SPI1);
	mySPI_Init();
	
	// Initial LIS302D
	mySPI_SendData(0x20, 0x67);
	
	USART_puts(USART2, "\n\n\n\r");
	
  /* Start audio playback on the first buffer (which is all zeros now) */
  Audio_MAL_Play((uint32_t)buff0, OUT_BUFFER_SAMPLES * 2);
  next_buff = 1;
	
	/* This is the main loop of the program. We simply wait for a buffer to be exhausted
   * and then we refill it. The callback (which is triggered by DMA completion) actually
   * handles starting the next buffer playing, so we don't need to be worried about that
   * latency here. The functionality of the fill_buffer() function determines what it is
   * that we are doing (e.g., playing tones, echoing the mic, etc.)
   */
	 
	char send_data[100];
	char send_dataX[20];
	char send_dataY[20];
	char send_dataZ[20];
	char sendTemp[30];
	int j=0,temp =0,i=0;
	
  while (1) {
    while (next_buff == 0);
		fill_buffer (buff0, OUT_BUFFER_SAMPLES);
		
		while (next_buff == 1);
    fill_buffer (buff1, OUT_BUFFER_SAMPLES);
		
		x = mySPI_GetData(0x29);
		y = mySPI_GetData(0x2B);
		z = mySPI_GetData(0x2D);
		
		// calibrate the value of X, Y, Z
		x = x-253;
		y = y-128;
		
		i = (i+1)%500;
		j = (j+1)%200;
		
		checkXY(x,y,dat);
		
		temp = dat;
		sprintf(send_data, "%5d ", temp);
		sprintf(send_dataX, "x: %d, ", x);
		sprintf(send_dataY, "y: %d, ", y);
		sprintf(send_dataZ, "z: %d\n\r",z);
		// print x y z
//			USART_puts(USART2, send_dataX);
//			USART_puts(USART2, send_dataY);
//			USART_puts(USART2, send_dataZ);
		
		
		// print loudness
//			USART_puts(USART2, send_data); 
		
		
		
		// Toggle RED LED while running
		if(j%4 == 0){
			STM_EVAL_LEDToggle(LED5);
		}
  }
}

/****************************************************************************/
/****************************************************************************/
/****//* ALL THREE PROBLEM ARE BEING INITIALISED AND CONFIGURED ABOVE*/ /****/
/****************************************************************************/
/****************************************************************************/

int WavePlayerInit(uint32_t AudioFreq)
{ 
  /* Initialize I2S interface */  
  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);
  
  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  EVAL_AUDIO_Init(OUTPUT_DEVICE_AUTO, volume, AudioFreq );  
  
  return 0;
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
  if (next_buff == 0) {
    Audio_MAL_Play((uint32_t)buff0, OUT_BUFFER_SAMPLES * 2);
    next_buff = 1; 
  }
  else {
    Audio_MAL_Play((uint32_t)buff1, OUT_BUFFER_SAMPLES * 2);
    next_buff = 0; 
  }
}
 
void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size)
{  
}

void EVAL_AUDIO_Error_CallBack(void* pData)
{
  while (1) /* Stop the program with an infinite loop */
  {} /* could also generate a system reset to recover from the error */
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
  return 0;
}

static void fill_buffer (int16_t *buffer, int num_samples)
{
	int count = num_samples / 2;
	int16_t amplitude = 8000;
	int16_t _buffer;
	
	//dsp(micbuff + mic_tail, count);
	
	// make mono to stereo here
  while (count--) {
		if(frame > 0) {
			/* Generate amplitude of current note using sine wave */
			double sample_value = sin(current_sample / samples_per_cycle * 2 * PI) * amplitude;
			
			_buffer = (int16_t)sample_value;
			current_sample++;
			if(current_sample >= samples_per_cycle){
				current_sample = 0;
			}
		} else {
			_buffer = 0;
		}
		*buffer++ = _buffer;
		*buffer++ = _buffer;
  }
	
	if(frame > 0)
		frame--;
}


/* This funcion initializes the USART2 peripheral
 * 
 * Arguments: baudrate --> the baudrate at which the USART is 
 * 						   supposed to operate
 */
void init_USART2(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
	/* enable APB2 peripheral clock for USART 
	 * note that only USART and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART, PB2 for TX and PB3 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART2 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 2(TX) and 3(RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART_IRQHandler() function
	 * if the USART receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART peripheral
	USART_Cmd(USART2, ENABLE);
}

/* This function is used to transmit a string of characters via 
 * the USART specified in USARTx.
 * 
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 * 
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 * 
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){
	
	

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}

// this is the interrupt request handler (IRQ) for ALL USART interrupts
void USART2_IRQHandler(void){
	
	// check if the USART receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
		char t = USART2->DR; // the character from the USART data register is saved in t
		
		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
		
		// change samples_per_cycle while receive input
		if (t == 'a') {
			samples_per_cycle = SAMPLE_RATE / C_FREQUENCY;
			USART_puts(USART2, "C4 ");
		} else if (t == 's') {
			samples_per_cycle = SAMPLE_RATE / D_FREQUENCY;
			USART_puts(USART2, "D4 ");
		} else if (t == 'd') {
			samples_per_cycle = SAMPLE_RATE / E_FREQUENCY;
			USART_puts(USART2, "E4 ");
		} else if (t == 'f') {
			samples_per_cycle = SAMPLE_RATE / F_FREQUENCY;
			USART_puts(USART2, "F4 ");
		} else if (t == 'g') {
			samples_per_cycle = SAMPLE_RATE / G_FREQUENCY;
			USART_puts(USART2, "G4 ");
		} else if (t == 'h') {
			samples_per_cycle = SAMPLE_RATE / A_FREQUENCY;
			USART_puts(USART2, "A4 ");
		} else if (t == 'j') {
			samples_per_cycle = SAMPLE_RATE / B_FREQUENCY;
			USART_puts(USART2, "B4 ");
		} else if (t == 'k') {
			samples_per_cycle = SAMPLE_RATE / C5_FREQUENCY;
			USART_puts(USART2, "C5 ");
		} else {
			USART_puts(USART2, "\r\n");
			return ;
		}
		
		// initail value for note playing
		frame = total_frame; 
		current_sample = 0;
	}
}

void _SPI_Init(void){
//		/* Configure SPI_InitStruct */
		SPI_InitTypeDef* SPI_InitStruct;
//		SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//		SPI_InitStruct->SPI_Mode = SPI_Mode_Master;           
//		SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;        
//		SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;       
//		SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;       
//		SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;  
//		SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
//		SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
//		SPI_InitStruct->SPI_CRCPolynomial = 10;
//		
//		// Initial SPI1
//		SPI_Init(SPI1,SPI_InitStruct);
//		SPI_Cmd(SPI1,ENABLE);
	
	
		SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
		SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
		SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStruct->SPI_CPOL = SPI_CPOL_High;
		SPI_InitStruct->SPI_CPHA = SPI_CPHA_2Edge;
	 
		SPI_Init(SPI1, SPI_InitStruct);
}

void mySPI_SendData(uint8_t adress, uint8_t data){
 
		GPIO_ResetBits(GPIOE, GPIO_Pin_3);
		 
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
		SPI_I2S_SendData(SPI1, adress);
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI_I2S_ReceiveData(SPI1);
		 
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
		SPI_I2S_SendData(SPI1, data);
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI_I2S_ReceiveData(SPI1);
		 
		GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

uint8_t mySPI_GetData(uint8_t adress){
 
			GPIO_ResetBits(GPIOE, GPIO_Pin_3); 
			 
			adress = 0x80 | adress;
			 
			while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
			SPI_I2S_SendData(SPI1, adress);
			while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
			SPI_I2S_ReceiveData(SPI1); //Clear RXNE bit
			 
			while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
			SPI_I2S_SendData(SPI1, 0x00); //Dummy byte to generate clock
			while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
			 
			GPIO_SetBits(GPIOE, GPIO_Pin_3);
			 
			return  SPI_I2S_ReceiveData(SPI1);
}

void mySPI_Init(void){
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitTypeDefStruct;

	SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitTypeDefStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitTypeDefStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitTypeDefStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitTypeDefStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitTypeDefStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitTypeDefStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitTypeDefStruct.SPI_CPHA = SPI_CPHA_2Edge;

	SPI_Init(SPI1, &SPI_InitTypeDefStruct);


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE , ENABLE);

	GPIO_InitTypeDef GPIO_InitTypeDefStruct;

	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitTypeDefStruct);

	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOE, &GPIO_InitTypeDefStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_SetBits(GPIOE, GPIO_Pin_3);


	SPI_Cmd(SPI1, ENABLE);

}