/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

//Ö±×ß·½Ïò --+-   ÄæÊ±Õë +++-  Ë³Ê±Õë---+  Ô¶Àë+---  ¿¿½ü-+++
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CanTxMsgTypeDef TxMessage; 
CanRxMsgTypeDef RxMessage;

CanTxMsgTypeDef hcan2TxMessage; 
CanRxMsgTypeDef hcan2RxMessage;

CAN_FilterConfTypeDef hcan1filter;
CAN_FilterConfTypeDef hcan2filter;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

int8_t ResetData[8]={0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};          //µç»ú³õÊ¼»¯
int8_t VeloticyModeData[8]={0x03,0x55,0x55,0x55,0x55,0x55,0x55,0x55};   //Ñ¡Ôñ0x03 ËÙ¶ÈÄ£Ê½
int8_t VeloticyData[8]={0x10,0xc4,0x00,0x64,0x55,0x55,0x55,0x55};				//1ºÅµç»úËÙ¶È²ÎÊý
int8_t VeloticyData1[8]={0x10,0xc4,0xFF,0x9c,0x55,0x55,0x55,0x55};			//2ºÅµç»úËÙ¶È²ÎÊý
int8_t VeloticyData2[8]={0x10,0xc4,0xFF,0x9c,0x55,0x55,0x55,0x55};  		//3ºÅµç»úËÙ¶È²ÎÊý
int8_t VeloticyData3[8]={0x10,0xc4,0xFF,0x9c,0x55,0x55,0x55,0x55};			//4ºÅµç»úËÙ¶È²ÎÊý
int8_t LocationModeData[8]={0x04,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
int8_t LocationData[8]={0x05,0x14,0x55,0x55,0xFF,0xFF,0x28,0x90};
int8_t LocationData1[8]={0x04,0x44,0x55,0x55,0xFF,0xFF,0xE8,0x90};
int8_t LimitModeData[8]={0x00,0x01,0x55,0x55,0x55,0x55,0x55,0x55};
int8_t ShootData1[8]={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};					 //·¢Éä·ÉÅÌ
int8_t ShootData2[8]={0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00};					 //·¢Éä·ÉÅÌ
int8_t ShootData3[8]={0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00};					 //·¢Éä·ÉÅÌ
int8_t ShootModeData1[8]={0x0F,0x00,0x55,0x55,0xFF,0xFF,0xE5,0x66};    //µÚÒ»¸öÎ»ÖÃ
int8_t ShootModeData2[8]={0x0F,0x00,0x55,0x55,0xFF,0xFF,0x0C,0x66};    //µÚ¶þ¸öÎ»ÖÃ
int8_t ShootModeData3[8]={0x0F,0x00,0x55,0x55,0xFF,0xFE,0x33,0x66};    //µÚÈý¸öÎ»ÖÃ
int ControlData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};						 //Ò£¿ØÆ÷½ÓÊÕÊý¾Ý
int LimData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};								 //ÏÞÎ»½ÓÊÕÊý¾Ý
int8_t brushlessData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   
int16_t speed_rotate;                                                    //Ðý×ªµ÷ÕûµÄ´óÐ¡
int16_t speed_distance;                                                  //¾àÀëµ÷ÕûµÄ´óÐ¡
int16_t speed_error=0;
int16_t speed_lasterror=0;
int16_t speed_change[4];                                                 //µ÷ÕûºóµÄËÙ¶ÈÊý×é
int16_t speed_init[4]={-5000,-5000,5000,-5000};                          //Éè¶¨ËÙ¶ÈÖµ
int8_t  Encoder_flag_up=0;																							 //±àÂëÆ÷Ç°½ø±êÖ¾Î»
int8_t  Encoder_flag_down=0;																						 //±àÂëÆ÷ºóÍË±êÖ¾Î»
int16_t Angle_x,Angle_y,Angle_z;
uint8_t start_k=0;
uint8_t JYRxBuffer[11]={1};																						  	//ÍÓÂÝÒÇ·µ»ØµÄ½Ç¶ÈÖµ Ç°Á½Î»Îª0x55 0x53
char uart3_flag=0;																												//½ÓÊÕÖÐ¶Ï±êÖ¾
char can1Rx_flag=0;																												//can½ÓÊÜ±êÖ¾Î»
char can2Rx_flag=0;																												//can½ÓÊÜ±êÖ¾Î»
uint16_t AD1=0;																														//¼¤¹â´«¸ÐÆ÷¶ÁÊý
uint16_t AD2=0;                                                        
uint16_t AD_INIT;  																												//³õÊ¼»¯¼¤¹âÖµ
uint8_t ex_flag=0;                                                    		//É«±êÍ¿ÖÐ¶Ï±êÖ¾  ÖÐ¶ÏºóÖÃ1
int16_t angle_init_z=0;                                               		//ÍÓÂÝÒÇZÖá³õÊ¼Öµ 
int LimID = 0;																														//ÏÞÎ»µÄCAN ID
float kp_rotate=3;  //20                                                    		//Ðý×ª±ÈÀý²ÎÊý
float kp_distance=2.5;    //22                                              		//¾àÀëÔ¶½ü±ÈÀý²ÎÊý1.5char Shoot_flag = 0;                                                  		//·¢Éä·ÉÅÌ±êÖ¾Î»
float kd_distance=2.5;    //22                                              		//¾àÀëÔ¶½ü±ÈÀý²ÎÊý1.5char Shoot_flag = 0;                                                  		//·¢Éä·ÉÅÌ±êÖ¾Î»																											//Í£Ö¹±êÖ¾Î»
char Anglim_up = 0; 
char Anglim_down = 0;
char Anglim_normal = 0;
char Poslim_up = 0; 
char Poslim_down = 0;
char Poslim_normal = 0;
uint8_t Pos_last = 0;
uint8_t Ang_last = 0;
int16_t Encoder_data=0;
int32_t position = -2000;
int32_t angle = -800;
uint8_t Receive_Flag=0;                                                   //×Ô¶¯°æ±¾Ò£¿ØÆ÷½ÓÊÕ±êÖ¾
uint8_t Now_state=0;
uint8_t Next_state=0;
uint16_t speed_set=0;
int32_t position_start = -2000;
int32_t position_last = 0;
int32_t angle_start = -800;
int32_t angle_last = 0;
int16_t n;
int32_t angleL = -800;
struct state
{
 uint8_t flag;
 uint8_t data;
}Left,Right,Forward,Backward,Pos_up,Pos_down,Ang_up,Ang_down,Ace_up,Ace_down;

//À¶³¡²ÎÊý
int8_t position_stateB[8]  ={  0,	  0,	166,	249,	166,	  0,	249,	249};		//¶ÔÓ¦×´Ì¬·Ö±ðÎª0 1 2 3 4 5 ½ü Ô¶
int8_t angle_stateB[8]     ={	0,	170,	165 ,	185,	172,	174,	183,	139};
int8_t brushless_stateB[8] ={	0,	128,	128,	132,	129,	130,	 82,	172};
int16_t Counter_Data[8][8]={
														{		 0, 1800, 2450, 2725, 2700, 3150, 1000,   80},
														{	2250,	   0,	 500,	 865,  900, 1300, 3300, 2330},
														{	1650, 3430,		 0,  150,   30,	 700,	2550, 1730},
														{	1350,	3165,	3830,		 0,	3930,  385, 2300, 1590},
														{	1350,	3250,	3900,	  50,		 0,	 400, 2400, 1600},
														{	 990,	2850,	3400, 3675,	3650,		 0, 1950, 1080},
														{	3070,	 800,	1550,	1725,	1700,	2150,    0, 3450},
														{	4050,	1770,	2430,	2470,	2450,	2970, 570,    0}														
													 };


													 
//ºì³¡²ÎÊý											 
int8_t position_state[8]  ={  0,	  0,	166,	249,	166,	  0,	249,	249};		//¶ÔÓ¦×´Ì¬·Ö±ðÎª0 1 2 3 4 5 3½ü 3Ô¶
int8_t angle_state[8]     ={	0,	170,	170,	194,	172,	177,	176,	129};
int8_t brushless_state[8] ={	0,	129,	127,	134,	129,	129,	 82,	165};
int16_t CounterF_Data[8][8]={																															//ºì³¡
														{		 0, 2700, 2150, 2250, 2020, 1250,  500, 3800},
														{	1400,	   0,	3570,	3720, 3450, 2750, 2000, 1020},
														{	2000,  450,		 0,   80,	3970,	3200, 2450, 1650},
														{	1800,	 380, 4000,	   0,	3870,	3200, 2400, 1650},
														{	2100,	 600,	 180,  230,		 0,	3430, 2530, 1760},
														{	2850,	1300,	 950, 1000,	 770,		 0, 3230, 2440},
												  	{	3550,	2050,	1650, 1700,	1520,	 820,    0, 3300},
												   	{	 250,	2950,	2450, 2550,	2320,	1650,  800,    0},													
													 };
enum location_FLAG{ one,three,five }location_flag;  // 135Î»ÖÃÃ¶¾Ù
enum key_FLAG{ ON,OFF,ZERO1}key_flag_s1,key_flag_s2,key_flag_s3,key_flag_s4;									//  s1-s4°²¼ì×´Ì¬Ã¶¾Ù
enum CHOOSE_FLAG{ GO,SHOOT1,SHOOT5,STOP,ZERO }choose_flag;									//Ò£¿Ø×´Ì¬Ã¶¾Ù£ºÔË¶¯£¬·¢Éä£¬Í£Ö¹
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
int Shootcount=0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C2_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void TIM5_IRQHandler(void);  																								//±àÂëÆ÷ÖÐ¶Ï·þÎñ³ÌÐò
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim); 													//±àÂëÆ÷ÖÐ¶Ï·þÎñ³ÌÐò
void stop_speed(void); 																											//Í£³µ
void get_AD1_and_AD2(void);  																								//»ñÈ¡Á½¸ö¼¤¹âÈ¡Öµ

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void MotorVeloticy_Init(void);																				//³õÊ¼»¯º¯Êý
static void MotorLocation_Init(void);
static void SendData(CAN_HandleTypeDef hcan,uint32_t ID,int8_t* Send);			//Ïòµç»ú·¢ËÍÖ¸Áî 
static void speed_transform_10to16(int8_t* Veloticy,int16_t speed);					//½«10½øÖÆËÙ¶ÈËÍµ½ËÙ¶È²ÎÊýÊý×é  Veloticy£ºÊý×é  speed£ºÊ®½øÖÆËÙ¶È
static void speed_send(int16_t x1,int16_t x2,int16_t x3,int16_t x4);               //·¢ËÍµç¼«Ö¸Áî µ×ÅÌËÄ¸öµç»ú×¨ÓÃ 
static void location_transform_10to32(int8_t* Position,int32_t location);   //½«10½øÖÆÎ»ÖÃËÍµ½Î»ÖÃ²ÎÊýÊý×é  Position£ºÊý×é  location£ºÊ®½øÖÆÎ»ÖÃ                              
static void location_send(int32_t x1,int32_t x2);																	//·¢ËÍÎ»ÖÃÖ¸Áî x1£º x2:
static void brushless_speed_transform(int8_t* brushlessData,uint8_t speed);	//µç»úËÙ¶È×ª»»
static void brushless_speed_send(uint8_t speed);														//µç»úËÙ¶ÈÉè¶¨
static void state_movement(uint8_t P,uint8_t AN,uint8_t AC);								//×´Ì¬¶¯×÷
static void speed_0to1_ON();
static void speed_1to0_ON();
static void speed_0to2_ON();
static void speed_0to3_ON();
static void speed_0to4_ON();
static void speed_0to5_ON();
static void speed_2to0_ON();
static void speed_3to0_ON();
static void speed_4to0_ON();
static void speed_5to0_ON();
#define JY901 0
#define ADEN  0 
#define TIAOSHI 0 
#define MAIN 0
#define CAN 0
#define AUTO 1
#define ZHENG 1
#define FAN 0
#define Encode_counter __HAL_TIM_GET_COUNTER(&htim5)
#define Encode_set __HAL_TIM_SET_COUNTER(&htim5, 0x1000)
#define Encode_set1 __HAL_TIM_SET_COUNTER(&htim5, 0x0000)
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
int main(void)
{

	uint16_t i;
	if(MAIN)
	{
  /* USER CODE BEGIN 1 */
	GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_CRC_Init();
  MX_I2C2_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	
	hcan1.pTxMsg=&TxMessage;
	hcan1.pRxMsg=&RxMessage;	
	hcan2.pTxMsg=&hcan2TxMessage;
	hcan2.pRxMsg=&hcan2RxMessage;	
	uint8_t Text[512];																					//sprintfÊä³ö×ª»»
  get_AD1_and_AD2();
	AD_INIT=AD1+AD2;
  MotorVeloticy_Init(); 
	MotorLocation_Init();

	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);

	
  
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */
  
    /**TIM2 GPIO Configuration    
    PB3     ------> TIM2_CH2 
    */
//	GPIO_InitStruct.Pin = GPIO_PIN_3;															
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	kp_distance=1;
	kp_rotate=2;
//Ö§¼ÜÎ»ÖÃ£º-6000 -200600
//·¢ÉäÌ¨½Ç¶È£º-800 -19000
	//location_send(-2000,-800);
	state_movement(position_state[0],angle_state[0],brushless_state[0]);
//speed_send(-1000,-1000,1000,-1000);
//uint16_t i;
//for(i=1000;i<=5000;i+=500)
//{
//     	get_AD1_and_AD2();
//      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
//		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
//			speed_change[0]=-i- speed_rotate + speed_distance;
//			speed_change[1]=-i- speed_rotate - speed_distance;
//			speed_change[2]=i- speed_rotate - speed_distance;
//			speed_change[3]=-i+ speed_rotate - speed_distance;
//		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
//			HAL_Delay(100);
//}
//			SendData(hcan1,0x115,LocationData);
//			SendData(hcan1,0x125,LocationData1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
}
	
	
  if(AUTO)
		{
			key_flag_s1=ZERO1;
		key_flag_s2=ZERO1;
		choose_flag=ZERO;
    Now_state=0;
		GPIO_InitTypeDef GPIO_InitStruct;
		HAL_Init();
		SystemClock_Config();
		MX_GPIO_Init();
		MX_ADC1_Init();
		MX_ADC2_Init();
		MX_ADC3_Init();
		MX_CAN1_Init();
		MX_CAN2_Init();
		MX_CRC_Init();
		MX_I2C2_Init();
		MX_RNG_Init();
		MX_TIM2_Init();
		MX_TIM5_Init();
		MX_USART1_UART_Init();
		MX_USART3_UART_Init();
		
		hcan1.pTxMsg=&TxMessage;
		hcan1.pRxMsg=&RxMessage;	
		hcan2.pTxMsg=&hcan2TxMessage;
		hcan2.pRxMsg=&hcan2RxMessage;	
//		uint8_t Text[512];																					//sprintfÊä³ö×ª»»
		get_AD1_and_AD2();
		AD_INIT=AD1+AD2;
//		printf("ADC1:%d\t",AD1);
//		printf("ADC2:%d\t",AD2);
		MotorVeloticy_Init(); 
		MotorLocation_Init();
		SendData(hcan1,0x135,ShootModeData1);
		HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
		HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
		//location_send(-2000,-800);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
	}
  while(AUTO)
	{
		if(can1Rx_flag==1)
		{ 
//			printf("can1");
//			printf("¡·»ù±¾IDºÅExtId£º%d\n",hcan1.pRxMsg->StdId);
//			printf("¡·½ÓÊÕµ½Êý¾Ý¶Î³¤¶È£º%d\n",hcan1.pRxMsg->DLC);
//			printf("¡·Êý¾Ý¶ÎµÄÄÚÈÝ:Data[0]= 0x%X £¬Data[1]=0x%X , Data[2]= 0x%X £¬Data[3]=0x%X \n",hcan1.pRxMsg->Data[0],hcan1.pRxMsg->Data[1],hcan1.pRxMsg->Data[2],hcan1.pRxMsg->Data[3]);
//			printf("Data[4]= 0x%X £¬Data[5]=0x%X , Data[6]= 0x%X £¬Data[7]=0x%X \n",hcan1.pRxMsg->Data[4],hcan1.pRxMsg->Data[5],hcan1.pRxMsg->Data[6],hcan1.pRxMsg->Data[7]);
			LimID = hcan1.pRxMsg->StdId;
			for(int i=0;i<8;i++)
			{
				LimData[i]=hcan1.pRxMsg->Data[i];
			}
			if(LimID==284)
			{
				if((LimData[0]==0x01)&&(LimData[1]==0x01))
				{
					Poslim_normal = 1;
					Poslim_down = 0;
					Poslim_up =0;
					printf("Pn\n");
				}
				else if((LimData[0]==0x00)&&(LimData[1]==0x01))
				{
					Poslim_normal = 0;
					Poslim_down = 1;
					Poslim_up =0;
					printf("Pd\n");
				}
				else if((LimData[0]==0x01)&&(LimData[1]==0x00))
				{
					Poslim_normal = 0;
					Poslim_down = 0;
					Poslim_up =1;
					printf("Pu\n");
				}
			}
			else if(LimID==300)
			{
				if((LimData[0]==0x01)&&(LimData[1]==0x01))
				{
					Anglim_normal=1;
					Anglim_down=0;
					Anglim_up=0;
					printf("An");
				}
				else if((LimData[0]==0x00)&&(LimData[1]==0x01))
				{
			  	Anglim_normal=0;
					Anglim_down=1;
					Anglim_up=0;
					printf("Ad");
				}
				else if((LimData[0]==0x01)&&(LimData[1]==0x00))
				{
					Anglim_normal=0;
					Anglim_down=0;
					Anglim_up=1;
					printf("Au");
				}
			}
			can1Rx_flag=0;
		}
	  if(can2Rx_flag==1)   //Ò£¿ØÆ÷½ÓÊÕÖÐ¶Ï
		{
			for(int i=0;i<8;i++)
			{
				ControlData[i]=hcan2.pRxMsg->Data[i];
			}
//			printf("can2");
//			printf("¡·»ù±¾IDºÅExtId£º0x%x\n",hcan2.pRxMsg->StdId);
//			printf("¡·½ÓÊÕµ½Êý¾Ý¶Î³¤¶È£º%d\n",hcan2.pRxMsg->DLC);
//			printf("¡·Êý¾Ý¶ÎµÄÄÚÈÝ:Data[0]= 0x%X £¬Data[1]=0x%X , Data[2]= 0x%X £¬Data[3]=0x%X \n",ControlData[0],ControlData[1],ControlData[2],ControlData[3]);
//			printf("Data[4]= 0x%X £¬Data[5]=0x%X , Data[6]= 0x%X £¬Data[7]=0x%X \n",ControlData[4],ControlData[5],ControlData[6],ControlData[7]);
			if((ControlData[0]==0x00)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				if(ControlData[4]==0x01) 
				{
					key_flag_s1=ON;
					Next_state=2;
				}
			else if(ControlData[4]==0x00) 
			{
				Next_state=4;
				key_flag_s1=OFF;
			}
			}
			else if((ControlData[0]==0x01)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				Next_state=3;
				if(ControlData[4]==0x01) key_flag_s1=ON;
			else if(ControlData[4]==0x00) key_flag_s1=OFF;
			}
			else if((ControlData[0]==0x02)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				if(ControlData[4]==0x01) 
				{
					key_flag_s1=ON;
					Next_state=4;
				}
			else if(ControlData[4]==0x00) 
			{
				Next_state=2;
				key_flag_s1=OFF;
			}
			}
			else if((ControlData[0]==0x03)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				Next_state=6;
				if(ControlData[4]==0x01) key_flag_s1=ON;
			else if(ControlData[4]==0x00) key_flag_s1=OFF;
			}
			else if((ControlData[0]==0x04)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				if(ControlData[4]==0x01) 
				{
					key_flag_s1=ON;
					Next_state=1;
				}
			else if(ControlData[4]==0x00) 
			{
				Next_state=5;
				key_flag_s1=OFF;
			}
			}
			else if((ControlData[0]==0x05)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				Next_state=7;
			if(ControlData[4]==0x01&&ControlData[5]==0x00) 
				{
					key_flag_s1=ON;
					key_flag_s2=ZERO1;
				}
				else if(ControlData[4]==0x01&&ControlData[5]==0x01) 
				{
					key_flag_s2=OFF;
					key_flag_s1=ZERO1;
				}
			else if(ControlData[4]==0x00&&ControlData[5]==0x00) 
				{
					key_flag_s1=OFF;
					key_flag_s2=ZERO1;
				}
			else if(ControlData[4]==0x00&&ControlData[5]==0x01)
			{
					key_flag_s2=ON;
				key_flag_s1=ZERO1;
			}				
			}
			else if((ControlData[0]==0x06)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00)) //í??1
			{
				choose_flag=STOP;
				if(ControlData[4]==0x01) key_flag_s1=ON;
			else if(ControlData[4]==0x00) key_flag_s1=OFF;
			}
			else if((ControlData[0]==0x07)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				if(ControlData[4]==0x01) 
				{
					key_flag_s1=ON;
					Next_state=5;
				}
			else if(ControlData[4]==0x00) 
			{
				Next_state=1;
				key_flag_s1=OFF;
			}
			}
			else if((ControlData[0]==0x08)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))  //1·?
			{
				choose_flag=SHOOT1;
				if(ControlData[4]==0x01) key_flag_s1=ON;
			else if(ControlData[4]==0x00) key_flag_s1=OFF;
			}
			else if((ControlData[0]==0x09)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))
			{
				choose_flag=GO;
				Next_state=0;
				if(ControlData[4]==0x01) key_flag_s1=ON;
			else if(ControlData[4]==0x00) key_flag_s1=OFF;
			}
			else if((ControlData[0]==0x0A)&&(ControlData[1]==0x00)&&(ControlData[2]==0x00)&&(ControlData[3]==0x00))  //5·?
			{
				choose_flag=SHOOT5;
				if(ControlData[4]==0x01) key_flag_s1=ON;
			else if(ControlData[4]==0x00) key_flag_s1=OFF;
			}
			can2Rx_flag=0;
	}

  if(key_flag_s1==ON)		//À¶³¡
	{
	if(choose_flag==GO)
	{
		angle=angleL;
		location_send(position,angle);
	if(Now_state==0)       //Æðµã
	{
	
		if(Next_state==1)
		{ 
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=2500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=1)||(Encode_counter<=1500))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]= 2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-1500- speed_rotate + speed_distance;
				speed_change[1]=-1500- speed_rotate - speed_distance;
				speed_change[2]= 1500- speed_rotate - speed_distance;
				speed_change[3]=-1500+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[0][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-Counter_Data[0][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
    for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=5)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=4)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==5)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[0][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
    for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=7)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=6)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==7)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[0][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
			HAL_Delay(100);
			state_movement(position_stateB[5],angle_stateB[0],brushless_stateB[5]);
			TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
     for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=9)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=8)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==9)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==9)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==9)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[0][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
	  else if(Next_state==6)
		{
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=6)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=5)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==6)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[0][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=5)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=4))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter<1000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>=1000)&&(Encode_counter<=2500))
			{
				speed_change[0]=-1500- speed_rotate + speed_distance;
				speed_change[1]=-1500- speed_rotate - speed_distance;
				speed_change[2]=1500- speed_rotate - speed_distance;
				speed_change[3]=-1500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>=2500))
			{
				speed_change[0]=-1000- speed_rotate + speed_distance;
				speed_change[1]=-1000- speed_rotate - speed_distance;
				speed_change[2]=1000- speed_rotate - speed_distance;
				speed_change[3]=-1000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[0][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	
	}
	else if(Now_state==1)   //Ä¿±ê1
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
	get_AD1_and_AD2();
	while(__fabs(AD1-AD2)>25)
{
	get_AD1_and_AD2();
}  //3  
		Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
    for(i=1000;i<=2500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==1)))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=1500- speed_rotate + speed_distance;
				speed_change[1]=1500- speed_rotate - speed_distance;
				speed_change[2]=-1500- speed_rotate - speed_distance;
				speed_change[3]=1500+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
		while(__fabs(Encode_counter-Counter_Data[1][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==2)
		{
						speed_send(-100,-100,-100,100);
	get_AD1_and_AD2();
	while(__fabs(AD1-AD2)>25)
{
	get_AD1_and_AD2();
}  //3  
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();
		while(__fabs(Encode_counter-Counter_Data[1][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;	
		Now_state=2;
		}
		else if(Next_state==3)
		{
		speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}  //3  
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to2_ON();
		while(__fabs(Encode_counter-Counter_Data[1][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
		speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		} 
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to3_ON();
		while(__fabs(Encode_counter-Counter_Data[1][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
		speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}	
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[5],angle_stateB[5],brushless_stateB[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to4_ON();
		while(__fabs(Encode_counter-Counter_Data[1][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to2_ON();
		while(__fabs(Encode_counter-Counter_Data[1][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1400- speed_rotate + speed_distance;
				speed_change[1]=-1400- speed_rotate - speed_distance;
				speed_change[2]=1400- speed_rotate - speed_distance;
				speed_change[3]=-1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[1][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	}
	else if(Now_state==2)   // Ä¿±ê2
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==3)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
		while(__fabs(Encode_counter-Counter_Data[2][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-Counter_Data[2][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();
		while(__fabs(Encode_counter-Counter_Data[2][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
	get_AD1_and_AD2();
	while(__fabs(AD1-AD2)>25)
{
	get_AD1_and_AD2();
}  //3  
		Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==4)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>1500)&&(Encode_counter<=2500))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>=2500))
			{
				speed_change[0]=-1400- speed_rotate + speed_distance;
				speed_change[1]=-1400- speed_rotate - speed_distance;
				speed_change[2]=1400- speed_rotate - speed_distance;
				speed_change[3]=-1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[2][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[5],angle_stateB[5],brushless_stateB[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to3_ON();
		while(__fabs(Encode_counter-Counter_Data[2][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
	  else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();
		while(__fabs(Encode_counter-Counter_Data[2][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>1500)&&(Encode_counter<=2500))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=2500))
			{
				speed_change[0]=-1400- speed_rotate + speed_distance;
				speed_change[1]=-1400- speed_rotate - speed_distance;
				speed_change[2]=1400- speed_rotate - speed_distance;
				speed_change[3]=-1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[2][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	}
	else if(Now_state==3)		//  Ä¿±ê3
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=5)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=4)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==5)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
		while(__fabs(Encode_counter-Counter_Data[3][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_2to0_ON();
		while(__fabs(Encode_counter-Counter_Data[3][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-Counter_Data[3][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1800- speed_rotate + speed_distance;
				speed_change[1]=-1800- speed_rotate - speed_distance;
				speed_change[2]=1800- speed_rotate - speed_distance;
				speed_change[3]=-1800+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[3][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1300- speed_rotate + speed_distance;
			speed_change[1]=-1300- speed_rotate - speed_distance;
			speed_change[2]=1300- speed_rotate - speed_distance;
			speed_change[3]=-1300+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
  	}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[5],angle_stateB[5],brushless_stateB[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to2_ON();
		while(__fabs(Encode_counter-Counter_Data[3][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
	  else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-Counter_Data[3][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(1000,1000,1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-Counter_Data[3][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	}
	else if(Now_state==4)		//	Ä¿±ê4
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=7)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=6)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==7)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-Counter_Data[4][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_3to0_ON();
		while(__fabs(Encode_counter-Counter_Data[4][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_2to0_ON();
		while(__fabs(Encode_counter-Counter_Data[4][2])>15)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=3500- speed_rotate + speed_distance;
				speed_change[1]=3500- speed_rotate - speed_distance;
				speed_change[2]=-3500- speed_rotate - speed_distance;
				speed_change[3]=3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=1000))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=1000))
			{
				speed_change[0]=1800- speed_rotate + speed_distance;
				speed_change[1]=1800- speed_rotate - speed_distance;
				speed_change[2]=-1800- speed_rotate - speed_distance;
				speed_change[3]=1800+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-Counter_Data[4][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1300- speed_rotate + speed_distance;
			speed_change[1]=1300- speed_rotate - speed_distance;
			speed_change[2]=-1300- speed_rotate - speed_distance;
			speed_change[3]=1300+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[5],angle_stateB[5],brushless_stateB[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();
		while(__fabs(Encode_counter-Counter_Data[4][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0; 
		Now_state=5;
		}
    else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï	
			for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=3500- speed_rotate + speed_distance;
				speed_change[1]=3500- speed_rotate - speed_distance;
				speed_change[2]=-3500- speed_rotate - speed_distance;
				speed_change[3]=3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=1000))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=1000))
			{
				speed_change[0]=1800- speed_rotate + speed_distance;
				speed_change[1]=1800- speed_rotate - speed_distance;
				speed_change[2]=-1800- speed_rotate - speed_distance;
				speed_change[3]=1800+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-Counter_Data[4][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
	 else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  speed_1to0_ON();	
		while(__fabs(Encode_counter-Counter_Data[4][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	
	}
	else if(Now_state==5)		//	Ä¿±ê5
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=10000;i+=1000)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + 4*speed_distance;
			speed_change[1]=i- speed_rotate - 4*speed_distance;
			speed_change[2]=-i- speed_rotate - 4*speed_distance;
			speed_change[3]=i+ speed_rotate - 4*speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=9)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if(Encoder_flag_up+Encoder_flag_down<=7)
			{
				speed_change[0]=10000- speed_rotate + 4*speed_distance;
				speed_change[1]=10000- speed_rotate - 4*speed_distance;
				speed_change[2]=-10000- speed_rotate - 4*speed_distance;
				speed_change[3]=10000+ speed_rotate - 4*speed_distance;
			}
			else if((Encoder_flag_up+Encoder_flag_down)==8)
			{
				speed_change[0]=5000- speed_rotate + 2*speed_distance;
				speed_change[1]=5000- speed_rotate - 2*speed_distance;
				speed_change[2]=-5000- speed_rotate - 2*speed_distance;
				speed_change[3]=5000+ speed_rotate - 2*speed_distance;
			}
			else if((Encoder_flag_up+Encoder_flag_down)==9)
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[5][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_4to0_ON();
		while(__fabs(Encode_counter-Counter_Data[5][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_3to0_ON();
		while(__fabs(Encode_counter-Counter_Data[5][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_2to0_ON();
		while(__fabs(Encode_counter-Counter_Data[5][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
		Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();
		while(__fabs(Encode_counter-Counter_Data[5][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}

    else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);		
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=3500- speed_rotate + speed_distance;
				speed_change[1]=3500- speed_rotate - speed_distance;
				speed_change[2]=-3500- speed_rotate - speed_distance;
				speed_change[3]=3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>=1000))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter<=1000))
			{
				speed_change[0]=1800- speed_rotate + speed_distance;
				speed_change[1]=1800- speed_rotate - speed_distance;
				speed_change[2]=-1800- speed_rotate - speed_distance;
				speed_change[3]=1800+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-Counter_Data[5][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_2to0_ON();
		while(__fabs(Encode_counter-Counter_Data[5][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	
	}
	else if(Now_state==6)		//  Ä¿±ê3
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=6)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=5)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==6)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
		while(__fabs(Encode_counter-Counter_Data[6][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_2to0_ON();
		while(__fabs(Encode_counter-Counter_Data[6][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-Counter_Data[6][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-Counter_Data[6][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1800- speed_rotate + speed_distance;
				speed_change[1]=-1800- speed_rotate - speed_distance;
				speed_change[2]=1800- speed_rotate - speed_distance;
				speed_change[3]=-1800+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[6][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
  	}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[5],angle_stateB[5],brushless_stateB[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1800- speed_rotate + speed_distance;
				speed_change[1]=-1800- speed_rotate - speed_distance;
				speed_change[2]=1800- speed_rotate - speed_distance;
				speed_change[3]=-1800+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[6][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
	
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[7],angle_stateB[7],brushless_stateB[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
					while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=1200- speed_rotate + speed_distance;
				speed_change[1]=1200- speed_rotate - speed_distance;
				speed_change[2]=-1200- speed_rotate - speed_distance;
				speed_change[3]=1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-Counter_Data[6][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	}
	else if(Now_state==7)		//  Ä¿±ê3
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[0],angle_stateB[0],brushless_stateB[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=5)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=4)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==5)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>2000)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter>1000)&&(Encode_counter<=2000))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==5)&&(Encode_counter<=1000))
			{
				speed_change[0]=1000- speed_rotate + speed_distance;
				speed_change[1]=1000- speed_rotate - speed_distance;
				speed_change[2]=-1000- speed_rotate - speed_distance;
				speed_change[3]=1000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[7][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_stateB[1],angle_stateB[1],brushless_stateB[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter>=3000)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=3500- speed_rotate + speed_distance;
				speed_change[1]=3500- speed_rotate - speed_distance;
				speed_change[2]=-3500- speed_rotate - speed_distance;
				speed_change[3]=3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter<=1500))
			{
				speed_change[0]=1400- speed_rotate + speed_distance;
				speed_change[1]=1400- speed_rotate - speed_distance;
				speed_change[2]=-1400- speed_rotate - speed_distance;
				speed_change[3]=1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[7][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_stateB[2],angle_stateB[2],brushless_stateB[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=1400- speed_rotate + speed_distance;
				speed_change[1]=1400- speed_rotate - speed_distance;
				speed_change[2]=-1400- speed_rotate - speed_distance;
				speed_change[3]=1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[7][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[3],angle_stateB[3],brushless_stateB[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-Counter_Data[7][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[4],angle_stateB[4],brushless_stateB[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();
		while(__fabs(Encode_counter-Counter_Data[7][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
  	}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[5],angle_stateB[5],brushless_stateB[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to2_ON();
		while(__fabs(Encode_counter-Counter_Data[7][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_stateB[6],angle_stateB[6],brushless_stateB[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1400- speed_rotate + speed_distance;
				speed_change[1]=-1400- speed_rotate - speed_distance;
				speed_change[2]=1400- speed_rotate - speed_distance;
				speed_change[3]=-1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-Counter_Data[7][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		
		
	}
	angleL=angle;
	choose_flag=ZERO;
	}
	else if(choose_flag==SHOOT1)
	{	
		  Shootcount++;
		
    
     if(Shootcount<18)
		 {
			SendData(hcan1,0x212,ShootData1);
			HAL_Delay(500);
			SendData(hcan1,0x211,ShootData1);
			HAL_Delay(500);
		 }
     if(Shootcount>18&&Shootcount<36)
		 {
			SendData(hcan1,0x212,ShootData2);
			HAL_Delay(500);
			SendData(hcan1,0x211,ShootData2);
			HAL_Delay(500);
		 }
		  if(Shootcount>36)
		 {
			SendData(hcan1,0x212,ShootData3);
			HAL_Delay(500);
			SendData(hcan1,0x211,ShootData3);
			HAL_Delay(500);
		 }
		 
		 if(Shootcount == 18)
		 {
			 SendData(hcan1,0x135,ShootModeData2);
		 }
	   if(Shootcount == 36)
		 {
			 SendData(hcan1,0x135,ShootModeData3);
		 }
		/* if(Shootcount == 52)
		 {
			 SendData(hcan1,0x135,ShootModeData1);
		 }
		*/ 
			choose_flag=ZERO;
	}
	else if(choose_flag==SHOOT5)
	{
		  n=n+1;
			angle=angle-71;
			location_send(position,angle);
		 	choose_flag=ZERO;
	}
	else if(choose_flag==STOP)
	{
		n=n-1;
		angle=angle+71;
	  location_send(position,angle);
		choose_flag=ZERO;
	}
	}
  
	if(key_flag_s1==OFF)  //ºì³¡
	{
		if(choose_flag==GO)
	{

		angle=angleL;
		location_send(position,angle);
	if(Now_state==0)       //Æðµã
	{
		if(Next_state==1)
		{
		Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||((Encode_counter>=3500)&&((Encoder_flag_up+Encoder_flag_down)==4)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>2000)&&(Encode_counter<=3500))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter<=2000))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}					
		while(__fabs(Encode_counter-CounterF_Data[0][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=6)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=5)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==6)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}					
		while(__fabs(Encode_counter-CounterF_Data[0][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
    state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=8)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=7)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==8)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}						
		while(__fabs(Encode_counter-CounterF_Data[0][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=10)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=9)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==10)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}					
		while(__fabs(Encode_counter-CounterF_Data[0][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
					for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=12)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=11)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==12)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==12)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==12)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}					
		while(__fabs(Encode_counter-CounterF_Data[0][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=7)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=6)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==7)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}						
		while(__fabs(Encode_counter-CounterF_Data[0][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
	  else if(Next_state==7)
		{
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
	  state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=9)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=8))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==9)&&(Encode_counter>2500))
			{
				speed_change[0]=2800- speed_rotate + speed_distance;
				speed_change[1]=2800- speed_rotate - speed_distance;
				speed_change[2]=-2800- speed_rotate - speed_distance;
				speed_change[3]=2800+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==9)&&(Encode_counter<=2500))
			{
				speed_change[0]=1400- speed_rotate + speed_distance;
				speed_change[1]=1400- speed_rotate - speed_distance;
				speed_change[2]=-1400- speed_rotate - speed_distance;
				speed_change[3]=1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}						
		while(__fabs(Encode_counter-CounterF_Data[0][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		while((__fabs(AD1-AD2)>20)||(__fabs(AD1+AD2-AD_INIT)>30))  //3  6
	{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(3*(AD1-AD2));//30
		  speed_distance= (int16_t)(2*(AD1+AD2-AD_INIT)); //20
			speed_change[0]=- speed_rotate + speed_distance;
			speed_change[1]=- speed_rotate - speed_distance;
			speed_change[2]=- speed_rotate - speed_distance;
			speed_change[3]= speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
	}
			  speed_transform_10to16(VeloticyData,0);
				speed_transform_10to16(VeloticyData1,0);
				speed_transform_10to16(VeloticyData2,0);
				speed_transform_10to16(VeloticyData3,0);
	
				SendData(hcan1,0x014,VeloticyData);	
				SendData(hcan1,0x024,VeloticyData1);
				SendData(hcan1,0x034,VeloticyData2);	
				SendData(hcan1,0x044,VeloticyData3);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	
	}
	else if(Now_state==1)   //Ä¿±ê1
	{
		if(Next_state==0)
		{
			if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
    state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==4)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}					
		while(__fabs(Encode_counter-CounterF_Data[1][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
	  while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
		Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-CounterF_Data[1][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;	
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_2to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[1][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_3to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[1][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_4to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[1][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==3)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
		while(__fabs(Encode_counter-CounterF_Data[1][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==4)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
		while(__fabs(Encode_counter-CounterF_Data[1][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	
	}
	else if(Now_state==2)   // Ä¿±ê2
	{
		if(Next_state==0)
		{
			if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=6)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=5)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==6)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1200- speed_rotate + speed_distance;
				speed_change[1]=-1200- speed_rotate - speed_distance;
				speed_change[2]=1200- speed_rotate - speed_distance;
				speed_change[3]=-1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
		while(__fabs(Encode_counter-CounterF_Data[2][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();	
		while(__fabs(Encode_counter-CounterF_Data[2][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=1200- speed_rotate + speed_distance;
				speed_change[1]=1200- speed_rotate - speed_distance;
				speed_change[2]=-1200- speed_rotate - speed_distance;
				speed_change[3]=1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
		while(__fabs(Encode_counter-CounterF_Data[2][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_2to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[2][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
				speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_3to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[2][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
			while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=1200- speed_rotate + speed_distance;
				speed_change[1]=1200- speed_rotate - speed_distance;
				speed_change[2]=-1200- speed_rotate - speed_distance;
				speed_change[3]=1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-CounterF_Data[2][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	   speed_1to0_ON();	
		while(__fabs(Encode_counter-CounterF_Data[2][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	
	}
	else if(Now_state==3)		//  Ä¿±ê3
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=8)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=7)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==8)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
	
		while(__fabs(Encode_counter-CounterF_Data[3][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to2_ON();
		while(__fabs(Encode_counter-CounterF_Data[3][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=-1500- speed_rotate + speed_distance;
				speed_change[1]=-1500- speed_rotate - speed_distance;
				speed_change[2]=1500- speed_rotate - speed_distance;
				speed_change[3]=-1500+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-CounterF_Data[3][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
				speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-CounterF_Data[3][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_2to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[3][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï		
		while(__fabs(Encode_counter-CounterF_Data[3][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}	
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-CounterF_Data[3][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	}
	else if(Now_state==4)		//	Ä¿±ê4
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
				speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=10)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=9)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==10)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
		while(__fabs(Encode_counter-CounterF_Data[4][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
				speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to3_ON();
		while(__fabs(Encode_counter-CounterF_Data[4][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
				speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to2_ON();
		while(__fabs(Encode_counter-CounterF_Data[4][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();	
		while(__fabs(Encode_counter-CounterF_Data[4][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
				speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-CounterF_Data[4][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();	
		while(__fabs(Encode_counter-CounterF_Data[4][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=-1200- speed_rotate + speed_distance;
				speed_change[1]=-1200- speed_rotate - speed_distance;
				speed_change[2]=1200- speed_rotate - speed_distance;
				speed_change[3]=-1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-CounterF_Data[4][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
		
	}
	else if(Now_state==5)		//	Ä¿±ê5
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
							
		for(i=1000;i<=10000;i+=1000)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + 4*speed_distance;
			speed_change[1]=-i- speed_rotate - 4*speed_distance;
			speed_change[2]=i- speed_rotate - 4*speed_distance;
			speed_change[3]=-i+ speed_rotate - 4*speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=12)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if(Encoder_flag_up+Encoder_flag_down<=9)
			{
				speed_change[0]=-10000- speed_rotate + 4*speed_distance;
				speed_change[1]=-10000- speed_rotate - 4*speed_distance;
				speed_change[2]=10000- speed_rotate - 4*speed_distance;
				speed_change[3]=-10000+ speed_rotate - 4*speed_distance;
			}
			else if((Encoder_flag_up+Encoder_flag_down)==10)
			{
				speed_change[0]=-6000- speed_rotate + 2*speed_distance;
				speed_change[1]=-6000- speed_rotate - 2*speed_distance;
				speed_change[2]=6000- speed_rotate - 2*speed_distance;
				speed_change[3]=-6000+ speed_rotate - 2*speed_distance;
			}
			else if((Encoder_flag_up+Encoder_flag_down)==11)
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-1500- speed_rotate + speed_distance;
				speed_change[1]=-1500- speed_rotate - speed_distance;
				speed_change[2]=1500- speed_rotate - speed_distance;
				speed_change[3]=-1500+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
	
		while(__fabs(Encode_counter-CounterF_Data[5][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to4_ON();
		while(__fabs(Encode_counter-CounterF_Data[5][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to3_ON();
		while(__fabs(Encode_counter-CounterF_Data[5][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to2_ON();
		while(__fabs(Encode_counter-CounterF_Data[5][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();		
		while(__fabs(Encode_counter-CounterF_Data[5][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}

		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		speed_0to2_ON();
		while(__fabs(Encode_counter-CounterF_Data[5][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
		while(__fabs(Encode_counter-CounterF_Data[5][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	}
	else if(Now_state==6)		//  Ä¿±ê3
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=7)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=6)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==7)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==7)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
	
		while(__fabs(Encode_counter-CounterF_Data[6][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-CounterF_Data[6][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		 while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=-1200- speed_rotate + speed_distance;
				speed_change[1]=-1200- speed_rotate - speed_distance;
				speed_change[2]=1200- speed_rotate - speed_distance;
				speed_change[3]=-1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
		while(__fabs(Encode_counter-CounterF_Data[6][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-CounterF_Data[6][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}	
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
				speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_1to0_ON();	
		while(__fabs(Encode_counter-CounterF_Data[6][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_2to0_ON();
		while(__fabs(Encode_counter-CounterF_Data[6][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		
		else if(Next_state==7)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[7],angle_state[7],brushless_state[7]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
			 while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=1200- speed_rotate + speed_distance;
				speed_change[1]=1200- speed_rotate - speed_distance;
				speed_change[2]=-1200- speed_rotate - speed_distance;
				speed_change[3]=1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-CounterF_Data[6][7])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=7;
		}
	}
	else if(Now_state==7)		//  Ä¿±ê3
	{
		if(Next_state==0)
		{
				if(Shootcount>=50)
			SendData(hcan1,0x135,ShootModeData1);
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[0],angle_state[0],brushless_state[0]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=9)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=8)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==9)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==9)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==9)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
	
		while(__fabs(Encode_counter-CounterF_Data[7][0])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=0;
		}
		else if(Next_state==1)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[1],angle_state[1],brushless_state[1]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==4)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-CounterF_Data[7][1])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=1;
		}
		else if(Next_state==2)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[2],angle_state[2],brushless_state[2]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		speed_0to1_ON();	
		while(__fabs(Encode_counter-CounterF_Data[7][2])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);

		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=2;
		}
		else if(Next_state==3)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[3],angle_state[3],brushless_state[3]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		while(__fabs(Encode_counter-CounterF_Data[7][3])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=3;
		}
		else if(Next_state==4)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
				speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[4],angle_state[4],brushless_state[4]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		 while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter>=2500)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter<=2500))
			{
				speed_change[0]=1200- speed_rotate + speed_distance;
				speed_change[1]=1200- speed_rotate - speed_distance;
				speed_change[2]=-1200- speed_rotate - speed_distance;
				speed_change[3]=1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-CounterF_Data[7][4])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=4;
		}
		else if(Next_state==5)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set1;
			speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		state_movement(position_state[5],angle_state[5],brushless_state[5]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=3)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=2)||((Encode_counter>=3000)&&(Encoder_flag_up+Encoder_flag_down==3)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==3)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
		while(__fabs(Encode_counter-CounterF_Data[7][5])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=1000- speed_rotate + speed_distance;
			speed_change[1]=1000- speed_rotate - speed_distance;
			speed_change[2]=-1000- speed_rotate - speed_distance;
			speed_change[3]=1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		while(Poslim_down)
		{
			position+=-10;
			location_send(position,angle);
		}
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=5;
		}
		else if(Next_state==6)
		{
			speed_send(-100,-100,-100,100);
		get_AD1_and_AD2();
		while(__fabs(AD1-AD2)>25)
		{
				get_AD1_and_AD2();
		}
			Encode_set;
			speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		state_movement(position_state[6],angle_state[6],brushless_state[6]);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
			 while(Encoder_flag_up+Encoder_flag_down<=1)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=0)||((Encode_counter<=2000)&&(Encoder_flag_up+Encoder_flag_down==1)))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==1)&&(Encode_counter>=2000))
			{
				speed_change[0]=-1200- speed_rotate + speed_distance;
				speed_change[1]=-1200- speed_rotate - speed_distance;
				speed_change[2]=1200- speed_rotate - speed_distance;
				speed_change[3]=-1200+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		while(__fabs(Encode_counter-CounterF_Data[7][6])>25)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-1000- speed_rotate + speed_distance;
			speed_change[1]=-1000- speed_rotate - speed_distance;
			speed_change[2]=1000- speed_rotate - speed_distance;
			speed_change[3]=-1000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
		Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		Now_state=6;
		}	
		
	}
	
	angleL=angle;
	choose_flag=ZERO;
}
	else if(choose_flag==SHOOT1)
	{	
		
	   Shootcount++;
     
     if(Shootcount<18)
		 {
			SendData(hcan1,0x212,ShootData1);
			HAL_Delay(500);
			SendData(hcan1,0x211,ShootData1);
			HAL_Delay(500);
		 }
     if(Shootcount>18&&Shootcount<36)
		 {
			SendData(hcan1,0x212,ShootData2);
			HAL_Delay(500);
			SendData(hcan1,0x211,ShootData2);
			HAL_Delay(500);
		 }
		  if(Shootcount>36)
		 {
			SendData(hcan1,0x212,ShootData3);
			HAL_Delay(500);
			SendData(hcan1,0x211,ShootData3);
			HAL_Delay(500);
		 }
		 
		 if(Shootcount == 18)
		 {
			 SendData(hcan1,0x135,ShootModeData2);
		 }
	   if(Shootcount == 36)
		 {
			 SendData(hcan1,0x135,ShootModeData3);
		 }
		/* if(Shootcount == 52)
		 {
			 SendData(hcan1,0x135,ShootModeData1);
		 }*/
		 
		  choose_flag=ZERO;
	}
	else if(choose_flag==SHOOT5)
	{   
			n=n+1;
			angle=angle-71;
			location_send(position,angle);
		 	choose_flag=ZERO;
	}
	else if(choose_flag==STOP)
	{
		n=n-1;
		angle=angle+71;
	  location_send(position,angle);
		choose_flag=ZERO;
	}
}
 if(key_flag_s2==ON)   //ºì³¡Æð²½
	{
		start_k =0;
		speed_send(-1000,-1000,1000,-1000);
		HAL_Delay(100);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  for(i=1000;i<=20000;i+=1000)
		{
			start_k++;
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + start_k/3*speed_distance;
			speed_change[1]=-i- speed_rotate - start_k/3*speed_distance;
			speed_change[2]=i- speed_rotate - start_k/3*speed_distance;
			speed_change[3]=-i+ speed_rotate - start_k/3*speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}	
		while(Encoder_flag_up+Encoder_flag_down<=13)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down)<=10)
			{
				speed_change[0]=-15000- speed_rotate + 4*speed_distance;
				speed_change[1]=-15000- speed_rotate - 4*speed_distance;
				speed_change[2]=15000- speed_rotate - 4*speed_distance;
				speed_change[3]=-15000+ speed_rotate - 4*speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==11))
			{
				speed_change[0]=-10000- speed_rotate + 3*speed_distance;
				speed_change[1]=-10000- speed_rotate - 3*speed_distance;
				speed_change[2]=10000- speed_rotate - 3*speed_distance;
				speed_change[3]=-10000+ speed_rotate - 3*speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==12))
			{
				speed_change[0]=-6000- speed_rotate + speed_distance;
				speed_change[1]=-6000- speed_rotate - speed_distance;
				speed_change[2]=6000- speed_rotate - speed_distance;
				speed_change[3]=-6000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}		
	while(__fabs(Encode_counter-4050)>40)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-2000- speed_rotate + speed_distance;
			speed_change[1]=-2000- speed_rotate - speed_distance;
			speed_change[2]=2000- speed_rotate - speed_distance;
			speed_change[3]=-2000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
	  Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
	key_flag_s2=ZERO1;
	}
	if(key_flag_s2==OFF)  //À¶³¡Æð²½
	{
		start_k=0;
		speed_send(1000,1000,-1000,1000);
		HAL_Delay(100);
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
	  for(i=1000;i<=20000;i+=1000)
		{
			start_k++;
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		//	speed_error=(int16_t)(kp_distance*(AD1+AD2-AD_INIT));
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + start_k/3*speed_distance;
			speed_change[1]=i- speed_rotate - start_k/3*speed_distance;
			speed_change[2]=-i- speed_rotate - start_k/3*speed_distance;
			speed_change[3]=i+ speed_rotate - start_k/3*speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
			speed_lasterror=speed_error;
		}	
		while(Encoder_flag_up+Encoder_flag_down<=10)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down)<=10)
			{
				speed_change[0]=15000- speed_rotate + 5*speed_distance;
				speed_change[1]=15000- speed_rotate - 5*speed_distance;
				speed_change[2]=-15000- speed_rotate - 5*speed_distance;
				speed_change[3]=15000+ speed_rotate - 5*speed_distance;
			}
//			else if(((Encoder_flag_up+Encoder_flag_down)==11))
//			{
//				speed_change[0]=12000- speed_rotate + 2*speed_distance;
//				speed_change[1]=12000- speed_rotate - 2*speed_distance;
//				speed_change[2]=-12000- speed_rotate - 2*speed_distance;
//				speed_change[3]=12000+ speed_rotate - 2*speed_distance;
//			}
//			else if(((Encoder_flag_up+Encoder_flag_down)==12))
//			{
//				speed_change[0]=6000- speed_rotate + speed_distance;
//				speed_change[1]=6000- speed_rotate - speed_distance;
//				speed_change[2]=-6000- speed_rotate - speed_distance;
//				speed_change[3]=6000+ speed_rotate - speed_distance;
//			}
//			else
//			{
//				speed_change[0]=3000- speed_rotate + speed_distance;
//				speed_change[1]=3000- speed_rotate - speed_distance;
//				speed_change[2]=-3000- speed_rotate - speed_distance;
//				speed_change[3]=3000+ speed_rotate - speed_distance;
//			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			speed_lasterror=speed_error;
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
		start_k=15;
			  for(i=15000;i>=2000;i-=1000)
		{
			start_k--;
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		//	speed_error=(int16_t)(kp_distance*(AD1+AD2-AD_INIT));
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + start_k/3*speed_distance;
			speed_change[1]=i- speed_rotate - start_k/3*speed_distance;
			speed_change[2]=-i- speed_rotate - start_k/3*speed_distance;
			speed_change[3]=i+ speed_rotate - start_k/3*speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(50);
			speed_lasterror=speed_error;
		}	
		while(Encoder_flag_up+Encoder_flag_down<=13)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			speed_lasterror=speed_error;
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
	while(__fabs(Encode_counter-50)>40)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=2000- speed_rotate + speed_distance;
			speed_change[1]=2000- speed_rotate - speed_distance;
			speed_change[2]=-2000- speed_rotate - speed_distance;
			speed_change[3]=2000+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}
		stop_speed();
		TIM_ITConfig(TIM5,TIM_IT_UPDATE,DISABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶
	  Encoder_data=0;
		Encoder_flag_up=0;
		Encoder_flag_down=0;
		key_flag_s2=ZERO1;
	}



	}
 
  /* USER CODE END 3 */

}

static void speed_0to1_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=2)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=1)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==2)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==2)&&(Encode_counter>1500)&&(Encode_counter<=2500))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else
			{
				speed_change[0]=-1500- speed_rotate + speed_distance;
				speed_change[1]=-1500- speed_rotate - speed_distance;
				speed_change[2]=1500- speed_rotate - speed_distance;
				speed_change[3]=-1500+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}			
}
static void speed_1to0_ON()
{
	uint16_t i;
	for(i=1000;i<=3000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=2)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=1)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==2)))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==2)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==2)&&(Encode_counter<=1500))
			{
				speed_change[0]=1500- speed_rotate + speed_distance;
				speed_change[1]=1500- speed_rotate - speed_distance;
				speed_change[2]=-1500- speed_rotate - speed_distance;
				speed_change[3]=1500+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
}

static void speed_0to2_ON()
{
	uint16_t i;
	for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==4)))
			{
				speed_change[0]=-3500- speed_rotate + speed_distance;
				speed_change[1]=-3500- speed_rotate - speed_distance;
				speed_change[2]=3500- speed_rotate - speed_distance;
				speed_change[3]=-3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-2500- speed_rotate + speed_distance;
				speed_change[1]=-2500- speed_rotate - speed_distance;
				speed_change[2]=2500- speed_rotate - speed_distance;
				speed_change[3]=-2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>=3000))
			{
				speed_change[0]=-1400- speed_rotate + speed_distance;
				speed_change[1]=-1400- speed_rotate - speed_distance;
				speed_change[2]=1400- speed_rotate - speed_distance;
				speed_change[3]=-1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
}

static void speed_0to3_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=6)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=5)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==6)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
}
static void speed_0to4_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=8)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=7)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==8)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
}

static void speed_0to5_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=-i- speed_rotate + speed_distance;
			speed_change[1]=-i- speed_rotate - speed_distance;
			speed_change[2]=i- speed_rotate - speed_distance;
			speed_change[3]=-i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=10)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=9)||((Encode_counter<=1500)&&(Encoder_flag_up+Encoder_flag_down==10)))
			{
				speed_change[0]=-4000- speed_rotate + speed_distance;
				speed_change[1]=-4000- speed_rotate - speed_distance;
				speed_change[2]=4000- speed_rotate - speed_distance;
				speed_change[3]=-4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=-3000- speed_rotate + speed_distance;
				speed_change[1]=-3000- speed_rotate - speed_distance;
				speed_change[2]=3000- speed_rotate - speed_distance;
				speed_change[3]=-3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter>=3000))
			{
				speed_change[0]=-2000- speed_rotate + speed_distance;
				speed_change[1]=-2000- speed_rotate - speed_distance;
				speed_change[2]=2000- speed_rotate - speed_distance;
				speed_change[3]=-2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}	
}

static void speed_2to0_ON()
{
	uint16_t i;
	for(i=1000;i<=3500;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=4)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=3)||(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>3000)))
			{
				speed_change[0]=3500- speed_rotate + speed_distance;
				speed_change[1]=3500- speed_rotate - speed_distance;
				speed_change[2]=-3500- speed_rotate - speed_distance;
				speed_change[3]=3500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=2500- speed_rotate + speed_distance;
				speed_change[1]=2500- speed_rotate - speed_distance;
				speed_change[2]=-2500- speed_rotate - speed_distance;
				speed_change[3]=2500+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==4)&&(Encode_counter<=1500))
			{
				speed_change[0]=1400- speed_rotate + speed_distance;
				speed_change[1]=1400- speed_rotate - speed_distance;
				speed_change[2]=-1400- speed_rotate - speed_distance;
				speed_change[3]=1400+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
}

static void speed_3to0_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=6)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=5)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==6)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==6)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
}
static void speed_4to0_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=8)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=7)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==8)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==8)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
}
static void speed_5to0_ON()
{
	uint16_t i;
	for(i=1000;i<=4000;i+=750)
		{
     	get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			speed_change[0]=i- speed_rotate + speed_distance;
			speed_change[1]=i- speed_rotate - speed_distance;
			speed_change[2]=-i- speed_rotate - speed_distance;
			speed_change[3]=i+ speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			HAL_Delay(100);
		}
		while(Encoder_flag_up+Encoder_flag_down<=10)
		{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(kp_rotate*(AD1-AD2)); 
		  speed_distance= (int16_t)(kp_distance*(AD1+AD2-AD_INIT));
			if((Encoder_flag_up+Encoder_flag_down<=9)||((Encode_counter>=3000)&&((Encoder_flag_up+Encoder_flag_down)==10)))
			{
				speed_change[0]=4000- speed_rotate + speed_distance;
				speed_change[1]=4000- speed_rotate - speed_distance;
				speed_change[2]=-4000- speed_rotate - speed_distance;
				speed_change[3]=4000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter>1500)&&(Encode_counter<=3000))
			{
				speed_change[0]=3000- speed_rotate + speed_distance;
				speed_change[1]=3000- speed_rotate - speed_distance;
				speed_change[2]=-3000- speed_rotate - speed_distance;
				speed_change[3]=3000+ speed_rotate - speed_distance;
			}
			else if(((Encoder_flag_up+Encoder_flag_down)==10)&&(Encode_counter<=1500))
			{
				speed_change[0]=2000- speed_rotate + speed_distance;
				speed_change[1]=2000- speed_rotate - speed_distance;
				speed_change[2]=-2000- speed_rotate - speed_distance;
				speed_change[3]=2000+ speed_rotate - speed_distance;
			}
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
			Encoder_data=__HAL_TIM_GET_COUNTER(&htim5);
		}				
}

static void speed_send(int16_t x1,int16_t x2,int16_t x3,int16_t x4)
{
		      speed_transform_10to16(VeloticyData,x4);
					speed_transform_10to16(VeloticyData1,x2);
					speed_transform_10to16(VeloticyData2,x3);
					speed_transform_10to16(VeloticyData3,x1);
	
					SendData(hcan1,0x014,VeloticyData);	
					SendData(hcan1,0x024,VeloticyData1);
					SendData(hcan1,0x034,VeloticyData2);	
					SendData(hcan1,0x044,VeloticyData3);
}


static void location_send(int32_t x1,int32_t x2)
{
					location_transform_10to32(LocationData,x1);
					location_transform_10to32(LocationData1,x2);
	
				  SendData(hcan1,0x115,LocationData);
		      SendData(hcan1,0x125,LocationData1);
}
static void brushless_speed_send(uint8_t speed)
{
	brushless_speed_transform(brushlessData,speed);
	SendData(hcan1,0x214,brushlessData);
}
static void SendData(CAN_HandleTypeDef hcan,uint32_t ID,int8_t* Send)
{
	hcan.pTxMsg->StdId=ID;
	hcan.pTxMsg->RTR=CAN_RTR_DATA;
	hcan.pTxMsg->IDE=CAN_ID_STD;
	hcan.pTxMsg->DLC=8;
	hcan.pTxMsg->Data[0]=Send[0];
	hcan.pTxMsg->Data[1]=Send[1];
	hcan.pTxMsg->Data[2]=Send[2];
	hcan.pTxMsg->Data[3]=Send[3];
	hcan.pTxMsg->Data[4]=Send[4];
	hcan.pTxMsg->Data[5]=Send[5];
	hcan.pTxMsg->Data[6]=Send[6];
	hcan.pTxMsg->Data[7]=Send[7];
	HAL_CAN_Transmit(&hcan1,200);
}

static void speed_transform_10to16(int8_t* Veloticy,int16_t speed)  
{
	Veloticy[3]= (uint8_t )(speed&0xff);
	Veloticy[2]= (uint8_t )((speed>>8)&0xff); 
}
static void location_transform_10to32(int8_t* Position,int32_t location)  
{
	Position[7]= (uint8_t )(location&0xff);
	Position[6]= (uint8_t )((location>>8)&0xff); 
	Position[5]= (uint8_t )((location>>16)&0xff); 
	Position[4]= (uint8_t )((location>>24)&0xff); 
}
static void brushless_speed_transform(int8_t* brushlessData,uint8_t speed)
{
	brushlessData[0]=(((speed*20) >> 8) & 0xFF);
	brushlessData[1]=((speed*20) & 0xFF);
}
static void MotorVeloticy_Init(void)
{
	SendData(hcan1,0x010,ResetData);       //³õÊ¼»¯ 0×é1ºÅ
	SendData(hcan1,0x020,ResetData);       //³õÊ¼»¯ 
	SendData(hcan1,0x030,ResetData);			 //³õÊ¼»¯
	SendData(hcan1,0x040,ResetData);			 //³õÊ¼»¯
	
	HAL_Delay(1000);
	
	SendData(hcan1,0x011,VeloticyModeData);			//Ä£Ê½Ñ¡Ôñ
	SendData(hcan1,0x021,VeloticyModeData);			//Ä£Ê½Ñ¡Ôñ
	SendData(hcan1,0x031,VeloticyModeData);			//Ä£Ê½Ñ¡Ôñ
	SendData(hcan1,0x041,VeloticyModeData);			//Ä£Ê½Ñ¡Ôñ
}




static void MotorLocation_Init(void)
{
	SendData(hcan1,0x110,ResetData);
	SendData(hcan1,0x120,ResetData);
  SendData(hcan1,0x130,ResetData);
	HAL_Delay(1000);
	
	SendData(hcan1,0x111,LocationModeData);
	SendData(hcan1,0x121,LocationModeData);
	SendData(hcan1,0x131,LocationModeData);
	HAL_Delay(1000);
	
	SendData(hcan1,0x11A,LimitModeData);
	SendData(hcan1,0x12A,LimitModeData);
	HAL_Delay(1000);
}
/** System Clock Configuration
*/
static void state_movement(uint8_t P,uint8_t AN,uint8_t AC)
{
	position=position-760*(P-position_last);
	angle=angle-71*(AN-angle_last);
//	if(position>position_start)
//	{
//		position=position_start;
//	}
//	if(angle>angle_start)
//	{
//		angle=angle_start;
//	}
	angle_last=AN;
	position_last=P;
	location_send(position,angle);
	brushless_speed_send(AC);
}
void stop_speed(void)
{
	while((__fabs(AD1-AD2)>20)||(__fabs(AD1+AD2-AD_INIT)>30))  //3  6
	{
			get_AD1_and_AD2();
      speed_rotate= (int16_t)(3*(AD1-AD2));//30
		  speed_distance= (int16_t)(2*(AD1+AD2-AD_INIT)); //20
			speed_change[0]=- speed_rotate + speed_distance;
			speed_change[1]=- speed_rotate - speed_distance;
			speed_change[2]=- speed_rotate - speed_distance;
			speed_change[3]= speed_rotate - speed_distance;
		  speed_send(speed_change[0],speed_change[1],speed_change[2],speed_change[3]);
	}
	speed_send(100,100,100,-100);
	get_AD1_and_AD2();
	while((AD1-AD2)<70)
{
	get_AD1_and_AD2();
}  //3  
			  speed_transform_10to16(VeloticyData,0);
				speed_transform_10to16(VeloticyData1,0);
				speed_transform_10to16(VeloticyData2,0);
				speed_transform_10to16(VeloticyData3,0);
	
				SendData(hcan1,0x014,VeloticyData);	
				SendData(hcan1,0x024,VeloticyData1);
				SendData(hcan1,0x034,VeloticyData2);	
				SendData(hcan1,0x044,VeloticyData3);
}

void get_AD1_and_AD2(void)
{
	      HAL_ADC_Start(&hadc1);
		    HAL_Delay(5);
		    HAL_ADC_Stop(&hadc1);
		    AD1=HAL_ADC_GetValue(&hadc1);
		
		    HAL_ADC_Start(&hadc2);
		    HAL_Delay(5);
		    HAL_ADC_Stop(&hadc2);
		    AD2=HAL_ADC_GetValue(&hadc2);
}
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
	//SEN1 ¼¤¹â´«¸ÐÆ÷1 PC1
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{
	//SEN2 ¼¤¹â´«¸ÐÆ÷2  PC2
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_9TQ;
  hcan1.Init.BS2 = CAN_BS2_4TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	hcan1filter.FilterNumber=0;
	hcan1filter.FilterMode=CAN_FILTERMODE_IDMASK;
	hcan1filter.FilterScale=CAN_FILTERSCALE_32BIT;
	hcan1filter.FilterIdHigh=0x0000;
	hcan1filter.FilterIdLow=0x0000;
	hcan1filter.FilterMaskIdHigh=0x0000;
	hcan1filter.FilterMaskIdLow=0x0000;
	hcan1filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	hcan1filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &hcan1filter);
}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_9TQ;
  hcan2.Init.BS2 = CAN_BS2_4TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
	hcan2filter.FilterNumber=0;
	hcan2filter.FilterMode=CAN_FILTERMODE_IDMASK;
	hcan2filter.FilterScale=CAN_FILTERSCALE_32BIT;
	hcan2filter.FilterIdHigh=0x0000;
	hcan2filter.FilterIdLow=0x0000;
	hcan2filter.FilterMaskIdHigh=0x0000;
	hcan2filter.FilterMaskIdLow=0x0000;
	hcan2filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	hcan2filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan2, &hcan2filter);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)    //±àÂëÆ÷³õÊ¼»¯º¯Êý  PA0 PA1
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0x1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;	
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
//TIM_ITConfig(TIM5,TIM_IT_UPDATE,ENABLE);   //Ê¹ÄÜ¸üÐÂÖÐ¶Ï
HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);  //¿ªÊ¼¼ÆÊý
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
	//´®¿Ú
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6 
                           PE7 PE8 PE9 PE10 
                           PE11 PE12 PE13 PE14 
                           PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC3 PC4 PC5 PC6 
                           PC7 PC8 PC9 PC10 
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 
                           PA6 PA7 PA8 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB4 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13 
                           PD14 PD15 PD0 PD1 
                           PD2 PD3 PD4 PD5 
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1 
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}



void TIM5_IRQHandler(void)    //±àÂëÆ÷ÖÕÖÕÖÐ¶Ï
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim)   //±àÂëÆ÷ÖÕÖÐ¶Ï
{
  /* Capture compare 1 event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) !=RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
        
        /* Input capture event */
        if((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)
        {
          HAL_TIM_IC_CaptureCallback(htim);
        }
        /* Output compare event */
        else
        {
          HAL_TIM_OC_DelayElapsedCallback(htim);
          HAL_TIM_PWM_PulseFinishedCallback(htim);
        }
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
      }
    }
  }
  /* Capture compare 2 event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
      /* Input capture event */
      if((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U)
      {          
        HAL_TIM_IC_CaptureCallback(htim);
      }
      /* Output compare event */
      else
      {
        HAL_TIM_OC_DelayElapsedCallback(htim);
        HAL_TIM_PWM_PulseFinishedCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* Capture compare 3 event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC3) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
      /* Input capture event */
      if((htim->Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00U)
      {          
        HAL_TIM_IC_CaptureCallback(htim);
      }
      /* Output compare event */
      else
      {
        HAL_TIM_OC_DelayElapsedCallback(htim);
        HAL_TIM_PWM_PulseFinishedCallback(htim); 
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* Capture compare 4 event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC4) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
      /* Input capture event */
      if((htim->Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00U)
      {          
        HAL_TIM_IC_CaptureCallback(htim);
      }
      /* Output compare event */
      else
      {
        HAL_TIM_OC_DelayElapsedCallback(htim);
        HAL_TIM_PWM_PulseFinishedCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* TIM Update event */
	
	
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)   //¸üÐÂÖÐ¶Ï
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) !=RESET)  //Ç°½øÖÐ¶Ï
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)!= RESET)
		{
	    Encoder_flag_up++;   //Ç°½ø±êÖ¾Î»++
			
		}
		else if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)!=SET)   //ºóÍËÖÐ¶Ï
		{
		  Encoder_flag_down++; //ºóÍË±êÖ¾Î»++
		}
      HAL_TIM_PeriodElapsedCallback(htim);
    }
  }
	
	
  /* TIM Break input event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_BREAK) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_BREAK) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_BREAK);
      HAL_TIMEx_BreakCallback(htim);
    }
  }
  /* TIM Trigger detection event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_TRIGGER) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_TRIGGER) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_TRIGGER);
      HAL_TIM_TriggerCallback(htim);
    }
  }
  /* TIM commutation event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_COM) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_COM) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_FLAG_COM);
      HAL_TIMEx_CommutationCallback(htim);
    }
  }
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/**
  * º¯Êý¹¦ÄÜ: ÖØ¶¨Ïòc¿âº¯Êýprintfµ½USART3
  * ÊäÈë²ÎÊý: ÎÞ
  * ·µ »Ø Öµ: ÎÞ
  * Ëµ    Ã÷£ºÎÞ
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * º¯Êý¹¦ÄÜ: ÖØ¶¨Ïòc¿âº¯Êýgetchar,scanfµ½USART3
  * ÊäÈë²ÎÊý: ÎÞ
  * ·µ »Ø Öµ: ÎÞ
  * Ëµ    Ã÷£ºÎÞ
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1,&ch, 1, 0xffff);
  return ch;
}
/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
