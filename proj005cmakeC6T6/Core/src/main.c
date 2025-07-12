#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_bkp.h"
//#include "stdio.h"
//#include "misc.h"
//#include "001.h"
#include "tim2_delay.h"
//#include "onewire.h"
#include <string.h>
//#include "libmodbus.h"
//#include "modbus.h"









//#include "stm32f10x.h"
//#include "stm32f10x_bkp.h"

#define PERIOD 1000
#define SYSCLK 72000000
#define PRESCALER 72

#define RX_BUF_SIZE 80
static volatile int temp;
static volatile char RX_FLAG_END_LINE;
static volatile unsigned int RXi;
static volatile char RXc;
static char RX_BUF[RX_BUF_SIZE]; //= {"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
static u8 RX_BUF08[RX_BUF_SIZE];
static char buffer[80];// = {"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
static volatile int TimeResult;
static volatile int iResult;
static volatile float fResult;
static volatile int TimeSec;
static volatile uint8_t TimeState;
static volatile uint8_t FLAG_ECHO;
//static uint16_t ds18b20Value;
// (UnixTime = 00:00:00 01.01.1970 = JD0 = 2440588)
#define JULIAN_DATE_BASE    2440588

void GETonGPIO(void);
#define USART1PPport GPIOA
#define USART1PPpin GPIO_Pin_11
void usart1_init(void);
void USART1_IRQHandler(void);
#define USARTSend USART1Send485
void clear_RXBuffer(void);
void USART01Send(u8 *pucBuffer);
void USART1Send(char *pucBuffer);
void USART1Send485(char *pucBuffer);
unsigned char RTC_Init(void);

char get_ab_xFF(int a);
u8 convT_DS18B20(u8 LSB, u8 MSB);
void schitatTemp(char* imya);
void vvhex(char vv);
void sendaddrow (void);

//float schitatfTemp(char* imya);
uint16_t schitatU16Temp(char* imya);
void oprosite (void);

#define DHT11_SUCCESS         1
#define DHT11_ERROR_CHECKSUM  2
#define DHT11_ERROR_TIMEOUT   3


void wwdgenable(void);
void WWDG_IRQHandler(void);
void iwdg_init(void);

static uint16_t millisec2;
static uint32_t globalsecs;
static uint32_t millisec003delay_ms;
void SETglobalsecs(uint32_t count);
uint32_t GETglobalsecs(void);


void TIM2_init(void);
void TIM2_IRQHandler(void);
void delay_us(uint32_t n_usec);
void delay_ms(uint32_t n_msec);
void TIM4_init(void);
void TIM4_IRQHandler(void);
void TIM3_init(void);
void TIM3_IRQHandler(void);

void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//static uint16_t servo001max;
//static uint16_t servo001min;
static uint16_t servo001use;
//static uint16_t servo002max;
//static uint16_t servo002min;
static uint16_t servo002use;
//static uint16_t servo003max;
//static uint16_t servo003min;
static uint16_t servo003use;
//static uint16_t servo004max;
//static uint16_t servo004min;
static uint16_t servo004use;
//static uint16_t servo005max;
//static uint16_t servo005min;
static uint16_t servo005use;
//void SERVOinit(void);


//MODBUS
//#include "modbus.h"
#define OBJ_SZ 36 //это количество объектов
#define SETUP 4 //это просто количество данных в массиве 0-элемент которого означает адрес
//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
static uint8_t SET_PAR[SETUP];//0-элемент это адрес
//OBJECT ARRAY WHERE READING AND WRITING OCCURS
//uint16_t res_table[OBJ_SZ];//массив с объектами то откуда мы читаем и куда пишем
//float res_ftable[OBJ_SZ];
//buffer uart
#define BUF_SZ 128 //размер буфера
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //максимальное количество регистров в ответе
//uart structure
typedef struct UART_DATA {
    uint8_t buffer[128];//буфер
    uint16_t rxtimer;//этим мы считаем таймоут
    uint8_t rxcnt; //количество принятых символов
    uint8_t txcnt;//количество переданных символов
    uint8_t txlen;//длина посылки на отправку
    uint8_t volatile rxgap;//окончание приема
    uint8_t protocol;//тип протокола - здесь не используется
    uint16_t delay;//задержка
    uint8_t ddddddDOBAVKA[1];
} UART_DATA;
struct UART_DATA uart1;//структуры для соответсвующих усартов
//timer 0.0001sec one symbol on 9600 ~1ms
//uart3.delay=30; //modbus gap 9600
//uart3.delay=10; //modbus gap 38400
// /////////////////////////////////////////////////////////////////////////////////////////////////////
void net_tx1(UART_DATA *uart);
void MODBUS_SLAVE(UART_DATA *MODBUS);//функция обработки модбас и формирования ответа
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);

void TX_01(UART_DATA *MODBUS);
void TX_02(UART_DATA *MODBUS);

static uint8_t Coils_RW[OBJ_SZ];
static uint8_t Discrete_Inputs_RO[OBJ_SZ];
void setCOILS(uint8_t *Coils_RW);
void read_Discrete_Inputs_RO(void);
//void startCOILS(uint8_t *Coils_RW);
void read_Coils_RW(void);
void TX_05(UART_DATA *MODBUS);
void TX_04(UART_DATA *MODBUS);
void TX_03(UART_DATA *MODBUS);
union FloatU8  {
    float tmp_val_float;
    uint8_t tmp_val_u8[4];
    uint32_t tmp_val_u32;
    uint16_t tmp_val_u16[2];
    int16_t tmp_val_i16[2];
    int32_t tmp_val_i32;
} f001;
//typedef union FloatU8 ;
void coilTOback(void);
void coilFROMback(void);
void TX_16(UART_DATA *MODBUS);
typedef union   {
    float tmp_float[OBJ_SZ];
    uint8_t tmp_u8[OBJ_SZ*4];
    uint32_t tmp_u32[OBJ_SZ];
    uint16_t tmp_u16[OBJ_SZ*2];
    int16_t tmp_i16[OBJ_SZ*2];
    int32_t tmp_i32[OBJ_SZ];
} REGISTRS001;
static REGISTRS001 hold_reg;
static REGISTRS001 input_reg;
//hold_reg, input_reg;

void rs485GPIOon (void);
void rs485GPIOoff (void);


//счетчик воды
uint8_t waterplus;
uint8_t waterpluscount;
uint8_t waterplusSET;
void watercounter (void);//hold reg8-9 hold.u32 Nomer5      bcp27-28







void atSTART(void);

void COILtimerMINUTES (uint8_t coilSETED, uint16_t inREGcount, uint16_t inREGbkp, uint16_t holdREGtimer, uint16_t holdREGbkp);


int main(void) {

  uint32_t RTC_Counter01 = 0;
  uint32_t RTC_Counter02 = 0;
  uint32_t RTC_Counter03 = 0;
  uint32_t n = 0;
  // Включить тактирование модулей управления питанием и управлением резервной областью
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  // Разрешить доступ к области резервных данных
  PWR_BackupAccessCmd(ENABLE);

  //uint16_t res003;
  SET_PAR[0] = 30; //адрес этого устройства 20 (modbus) 1-247

  GETonGPIO(); //B11-B1 PP B4 B4 IPU A7 IPU A6 IPD A5 IPU
  TIM2_init(); // мkс 0-19999 TIM2->CNT servo A1 A3
  TIM3_init();
  TIM4_init(); // мкс 0-19999 TIM4->CNT servo B6 B7 B9
  usart1_init(); //A9 PP RXD A10 TXD жёлый //RS232 A11 ResetBits //485     //USART 1 and GPIO A (9/10/11) ON A11pp A8invertA11
  //OW_Init(); //usart2 А2 А3
  //dev001.port = GPIOA;
  //dev001.pin = GPIO_Pin_12;
  //dev001.humidity = 0;
  //dev001.temparature = 0;
  //for(millisec003delay_ms=0; millisec003delay_ms<= 7200000; millisec003delay_ms++);
  //dev001.pointtemparature = 0;
  //DHT11_init(&dev001, dev001.port, dev001.pin);
  GPIO_ToggleBits(GPIOC,GPIO_Pin_13);
  //GPIO_ResetBits(GPIOC, GPIO_Pin_13);   // C13 -- 0 VCC
  //GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1 GDN set!
  uart1.delay=150; //modbus gap 9600
  uart1.rxtimer = 0;
  delay_ms(1000);
  GPIO_ToggleBits(GPIOC,GPIO_Pin_13);
  //GPIO_ResetBits(GPIOC, GPIO_Pin_13);   // C13 -- 0 VCC
  //GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1 GDN set!
  atSTART();
  //oprosite();

  iwdg_init();


  while (1) {
      if (Coils_RW[8] == 0) {
          IWDG_ReloadCounter();
        }
      if(uart1.rxgap==1) {

          //GPIO_SetBits(USART1PPport, USART1PPpin);
          MODBUS_SLAVE(&uart1);
          net_tx1(&uart1);

          //GPIO_ResetBits(USART1PPport, USART1PPpin);
      }

      if ( ((RTC_Counter02 = globalsecs)  - RTC_Counter01) >= 4) {
          //GPIO_ToggleBits(GPIOC,GPIO_Pin_13);
          //USART1Send485("test\r\n");
          RTC_Counter01 = RTC_Counter02;
          if ( (RTC_Counter02 - RTC_Counter03) >= 60) {
              n++;
              if (n > 6) {
                  if ((hold_reg.tmp_u16[24] - hold_reg.tmp_u16[25]) > 10) {
                      Coils_RW[8] = 1;
                  }
                }else if (n > 100) {
                  n = 0;
                }
              hold_reg.tmp_u16[24] = hold_reg.tmp_u16[24] + 1;
              RTC_Counter03 = RTC_Counter02;
              input_reg.tmp_u16[2] = (RTC_Counter02 / 3600) % 24;   //Number STM20hour   "hour [%d]"                 (gmod20_INreg)     {modbus="<[slave20_4:2]"}
              input_reg.tmp_u16[1] = (RTC_Counter02 / 60) % 60;     //Number STM20minute   "minute [:%d]"            (gmod20_INreg)     {modbus="<[slave20_4:1]"}
              input_reg.tmp_u16[0] = RTC_Counter02 % 60;            //Number STM20second  "seconds [:%d]"            (gmod20_INreg)     {modbus="<[slave20_4:0]"}
              input_reg.tmp_u16[3] = (RTC_Counter02 / (3600 * 24)); //Number STM20date  "date [%d]"                  (gmod20_INreg)     {modbus="<[slave20_4:3]"}
              COILtimerMINUTES(1, input_reg.tmp_u16[12], BKP_DR5, hold_reg.tmp_u16[28], BKP_DR9);   //B11     slave20_403:4       slave20_302:4
              COILtimerMINUTES(2, input_reg.tmp_u16[13], BKP_DR6, hold_reg.tmp_u16[29], BKP_DR10);  //B10     slave20_403:5       slave20_302:5
              COILtimerMINUTES(3, input_reg.tmp_u16[14], BKP_DR7, hold_reg.tmp_u16[30], BKP_DR11);  //B1      slave20_403:6       slave20_302:6
              COILtimerMINUTES(4, input_reg.tmp_u16[15], BKP_DR8, hold_reg.tmp_u16[31], BKP_DR12);  //B0      slave20_403:7       slave20_302:7

              BKP_WriteBackupRegister(BKP_DR13, hold_reg.tmp_u16[10]);
              BKP_WriteBackupRegister(BKP_DR14, hold_reg.tmp_u16[11]);
              BKP_WriteBackupRegister(BKP_DR15, hold_reg.tmp_u16[12]);
              BKP_WriteBackupRegister(BKP_DR16, hold_reg.tmp_u16[13]);
              BKP_WriteBackupRegister(BKP_DR17, hold_reg.tmp_u16[14]);
              BKP_WriteBackupRegister(BKP_DR18, hold_reg.tmp_u16[15]);
              BKP_WriteBackupRegister(BKP_DR19, hold_reg.tmp_u16[16]);
              BKP_WriteBackupRegister(BKP_DR20, hold_reg.tmp_u16[17]);
              BKP_WriteBackupRegister(BKP_DR21, hold_reg.tmp_u16[18]);
              BKP_WriteBackupRegister(BKP_DR22, hold_reg.tmp_u16[19]);
          }
          //ds18b20Value = schitatU16Temp("\x28\xee\xe8\x19\x17\x16\x02\xa1");
          //input_reg.tmp_float[9] = (float) (ds18b20Value / 16.0);   //Number STM20DS03f "DS01 floatTemp [%.2f °C]"   (gmod20_INreg)     {modbus="<[slave20_402:1]"}
          //input_reg.tmp_u16[4] = DHT11_read(&dev001);               //Number STM20DHTres "DHTstatus [%d]"            (gmod20_INreg)     {modbus="<[slave20_4:4]"}
          //if (input_reg.tmp_u16[4] == DHT11_SUCCESS) {
          //    input_reg.tmp_u16[5] = dev001.humidity;               //Number STM20DHThum "humidity [%d %%]"          (gmod20_INreg)     {modbus="<[slave20_4:5]"}
          //    input_reg.tmp_float[8] = ((float)dev001.temparature + (0.1 * dev001.pointtemparature) );
          //    //Number STM20DHTtemp "DHTtemp [%.1f °C]"  (gmod20_INreg)     {modbus="<[slave20_402:0]"}
          //  }
          input_reg.tmp_u16[11] = hold_reg.tmp_u16[27];             //Number STM20countPPRO  "ROcountPP [%d]"        (gmod20_INreg)     {modbus="<[slave20_4:11]"}
          hold_reg.tmp_u16[26] = hold_reg.tmp_u16[25];              //prov2
          input_reg.tmp_float[11] = (float) RTC_Counter01;          //Number STM20count "count [%.1f ]"              (gmod20_INreg)     {modbus="<[slave20_402:3]"}

          //oprosite(); //OW opros
          if (Coils_RW[9] != 0) {
              if (input_reg.tmp_i16[11] > 0) {
                  RTC_Counter02 = RTC_Counter02 + ((uint32_t)input_reg.tmp_i16[7]);
              } else {
                  input_reg.tmp_i16[11] = input_reg.tmp_i16[11] * (-1);
                  RTC_Counter02 = RTC_Counter02 + ((uint32_t)input_reg.tmp_i16[7]);
              }
              SETglobalsecs(RTC_Counter02);
              //res_ftable[5] = 0;
              Coils_RW[9] = 0;
          }
      }
  }
}

void atSTART(void) {
  coilFROMback(); //######################################## coilFROMback();coilFROMback();coilFROMback(); BKP_DR2 BKP_DR1
  Coils_RW[8] = 0;

  //setCOILS(Coils_RW);
  //for(u8 i = 0; i < OBJ_SZ; i++) {
  //    input_reg.tmp_u32[i] = 0;
  //    hold_reg.tmp_u32[i] = 0;
  //  }


  hold_reg.tmp_u16[28] = BKP_ReadBackupRegister(BKP_DR9);
  hold_reg.tmp_u16[29] = BKP_ReadBackupRegister(BKP_DR10);
  hold_reg.tmp_u16[30] = BKP_ReadBackupRegister(BKP_DR11);
  hold_reg.tmp_u16[31] = BKP_ReadBackupRegister(BKP_DR12);

  hold_reg.tmp_u16[10] = BKP_ReadBackupRegister(BKP_DR13);
  hold_reg.tmp_u16[11] = BKP_ReadBackupRegister(BKP_DR14);
  //servo001max = hold_reg.tmp_u16[11];
  //servo001use = 900;
  //servo001min = hold_reg.tmp_u16[10];
  hold_reg.tmp_u16[12] = BKP_ReadBackupRegister(BKP_DR15);
  hold_reg.tmp_u16[13] = BKP_ReadBackupRegister(BKP_DR16);
  //servo002max = hold_reg.tmp_u16[13];
  //servo002use = 500;
  //servo002min = hold_reg.tmp_u16[12];
  hold_reg.tmp_u16[14] = BKP_ReadBackupRegister(BKP_DR17);
  hold_reg.tmp_u16[15] = BKP_ReadBackupRegister(BKP_DR18);
  //servo003max = hold_reg.tmp_u16[15];
  //servo003use = 1000;
  //servo003min = hold_reg.tmp_u16[14];
  hold_reg.tmp_u16[16] = BKP_ReadBackupRegister(BKP_DR19);
  hold_reg.tmp_u16[17] = BKP_ReadBackupRegister(BKP_DR20);
  //servo004max = hold_reg.tmp_u16[17];
  //servo004use = 1000;
  //servo004min = hold_reg.tmp_u16[16];
  hold_reg.tmp_u16[18] = BKP_ReadBackupRegister(BKP_DR21);
  hold_reg.tmp_u16[19] = BKP_ReadBackupRegister(BKP_DR22);
  //servo005max = hold_reg.tmp_u16[19];
  //servo005use = 1000;
  //servo005min = hold_reg.tmp_u16[18];
  //WATER COUNT
  hold_reg.tmp_u16[8] = BKP_ReadBackupRegister(BKP_DR27);
  hold_reg.tmp_u16[9] = BKP_ReadBackupRegister(BKP_DR28);
  setCOILS(Coils_RW);

waterplus=0;
//waterminus=0;
waterpluscount=0;
//waterminuscount=0;
waterplusSET=1;
}

void COILtimerMINUTES (uint8_t coilSETED, uint16_t inREGcount,uint16_t inREGbkp, uint16_t holdREGtimer ,uint16_t holdREGbkp) {
  inREGcount = BKP_ReadBackupRegister(inREGbkp);

  if ( Coils_RW[coilSETED] != 0) {
      inREGcount--;
    } else {
      inREGcount = holdREGtimer;
    }
  if (inREGcount < 1) {
      Coils_RW[coilSETED] = 0;
    }
  BKP_WriteBackupRegister(inREGbkp, inREGcount);
  BKP_WriteBackupRegister(holdREGbkp, holdREGtimer);
}



/*#include "001.h"
//#include "onewire.h"
#include "tim2_delay.h"
#include "string.h"
#include "stdio.h"
//#include "libmodbus.h"
#include "modbus.h"*/


void GETonGPIO() { //PP B(11/10/1/0) C13 A(7/6) | IPU B4 | IPD B8 FLOAT B9
  GPIO_InitTypeDef GPIO_InitStructure;
  //LED C.13
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  //A6 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // ///////// 4 OUT B11 B10 B1 B0
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  // B11 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //B 10 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // B1 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //B 0 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //KNOPKA B3 IPU
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // A7 IPU
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // A6 IPU
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // A5 IPU
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //KNOPKA B4 IPU
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

}
void usart1_init(void) { //USART 1 and GPIO A (9/10/11) ON A11pp
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
  //  A 9 TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //  A 10 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // A11 PP
  GPIO_InitStructure.GPIO_Pin = USART1PPpin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART1PPport, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //  USART 1
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init (USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //GDN on A11
  //GPIO_SetBits(USART1PPport, USART1PPpin);
  GPIO_ResetBits(USART1PPport, USART1PPpin);
  GPIO_SetBits(GPIOA,GPIO_Pin_8);
  NVIC_EnableIRQ(USART1_IRQn);


}
void USART1_IRQHandler(void) {
  /*if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET) {
      RXc =(char) USART_ReceiveData(USART1);
      RX_BUF[RXi] = RXc;
      RXi++;
      RX_FLAG_END_LINE = 0;
      if (RXc != 13) {
          if (RXi > RX_BUF_SIZE - 1) {
              clear_RXBuffer();
            }
        } else {
          RX_FLAG_END_LINE = 1;
        }
      //Echo
      USART_SendData(USART1,(u16) RXc);
    }*/
  //Receive Data register not empty interrupt
  //GPIO_ToggleBits(GPIOC,GPIO_Pin_13);
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  {
      USART_ClearITPendingBit(USART1, USART_IT_RXNE); //очистка признака прерывания
      uart1.rxtimer = 0;
      if(uart1.rxcnt > (BUF_SZ-2)) {
          uart1.rxcnt=0;
        }
      uart1.buffer[uart1.rxcnt++]=USART_ReceiveData (USART1);
    }
  //Transmission complete interrupt
  /*if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)  {
      USART_ClearITPendingBit(USART1, USART_IT_TC);//очистка признака прерывания

      if(uart1.txcnt < uart1.txlen)  {
          //GPIO_SetBits(USART1PPport, USART1PPpin);  // +++++++++++++++++ключаем 485
          USART_SendData(USART1,((uint16_t) uart1.buffer[uart1.txcnt++]));//Передаем
        }
      else {
          //посылка закончилась и мы снимаем высокий уровень сRS485 TXE
          //uart1.buffer[255] = uart1.txlen;
          uart1.txlen=0;
          GPIO_WriteBit(USART1PPport, USART1PPpin,Bit_RESET);
          USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
          USART_ITConfig(USART1, USART_IT_TC, DISABLE);
          //TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
        }
    }
  /*if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET) {
      RXu = (u8) USART_ReceiveData (USART1);
      uart1.buffer[uart1.rxcnt]= RXu;
      uart1.rxcnt++;
      uart1.rxtimer = 0;
      //uart1.rxgap = 0;
      if(uart1.rxcnt > (BUF_SZ-2)) {
          uart1.rxcnt=0;
        }
      //Echo
      USART_SendData(USART1,(u16) uart1.buffer[uart1.rxcnt]);
    }*/
}
void clear_RXBuffer(void) {
  for (RXi = 0; RXi < RX_BUF_SIZE; RXi++)
    RX_BUF[RXi] = '\0';
  RXi = 0;
}
void USART01Send(u8 *pucBuffer) {
  while (*pucBuffer) {
      USART_SendData(USART1,(uint16_t) *pucBuffer++);
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
        }
    }
}
void USART1Send(char *pucBuffer) {
  while (*pucBuffer) {
      USART_SendData(USART1,(uint16_t) *pucBuffer++);
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
void USART1Send485(char *pucBuffer) {
  //GPIO_SetBits(USART1PPport, USART1PPpin);
  //GPIO_ResetBits(USART1PPport, USART1PPpin);
  rs485GPIOon();
  delay_ms(2);
  USART1Send(pucBuffer);
  delay_ms(2);
  rs485GPIOoff();
  //GPIO_SetBits(USART1PPport, USART1PPpin);
  //GPIO_ResetBits(USART1PPport, USART1PPpin);
}
unsigned char RTC_Init(void) {
  // Включить тактирование модулей управления питанием и управлением резервной областью
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  // Разрешить доступ к области резервных данных
  PWR_BackupAccessCmd(ENABLE);

  // Если RTC выключен - инициализировать
  if((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
      // Сброс данных в резервной области
      RCC_BackupResetCmd(ENABLE);
      RCC_BackupResetCmd(DISABLE);

      // Установить источник тактирования кварц 32768
      RCC_LSEConfig(RCC_LSE_ON);
      while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY){}
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

      RTC_SetPrescaler(0x7FFF); // Устанавливаем делитель, чтобы часы считали секунды
      // Включаем RTC
      RCC_RTCCLKCmd(ENABLE);
      // Ждем синхронизацию
      RTC_WaitForSynchro();

      return 1;
    }
  return 0;
}

/*void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct) { //get cirrent date
  unsigned long time;
  unsigned long t1, a, b, c, d, e, m;
  uint16_t year = 0;
  uint8_t mon = 0;
  uint8_t wday = 0;
  uint8_t mday = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint64_t jd = 0;
  uint64_t jdn = 0;

  jd = ((RTC_Counter+43200)/(86400>>1)) + (2440587<<1) + 1;
  jdn = jd>>1;

  time = RTC_Counter;
  t1 = time / 60;
  sec =(uint8_t) (time - t1 * 60);

  time = t1;
  t1 = time / 60;
  min =(uint8_t) (time - t1 * 60);

  time = t1;
  t1 = time / 24;
  hour =(uint8_t) (time - t1 * 24);

  wday =(uint8_t) ( jdn%7);

  a =(unsigned long) (jdn + 32044);
  b = (4 * a + 3) / 146097;
  c = a - (146097 * b) / 4;
  d = (4 * c + 3) / 1461;
  e = c - (1461 * d) / 4;
  m = (5 * e + 2) / 153;
  mday =(uint8_t) ( e - (153 * m + 2) / 5 + 1);
  mon =(uint8_t) ( m + 3 - 12 * (m / 10));
  year =(uint16_t) (100 * b + d - 4800 + (m / 10));

  RTC_DateTimeStruct->RTC_Year = year;
  RTC_DateTimeStruct->RTC_Month = mon;
  RTC_DateTimeStruct->RTC_Date = mday;
  RTC_DateTimeStruct->RTC_Hours = hour;
  RTC_DateTimeStruct->RTC_Minutes = min;
  RTC_DateTimeStruct->RTC_Seconds = sec;
  RTC_DateTimeStruct->RTC_Wday = wday;
}
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct) {  // Convert Date to Counter
  uint32_t a;
  uint32_t y;
  uint32_t m;
  uint32_t JDN;

  a =(uint32_t) ( (14 - RTC_DateTimeStruct->RTC_Month) / 12);
  y =(uint32_t) ( RTC_DateTimeStruct->RTC_Year + 4800 - a);
  m =(uint32_t) ( RTC_DateTimeStruct->RTC_Month + (12 * a) - 3);

  JDN = RTC_DateTimeStruct->RTC_Date;
  JDN += (153 * m + 2) / 5;
  JDN += 365 * y;
  JDN += y / 4;
  JDN += -y / 100;
  JDN += y / 400;
  JDN = JDN - 32045;
  JDN = JDN - JULIAN_DATE_BASE;
  JDN *= 86400;
  JDN +=(uint32_t) (RTC_DateTimeStruct->RTC_Hours * 3600);
  JDN +=(uint32_t) (RTC_DateTimeStruct->RTC_Minutes * 60);
  JDN +=(uint32_t) (RTC_DateTimeStruct->RTC_Seconds);

  return JDN;
}
void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char *  buffer01) {
  const char WDAY0[] = "Monday";
  const char WDAY1[] = "Tuesday";
  const char WDAY2[] = "Wednesday";
  const char WDAY3[] = "Thursday";
  const char WDAY4[] = "Friday";
  const char WDAY5[] = "Saturday";
  const char WDAY6[] = "Sunday";
  const char * WDAY[7]={WDAY0, WDAY1, WDAY2, WDAY3, WDAY4, WDAY5, WDAY6};

  const char MONTH1[] = "January";
  const char MONTH2[] = "February";
  const char MONTH3[] = "March";
  const char MONTH4[] = "April";
  const char MONTH5[] = "May";
  const char MONTH6[] = "June";
  const char MONTH7[] = "July";
  const char MONTH8[] = "August";
  const char MONTH9[] = "September";
  const char MONTH10[] = "October";
  const char MONTH11[] = "November";
  const char MONTH12[] = "December";
  const char * MONTH[12]={MONTH1, MONTH2, MONTH3, MONTH4, MONTH5,
                          MONTH6, MONTH7, MONTH8, MONTH9, MONTH10, MONTH11, MONTH12};

  sprintf(buffer01, "%s %d %s %04d",
          WDAY[RTC_DateTimeStruct->RTC_Wday],
      RTC_DateTimeStruct->RTC_Date,
      MONTH[RTC_DateTimeStruct->RTC_Month -1],
      RTC_DateTimeStruct->RTC_Year);
}
*/
char get_ab_xFF(int a){
  char ff;
  switch (a) {
    case 0:
      ff = '0';
      break;
    case 1:
      ff = '1';
      break;
    case 2:
      ff = '2';
      break;
    case 3:
      ff = '3';
      break;
    case 4:
      ff = '4';
      break;
    case 5:
      ff = '5';
      break;
    case 6:
      ff = '6';
      break;
    case 7:
      ff = '7';
      break;
    case 8:
      ff = '8';
      break;
    case 9:
      ff = '9';
      break;
    case 10:
      ff = 'a';
      break;
    case 11:
      ff = 'b';
      break;
    case 12:
      ff = 'c';
      break;
    case 13:
      ff = 'd';
      break;
    case 14:
      ff = 'e';
      break;
    case 15:
      ff = 'f';
      break;

    default:
      ff = 'X';
      break;
    }
  return ff;
}
u8 convT_DS18B20(u8 LSB, u8 MSB)
{
  LSB >>= 4; // убираем дробную часть
  MSB = (u8) (MSB * 16); // убираем лишние знаки
  return(MSB | LSB); // объединяем 2 байта -> возврат
}

/*void schitatTemp(char* imya) {
  //-----------------------------------------------------------------------------
  // процедура общения с шиной 1-wire
  // sendReset - посылать RESET в начале общения.
  // 		OW_SEND_RESET или OW_NO_RESET
  // command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOT
  // cLen - длина буфера команд, столько байт отошлется в шину
  // data - если требуется чтение, то ссылка на буфер для чтения
  // dLen - длина буфера для чтения. Прочитается не более этой длины
  // readStart - с какого символа передачи начинать чтение (нумеруются с 0)
  //		можно указать OW_NO_READ, тогда можно не задавать data и dLen
  //-----------------------------------------------------------------------------
  //OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart)
  //OW_Send(OW_SEND_RESET, '\x28\xEE\x09\x03\x1A\x16\x01\x67\x88\xbe\xff\xff", 12, RX_BUF, 2, 10);
  uint8_t buf[2];
  //OW_Send(OW_SEND_RESET, "\xcc\xbe\xff\xff", 4, buf,2, 2);
  u8 command01[12] = {(u8)'\x55',(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],(u8) imya[5],
                      (u8) imya[6],(u8) imya[7],(u8)'\xbe',(u8) '\xff',(u8) '\xff'};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  //USARTSend("\n\rTHIS IS 000\n\r");
  //USARTSend(buf);
  //USARTSend("\n\r");
  //int temp = ((buf[1] * 256) + buf[0]) * 16;
  char cifry[20];
  vvhex((char)buf[1]);
  vvhex((char)buf[0]);
  USARTSend("\n\r");
  //USARTSend(imya);
  for(int i = 0; i <= 7; i++)
    vvhex(imya[i]);
  USARTSend("\n\r");

   temp = convT_DS18B20(buf[0], buf[1]);
  //sprintf(cifry, "termperature :%d.%d\r\n", temp, (int) ((125*(buf[0] % 16))/2 + buf[0] % 2));
  USARTSend(cifry);
  //sprintf(cifry, ".%d\r\n", (int) (0.0625*1000)*(buf[0] % 16));
  //USARTSend(cifry);
}*/
void vvhex(char vv) {
  int a, b;
  char ff[2];
  ff[1] =(char) '\0';
  ff[0] =(char) 'Z';
  a = vv / 16;
  b = vv % 16;
  ff[0] = get_ab_xFF(a);
  USARTSend(ff);
  ff[0] = get_ab_xFF(b);
  USARTSend(ff);
}
void sendaddrow (void) {
  for(int i=0; i < RX_BUF_SIZE && i < 20;i++) {
      if (RX_BUF[i] != 0) {
          int a, b;
          char ff[2];
          ff[1] =(char) '\0';
          ff[0] =(char) 'Z';
          a = RX_BUF[i] / 16;
          b = RX_BUF[i] % 16;
          ff[0] = get_ab_xFF(a);
          USARTSend(ff);
          ff[0] = get_ab_xFF(b);
          USARTSend(ff);
          /*sprintf(cifry, "%\xd", a);
          USARTSend(cifry);
          sprintf(cifry, "%d\r\n", b);
          USARTSend(cifry);*/
        }
      if ((i+1) % 8 == 0)
        USARTSend("\n\r");
      //USARTSend("\n\rTHIS IS 8\n\r");
    }
}

/*u16 schitatTemp(char* imya) {
  //-----------------------------------------------------------------------------
  // процедура общения с шиной 1-wire
  // sendReset - посылать RESET в начале общения.
  // 		OW_SEND_RESET или OW_NO_RESET
  // command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOT
  // cLen - длина буфера команд, столько байт отошлется в шину
  // data - если требуется чтение, то ссылка на буфер для чтения
  // dLen - длина буфера для чтения. Прочитается не более этой длины
  // readStart - с какого символа передачи начинать чтение (нумеруются с 0)
  //		можно указать OW_NO_READ, тогда можно не задавать data и dLen
  //-----------------------------------------------------------------------------
  //OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart)
  //OW_Send(OW_SEND_RESET, "\x28\xEE\x09\x03\x1A\x16\x01\x67\x88\xbe\xff\xff", 12, RX_BUF, 2, 10);
  uint8_t buf[2];
  //OW_Send(OW_SEND_RESET, "\xcc\xbe\xff\xff", 4, buf,2, 2);
  //char command01[12] = {'\x55', imya[0], imya[1], imya[2], imya[3], imya[4],
    imya[5], imya[6], imya[7],'\xbe', '\xff', '\xff'};
  u8 command01[12] = {(u8)'\x55',(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8)
imya[3],(u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7],(u8)'\xbe',(u8) '\xff',(u8) '\xff'};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  //USARTSend("\n\rTHIS IS 000\n\r");
  //USARTSend(buf);
  //USARTSend("\n\r");
  //int temp = ((buf[1] * 256) + buf[0]) * 16;
  char cifry[20];
  vvhex(buf[1]);
  vvhex(buf[0]);
  USARTSend("\n\r");
  //USARTSend(imya);
  for(int i = 0; i <= 7; i++)
    vvhex(imya[i]);
  USARTSend("\n\r");
  int temp = convT_DS18B20(buf[0], buf[1]);
  sprintf(cifry, "termperature :%d.%d\r\n", temp, (int) (0.0625*1000)*(buf[0] % 16));
  USARTSend(cifry);
  //sprintf(cifry, ".%d\r\n", (int) (0.0625*1000)*(buf[0] % 16));
  //USARTSend(cifry);
  retern  ;
}*/

/*float schitatfTemp(char* imya) {
  uint8_t buf[2];
  u8 command01[12] = { 0x55,(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],
                       (u8) imya[5],(u8) imya[6],(u8) imya[7], 0xbe, 0xff, 0xff};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  float ftemp;
  ftemp = (float) ( (float) ((buf[1] << 8) | buf[0]) / 16.0);
  return ftemp;
}*/
/*uint16_t schitatU16Temp(char* imya) {
  uint8_t buf[2];
  u8 command01[12] = { 0x55,(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],
                       (u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7], 0xbe, 0xff, 0xff};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  //int itemp;
  //itemp = ((buf[1] << 8) | buf[0]) *1000 / 16;
  //delay_ms(10);
  return ((uint16_t) ((buf[1]<<8) + (buf[0])));
}*/

/*void oprosite(void) {
  u8 comm[2];
  comm[0] = 0xcc;
  comm[1] = 0x44;
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(100);
  comm[1] = 0x4e;
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  //delay_ms(100);
  //USARTSend("oprosheno\n\r");
}*/
// ////////////////////////////////////////////////////////DHT11

/*int DHT11_init(struct DHT11_Dev* dev, GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStructure;

  dev->port = port;
  dev->pin = pin;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  //Initialise GPIO DHT11
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(dev->port, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA, dev->pin,Bit_SET);
  return 0;
}
uint16_t DHT11_read(struct DHT11_Dev* dev) {
  dev->temparature = 0;
  dev->humidity = 0;
  dev->pointtemparature = 0;
  //Initialisation
  uint8_t i, j, temp;
  //uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data[5] = {0, 0, 0, 0, 0};
  GPIO_InitTypeDef GPIO_InitStructure;

  //Generate START condition
  //o
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);

  //Put LOW for at least 18ms
  GPIO_ResetBits(dev->port, dev->pin);
  delay_ms(18);
  //wait 18ms
  //Put HIGH for 20-40us
  GPIO_SetBits(dev->port, dev->pin);
  delay_us(40);
  //wait 40us
  //End start condition

  //io();
  //Input mode to receive data
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(dev->port, &GPIO_InitStructure);

  TIM4->CNT = 0;
  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)){
      if(TIM4->CNT > 100)
        return DHT11_ERROR_TIMEOUT;
    }
  //should be HIGH for at least 80us
  TIM4->CNT = 0;
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      if(TIM4->CNT > 100)
        return DHT11_ERROR_TIMEOUT;
    }
  //Read 40 bits (8*5)
  for(j = 0; j < 5; ++j) {
      for(i = 0; i < 8; ++i) {
          //TIM4->CNT = 0;
          //LOW for 50us
          while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
          //while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
          //                                if(TIM2->CNT > 60)
          //                                        return DHT11_ERROR_TIMEOUT;
          //                        }
          //Start counter
            //TIM4->CNT = 0;
            ////HIGH for 26-28us = 0 / 70us = 1
            //while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {      }
              //while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
            //                              if(TIM2->CNT > 100)
            //                                      return DHT11_ERROR_TIMEOUT;
            //                      }
          //Calc amount of time passed
          temp = TIM4->CNT;

          //shift 0
          data[j] = data[j] << 1;

          //if > 30us it's 1
          if(temp > 40) {
              data[j] = data[j]+1;
            }

        }
    }

  //verify the Checksum
  if(data[4] != (u8) (data[0] + data[2] + data[1] + data[3]))
    return DHT11_ERROR_CHECKSUM;
  //set data
  dev->temparature = data[2];
  dev->pointtemparature = data[3];
  dev->humidity = data[0];
  return DHT11_SUCCESS;
}
*/
void wwdgenable(void){
  // Enable Watchdog
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE);
  WWDG_DeInit();
  WWDG_SetPrescaler(WWDG_Prescaler_8); //1, 2, 4, 8
  WWDG_SetWindowValue(127); // 64...127
  WWDG_Enable(100);
  WWDG_EnableIT();

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;    //WWDG interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   // NVIC initialization
}
void WWDG_IRQHandler(void) {
  //int i;
  WWDG_ClearFlag(); //This function reset flag WWDG->SR and cancel the resetting
  WWDG_SetCounter(100);

  // Toggle LED which connected to PC13
  GPIOC->ODR ^= GPIO_Pin_13;
}
void iwdg_init(void) {
  // включаем LSI
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
  // разрешается доступ к регистрам IWDG
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  // устанавливаем предделитель
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  // значение для перезагрузки
  IWDG_SetReload(0x300); //256/40000*0x300=4.9152
  // перезагрузим значение
  IWDG_ReloadCounter();
  // LSI должен быть включен
  IWDG_Enable();
}

void SETglobalsecs(uint32_t count) {
    /*uint16_t a,b;
    a = (uint16_t) count >> 16;
    b = (uint16_t) count;*/
    BKP_WriteBackupRegister(BKP_DR3, ((uint16_t) (count >> 16)));
    BKP_WriteBackupRegister(BKP_DR4, ((uint16_t) count));
}
uint32_t GETglobalsecs(void) {
    return (((uint32_t) BKP_ReadBackupRegister(BKP_DR3) << 16) + ((uint32_t) BKP_ReadBackupRegister(BKP_DR4)));
}
void TIM2_init(void) {

  /*
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseStructInit(&TIMER_InitStructure);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 36000;
  TIMER_InitStructure.TIM_Period = 49999;
  TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
  TIM_Cmd(TIM2, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // ñ÷èòàåì îäèí ðàç
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
  NVIC_EnableIRQ(TIM2_IRQn);
  */

   TIM_TimeBaseInitTypeDef TIMER_InitStructure;
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   TIM_TimeBaseStructInit(&TIMER_InitStructure);

   TIMER_InitStructure.TIM_Prescaler = 71;
   TIMER_InitStructure.TIM_Period = 19999;
   TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);

   TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
   //SERVOinit();
    TIM_Cmd(TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  // Настроим ногу (PA1)A3 к которой подключен сервопривод
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_3;
  //Будем использовать альтернативный режим а не обычный GPIO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

TIM_OCInitTypeDef timerPWM;
  TIM_OCStructInit (&timerPWM);
  timerPWM.TIM_Pulse = 900;
  timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
  timerPWM.TIM_OutputState = TIM_OutputState_Enable;
  timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM2, &timerPWM);
  TIM_OC4Init(TIM2, &timerPWM);

   NVIC_EnableIRQ(TIM2_IRQn);
}
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)   {
      millisec2 = millisec2 + 20;
      watercounter();
      //millisec003delay_ms = millisec003delay_ms + 20;
      if (millisec2 >= 1000) {
          globalsecs = GETglobalsecs();
          globalsecs++;
          SETglobalsecs(globalsecs);
          millisec2 = 0;
        //TIM_SetCounter(TIM2, 0);
      }
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }

}
void delay_us(uint32_t n_usec) {
  TIM4->CNT = 0;
  while (TIM4->CNT < n_usec);

}
void delay_ms(uint32_t n_msec) {
  millisec003delay_ms = 0;

  //for (millisec003delay_ms = 1000000; millisec003delay_ms !=0; millisec003delay_ms--) {
  //delay_us(1000);

  //}
  //TIM2->CNT = 0;
  //while (TIM2->CNT < (2 * n_msec)){}
  while (millisec003delay_ms <= n_msec);
}

void TIM4_init(void) {
  TIM_TimeBaseInitTypeDef TIM_TimBaseStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  //Initialise TIMER4
  TIM_TimBaseStructure.TIM_Period = 19999;
  TIM_TimBaseStructure.TIM_Prescaler = 71;
  TIM_TimBaseStructure.TIM_ClockDivision = 0;
  TIM_TimBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimBaseStructure);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  // Настроим ногу (Pb6)B7 B9 к которой подключен сервопривод
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 | GPIO_Pin_9;
  //Будем использовать альтернативный режим а не обычный GPIO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_OCInitTypeDef timerPWM;
  TIM_OCStructInit (&timerPWM);
  timerPWM.TIM_Pulse = 1000;
  timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
  timerPWM.TIM_OutputState = TIM_OutputState_Enable;
  timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &timerPWM);
  TIM_OC2Init(TIM4, &timerPWM);

  TIM_OC4Init(TIM4, &timerPWM);
  TIM_Cmd(TIM4, ENABLE);

  // NVIC Configuration
  // Enable the TIM4_IRQn Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_EnableIRQ(TIM4_IRQn);
  //и так есть основной void TIM2_init(void);
}
void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
      //TimeSec++;
      //millisec003delay_ms = millisec003delay_ms + 20;
    }
}

void TIM3_init(void) {
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  //TIM_TimeBaseStructInit(&TIMER_InitStructure);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 72;
  TIMER_InitStructure.TIM_Period = 1000;
  TIMER_InitStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_EnableIRQ(TIM3_IRQn);
  // ñ÷èòàåì îäèí ðàç
  //TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);
}
void TIM3_IRQHandler(void) {
  millisec003delay_ms++;
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)   {
      TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//очищаем прерывания
      //если наш таймер больше уставки задержки и есть символы то есть gap -перерыв в посылке
      //и можно ее обрабатывать

      uart1.rxtimer++;
      if((uart1.rxtimer > uart1.delay)&(uart1.rxcnt > 1)) {
          uart1.rxgap=1;
        }
      else {
          uart1.rxgap=0;
        }
    }

}

void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR ^= GPIO_Pin;
}

/*void SERVOinit(void)
{
  //Включем порт А
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
  //Включаем Таймер 2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  // Настроим ногу (Pb6)B7 B8 B9 к которой подключен сервопривод
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  //Будем использовать альтернативный режим а не обычный GPIO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // A2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // A3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);



}*/


//#include "001.h"
//#include "onewire.h"
//#include "tim2_delay.h"
//#include "libmodbus.h"
//#include "modbus.h"


uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
// Table of CRC values for high-order byte
static const uint8_t table_crc_hi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
// Table of CRC values for low-order byte
static const uint8_t table_crc_lo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
  0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
  0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
  0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
  0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
  0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
  0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
  0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
  0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
  0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
  0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
  0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
  0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
  0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
  0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
  0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
  uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
  uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
  unsigned int i; /* will index into CRC lookup */

  /* pass through message buffer */
  while (buffer_length--) {
      i = crc_hi ^ *buffer++; /* calculate the CRC  */
      crc_hi = crc_lo ^ table_crc_hi[i];
      crc_lo = table_crc_lo[i];
    }

  return (crc_hi << 8 | crc_lo);
}

void net_tx1(UART_DATA *uart) {

  //GPIO_WriteBit(USART1PPport,USART1PPpin,Bit_SET);
  if((uart->txlen>0) && (uart->txcnt==0)) {
      //включаем rs485 на передачу
      //GPIO_WriteBit(USART1PPport,USART1PPpin,Bit_SET);
      rs485GPIOon();
      delay_ms(2);
      //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //выкл прерывание на прием
      //USART_ITConfig(USART1, USART_IT_TC, ENABLE); //включаем на окочание передачи


      for (uart->txcnt=0; uart->txcnt < uart->txlen; uart->txcnt++) {
          USART_SendData(USART1,(u16) uart->buffer[uart->txcnt]);
          while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
            {
            }
        }
      delay_ms(2);
      rs485GPIOoff();
      //GPIO_WriteBit(USART1PPport,USART1PPpin,Bit_RESET);
      //USART01Send(uart1.buffer);
      //USART_SendData(USART1,(u16) uart->buffer[uart->txcnt]);
    }
}

void MODBUS_SLAVE(UART_DATA *MODBUS) {
  uint16_t tmp, tmp001;
  //recive and checking rx query
  if((MODBUS->buffer[0] != 0) && (MODBUS->rxcnt > 5) && ((MODBUS->buffer[0]==SET_PAR[0]) ||(MODBUS->buffer[0]==255)))  {
      tmp=crc16(MODBUS->buffer,MODBUS->rxcnt-2);
      tmp001 = (uint16_t) ( ( (uint16_t)MODBUS->buffer[MODBUS->rxcnt - 2])<<8)+MODBUS->buffer[MODBUS->rxcnt - 1];
      if(tmp == tmp001)  {
          //если мы сюда попали значит пакет наш и crc совпало - надо проверить поддерживаем ли мы такой запрос
          //choosing function
          switch(MODBUS->buffer[1])
            {
            case 1:
              TX_01(MODBUS);
              break;
            case 2:
              TX_02(MODBUS);
              break;
            case 3:
              TX_03(MODBUS);
              break;
            case 4:
              TX_04(MODBUS); //Read Input Registers
              break;
            case 5:
              TX_05(MODBUS);
              break;
            case 6:
              TX_06(MODBUS);
              break;
            case 16:
              TX_16(MODBUS);
              break;
            default://если нет то выдаем ошибку
              //illegal operation
              TX_EXCEPTION(MODBUS,0x01);
            }
          //добавляем CRC и готовим к отсылке
          //adding CRC16 to reply
          tmp=crc16(MODBUS->buffer,MODBUS->txlen-2);
          MODBUS->buffer[MODBUS->txlen-1]=(uint8_t) tmp;
          MODBUS->buffer[MODBUS->txlen-2]=(uint8_t) (tmp>>8);
          MODBUS->txcnt=0;
        }
    }
  //сброс индикаторов окончания посылки
  MODBUS->rxgap=0;
  MODBUS->rxcnt=0;
  MODBUS->rxtimer=0xFFFF;
}
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type)
{
  //modbus exception - illegal data=01 ,adress=02 etc
  //illegal operation
  MODBUS->buffer[2]=error_type; //exception
  MODBUS->txlen=5; //responce length
}

void TX_01(UART_DATA *MODBUS) {
  read_Coils_RW();
  uint32_t tmp,tmp1;
  uint32_t m=0,n=0;
  int tmp_val;
  u16 tmp_val_pos = 0, tmp001 = 0, tmp002;

  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
  //MODBUS->buffer[2] = data byte count

  //2-3  - starting address
  tmp=(uint32_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

  //4-5 - number of registers
  tmp1=(uint32_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

  //default answer length if error
  n=3;

  //если нас устраивает длина запроса и и стартовый адрес
  if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1))) {
      for(m=0;m<tmp1;m++)  {
          //m == 0 ? tmp001 = 1 : tmp001 *= 2;
          if (tmp001 == 0) {
              tmp001 = 1;
            } else {
              tmp001 *= 2;
            }
          tmp_val= Coils_RW[m+tmp];//читаем текущее значение
          if (tmp_val) {
              tmp_val_pos = tmp_val_pos + tmp001;
            }
          if (m > 3 && (m+1)% 16 == 0) {
              MODBUS->buffer[n+1]=(uint8_t) (tmp_val_pos>>8);
              MODBUS->buffer[n]=(uint8_t) tmp_val_pos;
              n=n+2;
              tmp_val_pos = 0;
              tmp001 = 0;
            }
        }
      if (tmp1 <= 16) {
          MODBUS->buffer[n+1]=(uint8_t) (tmp_val_pos>>8);
          MODBUS->buffer[n]=(uint8_t) tmp_val_pos;
          n=n+2;
        }
      //m%8 == 0 ? tmp002 = 0 : tmp002 = 1;
      if (tmp1%8 == 0) {
          tmp002 = 0;
        } else {
          tmp002 = 1;
        }
      tmp001 = tmp1/8;
      m = tmp001 + tmp002;
      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=(uint8_t) (m); //byte count
      //подготовим к отправке
      MODBUS->txlen=(uint8_t) (m+5); //responce length
    }
  else  {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }
}
void TX_02(UART_DATA *MODBUS) {
  read_Discrete_Inputs_RO();
  uint32_t tmp,tmp1;
  uint32_t m=0,n=0;
  int tmp_val;
  u16 tmp_val_pos = 0, tmp001 = 0, tmp002;

  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
  //MODBUS->buffer[2] = data byte count

  //2-3  - starting address
  tmp=(uint32_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

  //4-5 - number of registers
  tmp1=(uint32_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

  //default answer length if error
  n=3;

  //если нас устраивает длина запроса и и стартовый адрес
  if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1))) {
      for(m=0;m<tmp1;m++)  {
          //m == 0 ? tmp001 = 1 : tmp001 *= 2;
          if (tmp001 == 0) {
              tmp001 = 1;
            } else {
              tmp001 *= 2;
            }
          tmp_val= Discrete_Inputs_RO[m+tmp];//читаем текущее значение
          if (tmp_val) {
              tmp_val_pos = tmp_val_pos + tmp001;
            }
          if (m > 3 && (m+1)% 16 == 0) {
              MODBUS->buffer[n+1]=(uint8_t) (tmp_val_pos>>8);
              MODBUS->buffer[n]=(uint8_t) tmp_val_pos;
              n=n+2;
              tmp_val_pos = 0;
              tmp001 = 0;
            }
        }
      if (tmp1 <= 16) {
          MODBUS->buffer[n+1]=(uint8_t) (tmp_val_pos>>8);
          MODBUS->buffer[n]=(uint8_t) tmp_val_pos;
          n=n+2;
        }
      //m%8 == 0 ? tmp002 = 0 : tmp002 = 1;
      if (tmp1%8 == 0) {
          tmp002 = 0;
        } else {
          tmp002 = 1;
        }
      tmp001 = tmp1/8;
      m = tmp001 + tmp002;
      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=(uint8_t) (m); //byte count
      //подготовим к отправке
      MODBUS->txlen=(uint8_t) (m+5); //responce length
    }
  else  {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }
}
void setCOILS(uint8_t *Coils_RW) {
  //coilTOback();
  if (!Coils_RW[0]) { GPIO_SetBits(GPIOC, GPIO_Pin_13);    } else { GPIO_ResetBits(GPIOC, GPIO_Pin_13); }
  if (Coils_RW[1])  { GPIO_SetBits(GPIOB, GPIO_Pin_11);    } else { GPIO_ResetBits(GPIOB, GPIO_Pin_11);  }
  if (Coils_RW[2])  { GPIO_SetBits(GPIOB, GPIO_Pin_10);    } else { GPIO_ResetBits(GPIOB, GPIO_Pin_10);  }
  if (Coils_RW[3])  { GPIO_SetBits(GPIOB, GPIO_Pin_1);     } else { GPIO_ResetBits(GPIOB, GPIO_Pin_1);   }
  if (Coils_RW[4])  { GPIO_SetBits(GPIOB, GPIO_Pin_0);     } else { GPIO_ResetBits(GPIOB, GPIO_Pin_0);   }
  if (Coils_RW[11])  { servo001use=hold_reg.tmp_u16[11];   } else { servo001use=hold_reg.tmp_u16[10];   }
  if (Coils_RW[12])  { servo002use=hold_reg.tmp_u16[13];   } else { servo002use=hold_reg.tmp_u16[12];   }
  if (Coils_RW[13])  { servo003use=hold_reg.tmp_u16[15];   } else { servo003use=hold_reg.tmp_u16[14];   }
  if (Coils_RW[14])  { servo004use=hold_reg.tmp_u16[17];   } else { servo004use=hold_reg.tmp_u16[16];   }
  if (Coils_RW[15])  { servo005use=hold_reg.tmp_u16[19];   } else { servo005use=hold_reg.tmp_u16[18];   }
  TIM2->CCR2 = servo001use;
  TIM4->CCR4 = servo002use;
  TIM4->CCR1 = servo003use;
  TIM4->CCR2 = servo004use;
  TIM2->CCR4 = servo005use;

  coilTOback();
}
void read_Discrete_Inputs_RO(void) {

  if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) != (uint8_t)Bit_SET) { Discrete_Inputs_RO[0] = 1; }else{ Discrete_Inputs_RO[0] = 0;}
  //GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET ? Discrete_Inputs_RO[1] = 1 : Discrete_Inputs_RO[1] = 0;
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET) { Discrete_Inputs_RO[1] = 1; }else{ Discrete_Inputs_RO[1] = 0;}
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10) == (uint8_t)Bit_SET) { Discrete_Inputs_RO[2] = 1; }else{ Discrete_Inputs_RO[2] = 0;}
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)  == (uint8_t)Bit_SET) { Discrete_Inputs_RO[3] = 1; }else{ Discrete_Inputs_RO[3] = 0;}
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)  == (uint8_t)Bit_SET) { Discrete_Inputs_RO[4] = 1; }else{ Discrete_Inputs_RO[4] = 0;}
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[5] = 1; }else{ Discrete_Inputs_RO[5] = 0;}
  //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == (uint8_t)Bit_SET ? Discrete_Inputs_RO[6] = 1; : Discrete_Inputs_RO[6] = 0;
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[6] = 1; }else{ Discrete_Inputs_RO[6] = 0;}
  //Discrete_Inputs_RO[6] = 1;
  //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == (uint8_t)Bit_SET ? Discrete_Inputs_RO[7] = 1; : Discrete_Inputs_RO[7] = 0;
  //if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[7] = 1; }else{ Discrete_Inputs_RO[7] = 0;}
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[8] = 1; }else{ Discrete_Inputs_RO[8] = 0;}
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[9] = 1; }else{ Discrete_Inputs_RO[9] = 0;}
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[10] = 1; }else{ Discrete_Inputs_RO[10] = 0;}
  Discrete_Inputs_RO[7] = 0;
  for(u8 i = 11; i < 16; i++) {
      Discrete_Inputs_RO[i] = 0;
    }
  for(u8 i = 16; i < 32; i++) {
      Discrete_Inputs_RO[i] = Discrete_Inputs_RO[i-16];
    }
}
/*void startCOILS(uint8_t *Coils_RW) {
  for(u8 i = 0; i < 32; i++) {
      Coils_RW[i] = 0;
    }
}*/
void read_Coils_RW(void) {
  if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) != (uint8_t)Bit_SET) { Coils_RW[0] = 1; }else{ Coils_RW[0] = 0;}
  //GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET ? Coils_RW[1] = 1 : Coils_RW[1] = 0;
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET) { Coils_RW[1] = 1; }else{ Coils_RW[1] = 0;}
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10) == (uint8_t)Bit_SET) { Coils_RW[2] = 1; }else{ Coils_RW[2] = 0;}
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)  == (uint8_t)Bit_SET) { Coils_RW[3] = 1; }else{ Coils_RW[3] = 0;}
  if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)  == (uint8_t)Bit_SET) { Coils_RW[4] = 1; }else{ Coils_RW[4] = 0;}
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)   == (uint8_t)Bit_SET) { Coils_RW[5] = 1; }else{ Coils_RW[5] = 0;}
  //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == (uint8_t)Bit_SET ? Coils_RW[6] = 1; : Coils_RW[6] = 0;
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)   == (uint8_t)Bit_SET) { Coils_RW[6] = 1; }else{ Coils_RW[6] = 0;}
  //Coils_RW[6] = 1;
  //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == (uint8_t)Bit_SET ? Coils_RW[7] = 1; : Coils_RW[7] = 0;
  //if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)   == (uint8_t)Bit_SET) { Coils_RW[7] = 1; }else{ Coils_RW[7] = 0;}
  Coils_RW[7] = 0;

  for(u8 i = 16; i < 32; i++) {
      Coils_RW[i] = Coils_RW[i-16];
    }
  coilTOback();
  setCOILS(Coils_RW);
}
void TX_05(UART_DATA *MODBUS) {
  uint16_t tmp,tmp1;
  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 5; //query type - - stay a same as in recived
  //2-3  -  address
  tmp=(uint16_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); // адрес
  //4-5 - значение 0xff00 ON / 0x0000 OFF
  tmp1=(uint16_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);
  //если нас устраивает
  //tmp = tmp - 1; //                  как-то ТАК :///////////////////////////////
  if(tmp <= 32) {
      if (tmp1 == (uint16_t) 0xff00) {
          Coils_RW[tmp] = 1;
        } else if (tmp1 == 0x0000) {
          Coils_RW[tmp] = 0;
        } else {
          TX_EXCEPTION(MODBUS,0x03);
        }
      //подготовим к отправке
      MODBUS->buffer[2] = (uint8_t) (tmp>>8); //#################    надеюсь поможет
      MODBUS->txlen=(uint8_t) (8); //responce length
    }
  else  {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }
  setCOILS(Coils_RW);
}
void TX_04(UART_DATA *MODBUS) {  //Read Input Registers
  uint16_t tmp,tmp1;
  uint8_t m=0,n=0;
  uint16_t tmp_val;

  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
  //MODBUS->buffer[2] = data byte count

  //2-3  - starting address
  tmp=((((uint16_t)MODBUS->buffer[2])<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения
  //4-5 - number of registers
  tmp1=((((uint16_t)MODBUS->buffer[4])<<8)+MODBUS->buffer[5]);//количество регистров для чтения
  //default answer length if error
  n=3;

  //если нас устраивает длина запроса и и стартовый адрес
  if(((tmp+tmp1) < OBJ_SZ) && (tmp1 < MODBUS_WRD_SZ+1)) {
      for(m=0;m<tmp1;m++)    {
          tmp_val=input_reg.tmp_u16[m+tmp];//читаем текущее значение uint16_t
          MODBUS->buffer[n]=(uint8_t) (tmp_val>>8);
          MODBUS->buffer[n+1]=(uint8_t) tmp_val;
          n=n+2;
        }
      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=m * 2; //byte count
      //подготовим к отправке
      MODBUS->txlen=m * 2 + 5; //responce length
    } else {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }
}
void TX_03(UART_DATA *MODBUS) {
  uint16_t tmp,tmp1, tmp_val = 0;
  uint16_t m=0,n=0;
  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
  //MODBUS->buffer[2] = data byte count
  //2-3  - starting address
  tmp=((((uint16_t)MODBUS->buffer[2])<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения
  //4-5 - number of registers
  tmp1=((((uint16_t)MODBUS->buffer[4])<<8)+MODBUS->buffer[5]);//количество регистров для чтения
  //default answer length if error
  n=3;

  //если нас устраивает длина запроса и и стартовый адрес
  if((((tmp+tmp1)<OBJ_SZ*2) && (tmp1<MODBUS_WRD_SZ+1)))    {
      for(m=0;m<tmp1;m++)    {
          tmp_val=hold_reg.tmp_u16[m+tmp];//читаем текущее значение uint16_t
          MODBUS->buffer[n]=(uint8_t) (tmp_val>>8);
          MODBUS->buffer[n+1]=(uint8_t) tmp_val;
          n=n+2;
        }
      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=(uint8_t) (m*2); //byte count
      //подготовим к отправке
      MODBUS->txlen=(uint8_t) (m*2+5); //responce length
    } else {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }
}
void TX_06(UART_DATA *MODBUS) {
  uint16_t tmp;

  //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived
  //2-3  - adress   , 4-5 - value
  tmp=(uint16_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress
  //MODBUS->buffer[2]  - byte count a same as in rx query

  if(tmp<OBJ_SZ)  {
      hold_reg.tmp_u16[tmp] =((((uint16_t)MODBUS->buffer[4])<<8)+MODBUS->buffer[5]);

      MODBUS->txlen=MODBUS->rxcnt; //responce length
    }
  else
    {
      //illegal data
      TX_EXCEPTION(MODBUS,0x02) ;
    }

}
void coilTOback(void) {
  u16 tmp_val_pos=0;
  u16 tmp001 = 0;
  for(u8 m=0;m<32;m++)  {

      if (tmp001 == 0) {
          tmp001 = 1;
        } else {
          tmp001 *= 2;
        }
      //читаем текущее значение
      if (Coils_RW[m]) {
          tmp_val_pos = tmp_val_pos + tmp001;
        }
      if (m == 15) {
          BKP_WriteBackupRegister(BKP_DR1,tmp_val_pos);
          tmp_val_pos = 0;
          tmp001 = 0;
        } else if (m == 31) {
          BKP_WriteBackupRegister(BKP_DR2,tmp_val_pos);
        }
    }
}
void coilFROMback(void) {
  u16 tmp_val_pos;
  tmp_val_pos = BKP_ReadBackupRegister(BKP_DR1);

  for(u8 m=0;m<32;m++)  {
      Coils_RW[m] = tmp_val_pos % 2;
      tmp_val_pos = tmp_val_pos / 2;
      if (m == 15) {
          tmp_val_pos = BKP_ReadBackupRegister(BKP_DR2);
        }
    }
}
/*void TX_16(UART_DATA *MODBUS) {
  uint16_t tmp, tmp1;
  uint16_t m=0,n=0;
  u8 i;
  //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived
  ///2-3  - starting address
  tmp=((((uint16_t)MODBUS->buffer[2])<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения
  //4-5 - number of registers
  tmp1=((((uint16_t)MODBUS->buffer[4])<<8)+MODBUS->buffer[5]);//количество регистров для чтения
  //default answer length if error
  n=3;

  if((((tmp+tmp1)<OBJ_SZ) && (tmp1<MODBUS_WRD_SZ+1)))    {
      for(m=0;m<tmp1;m++)   {
          i = m/2;


          MODBUS->buffer[n]=f001.tmp_val_u8[3];
          MODBUS->buffer[n+1]=f001.tmp_val_u8[2];
          MODBUS->buffer[n+2]=f001.tmp_val_u8[1];
          MODBUS->buffer[n+3]=f001.tmp_val_u8[0];
          f001.tmp_val_float=res_ftable[i+tmp];//пишем текущее значение
          m++; // ############# Второй раз)))))))))))
          n=n+4;
        }
      MODBUS->txlen=8;
    } else {
      //illegal data
      TX_EXCEPTION(MODBUS,0x02) ;
    }

}
*/
void TX_16(UART_DATA *MODBUS) {
  uint16_t tmp, tmp1;
  //uint16_t n=0;
  //u8 i;
  //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived
  ///2-3  - starting address
  tmp=((((uint16_t)MODBUS->buffer[2])<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения
  //2-3  - adress   , 4-5 - number of value 6==2 num bytes 7-8 value
  tmp1=((((uint16_t)MODBUS->buffer[4])<<8)+MODBUS->buffer[5]);//количество регистров для чтения
  //default answer length if error
  //MODBUS->buffer[6] == 2
  //n=7;
  if((((tmp+tmp1)<OBJ_SZ) && (tmp1<MODBUS_WRD_SZ+1)))    {
      hold_reg.tmp_u16[tmp] =((((uint16_t)MODBUS->buffer[7])<<8)+MODBUS->buffer[8]);

      MODBUS->txlen=8;
    } else {
      //illegal data
      TX_EXCEPTION(MODBUS,0x02) ;
    }

}
void rs485GPIOon (void) {
  GPIO_SetBits(GPIOA,GPIO_Pin_11);
  GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}
void rs485GPIOoff (void) {
  GPIO_ResetBits(GPIOA,GPIO_Pin_11);
  GPIO_SetBits(GPIOA,GPIO_Pin_8);
}
void watercounter (void) {
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)   == (uint8_t)Bit_SET) {

    if (waterplus == 1 ) {
      if (waterpluscount <= 9) {
        waterpluscount++;
          if (waterpluscount == 9) {
          waterplusSET =0;
          }
        } else {
        if (waterplusSET == 0 ) {
          hold_reg.tmp_u32[5] = hold_reg.tmp_u32[5] +5;
          BKP_WriteBackupRegister(BKP_DR27, hold_reg.tmp_u16[8]);
          BKP_WriteBackupRegister(BKP_DR28, hold_reg.tmp_u16[9]);
          waterplusSET =1;
          }
        }
    } else {
    waterplus =1;
    waterpluscount =0;
    }
  }else{
    if (waterplus == 0 ) {
      if (waterpluscount <= 9) {
        waterpluscount++;
          if (waterpluscount == 9) {
          waterplusSET =0;
          }
        } else {
        if (waterplusSET == 0 ) {
          hold_reg.tmp_u32[5] = hold_reg.tmp_u32[5] +5;
          BKP_WriteBackupRegister(BKP_DR27, hold_reg.tmp_u16[8]);
          BKP_WriteBackupRegister(BKP_DR28, hold_reg.tmp_u16[9]);
          waterplusSET =1;
          }
        }
    } else {
    waterplus =0;
    waterpluscount =0;
    }

  }
}
