/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */


#define PB5_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET)
#define PB5_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET)

#define PB6_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET)
#define PB6_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET)

#define PB7_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET)
#define PB7_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET)

#define PB10_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET)
#define PB10_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET)

#define PC13_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET)
#define PC13_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET)

#define PC3_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_RESET)
#define PC3_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET)

#define PC1_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_RESET)
#define PC1_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET)

#define PC2_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_RESET)
#define PC2_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_SET)

#define PC8_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET)
#define PC8_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET)

#define PC14_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET)
#define PC14_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET)

#define PC15_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET)
#define PC15_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET)


#define PD0_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET)
#define PD0_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_SET)

#define PD1_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_RESET)
#define PD1_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_SET)

#define PD2_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_RESET)
#define PD2_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_SET)

#define PD3_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_RESET)
#define PD3_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_SET)

#define PD4_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_RESET)
#define PD4_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_SET)

#define PD5_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,GPIO_PIN_RESET)
#define PD5_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,GPIO_PIN_SET)

#define PD7_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,GPIO_PIN_RESET)
#define PD7_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,GPIO_PIN_SET)

#define PD8_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_RESET)
#define PD8_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_SET)

#define PD9_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_RESET)
#define PD9_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_SET)

#define PD10_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_RESET)
#define PD10_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_SET)

#define PD11_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,GPIO_PIN_RESET)
#define PD11_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,GPIO_PIN_SET)

#define PD12_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET)
#define PD12_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET)

#define PD13_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET)
#define PD13_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET)

#define PD14_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET)
#define PD14_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET)

#define PD15_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET)
#define PD15_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET)

//---------------------------
#define PE8_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_RESET)
#define PE8_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_SET)

#define PE9_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_RESET)
#define PE9_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_SET)

#define PE10_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,GPIO_PIN_RESET)
#define PE10_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,GPIO_PIN_SET)

#define PE11_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET)
#define PE11_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_SET)

#define PE12_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_RESET)
#define PE12_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_SET)

#define PE13_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_RESET)
#define PE13_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_SET)

//-----

#define PE0_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,GPIO_PIN_RESET)
#define PE0_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,GPIO_PIN_SET)

#define PE1_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_RESET)
#define PE1_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET)

#define PE2_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,GPIO_PIN_RESET)
#define PE2_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,GPIO_PIN_SET)

#define PE3_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET)
#define PE3_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET)

#define PE4_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,GPIO_PIN_RESET)
#define PE4_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,GPIO_PIN_SET)

#define PE5_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,GPIO_PIN_RESET)
#define PE5_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,GPIO_PIN_SET)

#define PE6_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,GPIO_PIN_RESET)
#define PE6_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,GPIO_PIN_SET)

#define PE7_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_RESET)
#define PE7_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_SET)

//----------------------


#define PE14_0 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,GPIO_PIN_RESET)
#define PE14_1 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,GPIO_PIN_SET)

#define PE15_0 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_RESET)
#define PE15_1 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_SET)

#define CS_FPGA2_0 PE8_0 
#define CS_FPGA2_1 PE8_1

#define CS_FPGA1_0 PE15_0
#define CS_FPGA1_1 PE15_1

#define FPGA_CS_0 PE15_0
#define FPGA_CS_1 PE15_1

#define CS_LMK_0 PE14_0
#define CS_LMK_1 PE14_1

#define CS_adc1_0 PE10_0
#define CS_adc1_1 PE10_1

#define CS_adc2_0 PE11_0
#define CS_adc2_1 PE11_1

#define CS_DAC1_0 PE12_0
#define CS_DAC1_1 PE12_1 

#define CS_DAC2_0 PE13_0
#define CS_DAC2_1 PE13_1

#define CS_FLASH_0 PC8_0
#define CS_FLASH_1 PC8_1

#define LED1_1 PC13_1
#define LED1_0 PC13_0

#define LED2_1 PC14_1
#define LED2_0 PC14_0

#define LED3_1 PC15_1
#define LED3_0 PC15_0 

#define SYNC_LMK_0 PD0_0
#define SYNC_LMK_1 PD0_1

#define RESETB_DAC_0 PD1_0
#define RESETB_DAC_1 PD1_1

#define RESET_ADC_0 PD2_0
#define RESET_ADC_1 PD2_1


#define LED1(a) ((a==1)?LED1_1 : LED1_0)
#define LED2(a) ((a==1)?LED2_1 : LED2_0)
#define LED3(a) ((a==1)?LED3_1 : LED3_0)

#define CN0led(a) ((a==1)?PD8_1 : PD8_0)
#define CN1led(a) ((a==1)?PD9_1 : PD9_0)
#define CN2led(a) ((a==1)?PD10_1:PD10_0)
#define CN3led(a) ((a==1)?PD11_1:PD11_0)
#define CN4led(a) ((a==1)?PD12_1:PD12_0)
#define CN5led(a) ((a==1)?PD13_1:PD13_0)
#define CN6led(a) ((a==1)?PD14_1:PD14_0)
#define CN7led(a) ((a==1)?PD15_1:PD15_0)

#define WR(a) ((a==1)?PB6_1:PB6_0)
#define RD(a) ((a==1)?PB7_1:PB7_0)

#define u64 unsigned long long
#define u32 unsigned int
#define u16    unsigned short
#define u8     uint8_t
#define uint8  uint8_t

//---------------------------------------------------------------------
#define Bufer_size   16384u   //16384


// USART1 Receiver buffer
#define RX_BUFFER_SIZE1 64u

#define buf_IO   32u 
#define buf_Word 32u 
#define buf_DATA_Word 200u 
#define BUFFER_SR 200u
#define BUF_STR 64
#define MAX_PL 157u

//------------------------------------------------
//------------------------------------------------

/* typedef struct 		//структура ответной квитанции
{
	u32 Cmd_size;		//размер данных
	u32 Cmd_type;		//тип квитанции
	u32 Cmd_id;			//ID квитанции
	u32 Cmd_time;		//время формирования квитанции (сразу после выполнения команды)
	u8  Cmd_data[32];	//данные квитанции - 0-выполненна , 1 - не выполненна
	u32 N_sch;			//число подготовленных квитанций
}INVOICE;
 */


#define MSG_REPLY 		1
#define MSG_ERROR		2
#define ERROR_CMD_BUF 	1   //были затёртые команды в буфере
#define MSG_CMD_OK		3   //команда выполненна успешно
#define MSG_STATUS_OK	100 //Квитация на статус

#define MSG_ID_CH1   	101 //сообщаем ID миксросхемы LM в канале 1
#define MSG_ID_CH2   	102 //
#define MSG_ID_CH3   	103 //
#define MSG_ID_CH4   	104 //
#define MSG_ID_CH5   	105 //
#define MSG_ID_CH6   	106 //
#define MSG_ID_CH7   	107 //
#define MSG_ID_CH8   	108 //

#define MSG_TEMP_CH1   	111 //сообщаем температуру миксросхемы LM в канале 1
#define MSG_TEMP_CH2   	112 //
#define MSG_TEMP_CH3   	113 //
#define MSG_TEMP_CH4   	114 //
#define MSG_TEMP_CH5   	115 //
#define MSG_TEMP_CH6   	116 //
#define MSG_TEMP_CH7   	117 //
#define MSG_TEMP_CH8   	118 //

#define MSG_U_CH1   	121 //сообщаем напряжение миксросхемы LM в канале 1
#define MSG_U_CH2   	122 //
#define MSG_U_CH3   	123 //
#define MSG_U_CH4   	124 //
#define MSG_U_CH5   	125 //
#define MSG_U_CH6   	126 //
#define MSG_U_CH7   	127 //
#define MSG_U_CH8   	128 //

#define MSG_I_CH1   	131 //сообщаем напряжение миксросхемы LM в канале 1
#define MSG_I_CH2   	132 //
#define MSG_I_CH3   	133 //
#define MSG_I_CH4   	134 //
#define MSG_I_CH5   	135 //
#define MSG_I_CH6   	136 //
#define MSG_I_CH7   	137 //
#define MSG_I_CH8   	138 //

#define MSG_P_CH1   	141 //сообщаем напряжение миксросхемы LM в канале 1
#define MSG_P_CH2   	142 //
#define MSG_P_CH3   	143 //
#define MSG_P_CH4   	144 //
#define MSG_P_CH5   	145 //
#define MSG_P_CH6   	146 //
#define MSG_P_CH7   	147 //
#define MSG_P_CH8   	148 //

#define MSG_PWR_CHANNEL 150 //сообщаем состояние линий питания каналов

//---------команды управления---------------------------------------------
#define CMD_TIME_SETUP  1   //команда установки точного времени, реалтайм.
#define CMD_HELP		2   //вывести в консоль HELP()
#define CMD_TIME 		0   //вывести в консоль текущее время
#define CMD_12V			3   //включить - 1 , выключить - 0 питание +12 Вольт
#define CMD_STATUS		100 //сообщите состояние
#define CMD_LED			200 //команда управления светодиодами лицевой панели
#define CMD_xxx			300
#define CMD_CH_UP		4   //команда включения/выключения каналов питания , канал в данных передаётся (вкл/выкл - инверсные коды)
//------------------------------------------------------------------------
 
#define SIZE_SERVER   1024//размер буфера "Хранилище"  тут хранятся данные команды пришедших пакетов сами команды хранятся в реестре
#define SIZE_ID 	   32 //размер реестра	
#define quantity_CMD   64 //максимальное количество команд 
#define quantity_DATA 	8 //максимальная длинна данных
#define quantity_SENDER 2 //максимальное количество адресатов

 
typedef struct //структура команды
{
	u32 Cmd_size;
	u32 Cmd_type;
	u64 Cmd_id;
	u64 Cmd_time;
	u8  Cmd_data[quantity_DATA];
}Command;

typedef struct //структура сообщения с описанием команд
{
	u32 Msg_size;
	u32 Msg_type;
	u64 Num_cmd_in_msg;
    Command CMD[quantity_CMD];
}Message;

typedef struct //структура фрейма
{
	u16 Frame_size;
	u16 Frame_number;
	u8  Stop_bit;
	u32 Msg_uniq_id;
	u64 Sender_id;
	u64 Receiver_id;
    Message MSG;		
}Frame;

typedef struct //структура "Хранилище"
{
	u8 MeM[SIZE_SERVER];
	u32 INDEX;
	u32 INDEX_LAST;
	u64 CMD_ID;
	u64 SENDER_ID;
	u64 TIME;
	u32 x1;//начало диапазона удалённых индексов
	u32 x2;//конец  диапазона удалённых индексов
}SERVER;


typedef struct //структура "реестр" (ID)
{
	u32 INDEX    	  [SIZE_ID];
	u64 CMD_TYPE 	  [SIZE_ID];
	u64 CMD_ID		  [SIZE_ID];
	u64 SENDER_ID	  [SIZE_ID];
	u64 TIME     	  [SIZE_ID];
	u8  FLAG_REAL_TIME[SIZE_ID]; //флаг реального времени
	u32 N_sch;//число записей в структуре
}ID_SERVER;

typedef struct   //структура команды на исполнение
{
	u32 INDEX;  //индекс команды в структуре SERVER 
	u64 TIME;	//время исполнения
}CMD_RUN;

typedef struct
{
	u32 IDX;				//подсчитанное количество отправителей
	u64 A[quantity_SENDER];	//масств адресов ОТПРАВИТЕЛЕЙ	
}ADR_SENDER;

//--------------------------
void FRAME_DECODE (uint8 *,u32 );
void INIT_SERV_ARCHIV (SERVER *,ID_SERVER *,ADR_SENDER *);
void SERV_WR (u8 ,SERVER *);
void PRINT_SERV (void);
void PRINT_SERV_ID (void);
void STATUS_ID (ID_SERVER *);
u32 ADR_FINDER (u64 ,ADR_SENDER *); //ищем адрес отправителя в структуре адресов (его порядковый номер)
void Set_network(void);
void MSG_SHOW (void);
u32 SEND_UDP_MSG (void);


u32 ERROR_CMD_MSG	//формирует структуру квитанции уровня CMD
(
ID_SERVER *,	//указатель на структуру "реестр"	
Frame *,		//указатель на структуру квитанции
u32 ,			//индекс в реестре
u32 ,			//тип сообщения
u32 ,			//данные сообщения
u32 			//время составления квитанции
);

u32 SERV_ID_WR  //функция заполнения реестра команд
(
Frame  *,		 //указатель на структуру квитанции
SERVER *,     	 //указатель на структуру "Хранилище"
ID_SERVER *,	 //указатель на структуру "реестр"
u32 ,			 //индекс в Хранилище
u32 ,			 //CMD_ID
u32 ,			 //SENDER_ID
u64 ,			 //TIME
u64 			 //CMD_ID
); 

void SERV_ID_DEL //процедура удаления команды из реестра
(
ID_SERVER *,     //указатель на структуру "реестр"
u32 			 //индекс команды в реестре
);

u32 TX_MSG_BUFF (//заполняем транспортный массив
Frame *,		  //структура квитанции
uint8 *,		  //транспортный массив
u32				  //максимальный размер транспортного массива 
);

u32 SYS_CMD_MSG(
ID_SERVER *,	//реестр
Frame *, 		//структура квитанций	
u32 ,	 		//индекс в реестре
u32 ,			//тип сообщения
u32 ,		 	//объём данных сообщения в байтах
u8 *,			//данные сообщения - массив данных
u32 	  		//время составления квитанции
);
//--------------------------
typedef struct  reg_DAC37j82  // объявляю структуру 
{
	u32 R[128];

    u8 qmc_offsetab_ena            :1;
    u8 qmc_offsetcd_ena            :1;
    u8 qmc_corrab_ena              :1;
    u8 qmc_corrcd_ena              :1;
    u8 interp                      :4;
    u8 alarm_zeros_txenable_ena    :1;
    u8 outsum_ena                  :1;
    u8 alarm_zeros_jesddata_ena    :1;
    u8 alarm_out_ena               :1;
    u8 alarm_out_pol               :1;
    u8 pap_ena                     :1;
    u8 inv_sinc_ab_ena             :1;
    u8 inv_sinc_cd_ena             :1;

    u8 sfrac_ena_ab                :1;
    u8 sfrac_ena_cd                :1;
    u8 lfrac_ena_ab                :1;
    u8 lfrac_ena_cd                :1;
    u8 sfrac_sel_ab                :1;
    u8 sfrac_sel_cd                :1;
    u8 daca_compliment             :1;
    u8 dacb_compliment             :1;
    u8 dacc_compliment             :1;
    u8 dacd_compliment             :1;

    u8 dac_bitwidth                :2;
    u8 zer_invalid_data            :1;
    u8 shorttest_ena               :1;
    u8 sif4_ena                    :1;                             
    u8 mixer_ena                   :1;      
    u8 mixer_gain                  :1;                   
    u8 nco_ena                     :1;
    u8 twos                        :1;
    u8 sif_reset                   :1;

    u8 coarse_dac                  :4;
    u8 fifo_error_zeros_data_ena   :1;
    u8 sif_txenable                :1;

    u64 alarms_mask                :48;

    u8 memin_tempdata              :8;
    u8 memin_lane_skew             :5;

    u16 qmc_offseta                :13;
    u16 qmc_offsetb                :13;
    u16 qmc_offsetc                :13;
    u16 qmc_offsetd                :13;
    u16 qmc_gaina                  :11;

    u8 fs8                         :1;
    u8 fs4                         :1;
    u8 fs2                         :1;
    u8 fsm4                        :1;
    u16 qmc_gainb                  :11;
    u16 qmc_gainc                  :11;

    u8 output_delayab_reserved     :2;      
    u8 output_delaycd_reserved     :2;  
    u16 qmc_gaind                  :11;

    u16 qmc_phaseab                :12;
    u16 qmc_phasecd                :12;

    u16 phaseoffsetab              :16;
    u16 phaseoffsetcd              :16;

    u64 phaseaddab                 :48;
    u64 phaseaddcd                 :48;

    u8 vbgr_sleep                  :1;
    u8 biasopamp_sleep             :1;
    u8 tsense_sleep                :1;
    u8 pll_sleep                   :1;
    u8 clkrecv_sleep               :1;
    u8 daca_sleep                  :1;
    u8 dacb_sleep                  :1;
    u8 dacc_sleep                  :1;
    u8 dacd_sleep                  :1;

    u8 extref_ena                  :1;
    u8 dtest_lane                  :3;
    u8 dtest                       :3;
    u8 atest                       :6;

    u8 syncsel_qmoffsetab          :4;
    u8 syncsel_qmoffsetcd          :4;
    u8 syncsel_qmcorrab            :4;
    u8 syncsel_qmcorrcd            :4;  
                        
    u8 syncsel_mixerab             :4; 
    u8 syncsel_mixercd             :4; 
    u8 syncsel_nco                 :4; 
    u8 sif_sync                    :1;

    u8 syncsel_dither              :4;
    u8 syncsel_pap                 :4;
    u8 syncsel_fir5a               :4;

    u8 patha_in_sel                :2;
    u8 pathb_in_sel                :2;
    u8 pathc_in_sel                :2;
    u8 pathd_in_sel                :2;
    u8 patha_out_sel               :2;
    u8 pathb_out_sel               :2;
    u8 pathc_out_sel               :2;
    u8 pathd_out_sel               :2;

    u16 sleep_cntl                 :16;

    u8 cdrvser_sysref_mode         :3;
	u8 clkjesd_div				   :3;
    u8 dither_ena                  :4;
    u8 dither_mixer_ena            :4;
    u8 dither_sra_sel              :4;
    u8 dither_zero                 :1;

    u8 pap_dlylen_sel              :1;
    u8 pap_gain                    :3;

    u16 pap_vth                    :16;

    u8 titest_dieid_read_ena       :1;
    u8 sifdac_ena                  :1;

    u16 sifdac                     :16;

    u8 lockdet_adj                 :3;
    u8 pll_reset                   :1;
    u8 pll_ndivsync_ena            :1;
    u8 pll_ena                     :1;
    u8 pll_cp                      :2;
    u8 pll_n                       :5;
    u8 memin_pll_lfvolt            :3;

    u8 pll_m                       :8;
    u8 pll_p                       :4;

    u8 pll_vcosel                  :1;
    u8 pll_vco                     :6;
    u8 pll_vcoitune                :2;
    u8 pll_cp_adj                  :5;

    u8 syncb_lvds_lopwrb           :1;
    u8 syncb_lvds_lopwra           :1;
    u8 syncb_lvds_lpsel            :1;
    u8 syncb_lvds_effuse_sel       :1;
    u8 lvds_sleep                  :1;
    u8 lvds_sub_ena                :1;
	
	u8 syncb_lvds_sleep			   :1;
	u8 syncb_lvds_sub_ena    	   :1;

    u8 serdes_clk_sel              :1;
    u8 serdes_refclk_div           :3;

    u16 rw_cfgpll                  :16;

    u16 rw_cfgrx0_L                :15;
	
	u16 rw_cfgrx0_H                :15;

    u16 w_cfgrx0                   :16;

    u8 INVPAIR                     :8;

    u16 errorcnt_link0             :16;

    u16 errorcnt_link1             :16;

    u16 errorcnt_link2             :16;

    u16 errorcnt_link3             :16;

    u8 lid0                        :5;
    u8 lid1                        :5;
    u8 lid2                        :5;

    u8 lid3                        :5;
    u8 lid4                        :5;
    u8 lid5                        :5;

    u8 lid6                        :5;
    u8 lid7                        :5;
    u8 subclassv                   :3;
    u8 jesdv                       :1;

    u16 link_assign                :16;

    u8 lane_ena                    :8;
    u8 jesd_test_seq               :2;
    u8 dual                        :1;
    u8 init_state                  :4;
    u8 jesd_reset_n                :1;

    u8 rbd_m1                      :5;
    u8 f_m1                        :8;

    u8 k_m1                        :5;
    u8 l_m1                        :5;

    u8 m_m1                        :8;
    u8 s_m1                        :5;

    u8 nprime_m1                   :5;
    u8 hd                          :1;
    u8 scr                         :1;
    u8 n_m1                        :5;

    u8 match_data                  :8;
    u8 match_specific              :1;
    u8 match_ctrl                  :1;
    u8 no_lane_sync                :1;
    u8 jesd_commaalign_ena         :1;

    u8 adjcnt_link0                :4;
    u8 adjdir_link0                :1;
    u8 bid_link0                   :4;
    u8 cf_link0                    :5;
    u8 cs_link0                    :2;

    u8 did_link0                   :8;
    u8 sync_request_ena_link0      :8;
    u8 disable_err_report_link0    :1;
    u8 phadj_link0                 :1;
    u8 error_ena_link0             :8;

    u8 adjcnt_link1                :4;
    u8 adjdir_link1                :1;
    u8 bid_link1                   :4;
    u8 cf_link1                    :5;
    u8 cs_link1                    :2;

    u8 did_link1                   :8;
    u8 sync_request_ena_link1      :8;

    u8 disable_err_report_link1    :1;
    u8 phadj_link1                 :1;
    u8 error_ena_link1             :8;

    u8 adjcnt_link2                :4;
    u8 adjdir_link2                :1;
    u8 bid_link2                   :4;
    u8 cf_link2                    :5;
    u8 cs_link2                    :2;

    u8 did_link2                   :8;
    u8 sync_request_ena_link2      :8;

    u8 disable_err_report_link2    :1;
    u8 phadj_link2                 :1;
    u8 error_ena_link2             :8;

    u8 adjcnt_link3                :4;
    u8 adjdir_link3                :1;
    u8 bid_link3                   :4;
    u8 cf_link3                    :5;
    u8 cs_link3                    :2;
	u8 did_link3				   :8; 
	u8 sync_request_ena_link3	   :8;

    u8 disable_err_report_link3    :1;   
    u8 phadj_link3                 :1;
    u8 error_ena_link3             :8;

    u8  err_cnt_clr_link3           :1;
    u8  sysref_mode_link3           :3;
    u8  err_cnt_clr_link2           :1;
    u8  sysref_mode_link2           :3;
    u8  err_cnt_clr_link1           :1;
    u8  sysref_mode_link1           :3;
    u8  err_cnt_clr_link0           :1;
    u8  sysref_mode_link0           :3;

    u8  res1                        :8;
    u8  res2                        :8;

    u8  octetpath_sel0            :3;
    u8  octetpath_sel1            :3;
    u8  octetpath_sel2            :3;   
    u8  octetpath_sel3            :3;
    u8  octetpath_sel4            :3;
    u8  octetpath_sel5            :3;
    u8  octetpath_sel6            :3;
    u8  octetpath_sel7            :3;

    u8  syncn_pol                 :1;
    u8  syncncd_sel               :4;
    u8  syncnab_sel               :4;
    u8  syncn_sel                 :4;

    u8  alarm_l_error0            :8;
    u8  alarm_fifo_flags0         :4;

    u8  alarm_l_error1            :8;
    u8  alarm_fifo_flags1         :4;

    u8  alarm_l_error2            :8;
    u8  alarm_fifo_flags2         :4;

    u8  alarm_l_error3            :8;
    u8  alarm_fifo_flags3         :4;

    u8  alarm_l_error4            :8;
    u8  alarm_fifo_flags4         :4;

    u8  alarm_l_error5            :8;
    u8  alarm_fifo_flags5         :4;

    u8  alarm_l_error6            :8;
    u8  alarm_fifo_flags6         :4;
 
    u8  alarm_l_error7            :8;
    u8  alarm_fifo_flags7         :4;

    u8  alarm_sysref_err          :4;
    u8  alarm_pap                 :4;
    u8  alarm_rw0_pll             :1;
    u8  alarm_rw1_pll             :1;
    u8  alarm_from_pll            :1;

    u8  alarm_from_shorttest      :8;
    u8  memin_rw_losdct           :8;

    u8  sfrac_coef0_ab            :2;
    u8  sfrac_coef1_ab            :5;
    u8  sfrac_coef2_ab            :8;
    u16 sfrac_coef3_ab            :16;
    u8  sfrac_coef4_ab            :3;
    u16 sfrac_coef5_ab            :10;
    u16 sfrac_coef6_ab            :9;
    u8  sfrac_coef7_ab            :7;
    u8  sfrac_coef8_ab            :5;
    u8  sfrac_coef9_ab            :2;

    u32 sfrac_invgain_ab          :20;  
 
    u8  lfras_coefsel_a           :3;
    u8  lfras_coefsel_b           :3;

    u8  sfrac_coef0_cd              :2;
    u8  sfrac_coef1_cd              :5;
    u8  sfrac_coef2_cd              :8;
    u16  sfrac_coef3_cd             :10;
    u32 sfrac_coef4_cd              :19;
    u16 sfrac_coef5_cd              :10;
    u16  sfrac_coef6_cd             :9;
    u8  sfrac_coef7_cd              :7;
    u8  sfrac_coef8_cd              :5;
    u8  sfrac_coef9_cd              :2;
    u32 sfrac_invgain_cd            :20;
    u8  lfras_coefsel_c             :3;
    u8  fras_coefsel_d              :3;

    u8 memin_efc_autoload_done     :1;
    u8 memin_efc_error             :5;
    u8 vendorid                    :2;
    u8 versionid                   :3;
	u8 lfras_coefsel_d			   :3;
	
	u8 output_delayab			:2;
	u8 output_delaycd			:2;
	
	u8 alarm_i_error0 :8;
	u8 alarm_i_error1 :8;
	u8 alarm_i_error2 :8;
	u8 alarm_i_error3 :8;
	u8 alarm_i_error4 :8;
	u8 alarm_i_error5 :8;
	u8 alarm_i_error6 :8;
	u8 alarm_i_error7 :8;
	
	u8 lfrac_coefsel_c :3;
	u8 lfrac_coefsel_d :3;
 
} reg_DAC37j82;


//--------------------------
typedef struct  reg_ADC42jb49  // объявляю структуру 
{
	u8 R[64];

    u8 CLK_DIV        		:2;
    u8 SYSREF_DELAY   		:3;
    u8 PDN_CHA        		:1;
    u8 PDN_CHB        		:1;
    u8 STDBY          		:1;
    u8 DATA_FORMAT    		:1;
    u8 RESET          		:1;
    u8 CHA_GAIN       		:5;
    u8 CHA_GAIN_EN    		:1;
    u8 CHB_GAIN        		:5;
    u8 CHB_GAIN_EN    		:1;
    u8 HIGH_FREQ_1    		:1;
    u8 FAST_OVR_EN   		:1;
	u8 HIGH_FREQ_2    		:1;
	u8 CHA_TEST_PATTERNS 	:4;
	u8 CHB_TEST_PATTERNS 	:4;
	u16 CUSTOM_PATTERN1		:16;
	u16 CUSTOM_PATTERN2		:16;
	u8 FAST_OVR_THRESHOLD   :7;
	u8 SERDES_TEST_PATTERN	:2;
	u8 IDLE_SYNC			:1;
	u8 TESTMODE_EN			:1;
	u8 FLIP_ADC_DATA		:1;
	u8 LANE_ALIGN			:1;
	u8 FRAME_ALIGN			:1;
	u8 TX_LINK_CONFIG_DATA0 :1;
	u8 CTRLK				:1;
	u8 CTRLF				:1;
	u8 SCRAMBLE_EN			:1;
	u8 OCTETS_PER_FRAME		:1;
	u8 FRAMES_PER_MULTIFRAME:5;
	u8 SUBCLASS				:3;
	u8 SYNC_REQ				:1;
	u8 LMFC_RESET_MASK		:1;
	u8 OUTPUT_CURRENT_SEL	:4;
	u8 LINK_LAYER_TESTMODE	:3;
	u8 LINK_LAYER_RPAT		:1;
	u8 PULSE_DET_MODES		:3;
	u8 FORCE_LMFC_COUNT		:1;
	u8 LMFC_COUNT_INIT		:5;
	u8 RELEASE_ILANE_SEQ	:2; 
} reg_ADC42jb49;
//---------------------------
typedef struct reg_lmk04828   // объявляю структуру 
{
	u32 R[125];
	u8  RESET		  	  	: 1;
	u8  SPI_3WIRE_DIS 	  	: 1;
	u8  POWERDOWN	  	  	: 1;
	u8  ID_DEVICE_TYPE	  	: 8;
	u16 ID_PROD		  	  	:16;
	u8	ID_MASKREV	  	  	: 8;
    u16 ID_VNDR		  	  	:16;
    u8	CLKout0_1_ODL 	  	: 1;
    u8	CLKout2_3_ODL 	  	: 1;
    u8	CLKout4_5_ODL 	  	: 1;
    u8	CLKout6_7_ODL 	  	: 1;
	u8	CLKout8_9_ODL 	  	: 1;
	u8	CLKout10_11_ODL   	: 1;
	u8	CLKout12_13_ODL   	: 1;
	
    u8  CLKout0_1_IDL 	  	: 1;
	u8  CLKout2_3_IDL 	  	: 1;
	u8  CLKout4_5_IDL 	  	: 1;
	u8  CLKout6_7_IDL 	  	: 1;
	u8  CLKout8_9_IDL 	  	: 1;
	u8  CLKout10_11_IDL	  	: 1;
	u8  CLKout12_13_IDL   	: 1;
	
    u8  DCLKout0_DIV  	  	: 5;
    u8  DCLKout2_DIV  	  	: 5;
    u8  DCLKout4_DIV  	  	: 5;
    u8  DCLKout6_DIV  	  	: 5;
    u8  DCLKout8_DIV  	  	: 5;
    u8  DCLKout10_DIV  	  	: 5;
    u8  DCLKout12_DIV  	  	: 5;
    u8  DCLKout0_DDLY_CNTH	: 4;
    u8  DCLKout2_DDLY_CNTH	: 4;
    u8  DCLKout4_DDLY_CNTH	: 4;
    u8  DCLKout6_DDLY_CNTH	: 4;
    u8  DCLKout8_DDLY_CNTH	: 4;
    u8  DCLKout10_DDLY_CNTH	: 4;
    u8  DCLKout12_DDLY_CNTH	: 4;
    u8  DCLKout0_DDLY_CNTL	: 4;
    u8  DCLKout2_DDLY_CNTL	: 4;
    u8  DCLKout4_DDLY_CNTL	: 4;
    u8  DCLKout6_DDLY_CNTL	: 4;
    u8  DCLKout8_DDLY_CNTL	: 4;
    u8  DCLKout10_DDLY_CNTL	: 4;
    u8  DCLKout12_DDLY_CNTL	: 4;
	
    u8  DCLKout0_ADLY	  	: 5;
    u8  DCLKout2_ADLY	  	: 5;
    u8  DCLKout4_ADLY	  	: 5;
    u8  DCLKout6_ADLY	  	: 5;
    u8  DCLKout8_ADLY	  	: 5;
    u8  DCLKout10_ADLY	  	: 5;
    u8  DCLKout12_ADLY	  	: 5;
	
	u8  DCLKout0_ADLY_PD  	: 1;
    u8  DCLKout2_ADLY_PD  	: 1;
    u8  DCLKout4_ADLY_PD  	: 1;
    u8  DCLKout6_ADLY_PD  	: 1;
    u8  DCLKout8_ADLY_PD  	: 1;
    u8  DCLKout10_ADLY_PD  	: 1;
    u8  DCLKout12_ADLY_PD  	: 1;
	
    u8  DCLKout0_ADLY_MUX 	: 1;
    u8  DCLKout2_ADLY_MUX 	: 1;
    u8  DCLKout4_ADLY_MUX 	: 1;
    u8  DCLKout6_ADLY_MUX 	: 1;
    u8  DCLKout8_ADLY_MUX 	: 1;
    u8  DCLKout10_ADLY_MUX 	: 1;
    u8  DCLKout12_ADLY_MUX 	: 1;
    u8  DCLKout0_MUX       	: 2;
	u8  DCLKout2_MUX       	: 2;
	u8  DCLKout4_MUX       	: 2;
	u8  DCLKout6_MUX       	: 2;
	u8  DCLKout8_MUX       	: 2;
	u8  DCLKout10_MUX      	: 2;
	u8  DCLKout12_MUX      	: 2;
    u8	DCLKout0_HS			: 1;
    u8	DCLKout2_HS			: 1;
    u8	DCLKout4_HS			: 1;
    u8	DCLKout6_HS			: 1;
    u8	DCLKout8_HS			: 1;
    u8	DCLKout10_HS		: 1;
    u8	DCLKout12_HS		: 1;

    u8  SDCLKout1_MUX		: 1;
    u8  SDCLKout3_MUX		: 1;
    u8  SDCLKout5_MUX		: 1;
    u8  SDCLKout7_MUX		: 1;
    u8  SDCLKout9_MUX		: 1;
    u8  SDCLKout11_MUX		: 1;
    u8  SDCLKout13_MUX		: 1;
    u8	SDCLKout1_DDLY		: 4;
    u8	SDCLKout3_DDLY		: 4;
    u8	SDCLKout5_DDLY		: 4;
    u8	SDCLKout7_DDLY		: 4;
    u8	SDCLKout9_DDLY		: 4;
    u8	SDCLKout11_DDLY		: 4;
    u8	SDCLKout13_DDLY		: 4;
    u8  SDCLKout1_HS		: 1;
    u8  SDCLKout3_HS		: 1;
    u8  SDCLKout5_HS		: 1;
    u8  SDCLKout7_HS		: 1;
    u8  SDCLKout9_HS		: 1;
    u8  SDCLKout11_HS		: 1;
    u8  SDCLKout13_HS		: 1;

    u8  SDCLKout1_ADLY_EN	: 1;
    u8  SDCLKout3_ADLY_EN	: 1;
    u8  SDCLKout5_ADLY_EN	: 1;
    u8  SDCLKout7_ADLY_EN	: 1;
    u8  SDCLKout9_ADLY_EN	: 1;
    u8  SDCLKout11_ADLY_EN	: 1;
    u8  SDCLKout13_ADLY_EN	: 1;

    u8  SDCLKout1_ADLY 		: 4;
    u8  SDCLKout3_ADLY 		: 4;
    u8  SDCLKout5_ADLY 		: 4;
    u8  SDCLKout7_ADLY 		: 4;
    u8  SDCLKout9_ADLY 		: 4;
    u8  SDCLKout11_ADLY		: 4;
    u8  SDCLKout13_ADLY		: 4;
	
    u8  DCLKout0_DDLY_PD	: 1;
    u8  DCLKout2_DDLY_PD	: 1;
    u8  DCLKout4_DDLY_PD	: 1;
    u8  DCLKout6_DDLY_PD	: 1;
    u8  DCLKout8_DDLY_PD	: 1;
    u8  DCLKout10_DDLY_PD	: 1;
    u8  DCLKout12_DDLY_PD	: 1;

    u8  DCLKout0_HSg_PD		: 1;
    u8  DCLKout2_HSg_PD		: 1;
    u8  DCLKout4_HSg_PD		: 1;
    u8  DCLKout6_HSg_PD		: 1;
    u8  DCLKout8_HSg_PD		: 1;
    u8  DCLKout10_HSg_PD	: 1;
    u8  DCLKout12_HSg_PD	: 1;

	u8  DCLKout0_ADLYg_PD	: 1;
	u8  DCLKout2_ADLYg_PD	: 1;
	u8  DCLKout4_ADLYg_PD	: 1;
	u8  DCLKout6_ADLYg_PD	: 1;
	u8  DCLKout8_ADLYg_PD	: 1;
	u8  DCLKout10_ADLYg_PD	: 1;
	u8  DCLKout12_ADLYg_PD	: 1;
	
    u8  DCLKout_ADLYg_PD	: 1;
    u8  DCLKout_ADLY_PD 	: 1;
	
    u8  CLKout0_1_PD		: 1;
    u8  CLKout2_3_PD		: 1;
    u8  CLKout4_5_PD		: 1;
    u8  CLKout6_7_PD		: 1;
    u8  CLKout8_9_PD		: 1;
    u8  CLKout10_11_PD		: 1;
    u8  CLKout12_13_PD		: 1;

    u8  SDCLKout1_DIS_MODE	: 2;
    u8  SDCLKout3_DIS_MODE	: 2;
    u8  SDCLKout5_DIS_MODE	: 2;
    u8  SDCLKout7_DIS_MODE	: 2;
    u8  SDCLKout9_DIS_MODE	: 2;
    u8  SDCLKout11_DIS_MODE	: 2;
    u8  SDCLKout13_DIS_MODE	: 2;
    u8  SDCLKout1_PD		: 1;
    u8  SDCLKout3_PD		: 1;
    u8  SDCLKout5_PD		: 1;
    u8  SDCLKout7_PD		: 1;
    u8  SDCLKout9_PD		: 1;
    u8  SDCLKout11_PD		: 1;
    u8  SDCLKout13_PD		: 1;

    u8  SDCLKout1_POL		: 1;
    u8  SDCLKout3_POL		: 1;
    u8  SDCLKout5_POL		: 1;
    u8  SDCLKout7_POL		: 1;
    u8  SDCLKout9_POL		: 1;
    u8  SDCLKout11_POL		: 1;
    u8  SDCLKout13_POL		: 1;

    u8  SDCLKout1_FMT		: 3;
    u8  SDCLKout3_FMT		: 3;
    u8  SDCLKout5_FMT		: 3;
    u8  SDCLKout7_FMT		: 3;
    u8  SDCLKout9_FMT		: 3;
    u8  SDCLKout11_FMT		: 3;
    u8  SDCLKout13_FMT		: 3;

    u8  DCLKout0_POL		: 1;
    u8  DCLKout2_POL		: 1;
    u8  DCLKout4_POL		: 1;
    u8  DCLKout6_POL		: 1;
    u8  DCLKout8_POL		: 1;
    u8  DCLKout10_POL		: 1;
    u8  DCLKout12_POL		: 1;
	
    u8  CLKout0_FMT			: 3;
    u8  CLKout2_FMT			: 3;
    u8  CLKout4_FMT			: 3;
    u8  CLKout6_FMT			: 3;
    u8  CLKout8_FMT			: 3;
    u8  CLKout10_FMT		: 3;
    u8  CLKout12_FMT		: 3;

    u8  VCO_MUX				: 2;

    u8  OSCout_MUX			: 1;
    u8  OSCout_FMT			: 4;
    u8  SYSREF_CLKin0_MUX   : 1;
    u8  SYSREF_MUX			: 2;
    u16 SYSREF_DIV			:13;
    u16 SYSREF_DDLY			:13;
    u8 SYSREF_PULSE_CNT		: 2;
    u8 PLL2_NCLK_MUX		: 1;
    u8 PLL1_NCLK_MUX		: 1;
    u8 FB_MUX				: 2;
    u8 FB_MUX_EN			: 1;
    u8 PLL1_PD				: 1;
    u8 VCO_LDO_PD			: 1;
    u8 VCO_PD 				: 1;
    u8 OSCin_PD				: 1;
    u8 SYSREF_GBL_PD		: 1;
    u8 SYSREF_PD 			: 1;
    u8 SYSREF_DDLY_PD		: 1;
    u8 SYSREF_PLSR_PD  		: 1;
    u8 DDLYd_SYSREF_EN		: 1;
    u8 DDLYd12_EN			: 1;
    u8 DDLYd10_EN			: 1;
    u8 DDLYd8_EN			: 1;
	u8 DDLYd7_EN			: 1;
    u8 DDLYd6_EN			: 1;
    u8 DDLYd4_EN			: 1;
    u8 DDLYd2_EN			: 1;
    u8 DDLYd0_EN			: 1;
    u8 DDLYd_STEP_CNT		: 4;
    u8 SYSREF_CLR			: 1;
	u8 SYSREF_DDLY_CLR		: 1;
    u8 SYNC_1SHOT_EN		: 1;
    u8 SYNC_POL				: 1;
    u8 SYNC_EN 				: 1;
    u8 SYNC_PLL2_DLD		: 1;
    u8 SYNC_PLL1_DLD		: 1;
    u8 SYNC_MODE			: 2;
    u8 SYNC_DISSYSREF		: 1;
    u8 SYNC_DIS12			: 1;
    u8 SYNC_DIS10			: 1;
    u8 SYNC_DIS8			: 1;
    u8 SYNC_DIS6			: 1;
    u8 SYNC_DIS4			: 1;
    u8 SYNC_DIS2			: 1;
    u8 SYNC_DIS0			: 1;
    u8 Fixed_Register		: 8;
    u8 CLKin2_EN			: 1;
    u8 CLKin1_EN			: 1;
    u8 CLKin0_EN			: 1;
    u8 CLKin2_TYPE			: 1;
    u8 CLKin1_TYPE			: 1;
    u8 CLKin0_TYPE			: 1;
    u8 CLKin_SEL_POL		: 1;
    u8 CLKin_SEL_MODE		: 3;
    u8 CLKin1_OUT_MUX		: 2;
    u8 CLKin0_OUT_MUX		: 2;
    u8 CLKin_SEL0_MUX		: 3;
    u8 CLKin_SEL0_TYPE		: 3;
    u8 SDIO_RDBK_TYPE		: 1;
    u8 CLKin_SEL1_MUX		: 3;
    u8 CLKin_SEL1_TYPE		: 3;
    u8 RESET_MUX			: 3;
    u8 RESET_TYPE			: 3;
    u8 LOS_TIMEOUT			: 2;
    u8 LOS_EN				: 1;
    u8 TRACK_EN				: 1;
    u8 HOLDOVER_FORCE		: 1;
    u8  MAN_DAC_EN			: 1;
	u16 MAN_DAC			    : 10;
    u8 DAC_TRIP_LOW	  		: 6;
    u8 DAC_CLK_MULT			: 2;
    u8 DAC_TRIP_HIGH		: 6;
    u8 DAC_CLK_CNTR 		: 8;
    u8 HOLDOVER_PLL1_DET	: 1;
    u8 HOLDOVER_LOS_DET		: 1;
    u8 HOLDOVER_VTUNE_DET	: 1;
    u8 HOLDOVER_HITLESS_SWITCH:1;
    u8 HOLDOVER_EN 			: 1;
    u16 HOLDOVER_DLD_CNT	:14;
    u16 CLKin0_R			:14;
    u16 CLKin1_R			:14;
    u16 CLKin2_R			:14;
    u16 PLL1_N 				:14;
    u8 PLL1_WND_SIZE		: 2;
    u8 PLL1_CP_TRI			: 1;
    u8 PLL1_CP_POL			: 1;
    u8 PLL1_CP_GAIN			: 4;
    u16 PLL1_DLD_CNT		:14;
    u8 PLL1_R_DLY			: 3;
    u8 PLL1_N_DLY			: 3;
    u8 PLL1_LD_MUX			: 5;
    u8 PLL1_LD_TYPE			: 3;
    u16 PLL2_R				:12;
    u8 PLL2_P				: 3;
    u8 OSCin_FREQ			: 3;
    u8 PLL2_XTAL_EN			: 1;
    u8 PLL2_REF_2X_EN		: 1;
    u32 PLL2_N_CAL			:18;
    u32 PLL2_N  			:18;
    u8 PLL2_FCAL_DIS		: 1;
    u8 PLL2_WND_SIZE		: 2;
    u8 PLL2_CP_GAIN			: 2;
    u8 PLL2_CP_POL			: 1;
    u8 PLL2_CP_TRI			: 1;
    u8 Fixed_Value_0x169	: 1;
    u16 PLL2_DLD_CNT		:16;
    u8 SYSREF_REQ_EN		: 1;
    u8 PLL2_LF_R4			: 3;
    u8 PLL2_LF_R3			: 3;
    u8 PLL2_LF_C4			: 4;
    u8 PLL2_LF_C3			: 4;
    u8 PLL2_LD_MUX			: 5;
    u8 PLL2_LD_TYPE			: 3;
    u8 PLL2_PRE_PD			: 1;
    u8 PLL2_PD 				: 1;
    u8 OPT_REG_1 			: 8; 
    u8 OPT_REG_2			: 8;
    u8 RB_PLL1_LD_LOST		: 1;
    u8 RB_PLL1_LD 			: 1;
    u8 CLR_PLL1_LD_LOST		: 1;
    u8 RB_PLL2_LD_LOST 		: 1;
    u8 RB_PLL2_LD 			: 1;
    u8 CLR_PLL2_LD_LOST		: 1;
    u16 RB_DAC_VALUE		:10;
    u8 RB_CLKin2_SEL		: 1;
    u8 RB_CLKin1_SEL		: 1;
    u8 RB_CLKin0_SEL		: 1;
    u8 RB_CLKin1_LOS		: 1;
    u8 RB_CLKin0_LOS		: 1;
    u8 RB_HOLDOVER			: 1;
    u8 SPI_LOCK				: 8;

} reg_lmk04828;


//---------------------------

u32 IO ( char* );
void test_delay (u32 );
u8 PIN_control_PA8 (void);
u8 PIN_control (void);
void ATT_upr (u8 ,u8 );
void FAPCH_INIT (void);
void LED (void);
void SDRAM_test_wr (u32,u16);
u16 SDRAM_test_rd (u32 ,u16); 
void test3_sdram(u16 );
void test2_SDRAM(u16 );
void test_SDRAM (u16 );
char getchar1(void);
void Menu1(char );
u64 FPGA_rSPI (u8,u8);
u64 FPGA2_rSPI(u8,u8);
u32 FPGA_wSPI (u8,u8,u64);
u32 FPGA2_wSPI(u8,u8,u64);
void spisend_FPGA (u8,u8);
u8 spisend8 (u8);
void spisend32 (u32);
void h_out (u64,u8);
void hn_out(u64,u8);
void i_out (u64,u8);
void in_out(u64,u8);
void xn_out (char *,u32);
void x_out (char *,u32);
void x32_out (char *,u32);
void nu_out (char *,u32);
void un_out (char *,u32);
void u_out (char *,u32);
void d_out (char *,int);
void f_out (char *,double);
void Transf(char* );
unsigned int leng ( char *);
void itoa(int ,  char *, int);
volatile void delay_us( uint32_t );
void Delay( unsigned int ) ;
u32 alarm_dac1_serdes_pll (u8);
void dac1_info (u32);
void spisend_dac1(u32);
void write_reg_dac1 (u8,u16);
u16 spiread_dac1 (unsigned short);
u32 alarm_dac1_serdes_pll (u8 );
void dac1_info (u32 );
void dac1_write (char );
void dac1_read_reg (void);
void DAC1_dsp_init(u8 );
void DAC1_coarse_dac (u8);
void DAC1_mixer_gain (u8);
void DAC1_QMC_gain (u16);

void spisend_dac2 (u32);
void write_reg_dac2(u8,u16);
u16 spiread_dac2 (unsigned short);
u32 alarm_dac2_serdes_pll (u8 );
void dac2_info (u32);
void dac2_write (char);
void dac2_read_reg (void);
void DAC2_coarse_dac(u8);
void DAC2_mixer_gain(u8);
void DAC2_QMC_gain  (u16);
void DAC2_dsp_init (u8);
void pll_init (u8,u8,u8,u8,u8,u8);
void init_dac_temp (void);
void init_dac1 (u8);
void defrag_REG_ADC (reg_ADC42jb49 *);
void INIT_REG_ADC   (reg_ADC42jb49 *);
void write_reg_adc2 (u8,u8);
u8 spiread_adc2     (u8);
void adc2_write (char);
void adc2_test (u8);
void adc2_read_reg (void);
void adc2_fifo_read (u8,u8);
void write_reg_adc1 (u8,u8);
u8 spiread_adc1 (u8 );
void adc1_write (char);
void adc1_test (u8);
void adc1_read_reg (void);
void adc1_fifo_read (u8,u8);
void INIT_REG_LMK (void);
void spisend_lmk (unsigned int);
char spiread_lmk (unsigned short);
void lmk_write (char);
void init_lmk_temp (void);
void init_lmk (u8);
void ADC_test (void);
void ETH0_MAC_write (u64 );
void ETH1_MAC_write (u64 );
void ETH0_PORT_write (u32 ,u16 );
void ETH1_PORT_write (u32 ,u16 );
void ETH0_IP_write (u64 ,u32 );
void ETH1_IP_write (u64 ,u32 );

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
