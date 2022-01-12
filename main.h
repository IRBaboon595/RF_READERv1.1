/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f7xx_hal.h"
#include "math.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "dwt_delay.h"
#include <stdio.h>
#include <time.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define AV_COUNT					7
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
typedef union{
	uint16_t 														istd;
	uint8_t 														cstd[2];
}std_union;

typedef union{
	uint32_t 														listd;
	uint16_t 														istd[2];
	uint8_t 														cstd[4];
}long_std_union;

typedef union{
	uint64_t 														llistd;
	uint32_t 														listd[2];
	uint16_t 														istd[4];
	uint8_t 														cstd[8];
}long_long_std_union;

typedef struct{
	
	uint8_t 														b24;
	uint8_t 														dacen;
	uint8_t 														signal_type;
	uint8_t 														msbouten;
	uint8_t 														inc_type;
	uint8_t 														syncsel;
	uint8_t 														syncouten;
	uint8_t 														scan_dir;
	uint8_t 														tint_type;
	double	 														freq_delta;
	double	 														freq_start;
	uint16_t 														n_inc;
	uint16_t 														tint;
	
}AD5932_InitTypeDef;

typedef struct{
	uint16_t														r_counter;
	uint8_t															anti_backlash_pulse_width;
	uint8_t															lock_detect_precision;
	uint8_t															test_mode_bit;
	uint8_t															band_select_clock;
}ADF4360_RCounterTypeDef;
	
typedef struct{
	uint8_t 														a_counter;
	uint16_t 														b_counter;
	uint8_t															cp_gain_;
	uint8_t															divide_by_2;
	uint8_t															divide_by_2_select;
}ADF4360_NCounterTypeDef;

typedef struct{
	uint8_t															core_power_level;
	uint8_t															counter_reset;
	uint8_t															cp_gain;
	uint8_t															cp_three_state;
	uint8_t															current_setting_1;
	uint8_t															current_setting_2;
	uint8_t															mute_till_lock_detect;
	uint8_t															muxout_control;
	uint8_t															output_power_level;
	uint8_t															phase_detector_polarity;
	uint8_t															power_down_1;
	uint8_t															power_down_2;
	uint8_t															prescaler_value;
}ADF4360_ControlRegTypeDef;

typedef struct{
	ADF4360_ControlRegTypeDef						hctrlregi;
	ADF4360_RCounterTypeDef							hrcounteri;
	ADF4360_NCounterTypeDef							hncounteri;		
}ADF4360_InitTypeDef;

/*typedef enum{
	antenna_1	= 1,
	antenna_2,
	antenna_3
}antennas;

double tag_frequency[18];
double band_frequency[18];
double sweep_step_frequency[18];*/

extern ADC_HandleTypeDef 							hadc1;

extern SPI_HandleTypeDef 							hspi1;
extern SPI_HandleTypeDef 							hspi2;

extern UART_HandleTypeDef 						huart3;

extern AD5932_InitTypeDef 						hAD5932i;

extern ADF4360_InitTypeDef 						hADF4360i;
extern ADF4360_RCounterTypeDef				hADF4360_Ri;
extern ADF4360_NCounterTypeDef				hADF4360_Ni;
extern ADF4360_ControlRegTypeDef			hADF4360_CTRLi;

extern double 												att_steps[6];

extern uint8_t												ser_att;
extern uint8_t												*SPI1_RX_BUFF;
extern uint8_t												*SPI1_TX_BUFF;
extern uint8_t												*SPI2_RX_BUFF;
extern uint8_t												*SPI2_TX_BUFF;
extern uint8_t												*UART_RX_BUFF;
extern uint8_t												*UART_TX_BUFF;
extern uint8_t												UART_first_byte;
extern uint8_t												uart_command;
extern uint8_t												adc_enable;
extern uint8_t												tim2_overflow;
extern uint8_t 												adc_counter;
extern uint8_t												UART_BIG_TX_BUFF[3000];
extern uint8_t 												average_count;

extern long_std_union 								adc_result;
extern std_union											UART_length;

extern uint16_t 											adc_result_massive[AV_COUNT][1000];
extern uint16_t 											adc_result_counter;
extern uint16_t 											adc_global_counter;
extern uint16_t 											detected_signal;
extern uint16_t												rand_num;

/******************************* test parameters ***************************************/
extern double													center_freq;
extern double													band_freq;
extern uint16_t												steps_freq;
extern double													single_step_freq;

/******************************* working parameters ************************************/
extern double 												center_freq_a_1[18];
extern double 												band_freq_a_1[18];
extern double 												single_step_freq_a_1[18];
extern uint16_t												steps_freq_a_1[18];
extern uint8_t 												temp_k;
extern uint8_t 												antenna_start;
extern uint8_t												antenna_stop;
extern uint8_t												tag_start[3];
extern uint8_t												tag_stop[3];
extern uint8_t												mode;

extern const float 										RAND_MAX_F;



/******************* PROTOTYPES ********************/
void Error_Handler(void);
void attenuator_init(uint8_t att_address);
void attenuator_set_att(double attenuation);
void switch_PE_handle(uint8_t pe_state);
void switch_RF_handle(uint8_t rf_state);
void generator_init(double p_freq);
void user_param(double st_freq, double d_freq, uint16_t n_incr, uint8_t msb_out, uint8_t signal_t, uint8_t ext, uint16_t time);
void synthesizer_init(void);
void AD5932_init(AD5932_InitTypeDef *p_hAD5932i);
void ADF4360_init(ADF4360_InitTypeDef *phADF4360i);
void UART_pack_parser(void);
uint8_t xor_handler(uint8_t *mass);
void all_generation_off(void);
void init_transmitter(void);
void start_gen(double freq_vco, double delta, uint16_t nincr);
void start_gen_1(double freq_vco, double delta, uint16_t step);
void ref_gen_maintain(uint8_t ctrl);
void rf_path_ctrl(uint8_t path, uint8_t ant);
void select_path(uint8_t path);
uint16_t median_filter(uint16_t datum);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ON									0x01
#define OFF									0x00

#define MODE_TEST						0x00
#define MODE_WORK						0x01

#define RECEIVER 						0x00
#define TRANSMITTER 				0x01

#define X14_Pin 										GPIO_PIN_13
#define X14_GPIO_Port 							GPIOC
#define X15_Pin 										GPIO_PIN_14
#define X15_GPIO_Port 							GPIOC
#define X16_Pin 										GPIO_PIN_15
#define X16_GPIO_Port 							GPIOC
#define OSC_IN_Pin 									GPIO_PIN_0
#define OSC_IN_GPIO_Port  					GPIOH
#define OSC_OUT_Pin 								GPIO_PIN_1
#define OSC_OUT_GPIO_Port 					GPIOH
#define X10_Pin 										GPIO_PIN_0
#define X10_GPIO_Port 							GPIOC
#define X11_Pin 										GPIO_PIN_1
#define X11_GPIO_Port 							GPIOC
#define X12_Pin 										GPIO_PIN_2
#define X12_GPIO_Port 							GPIOC
#define X13_Pin 										GPIO_PIN_3
#define X13_GPIO_Port 							GPIOC
#define PE42430_V1_Pin 							GPIO_PIN_0
#define PE42430_V1_GPIO_Port 				GPIOA
#define PE42430_V2_Pin 							GPIO_PIN_1
#define PE42430_V2_GPIO_Port 				GPIOA
#define PE42430_V3_Pin 							GPIO_PIN_2
#define PE42430_V3_GPIO_Port 				GPIOA
#define ADC123_3_Pin 								GPIO_PIN_3
#define ADC123_3_GPIO_Port 					GPIOA
#define AD5932_FSYNC_Pin 						GPIO_PIN_4
#define AD5932_FSYNC_GPIO_Port 			GPIOA
#define SPI1_SCK_Pin 								GPIO_PIN_5
#define SPI1_SCK_GPIO_Port 					GPIOA
#define AD5932_CTRL_Pin 						GPIO_PIN_6
#define AD5932_CTRL_GPIO_Port 			GPIOA
#define SPI1_MOSI_Pin 							GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port 				GPIOA
#define AD5932_SYNCOUT_Pin 					GPIO_PIN_4
#define AD5932_SYNCOUT_GPIO_Port 		GPIOC
#define AD5932_INTER_Pin 						GPIO_PIN_0
#define AD5932_INTER_GPIO_Port			GPIOB
#define AD5932_MSBOUT_MCU_Pin 			GPIO_PIN_1
#define AD5932_MSBOUT_MCU_GPIO_Port GPIOB
#define RF_CTRL_2_Pin 							GPIO_PIN_2
#define RF_CTRL_2_GPIO_Port 				GPIOB
#define ADF4360_CE_Pin 							GPIO_PIN_12
#define ADF4360_CE_GPIO_Port 				GPIOB
#define SPI2_SCK_Pin 								GPIO_PIN_13
#define SPI2_SCK_GPIO_Port 					GPIOB
#define ADF4360_LE_Pin 							GPIO_PIN_14
#define ADF4360_LE_GPIO_Port 				GPIOB
#define SPI2_MOSI_Pin 							GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port 				GPIOB
#define ADF4360_MUX_Pin 						GPIO_PIN_6
#define ADF4360_MUX_GPIO_Port 			GPIOC
#define HMC8073_A0_Pin 							GPIO_PIN_7
#define HMC8073_A0_GPIO_Port 				GPIOC
#define HMC8073_A1_Pin 							GPIO_PIN_8
#define HMC8073_A1_GPIO_Port 				GPIOC
#define HMC8073_A2_Pin 							GPIO_PIN_9
#define HMC8073_A2_GPIO_Port 				GPIOC
#define RF_CTRL_AMP_TR_Pin 					GPIO_PIN_8
#define RF_CTRL_AMP_TR_GPIO_Port 		GPIOA
#define RF_CTRL_AMP_RC_Pin 					GPIO_PIN_9
#define RF_CTRL_AMP_RC_GPIO_Port 		GPIOA
#define TEST_LED_Pin 								GPIO_PIN_10
#define TEST_LED_GPIO_Port 					GPIOA
#define X9_Pin 											GPIO_PIN_11
#define X9_GPIO_Port 								GPIOA
#define RF_POWER_SUP_CTRL_Pin 			GPIO_PIN_12
#define RF_POWER_SUP_CTRL_GPIO_Port GPIOA
#define HMC8073_CLK_Pin 						GPIO_PIN_10
#define HMC8073_CLK_GPIO_Port 			GPIOC
#define HMC8073_LE_Pin 							GPIO_PIN_11
#define HMC8073_LE_GPIO_Port 				GPIOC
#define HMC8073_SI_Pin 							GPIO_PIN_12
#define HMC8073_SI_GPIO_Port 				GPIOC
#define RF_CTRL_0_Pin 							GPIO_PIN_2
#define RF_CTRL_0_GPIO_Port 				GPIOD
#define X5_Pin 											GPIO_PIN_5
#define X5_GPIO_Port 								GPIOB
#define X6_Pin 											GPIO_PIN_6
#define X6_GPIO_Port 								GPIOB
#define X7_Pin 											GPIO_PIN_7
#define X7_GPIO_Port 								GPIOB
#define UART3_TX_Pin 								GPIO_PIN_11
#define UART3_TX_GPIO_Port 					GPIOB
#define UART3_RX_Pin 								GPIO_PIN_10
#define UART3_RX_GPIO_Port 					GPIOB
#define X8_Pin 											GPIO_PIN_8
#define X8_GPIO_Port 								GPIOB
#define RE_DE_Pin 									GPIO_PIN_9
#define RE_DE_GPIO_Port 						GPIOB
/* USER CODE BEGIN Private defines */
/******************************************* HMC8073 DEFINES *******************************************/

#define ATT_ADDRESS				0x00

#define RF1								0x01
#define RF2								0x02
#define RF3								0x03

/******************************************* AD5932 DEFINES *******************************************/

/*		COMMON DEFINES		*/
#define FMCLK							50000000
#define TWO_POWER_24			16777216
#define	FSTART_DEF				10000000
#define	DELTA_F_DEF				0										//100000
#define	N_INC_DEF					1										//98								//+1 (the first CTRL STARTS BUT DO NOT GIVE AN INC)
#define	TINT_DEF					50000								//MCLK PERIODS FOR 1MSEC DELAY

/*		REGISTER ADDRESSES		*/
#define CREG							0x0
#define NINCR							0x1
#define	DELTA_F_LSB				0x2
#define	DELTA_F_MSB				0x3
#define	TINT							0x4
#define	F_START_LSB				0xC
#define	F_START_MSB				0xD

/*		CREG BITS		*/
#define B24								11
#define DAC_EN						10
#define SINE_TRI					9
#define MSBOUT_EN					8
#define INT_EXT_INCR			5
#define SYNCSEL						3
#define SYNCOUTEN					2

/*		DELTA FREQ MSB BITS		*/
#define SCAN_DIR					11

/*		TIME INTERVAL BITS		*/
#define MCLK_OR_OUTFREQ		13

/*		COMMON BITS		*/
#define ADD_BITS					12

/*		BIT ACTIONS		*/
#define B24_IND_BYTE			0
#define B24_SER_BYTE			1

#define DAC_OFF						0
#define DAC_ON						1

#define TRIA_SIGNAL				0
#define SINE_SIGNAL				1

#define MSBOUT_OFF				0
#define MSBOUT_ON					1

#define INT_INCRMT				0
#define EXT_INCRMT				1

#define EACH_INCR					0
#define EOS_PULSE					1

#define SYNCOUT_OFF				0
#define SYNCOUT_ON				1

#define SCAN_DIR_POS			0
#define SCAN_DIR_NEG			1

#define TINT_OUTFREQ			0
#define TINT_MCLK					1

/******************************************* ADF4360 DEFINES *******************************************/

/*		COMMON DEFINES		*/
#define R_PARAM						2	
#define A_PARAM						8
#define B_PARAM						28//478//28
#define REF_FREQ_IN				FSTART_DEF+(DELTA_F_DEF*N_INC_DEF)
#define FVCO							((16*B_PARAM/17)+A_PARAM)*REF_FREQ_IN/R_PARAM
#define P_PARAM						16
#define P_PARAM_2					17
#define K_PARAM						((P_PARAM*B_PARAM)+A_PARAM)/R_PARAM
#define FREQ_OUT					2463000000 //2412000000
#define BASE_CENTER_FREQ	2448000000
#define BASE_BAND					10000000
#define BASE_FREQ_STEP		40000

/******************************************* SWEEP DEFINES *******************************************/

#define SWEEP_LEN				200 //460  380  
#define TIME_TX					300
#define TIME_RX				550

/*		REGISTER ADDRESSES		*/
#define CONTROL_L					0x0
#define R_COUNTER					0x1
#define N_COUNTER					0x2

#define ADD_BITS_ADF4360	0

/*		 CONTROL LATCH BITS		*/
#define CPL								2
#define COUNTER_RESET			4
#define MUXOUT_CTRL				5
#define PH_DET_POL				8
#define CP_3_STATE				9
#define CP_GAIN_CL				10
#define MUTE_LD						11
#define OUT_POWER_LVL			12
#define CURRENT_SET_1			14
#define CURRENT_SET_2			17
#define POWER_DOWN_1			20
#define POWER_DOWN_2			21
#define PRESCALER					22

/*		 R COUNTER BITS		*/
#define REF_COUNTER				2
#define ABP_WID						16
#define LOCK_DET_PREC			18
#define TST_MODE_BIT			19
#define BAND_SEL_CLK			20

/*		 N COUNTER BITS		*/
#define A_COUNTER					2
#define B_COUNTER					8
#define CP_GAIN_N_C				21
#define DIV_2							22
#define DIV_2_SEL					23

/*		BIT ACTIONS		*/
/*		CTRL LATCH BIT ACTIONS		*/
#define	CPL_5MA						0x0
#define	CPL_10MA					0x1
#define	CPL_15MA					0x2
#define	CPL_25MA					0x3

#define	NORMAL_STATE			0x0
#define	RESET_STATE				0x1

#define	TRI_STATE					0x0
#define	DIG_LOCK_DET			0x1
#define	N_DIV							0x2
#define	DVDD							0x3
#define	R_DIV							0x4
#define	N_OP_LD						0x5
#define	SDO								0x6
#define	DGND							0x7

#define	PH_DET_NEG				0x0
#define	PH_DET_POS				0x1

#define	CP_NORMAL					0x0
#define	CP_TRISTATE				0x1

#define	CP_GAIN_SET_1			0x0
#define	CP_GAIN_SET_2			0x1

#define	MUTE_DIS					0x0
#define	MUTE_EN					``0x1

#define	OUT_POW_13dBm			0x0
#define	OUT_POW_11dBm			0x1
#define	OUT_POW_8dBm			0x2
#define	OUT_POW_6dBm			0x3

#define	CUR_SET_1_031			0x0
#define	CUR_SET_1_062			0x1
#define	CUR_SET_1_093			0x2
#define	CUR_SET_1_125			0x3
#define	CUR_SET_1_156			0x4
#define	CUR_SET_1_187			0x5
#define	CUR_SET_1_218			0x6
#define	CUR_SET_1_250			0x7

#define	CUR_SET_2_031			0x0
#define	CUR_SET_2_062			0x1
#define	CUR_SET_2_093			0x2
#define	CUR_SET_2_125			0x3
#define	CUR_SET_2_156			0x4
#define	CUR_SET_2_187			0x5
#define	CUR_SET_2_218			0x6
#define	CUR_SET_2_250			0x7

#define	PD1_DIS						0x0
#define	PD1_EN						0x1

#define	PD2_ASYN					0x0
#define	PD2_SYN						0x1

#define	PRESC_8_9					0x0
#define	PRESC_16_17				0x1
#define	PRESC_32_33				0x2
#define	PRESC_32_33_d			0x3

/*		R COUNTER BIT ACTIONS		*/
#define	ABP_3_0NS					0x0
#define	ABP_1_3NS					0x1
#define	ABP_6_0NS					0x2
#define	ABP_3_0NS_				0x3
	
#define	THREE_CYCLES			0x0
#define	FIVE_CYCLES				0x1

#define	BAND_1						0x0
#define	BAND_2						0x1
#define	BAND_4						0x2
#define	BAND_8						0x3

/*		N COUNTER BIT ACTIONS		*/
#define	CP_GAIN_SET_1			0x0
#define	CP_GAIN_SET_2			0x1
	
#define	FUND_OUT					0x0
#define	DIV_BY_2					0x1

#define	FUND_OUT_SEL			0x0
#define	DIV_BY_2_SEL			0x1

/******************************************* UART1 DEFINES *******************************************/

#define	SYNCHRO												0x79
#define UART_ADDR											0x0A

#define ECHO													0x00
#define ADC_CONT_1SEC									0x01
#define GENERATION_CTRL								0x02
#define OUTPUT_SWITCH									0x03
#define SET_ATT												0x04
#define AMP_MANAGE										0x05
#define ADC_ECHO											0x06
#define CRYSTAL_EN										0x07
#define AD5932_CTR										0x08
#define ADF4360_CTR										0x09
#define SEND_FREQ_PARAM								0x0A
#define SEND_FREQ_PARAM_TOTAL					0x0B
#define MODE_SELECT              		  0x0C

#define SERVICE_BITS_LEN							0x06

/******************************************* MEDIAN DEFINES *******************************************/

#define STOPPER 						0                                      /* Smaller than any datum */
#define MEDIAN_FILTER_SIZE  (13)



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
