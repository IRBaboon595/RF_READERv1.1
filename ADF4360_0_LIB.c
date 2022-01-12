/*****************************************************************************
Made by: Alexander Plotnikov
Date: 14.05.2019
Name: ADF4360_0_LIB.c
Version: 1.0
Discription: Source file for AD5932 library
*****************************************************************************/




//#include "ADF4360_0_LIB.h"
#include "main.h"

void ADF4360_init(ADF4360_InitTypeDef *phADF4360i)
{
	long_std_union temp;
	temp.listd = 0;
	HAL_GPIO_WritePin(ADF4360_CE_GPIO_Port, ADF4360_CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET);
	
	//R COUNTER 
	temp.listd |= (R_COUNTER << ADD_BITS_ADF4360);
	temp.listd |= ((phADF4360i->hrcounteri.r_counter << REF_COUNTER) & 0x00FFFC);
	temp.listd |= ((phADF4360i->hrcounteri.anti_backlash_pulse_width << ABP_WID) & 0x030000);
	temp.listd |= ((phADF4360i->hrcounteri.lock_detect_precision << LOCK_DET_PREC) & 0x040000);
	temp.listd &=~ (phADF4360i->hrcounteri.test_mode_bit << TST_MODE_BIT);
	temp.listd |= ((phADF4360i->hrcounteri.band_select_clock << BAND_SEL_CLK) & 0x300000);
	
	SPI2_TX_BUFF[0] = temp.cstd[2];
	SPI2_TX_BUFF[1] = temp.cstd[1];
	SPI2_TX_BUFF[2] = temp.cstd[0];
	
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, SPI2_TX_BUFF, 3, 1);
	while(hspi2.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET);
	
	// CTRL LATCH
	temp.listd = 0;
	temp.listd |= (CONTROL_L << ADD_BITS_ADF4360);
	temp.listd |= ((phADF4360i->hctrlregi.core_power_level << CPL) & 0x000006);
	temp.listd |= ((phADF4360i->hctrlregi.counter_reset << COUNTER_RESET) & 0x000010);
	temp.listd |= ((phADF4360i->hctrlregi.muxout_control << MUXOUT_CTRL) & 0x0000E0);
	temp.listd |= ((phADF4360i->hctrlregi.phase_detector_polarity << PH_DET_POL) & 0x000100);
	temp.listd |= ((phADF4360i->hctrlregi.cp_three_state << CP_3_STATE) & 0x000200);
	temp.listd |= ((phADF4360i->hctrlregi.cp_gain << CP_GAIN_CL) & 0x000400);
	temp.listd |= ((phADF4360i->hctrlregi.mute_till_lock_detect << MUTE_LD) & 0x000800);
	temp.listd |= ((phADF4360i->hctrlregi.output_power_level << OUT_POWER_LVL) & 0x003000);
	temp.listd |= ((phADF4360i->hctrlregi.current_setting_1 << CURRENT_SET_1) & 0x01C000);
	temp.listd |= ((phADF4360i->hctrlregi.current_setting_2 << CURRENT_SET_2) & 0x0E0000);
	temp.listd |= ((phADF4360i->hctrlregi.power_down_1 << POWER_DOWN_1) & 0x100000);
	temp.listd |= ((phADF4360i->hctrlregi.power_down_2 << POWER_DOWN_2) & 0x200000);
	temp.listd |= ((phADF4360i->hctrlregi.prescaler_value << PRESCALER) & 0xC00000);
	
	SPI2_TX_BUFF[0] = temp.cstd[2];
	SPI2_TX_BUFF[1] = temp.cstd[1];
	SPI2_TX_BUFF[2] = temp.cstd[0];
	
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, SPI2_TX_BUFF, 3, 1);
	while(hspi2.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET);
	
	HAL_Delay(6);
	
	//N COUNTER 
	temp.listd = 0;
	temp.listd |= (N_COUNTER << ADD_BITS_ADF4360);
	temp.listd |= ((phADF4360i->hncounteri.a_counter << A_COUNTER) & 0x00007C);
	temp.listd |= ((phADF4360i->hncounteri.b_counter << B_COUNTER) & 0x1FFF00);
	temp.listd |= ((phADF4360i->hncounteri.cp_gain_ << CP_GAIN_N_C) & 0x200000);
	temp.listd |= ((phADF4360i->hncounteri.divide_by_2 << DIV_2) & 0x400000);
	temp.listd |= ((phADF4360i->hncounteri.divide_by_2_select << DIV_2_SEL) & 0x800000);
	
	SPI2_TX_BUFF[0] = temp.cstd[2];
	SPI2_TX_BUFF[1] = temp.cstd[1];
	SPI2_TX_BUFF[2] = temp.cstd[0];
	
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, SPI2_TX_BUFF, 3, 1);
	while(hspi2.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET);
	
}

