/*****************************************************************************
Made by: Alexander Plotnikov
Date: 14.05.2019
Name: AD5932_Lib.c
Version: 1.0
Discription: Source file for AD5932 library
*****************************************************************************/




//#include "AD5932_Lib.h"
#include "main.h"

void AD5932_init(AD5932_InitTypeDef *p_hAD5932i)
{
	
	volatile std_union temp;
	temp.istd = 0;
	volatile uint32_t M = 0;
	volatile double M_1 = p_hAD5932i->freq_start;
	M_1 *= 16777216;
	M_1 /= FMCLK;
	M = round(M_1);
	HAL_GPIO_WritePin(AD5932_CTRL_GPIO_Port, AD5932_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	
	//CREG init
	
	temp.istd |= (CREG << ADD_BITS) & 0xF000;
	temp.istd |= (p_hAD5932i->b24 << B24) & (1 << B24);
	temp.istd |= (p_hAD5932i->dacen << DAC_EN) & (1 << DAC_EN);
	temp.istd |= (p_hAD5932i->signal_type << SINE_TRI) & (1 << SINE_TRI);
	temp.istd |= (p_hAD5932i->msbouten << MSBOUT_EN) & (1 << MSBOUT_EN);
	temp.istd |= (p_hAD5932i->inc_type << INT_EXT_INCR) & (1 << INT_EXT_INCR);
	temp.istd |= (p_hAD5932i->syncsel << SYNCSEL) & (1 << SYNCSEL);
	temp.istd |= (p_hAD5932i->syncouten << SYNCOUTEN) & (1 << SYNCOUTEN);
	temp.istd |= 0xD3; //RESERVED BITS
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	
	//F START init
	
	temp.istd = 0;
	temp.istd |= (F_START_LSB << ADD_BITS) & 0xF000;
	temp.istd |= (M & 0x00000FFF);
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	
	temp.istd = 0;
	temp.istd |= (F_START_MSB << ADD_BITS) & 0xF000;
	temp.istd |= (M & 0x00FFF000) >> 12;
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	
	//Delta F init
	
	M = 0;
	M_1 = p_hAD5932i->freq_delta;
	M_1 *= 16777216;
	M_1 /= FMCLK;
	M = round(M_1);
	
	temp.istd = 0;
	temp.istd |= (DELTA_F_LSB << ADD_BITS) & 0xF000;
	temp.istd |= (M & 0x00000FFF);
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	temp.istd = 0;
	temp.istd |= (p_hAD5932i->scan_dir << SCAN_DIR) & (1 << SCAN_DIR);
	temp.istd |= (DELTA_F_MSB << ADD_BITS) & 0xF000;
	temp.istd |= (M >> 12) & 0x000007FF;
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	
	//N increment init
	
	temp.istd = 0;
	temp.istd |= (NINCR << ADD_BITS) & 0xF000;
	temp.istd |= (p_hAD5932i->n_inc & 0x00000FFF);
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	//T interval init
	
	temp.istd = 0;
	temp.istd |= (TINT << ADD_BITS) & 0xF000;
	temp.istd |= (p_hAD5932i->tint & 0x1FFF);
	temp.istd |= (p_hAD5932i->tint_type << MCLK_OR_OUTFREQ) & (1 << MCLK_OR_OUTFREQ);
	
	SPI1_TX_BUFF[0] = temp.cstd[1];
	SPI1_TX_BUFF[1] = temp.cstd[0];
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_TX_BUFF, 2, 1);
	while(hspi1.State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(AD5932_FSYNC_GPIO_Port, AD5932_FSYNC_Pin, GPIO_PIN_SET);
	
	for(int i = 0; i < p_hAD5932i->n_inc; i++)
	{
		HAL_GPIO_WritePin(AD5932_CTRL_GPIO_Port, AD5932_CTRL_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(AD5932_CTRL_GPIO_Port, AD5932_CTRL_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
}
