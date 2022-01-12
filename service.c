/*****************************************************************************
Made by: Alexander Plotnikov
Date: 14.05.2019
Name: service.c
Version: 1.0
Discription: Source file for user middleware functions
*****************************************************************************/



#include "main.h"






/******************************* HMC8073 Attenuator Functions *********************************************/

void attenuator_init(uint8_t att_address)
{
	HAL_GPIO_WritePin(HMC8073_LE_GPIO_Port, HMC8073_LE_Pin, GPIO_PIN_SET);
	
	if((att_address & 0x01) == 0)
	{
		HAL_GPIO_WritePin(HMC8073_A0_GPIO_Port, HMC8073_A0_Pin, GPIO_PIN_RESET);
	}
	else if((att_address & 0x01) == 1)
	{
		HAL_GPIO_WritePin(HMC8073_A0_GPIO_Port, HMC8073_A0_Pin, GPIO_PIN_SET);
	}
	
	if(((att_address & 0x02) >> 1) == 0)
	{
		HAL_GPIO_WritePin(HMC8073_A1_GPIO_Port, HMC8073_A1_Pin, GPIO_PIN_RESET);
	}
	else  if(((att_address & 0x02) >> 1) == 1)
	{
		HAL_GPIO_WritePin(HMC8073_A1_GPIO_Port, HMC8073_A1_Pin, GPIO_PIN_SET);
	}
	
	if(((att_address & 0x04) >> 2) == 0)
	{
		HAL_GPIO_WritePin(HMC8073_A2_GPIO_Port, HMC8073_A2_Pin, GPIO_PIN_RESET);
	}
	else  if(((att_address & 0x04) >> 2) == 1)
	{
		HAL_GPIO_WritePin(HMC8073_A2_GPIO_Port, HMC8073_A2_Pin, GPIO_PIN_SET);
	}
	
	
}

void attenuator_set_att(double attenuation)
{
	ser_att = 0;
	uint8_t ser_att_n = 1;
	uint8_t att_address = ATT_ADDRESS;
	
	for (int i = 0; i < 6; i++)
	{
		attenuation -= att_steps[i];
		if(attenuation >= 0)
		{
			ser_att += (1 << i);
		}
		else
		{
			attenuation += att_steps[i];
		}
	}
	
	ser_att = (ser_att << 1) | 0x81;
	
	HAL_GPIO_WritePin(HMC8073_LE_GPIO_Port, HMC8073_LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HMC8073_CLK_GPIO_Port, HMC8073_CLK_Pin, GPIO_PIN_RESET);
	
	for(int i = 7; i >= 0; i--)
	{
		if((ser_att & (ser_att_n << i)) != 0)
		{
			HAL_GPIO_WritePin(HMC8073_SI_GPIO_Port, HMC8073_SI_Pin, GPIO_PIN_SET);
		}
		else if((ser_att & (ser_att_n << i)) == 0)
		{
			HAL_GPIO_WritePin(HMC8073_SI_GPIO_Port, HMC8073_SI_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(HMC8073_CLK_GPIO_Port, HMC8073_CLK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(HMC8073_CLK_GPIO_Port, HMC8073_CLK_Pin, GPIO_PIN_RESET);
	}	
	
	for(int i = 0; i < 8; i++)
	{
		if((att_address & (ser_att_n << i)) != 0)
		{
			HAL_GPIO_WritePin(HMC8073_SI_GPIO_Port, HMC8073_SI_Pin, GPIO_PIN_SET);
		}
		else if((att_address & (ser_att_n << i)) == 0)
		{
			HAL_GPIO_WritePin(HMC8073_SI_GPIO_Port, HMC8073_SI_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(HMC8073_CLK_GPIO_Port, HMC8073_CLK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(HMC8073_CLK_GPIO_Port, HMC8073_CLK_Pin, GPIO_PIN_RESET);
	}	
	
	HAL_GPIO_WritePin(HMC8073_LE_GPIO_Port, HMC8073_LE_Pin, GPIO_PIN_SET);
	
}

/******************************* PE42430 Switch Functions *********************************************/

void switch_PE_handle(uint8_t pe_state)
{
	//uint8_t status = 0;
	switch(pe_state)
	{
		case RF1:
			HAL_GPIO_WritePin(PE42430_V1_GPIO_Port, PE42430_V1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PE42430_V2_GPIO_Port, PE42430_V2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PE42430_V3_GPIO_Port, PE42430_V3_Pin, GPIO_PIN_RESET);
			//status = 1;
			break;
		case RF2:
			HAL_GPIO_WritePin(PE42430_V2_GPIO_Port, PE42430_V2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PE42430_V1_GPIO_Port, PE42430_V1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PE42430_V3_GPIO_Port, PE42430_V3_Pin, GPIO_PIN_RESET);
			//status = 2;
			break;
		case RF3:
			HAL_GPIO_WritePin(PE42430_V3_GPIO_Port, PE42430_V3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PE42430_V2_GPIO_Port, PE42430_V2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PE42430_V1_GPIO_Port, PE42430_V1_Pin, GPIO_PIN_RESET);
			//status = 3;
			break;
		default:
			//status = 0;
			break;
	}
	//return status;
}

/******************************* RF3025 Switch Functions *********************************************/

void switch_RF_handle(uint8_t rf_state)
{
	//uint8_t status = 0;
	switch(rf_state)
	{
		case RF1:
			HAL_GPIO_WritePin(RF_CTRL_2_GPIO_Port, RF_CTRL_2_Pin, GPIO_PIN_SET);
			//status = 1;
			break;
		case RF2:
			HAL_GPIO_WritePin(RF_CTRL_2_GPIO_Port, RF_CTRL_2_Pin, GPIO_PIN_RESET);
			//status = 2;
			break;
		default:
			//status = 0;
			break;
	}
	//return status;
}

/**************************************** AD5932 MAINTAIN FUNCTIONS ***********************************************/

void generator_init(double p_freq)
{	
	double freq = p_freq;
	p_freq = B_PARAM;
	p_freq *= P_PARAM;
	//p_freq /= P_PARAM_2;
	p_freq += A_PARAM;
	p_freq /= R_PARAM;
	freq /= p_freq;
	hAD5932i.b24 = B24_IND_BYTE;
	hAD5932i.dacen = DAC_ON;
	hAD5932i.freq_delta = DELTA_F_DEF;
	hAD5932i.freq_start = freq;
	hAD5932i.inc_type = EXT_INCRMT;
	hAD5932i.msbouten = MSBOUT_ON;
	hAD5932i.n_inc = N_INC_DEF;
	hAD5932i.scan_dir = SCAN_DIR_POS;
	hAD5932i.signal_type = SINE_SIGNAL;
	hAD5932i.syncouten = SYNCOUT_ON;
	hAD5932i.syncsel = EOS_PULSE;
	
	AD5932_init(&hAD5932i);
}

void user_param(double st_freq, double d_freq, uint16_t n_incr, uint8_t msb_out, uint8_t signal_t, uint8_t ext, uint16_t time)
{	
	hAD5932i.b24 = B24_IND_BYTE;
	hAD5932i.dacen = DAC_ON;
	hAD5932i.freq_delta = d_freq;
	hAD5932i.freq_start = st_freq;
	hAD5932i.inc_type = ext;
	hAD5932i.msbouten = msb_out;
	hAD5932i.n_inc = n_incr;
	hAD5932i.scan_dir = SCAN_DIR_POS;
	hAD5932i.signal_type = signal_t;
	hAD5932i.syncouten = SYNCOUT_ON;
	hAD5932i.syncsel = EOS_PULSE;
	hAD5932i.tint = time;
	hAD5932i.tint_type = 1; //Fixed number of clock cycles
	
	AD5932_init(&hAD5932i);
}

/**************************************** ADF4360-0 MAINTAIN FUNCTIONS ***********************************************/

void synthesizer_init(void)
{
	HAL_GPIO_WritePin(ADF4360_CE_GPIO_Port, ADF4360_CE_Pin, GPIO_PIN_SET);
	
	hADF4360i.hctrlregi.core_power_level = 							CPL_10MA;
	hADF4360i.hctrlregi.counter_reset = 								NORMAL_STATE;
	hADF4360i.hctrlregi.cp_gain = 											CP_GAIN_SET_1;
	hADF4360i.hctrlregi.cp_three_state = 								CP_NORMAL;
	hADF4360i.hctrlregi.current_setting_1 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.current_setting_2 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.mute_till_lock_detect = 				MUTE_DIS;
	hADF4360i.hctrlregi.muxout_control = 								DIG_LOCK_DET;
	hADF4360i.hctrlregi.output_power_level = 						OUT_POW_13dBm;	
	hADF4360i.hctrlregi.phase_detector_polarity = 			PH_DET_POS;
	hADF4360i.hctrlregi.power_down_1 = 									PD1_DIS;
	hADF4360i.hctrlregi.power_down_2 = 									PD2_ASYN;
	hADF4360i.hctrlregi.prescaler_value = 							PRESC_16_17;
	hADF4360i.hrcounteri.anti_backlash_pulse_width = 		ABP_3_0NS;
	hADF4360i.hrcounteri.band_select_clock = 						BAND_8;
	hADF4360i.hrcounteri.lock_detect_precision = 				FIVE_CYCLES;
	hADF4360i.hrcounteri.r_counter = 										R_PARAM;
	hADF4360i.hrcounteri.test_mode_bit = 								1;						//CLEARING IN INIT PROCESS
	hADF4360i.hncounteri.a_counter = 										A_PARAM;
	hADF4360i.hncounteri.b_counter = 										B_PARAM;
	hADF4360i.hncounteri.cp_gain_ = 										CP_GAIN_SET_1;
	hADF4360i.hncounteri.divide_by_2 = 									0;
	hADF4360i.hncounteri.divide_by_2_select = 					0;
	
	ADF4360_init(&hADF4360i);	
}

void synthesizer_user(uint8_t out_power, uint8_t presc, uint8_t a_c, uint16_t b_c, uint16_t r_c)
{
	HAL_GPIO_WritePin(ADF4360_CE_GPIO_Port, ADF4360_CE_Pin, GPIO_PIN_SET);
	
	hADF4360i.hctrlregi.core_power_level = 							CPL_10MA;
	hADF4360i.hctrlregi.counter_reset = 								NORMAL_STATE;
	hADF4360i.hctrlregi.cp_gain = 											CP_GAIN_SET_1;
	hADF4360i.hctrlregi.cp_three_state = 								CP_NORMAL;
	hADF4360i.hctrlregi.current_setting_1 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.current_setting_2 = 						CUR_SET_1_250;
	hADF4360i.hctrlregi.mute_till_lock_detect = 				MUTE_DIS;
	hADF4360i.hctrlregi.muxout_control = 								DIG_LOCK_DET;
	hADF4360i.hctrlregi.output_power_level = 						out_power;	
	hADF4360i.hctrlregi.phase_detector_polarity = 			PH_DET_POS;
	hADF4360i.hctrlregi.power_down_1 = 									PD1_DIS;
	hADF4360i.hctrlregi.power_down_2 = 									PD2_ASYN;
	hADF4360i.hctrlregi.prescaler_value = 							presc;
	hADF4360i.hrcounteri.anti_backlash_pulse_width = 		ABP_3_0NS;
	hADF4360i.hrcounteri.band_select_clock = 						BAND_8;
	hADF4360i.hrcounteri.lock_detect_precision = 				FIVE_CYCLES;
	hADF4360i.hrcounteri.r_counter = 										r_c;
	hADF4360i.hrcounteri.test_mode_bit = 								1;						//CLEARING IN INIT PROCESS
	hADF4360i.hncounteri.a_counter = 										a_c;
	hADF4360i.hncounteri.b_counter = 										b_c;
	hADF4360i.hncounteri.cp_gain_ = 										CP_GAIN_SET_1;
	hADF4360i.hncounteri.divide_by_2 = 									0;
	hADF4360i.hncounteri.divide_by_2_select = 					0;
	
	ADF4360_init(&hADF4360i);	
}

/**************************************** OSCILLATOR MAINTAIN FUNCTIONS *****************************************/

void ref_gen_maintain(uint8_t ctrl)
{
	if(ctrl == ON)
	{
		HAL_GPIO_WritePin(X9_GPIO_Port, X9_Pin, GPIO_PIN_SET);
	}
	else if(ctrl == OFF)
	{
		HAL_GPIO_WritePin(X9_GPIO_Port, X9_Pin, GPIO_PIN_RESET);
	}
}
/**************************************** UART MAINTAIN FUNCTIONS ***********************************************/

void all_generation_off(void)
{
	//ADF4360-0 RESET
	HAL_GPIO_WritePin(ADF4360_CE_GPIO_Port, ADF4360_CE_Pin, GPIO_PIN_RESET);
	
	//AD5932 RESET
	HAL_GPIO_WritePin(AD5932_INTER_GPIO_Port, AD5932_INTER_Pin, GPIO_PIN_SET);
	//DWT_Delay(1000);
	HAL_GPIO_WritePin(AD5932_INTER_GPIO_Port, AD5932_INTER_Pin, GPIO_PIN_RESET); 
	
	//50MHz GENERATOR RESET
	HAL_GPIO_WritePin(X9_GPIO_Port, X9_Pin, GPIO_PIN_RESET);
}

void UART_pack_parser(void)
{
	std_union temp;
	uint8_t counter = 0;
	uint8_t counter_ant = 0;
	uint8_t counter_tag = 0;
	long_long_std_union temp_1;
	long_long_std_union centerFreq_1;
	long_long_std_union deltaFreq_1;
	long_long_std_union stepFreq_1;
	AD5932_InitTypeDef ad5932_temp;
	ADF4360_InitTypeDef adf4360_temp;
	temp.istd = 0;
	double att_temp = 0;
	if(UART_TX_BUFF[3] == UART_ADDR)
	{
		if((xor_handler(UART_TX_BUFF) == 0))
		{
			switch(UART_TX_BUFF[4])
			{ 
				case ECHO:
					HAL_UART_Transmit(&huart3, UART_TX_BUFF, UART_length.istd, 1000);
					uart_command = 0;
				break;	
				case ADC_CONT_1SEC:
					adc_enable = UART_TX_BUFF[5];
					uart_command = 0x01;
				break;
				case GENERATION_CTRL:
					if(UART_TX_BUFF[5] == ON)
					{
						HAL_GPIO_WritePin(X9_GPIO_Port, X9_Pin, GPIO_PIN_SET);
					}
					else if(UART_TX_BUFF[5] == OFF)
					{
						HAL_GPIO_WritePin(X9_GPIO_Port, X9_Pin, GPIO_PIN_RESET);
					}
					if(UART_TX_BUFF[6] == ON)
					{
						generator_init(FREQ_OUT);
					}
					else if(UART_TX_BUFF[6] == OFF)
					{
						HAL_GPIO_WritePin(AD5932_INTER_GPIO_Port, AD5932_INTER_Pin, GPIO_PIN_SET);
						DWT_Delay(1000);
						HAL_GPIO_WritePin(AD5932_INTER_GPIO_Port, AD5932_INTER_Pin, GPIO_PIN_RESET); 
					}
					if(UART_TX_BUFF[7] == ON)
					{
						synthesizer_init();
					}
					else if(UART_TX_BUFF[7] == OFF)
					{
						HAL_GPIO_WritePin(ADF4360_CE_GPIO_Port, ADF4360_CE_Pin, GPIO_PIN_RESET);
					}
					uart_command = 0x02;
				break;
				case OUTPUT_SWITCH:
					switch_PE_handle(UART_TX_BUFF[5]);
					switch_RF_handle(UART_TX_BUFF[6]);
					temp.cstd[0] = UART_TX_BUFF[6];
					temp.cstd[0] = (temp.cstd[0] >> 1) & 0x01;
					if(temp.cstd[0] == 0)
					{
						HAL_GPIO_WritePin(RF_CTRL_AMP_TR_GPIO_Port, RF_CTRL_AMP_TR_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(RF_CTRL_AMP_RC_GPIO_Port, RF_CTRL_AMP_RC_Pin, GPIO_PIN_SET);
					}
					else
					{
						HAL_GPIO_WritePin(RF_CTRL_AMP_TR_GPIO_Port, RF_CTRL_AMP_TR_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(RF_CTRL_AMP_RC_GPIO_Port, RF_CTRL_AMP_RC_Pin, GPIO_PIN_RESET);
					}
					
					uart_command = 0x03;
				break;
				case SET_ATT:
					temp.cstd[1] = UART_TX_BUFF[5];
					temp.cstd[0] = UART_TX_BUFF[6];
					att_temp = temp.istd;
					attenuator_set_att((att_temp/10));
					uart_command = 0x04;
				case ADC_ECHO:
					//HAL_UART_Transmit(&huart3, UART_TX_BUFF, UART_length.istd, 1000);
					adc_enable = UART_TX_BUFF[5];
					uart_command = 0x05;
				break;
				case CRYSTAL_EN:		
					ref_gen_maintain(UART_TX_BUFF[5]);	
				break;
				case AD5932_CTR:
					temp_1.cstd[7] = UART_TX_BUFF[5];
					temp_1.cstd[6] = UART_TX_BUFF[6];
					temp_1.cstd[5] = UART_TX_BUFF[7];
					temp_1.cstd[4] = UART_TX_BUFF[8];
					temp_1.cstd[3] = UART_TX_BUFF[9];
					temp_1.cstd[2] = UART_TX_BUFF[10];
					temp_1.cstd[1] = UART_TX_BUFF[11];
					temp_1.cstd[0] = UART_TX_BUFF[12];
					ad5932_temp.freq_start = temp_1.llistd;
					temp_1.cstd[7] = UART_TX_BUFF[13];
					temp_1.cstd[6] = UART_TX_BUFF[14];
					temp_1.cstd[5] = UART_TX_BUFF[15];
					temp_1.cstd[4] = UART_TX_BUFF[16];
					temp_1.cstd[3] = UART_TX_BUFF[17];
					temp_1.cstd[2] = UART_TX_BUFF[18];
					temp_1.cstd[1] = UART_TX_BUFF[19];
					temp_1.cstd[0] = UART_TX_BUFF[20];
					ad5932_temp.freq_delta = temp_1.llistd;
					temp_1.cstd[1] = UART_TX_BUFF[21];
					temp_1.cstd[0] = UART_TX_BUFF[22];
					ad5932_temp.n_inc = temp_1.istd[0];
					temp_1.cstd[1] = UART_TX_BUFF[26];
					temp_1.cstd[0] = UART_TX_BUFF[27];
					ad5932_temp.tint = temp_1.istd[0];
					temp_1.llistd = 0;
					ad5932_temp.msbouten = UART_TX_BUFF[23];
					ad5932_temp.signal_type = UART_TX_BUFF[24];
					ad5932_temp.inc_type = UART_TX_BUFF[25];				
				
					user_param(ad5932_temp.freq_start, ad5932_temp.freq_delta, ad5932_temp.n_inc, ad5932_temp.msbouten, ad5932_temp.signal_type, ad5932_temp.inc_type, ad5932_temp.tint);		
				break;
				case ADF4360_CTR:
					temp_1.cstd[1] = UART_TX_BUFF[8];
					temp_1.cstd[0] = UART_TX_BUFF[9];
					adf4360_temp.hncounteri.b_counter = temp_1.istd[0];
					temp_1.cstd[1] = UART_TX_BUFF[10];
					temp_1.cstd[0] = UART_TX_BUFF[11];
					adf4360_temp.hrcounteri.r_counter = temp_1.istd[0];
					adf4360_temp.hctrlregi.output_power_level = UART_TX_BUFF[5];
					adf4360_temp.hctrlregi.prescaler_value = UART_TX_BUFF[6];
					adf4360_temp.hncounteri.a_counter = UART_TX_BUFF[7];
				
					synthesizer_user(adf4360_temp.hctrlregi.output_power_level, adf4360_temp.hctrlregi.prescaler_value, adf4360_temp.hncounteri.a_counter, adf4360_temp.hncounteri.b_counter, adf4360_temp.hrcounteri.r_counter);
				break;
				case SEND_FREQ_PARAM:
					for(int i = 8; i != 0; i--)
					{
						counter = 13 - i;
						centerFreq_1.cstd[i-1] = UART_TX_BUFF[counter];
					}
					for(int i = 8; i != 0; i--)
					{
						counter = 21 - i;
						deltaFreq_1.cstd[i-1] = UART_TX_BUFF[counter];
					}
					for(int i = 8; i != 0; i--)
					{
						counter = 29 - i;
						stepFreq_1.cstd[i-1] = UART_TX_BUFF[counter];
					}	
					center_freq = centerFreq_1.llistd;
					band_freq = deltaFreq_1.llistd;
					single_step_freq = stepFreq_1.llistd;
					steps_freq = band_freq / single_step_freq;
					uart_command = 0x0A;
					break;
				case SEND_FREQ_PARAM_TOTAL:
					for(int a = 0; a < 3; a++)
					{
						for(int t = 0; t < 6; t++)
						{
							for(int i = 8; i != 0; i--)
							{
								counter = 13 - i + 8 * t + a * 48;
								centerFreq_1.cstd[i - 1] = UART_TX_BUFF[counter];
								center_freq_a_1[8 - i + 8 * t] = centerFreq_1.llistd;
							}			
							for(int i = 8; i != 0; i--)
							{
								counter = 157 - i + 8 * t + a * 48;
								deltaFreq_1.cstd[i - 1] = UART_TX_BUFF[counter];
								band_freq_a_1[8 - i + 8 * t] = deltaFreq_1.llistd;
							}
							for(int i = 8; i != 0; i--)
							{
								counter = 301 - i + 8 * t + a * 48;
								stepFreq_1.cstd[i - 1] = UART_TX_BUFF[counter];
								single_step_freq_a_1[8 - i + 8 * t] = stepFreq_1.llistd;
							}	
						}
					}
					
					for(int i = 1; i < 4; i++)
					{
						counter = 0;
						counter_tag = 0;
						counter_ant = 0;
						for(int n = 0; n < 6; i++)
						{
							if(center_freq_a_1[n*i] == 0)
							{
								counter++;
								counter_ant = i - 1;
								counter_tag |= (1 << n);
							}		
						}
						if(counter != 6)
						{
							antenna_start |= (1 << (i - 1));
							tag_start[i - 1] = counter_tag;
						}
					}
					uart_command = 0x0B;
				break;
				case MODE_SELECT:
					if(UART_TX_BUFF[5])
					{
						mode = MODE_WORK;
					}
					else
					{
						mode = MODE_TEST;
					}
				break;
				default:
					uart_command = 255;
				break;
			}
		}
	}
	HAL_GPIO_WritePin(RE_DE_GPIO_Port, RE_DE_Pin, GPIO_PIN_RESET);
}

uint8_t xor_handler(uint8_t *mass)
{
	uint8_t result = 0;
	std_union temp_1;
	temp_1.cstd[1] = mass[1];
	temp_1.cstd[0] = mass[2];
	for(int i = 0; i < (temp_1.istd); i++)
	{
		result ^= mass[i];
	}
	return result;
}

void init_transmitter(void)
{
	HAL_GPIO_WritePin(X9_GPIO_Port, X9_Pin, GPIO_PIN_SET); //50MHz generator enable;
	generator_init(center_freq);
	synthesizer_init();
}

void start_gen(double freq_vco, double delta, uint16_t nincr)
{
	double freq = K_PARAM;
	freq_vco = freq_vco/freq;
	freq = freq_vco;
	hAD5932i.freq_start = freq;
	hAD5932i.freq_delta = delta;
	hAD5932i.n_inc = nincr;
	
	AD5932_init(&hAD5932i);
}

void start_gen_1(double freq_vco, double delta, uint16_t step)
{
	double temp = 0;
	temp = B_PARAM;
	temp *= P_PARAM;
	//temp /= P_PARAM_2;
	temp += A_PARAM;
	temp /= R_PARAM;
	freq_vco -= delta/2;
	freq_vco += single_step_freq*step;
	freq_vco /= temp;
	hAD5932i.freq_start = freq_vco;
	hAD5932i.freq_delta = 0;
	hAD5932i.n_inc = 1;
	
	AD5932_init(&hAD5932i);
}

void select_path(uint8_t path)
{
	if(path == RECEIVER)
	{
		switch_RF_handle(RF2);
		HAL_GPIO_WritePin(RF_CTRL_0_GPIO_Port, RF_CTRL_0_Pin, GPIO_PIN_RESET);		//change for maket
		//HAL_GPIO_WritePin(RF_CTRL_AMP_RC_GPIO_Port, RF_CTRL_AMP_RC_Pin, GPIO_PIN_SET);
	}
	else if(path == TRANSMITTER)
	{
		switch_RF_handle(RF1);
		HAL_GPIO_WritePin(RF_CTRL_0_GPIO_Port, RF_CTRL_0_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(RF_CTRL_AMP_RC_GPIO_Port, RF_CTRL_AMP_RC_Pin, GPIO_PIN_RESET);
	}
	
}

uint16_t median_filter(uint16_t datum)
{
 struct pair
 {
   struct pair   *point;                              /* Pointers forming list linked in sorted order */
   uint16_t  value;                                   /* Values to sort */
 };
 static struct pair buffer[MEDIAN_FILTER_SIZE] = {0}; /* Buffer of nwidth pairs */
 static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
 static struct pair small = {NULL, STOPPER};          /* Chain stopper */
 static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/

 struct pair *successor;                              /* Pointer to successor of replaced data item */
 struct pair *scan;                                   /* Pointer used to scan down the sorted list */
 struct pair *scanold;                                /* Previous value of scan */
 struct pair *median;                                 /* Pointer to median */
 uint16_t i;

 if (datum == STOPPER)
 {
   datum = STOPPER + 1;                             /* No stoppers allowed. */
 }

 if ( (++datpoint - buffer) >= MEDIAN_FILTER_SIZE)
 {
   datpoint = buffer;                               /* Increment and wrap data in pointer.*/
 }

 datpoint->value = datum;                           /* Copy in new datum */
 successor = datpoint->point;                       /* Save pointer to old value's successor */
 median = &big;                                     /* Median initially to first in chain */
 scanold = NULL;                                    /* Scanold initially null. */
 scan = &big;                                       /* Points to pointer to first (largest) datum in chain */

 /* Handle chain-out of first item in chain as special case */
 if (scan->point == datpoint)
 {
   scan->point = successor;
 }
 scanold = scan;                                     /* Save this pointer and   */
 scan = scan->point ;                                /* step down chain */

 /* Loop through the chain, normal loop exit via break. */
 for (i = 0 ; i < MEDIAN_FILTER_SIZE; ++i)
 {
   /* Handle odd-numbered item in chain  */
   if (scan->point == datpoint)
   {
     scan->point = successor;                      /* Chain out the old datum.*/
   }

   if (scan->value < datum)                        /* If datum is larger than scanned value,*/
   {
     datpoint->point = scanold->point;             /* Chain it in here.  */
     scanold->point = datpoint;                    /* Mark it chained in. */
     datum = STOPPER;
   };

   /* Step median pointer down chain after doing odd-numbered element */
   median = median->point;                       /* Step median pointer.  */
   if (scan == &small)
   {
     break;                                      /* Break at end of chain  */
   }
   scanold = scan;                               /* Save this pointer and   */
   scan = scan->point;                           /* step down chain */

   /* Handle even-numbered item in chain.  */
   if (scan->point == datpoint)
   {
     scan->point = successor;
   }

   if (scan->value < datum)
   {
     datpoint->point = scanold->point;
     scanold->point = datpoint;
     datum = STOPPER;
   }

   if (scan == &small)
   {
     break;
   }

   scanold = scan;
   scan = scan->point;
 }
 return median->value;
}
