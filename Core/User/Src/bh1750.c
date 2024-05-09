#include "bh1750.h"

#include "stdio.h"
#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;



BH1750_DATA BH1750;

void BH1750_Init(void)
{
	hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
	
	BH1750.time = 120;
	BH1750.mode	=	BH_CON_H_MODE;
	BH1750.status	= GOOG;
}

void BH1750_Cmd_Write(uint8_t cmd)
{
	uint8_t data[1];
	data[0] = cmd;
	while(HAL_I2C_Master_Transmit(&hi2c1,BH1750_Addr_L,data,1,BH1750.time)!=HAL_OK);
	__HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_STOPF);
	HAL_Delay(100);
}
void BH1750_Start(void)
{
	BH1750_Cmd_Write(BH1750_ON);	 //power on
	BH1750_Cmd_Write(BH1750_RESET);	//clear
	BH1750_Cmd_Write(BH1750.mode);  //mode 
}
uint16_t BH1750_Read(void)
{   	
	uint8_t data_re[2] = {0,0};
	while(HAL_I2C_Master_Receive(&hi2c1,BH1750_Addr_L,data_re,2,BH1750.time*2)!=HAL_OK);
	HAL_Delay(100);
	BH1750.result_lx=data_re[0];
	BH1750.result_lx=(data_re[0]<<8)+data_re[1];  //????,?????
	BH1750.result_lx= BH1750.result_lx/1.2;
	HAL_Delay(100);
	return BH1750.result_lx;
}

void Display_Lux (float Lux)
{
	char str[20] = {0};
	lcd_put_cur(0, 0);
	sprintf (str, "LUX: %.2f ", Lux);
	lcd_send_string(str);
}
