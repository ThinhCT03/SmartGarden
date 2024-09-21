#ifndef __DS3231
#define __DS3231

#include "main.h"
#define DS3231_ADDRESS 0xD0

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;





int bcdToDec(uint8_t val);
uint8_t decToBcd(int val);
void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void Get_Time (uint8_t *Get_Sec , uint8_t *Get_min ,uint8_t *Get_hour,uint8_t *Get_day ,uint8_t *Get_date,uint8_t *Get_month ,uint8_t *Get_year);
float Get_Temp (void);
void force_temp_conv (void);


#endif
