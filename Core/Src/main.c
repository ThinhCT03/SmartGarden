#include "main.h"
#include "i2c-lcd.h"
#include "DS3231.h"
#include "DHT11.h"
#include "stdio.h"
#include "bh1750.h"
//Biến cho RTC
char buffer[15];

//TIME mam;
 uint8_t  sec,min,hour,day,date,month,year;

//biến cho độ ẩm đất
uint32_t var = 0;
float gradient = 0.001221;
float output_var = 0;
float moisture = 0;
float voltage = 0;

//biến as
float Lux = 0;

//biến cho sky mưa
 int rain; //mặc định là không mưa
 int SW_Motor = 1;
 int SW_Light = 1;

 //BIẾN CHO DHT11
 uint8_t k ;
 uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
 uint16_t SUM, RH, TEMP;
 float Temperature;
 float Humidity;
 uint8_t Presence;

/* Private define ------------------------------------------------------------*/
	typedef enum {Schedule_Water, Sensor_Water} ChooseState_t;
	typedef enum {Water_With_Sensor_State, Water_With_Schedule_State} Watering_t;
	typedef enum {Turn_Light, Off_Light} ControlLight_t;
	static int mnt;
	static ChooseState_t ChooseState = Sensor_Water;
	static Watering_t Watering = Water_With_Sensor_State;
	static ControlLight_t ControlLight = Off_Light;
	uint8_t previous_button_state;
	uint8_t current_button_state;
	uint8_t previous_water_button_state;
	uint8_t current_water_button_state;
	uint8_t previous_light_button_state;
	uint8_t current_light_button_state;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
void Display_HUM (float Temp);
uint16_t Flux_Reading (void);
uint8_t Reading_SW_Water(void);

void ChooseStateMachineUpdate(void);
void WateringMachineUpdate(void);
void ControlLightMachineUpdate(void);
//------------------------------------------------------------------------------//
int main(void)
{
  HAL_Init();
  SystemClock_Config();


  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  HAL_ADC_Start_IT(&hadc1); //bật ngắt ADC

  BH1750_Init();
  BH1750_Start();
  HAL_Delay(500);
  lcd_init();

  while (1)
  {
	  ChooseStateMachineUpdate();
	  WateringMachineUpdate();
	  ControlLightMachineUpdate();
  }
}

//Đọc độ flux
uint16_t Flux_Reading(void){
	 	BH1750_Init();
	 	BH1750_Start();
	 	HAL_Delay(300);
	 	Lux = BH1750_Read();
	 	return Lux;
}


//Máy trạng thái tưới nước
void WateringMachineUpdate(void){
	switch (Watering){
	case Water_With_Sensor_State:
		  //DHT11
	  	  DHT11_Start();
	 	  Presence = DHT11_Check_Response();
	 	  Rh_byte1 = DHT11_Read ();
	 	  Rh_byte2 = DHT11_Read ();
	 	  Temp_byte1 = DHT11_Read ();
	 	  Temp_byte2 = DHT11_Read ();
	 	  SUM = DHT11_Read();
	 	  TEMP = Temp_byte1;
	 	  RH = Rh_byte1;
	 	  Temperature = (float) TEMP;
	 	  Humidity = (float) RH;
	 	  Display_Temp(Temperature);
	 	  Display_Rh(Humidity);

		  //DS3231 đ�?c
		  Get_Time(&sec,&min,&hour,&day,&date,&month,&year);
		  sprintf (buffer, "%02d:%02d:%02d", hour, min, sec);
		  lcd_put_cur (0,0);
		  lcd_send_string(buffer);
		  sprintf (buffer, "%02d-%02d-20%02d", date, month, year);
		  lcd_put_cur(1, 0);
		  lcd_send_string(buffer);

		  //Soil Moisture
		  output_var = ((var*1.0)/4095);
		  moisture = (100 - (output_var*100));
		  voltage = (gradient*var);
		  HAL_Delay(50);
		  Display_HUM (moisture);
		  //Rain Sensor
		  rain = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

		  //Đọc nút nhấn
		  Reading_SW_Water();

		  //Điều kiện tưới
		  if(rain == 0 && (Temperature > 36 || Humidity < 60 || (voltage > 3.5 && voltage < 5))){
		   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  }
		  else if (rain == 0 && (Temperature <= 36 || Humidity >= 60 || voltage < 3.5)){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  }
		  else if (rain == 1){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  }
		  if (previous_water_button_state == GPIO_PIN_SET && current_water_button_state == GPIO_PIN_RESET){
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  }
		  break;

	case Water_With_Schedule_State:
		  Get_Time(&sec,&min,&hour,&day,&date,&month,&year);
		  sprintf (buffer, "%02d:%02d:%02d", hour, min, sec);
		  lcd_put_cur (0,0);
		  lcd_send_string(buffer);
		  sprintf (buffer, "%02d-%02d-20%02d", date, month, year);
		  lcd_put_cur(1, 0);
		  lcd_send_string(buffer);
		  break;


	switch (Watering){
	case Water_With_Sensor_State:
		  		if (mnt == 0) Watering = Water_With_Schedule_State;
		  		else if (mnt == 1) Watering = Water_With_Sensor_State;
		  		break;
	case Water_With_Schedule_State:
		  		if (mnt == 1) Watering = Water_With_Sensor_State;
		  		else if (mnt == 0) Watering = Water_With_Schedule_State;
		  		break;
		  	}

	}
}


//Máy trạng thái cho chọn cách tưới nước
void ChooseStateMachineUpdate(void){
	switch (ChooseState)
		{
		case Sensor_Water:
			//bật led chân PA11 để thể hiện đang ở trạng thái tự tưới
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			mnt = 1;
			break;
		case Schedule_Water:
			//bật led chân PA12 để thể hiện đang ở trạng thái lên lịch tưới
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			mnt = 0;
			break;
		}

	switch (ChooseState){
	case Sensor_Water:
		current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
		HAL_Delay(50); //chống run
		current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
		if (previous_button_state == GPIO_PIN_SET && current_button_state == GPIO_PIN_RESET)
		{
			ChooseState = (ChooseState == Sensor_Water) ? Schedule_Water : Sensor_Water;
		}
		previous_button_state = GPIO_PIN_SET;
		break;
	case Schedule_Water:
		current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
		HAL_Delay(50); //chống run
		current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
		if (previous_button_state == GPIO_PIN_SET && current_button_state == GPIO_PIN_RESET)
		{
			ChooseState = (ChooseState == Schedule_Water) ? Sensor_Water : Schedule_Water;
		}
		previous_button_state = GPIO_PIN_SET;
		break;
	}
}

//Máy trạng thái bật đèn
void ControlLightMachineUpdate(void){
	switch(ControlLight){
	case Off_Light:
		//xuất mức 1 ra PA6, relay không bật
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		break;
	case Turn_Light:
		//xuất mức 0 ra PA6, bật relay
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		break;
	}
	switch(ControlLight){
	case Off_Light:
		current_light_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		HAL_Delay(50); //chống run
		current_light_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		if (previous_light_button_state == GPIO_PIN_SET && current_light_button_state == GPIO_PIN_RESET)
		{
			ControlLight = (ControlLight == Off_Light) ? Turn_Light : Off_Light;
		}
		else if (Flux_Reading() < 10){
			ControlLight = (ControlLight == Off_Light) ? Turn_Light : Off_Light;
		}
		else if (Flux_Reading() >= 10){
			ControlLight = Off_Light;
		}
		previous_light_button_state = GPIO_PIN_SET;
		break;
	case Turn_Light:
		current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		HAL_Delay(50); //chống run
		current_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		if (previous_light_button_state == GPIO_PIN_SET && current_light_button_state == GPIO_PIN_RESET)
		{
			ControlLight = (ControlLight == Turn_Light) ? Off_Light : Turn_Light;
		}
		else if (Flux_Reading() >= 10){
			ControlLight = (ControlLight == Turn_Light) ? Off_Light : Turn_Light;
		}
		else if (Flux_Reading() < 10){
			ControlLight = Turn_Light;
		}
		previous_light_button_state = GPIO_PIN_SET;
		break;

	}
}


//Hàm hiển thị độ ẩm đất
void Display_HUM (float Temp)
{
	char str[20] = {0};
	lcd_put_cur(0, 0);

	sprintf (str, "Mois: %.1f ", Temp);
	lcd_send_string(str);
}

//Hàm ngắt ADC đọc độ ẩm đất
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == hadc1.Instance)
	{
		var = HAL_ADC_GetValue(&hadc1);
	}

}


	//---------------CẤU HÌNH STM32F103------------------//

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
uint8_t Reading_SW_Water(void){
	current_water_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	HAL_Delay(50); //chống run
	current_water_button_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	return current_water_button_state;

}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  	GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE END 4 */



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
