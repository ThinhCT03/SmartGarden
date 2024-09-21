#ifndef __DHT11_H
#define __DHT11_H

#include "main.h" // Include các thư viện cần thiết cho hàm header này

// Định nghĩa các hằng số và biến cần thiết
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_0



// Khai báo các hàm
void DHT11_Start(void);
uint8_t DHT11_Read(void);
void delay(uint16_t delay);
uint8_t DHT11_Check_Response (void);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Display_Temp (float Temp);
void Display_Rh (float Rh);
#endif /* __DHT11_H */
