#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f1xx_hal.h"

// Inicializa el sensor
void BMP280_Init(I2C_HandleTypeDef *hi2c);

// Lee la altitud en metros
float BMP280_ReadAltitude(void);

#endif /* INC_BMP280_H_ */
