#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "bmp280.h"
#include "mpu9250.h"
#include "stdio.h"
#include <math.h>

double KalmanAngleX;
double KalmanAngleY;
#define RAD_TO_DEG 57.295779513082320876798154814105
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
char data[200];
const uint16_t i2c_timeout = 100;
float pressure, temperature, humidity;
const double Accel_Z_corrector = 14418.0;
BMP280_HandleTypedef bmp280;
uint8_t Data[256];
int16_t  AccData[3],  MagData[3], GyroData[3];
uint16_t size;
uint32_t timer;

Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };

Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f, };


void SystemClock_Config(void);

int main(void)
{
	  HAL_Init();
	  sprintf((char *)Data, "BMP280 initialization failed\n");
	  HAL_UART_Transmit(&huart2, Data, size, 1000);
	  SystemClock_Config();
	  MX_GPIO_Init();
	  MX_USART2_UART_Init();
	  MX_I2C1_Init();
	  MPU9250_Init();
	  MPU9250_SetAccelRange(ACCEL_RANGE_2G);
	  MPU9250_SetGyroRange(GYRO_RANGE_250DPS);
	  MPU9250_SetDLPFBandwidth(DLPF_BANDWIDTH_184HZ);
	  MPU9250_SetSampleRateDivider(LP_ACCEL_ODR_0_24HZ);

	  bmp280_init_default_params(&bmp280.params);
	  bmp280.addr = BMP280_I2C_ADDRESS_0;
	  bmp280.i2c = &hi2c1;

	  	while (!bmp280_init(&bmp280, &bmp280.params)) {
	  		size = sprintf((char *)Data, "BMP280 initialization failed\n");
	  		HAL_UART_Transmit(&huart2, Data, size, 1000);
	  		HAL_Delay(2000);
	  	}
	  	bool bme280p = bmp280.id == BME280_CHIP_ID;
	  	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	  	HAL_UART_Transmit(&huart2, Data, size, 1000);
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  MPU9250_GetData(AccData,MagData,GyroData);
		  //(float)AccData[0],(float)AccData[1],(float)AccData[2]
		  //(float)MagData[0],(float)MagData[1],(float)MagData[2]
		  //(float)GyroData[0],(float)GyroData[1],(float)GyroData[2]
		  //sprintf(data,"X:%.2f    Y:%.2f\r\n",);
		  float accxraw=AccData[0];
		  float accyraw=AccData[1];
		  float acczraw=AccData[2];

		  float gyroxraw=GyroData[0];
		  float gyroyraw=GyroData[1];
		  float gyrozraw=GyroData[2];

		  float ax=accxraw/16384.0;
		  float ay=accyraw/16384.0;
		  float az=acczraw/Accel_Z_corrector;

		  float gx = gyroxraw / 131.0;
		  float gy = gyroyraw / 131.0;
		  float gz = gyrozraw / 131.0;

		  double dt = (double) (HAL_GetTick() - timer) / 1000;
		  timer = HAL_GetTick();

		  double roll;
		  double roll_sqrt=sqrt(accxraw*accxraw+acczraw*acczraw);
		  if(roll_sqrt!=0.0){
			  roll = atan(accyraw / roll_sqrt) * RAD_TO_DEG;
		  }else {
				roll = 0.0;
		  }
		  double pitch=atan2(-accxraw,acczraw)*RAD_TO_DEG;
		  if((pitch < -90 && KalmanAngleY > 90) || (pitch > 90 && KalmanAngleY < -90)){
			  KalmanY.angle = pitch;
			  KalmanAngleY = pitch;
		  }else {
				KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, gy, dt);
		  }
		  if (fabs(KalmanAngleY) > 90)
		  		gx = -gx;

		  KalmanAngleX = Kalman_getAngle(&KalmanX, roll, gy, dt);
		  sprintf(data, "x:%f y:%f \r\n", (float)KalmanAngleX, (float)KalmanAngleY);

		  HAL_UART_Transmit(&huart2, (uint8_t*)data, sizeof(data), 100);
		  HAL_Delay(200);
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
	#if 1
		  		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
		  			size = sprintf((char *)Data,"Temperature/pressure reading failed\n");
		  			HAL_UART_Transmit(&huart2, Data, size, 1000);
		  			HAL_Delay(2000);
		  		}

		  		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
		  				pressure, temperature);
		  		HAL_UART_Transmit(&huart2, Data, size, 1000);
		  		if (bme280p) {
		  			size = sprintf((char *)Data,", Humidity: %.2f\n", humidity);
		  			HAL_UART_Transmit(&huart2, Data, size, 1000);
		  		}
		  		else {
		  			size = sprintf((char *)Data, "\n");
		  			HAL_UART_Transmit(&huart2, Data, size, 1000);
		  		}

	#endif
	  }
	  /* USER CODE END 3 */
	}

	/**
	  * @brief System Clock Configuration
	  * @retval None
	  */


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate,
		double dt) {
	double rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt
			* (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0]
					+ Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	double S = Kalman->P[0][0] + Kalman->R_measure;
	double K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	double y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	double P00_temp = Kalman->P[0][0];
	double P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
}

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
