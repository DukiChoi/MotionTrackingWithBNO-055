/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "TinyEKF.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#define imuBufSize 34
void SystemClock_Config(void);

/* STM handles*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void delay_ms(unsigned short i);

/* Private variables ---------------------------------------------------------*/
volatile uint8_t imuBuf[imuBufSize] = {0,};
double a[3], w[3], h[3], q[4], an[3] = {0.0,}; // acceleration, angular velocity, magnetic field intensity, quaternion, angle
double pi, theta, psi = 0.0;  //roll, pitch, yaw
int i = 0; //a integer used in while-loop
static double g = 9.80; //gravity(m/s^2)
double S_x, S_y, S_z= 0.0;
double V_x, V_y, V_z= 0.0; // initial velocity
static double t = 0.01;  //sensing period
double S[3] = {0,};
double return_x[4] = {0,};
float S_gps_x = 0.0;
float S_gps_y = 0.0;
float S_gps_z = 0.0;
double xp[4] = {1.0, 0.0, 0.0, 0.0};
bool returns;
double temp=0.0;

class IMUEKF : public TinyEKF {
  public:
  IMUEKF(){

	// state vector init = Quaternion
    this->setX(0, xp[0]); //w
    this->setX(1, xp[1]); //x
    this->setX(2, xp[2]); //y
    this->setX(3, xp[3]); //z

    // error cov
    this->setP(0, 0, 0.001);
    this->setP(1, 1, 0.001);
    this->setP(2, 2, 0.001);
    this->setP(3, 3, 0.001);

    this->setP(0,1,0.0002);
    this->setP(0,2,0.0002);
    this->setP(0,3,0.0002);
    this->setP(1,0,0.0002);
    this->setP(1,2,0.0002);
    this->setP(1,3,0.0002);
    this->setP(2,0,0.0002);
    this->setP(2,1,0.0002);
    this->setP(2,3,0.0002);
    this->setP(3,0,0.0002);
    this->setP(3,1,0.0002);
    this->setP(3,2,0.0002);

    // process noise
    this->setQ(0, 0, 2);
    this->setQ(1, 1, 2);
    this->setQ(2, 2, 2);
    this->setQ(3, 3, 2);

    // measurement noise
    this->setR(0, 0, pow(10,-6)); // Angle Noise
    this->setR(1, 1, pow(10,-6));
    this->setR(2, 2, pow(10,-6));
    this->setR(3, 3, pow(10,-6));
  }
  protected:
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]){

    // Process model hat[X_predict]
    fx[0] = this->x[0] + (this->x[1]*(-(w[0]*t/2))) + (this->x[2]*(-(w[1]*t/2))) + (this->x[3]*(-(w[2]*t/2)));
    fx[1] = this->x[1] + (this->x[0]*(w[0]*t/2)) + (this->x[2]*(w[2]*t/2)) + (this->x[3]*(-(w[1]*t/2)));
    fx[2] = this->x[2] + (this->x[0]*(w[1]*t/2)) + (this->x[1]*(-(w[2]*t/2))) + (this->x[3]*(w[0]*t/2));
    fx[3] = this->x[3] + (this->x[0]*(w[2]*t/2)) + (this->x[1]*(w[1]*t/2)) + (this->x[2]*(-(w[0]*t/2)));

    //state transition matrix
    F[0][0] = 1;
    F[0][1] = -(w[0]*(t/2));
    F[0][2] = -(w[1]*(t/2));
    F[0][3] = -(w[2]*(t/2));

    F[1][0] = w[0]*(t/2);
    F[1][1] = 1;
    F[1][2] = w[2]*(t/2);
    F[1][3] = -(w[1]*(t/2));

    F[2][0] = w[1]*(t/2);
    F[2][1] = -(w[2]*(t/2));
    F[2][2] = 1;
    F[2][3] = w[0]*(t/2);

    F[3][0] = w[2]*(t/2);
    F[3][1] = w[1]*(t/2);
    F[3][2] = -(w[0]*(t/2));
    F[3][3] = 1;

    // Measurement function simplifies the relationship between state and sensor readings for convenience.
    // A more realistic measurement function would distinguish between state value and measured value; e.g.:

    // step 1 hx
    hx[0] = x[0];
    hx[1] = x[1];
    hx[2] = x[2];
    hx[3] = x[3];

    // step 1 Jacobian of measurement function
    H[0][0] = 1;
    H[0][1] = 0;
    H[0][2] = 0;
    H[0][3] = 0;

    H[1][0] = 0;
    H[1][1] = 1;
    H[1][2] = 0;
    H[1][3] = 0;

    H[2][0] = 0;
    H[2][1] = 0;
    H[2][2] = 1;
    H[2][3] = 0;

    H[3][0] = 0;
    H[3][1] = 0;
    H[3][2] = 0;
    H[3][3] = 1;
  }
};

// q0: qw, q1: qx, q2: qy, q3: qz
// q0: qw, q1: qx, q2: qy, q3: qz
void quaternion_to_euler(double q0, double q1, double q2, double q3) {
   pi = atan2(2*(q2*q3 + q0*q1), (pow(q0, 2.0) - pow(q1, 2.0) - pow(q2, 2.0) + pow(q3, 2.0)));
   theta = asin(2*(q0*q2 - q1*q3)/(pow(q0, 2.0) + pow(q1, 2.0) + pow(q2, 2.0) + pow(q3, 2.0)));
   psi = atan2(2*(q1*q2 + q0*q3), (pow(q0, 2.0) + pow(q1, 2.0) - pow(q2, 2.0) - pow(q3, 2.0)));
}

void euler_to_quaternion(double roll, double pitch, double yaw){ // pi theta psi
  q[0] = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2); //w
  q[1] = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2); //x
  q[2] = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2); //y
  q[3] = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2); //z
}

double* calculation(double *a, double *w, double *an){

   //calculate the position with accerelation, angular velocity and angle
   //plus= world to body
   
    double acc_x = a[0];
    double acc_y = a[1];
    double acc_z = a[2];

    //angular-drift compensation
    if (abs(an[0]) < 0.01/180*3.14){
       an[0] = 0;
    }
    if (abs(an[1]) < 0.01/180*3.14){
       an[1] = 0;
    }
    if (abs(an[2]) < 0.01/180*3.14){
       an[2] = 0;
    }

    //plus zyx body to world
    double acc_x_world = cos(an[1])*cos(an[2])*acc_x + (sin(an[0])*sin(an[1])*cos(an[2])-cos(an[0])*sin(an[2]))*acc_y + (cos(an[0])*sin(an[1])*cos(an[2])+sin(an[0])*sin(an[2]))*acc_z;
    double acc_y_world = cos(an[1])*sin(an[2])*acc_x + (sin(an[0])*sin(an[1])*sin(an[2])+cos(an[0])*cos(an[2]))*acc_y + (cos(an[0])*sin(an[1])*sin(an[2])-sin(an[0])*cos(an[2]))*acc_z;
    double acc_z_world = - sin(an[1])*acc_x + sin(an[0])*cos(an[1])*acc_y + cos(an[0])*cos(an[1])*acc_z-9.806;

    //V-drift compensation
    double acc_sqrt = sqrt(pow(acc_x_world, 2) + pow(acc_y_world, 2) + pow(acc_z_world, 2));
    if (acc_sqrt < 0.004){
      acc_x_world = 0;
      acc_y_world = 0;
      acc_z_world = 0;
    }

    V_x = V_x + acc_x_world*t;
    V_y = V_y + acc_y_world*t;
    V_z = V_z + acc_z_world*t;

    S_x = V_x*t;
    S_y = V_y*t;
    S_z = V_z*t;

    S_gps_x = S_gps_x + S_x;
    S_gps_y = S_gps_y + S_y;
    S_gps_z = S_gps_z + S_z;

    S[0]=S_gps_x;
    S[1]=S_gps_y;
    S[2]=S_gps_z;
    return S;
}

int main(void)
{

  IMUEKF ekf;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  HAL_StatusTypeDef RcvStatIMU;
  float S[3] = {0.0,};

  while(1){
    int sum = 0;
    int sum2 = 0;
    RcvStatIMU = HAL_UART_Receive(&huart1, (uint8_t*)&imuBuf, imuBufSize, 1000);
    if (RcvStatIMU == HAL_OK){
      if ((imuBuf[1] == 0x55) && (imuBuf[2] == 0x51) && (imuBuf[13] == 0x52)){  //&& (imuBuf[24] == 0x53) &&(imuBuf[46] == 0x59)
        for (i=1; i<11; i++){
          sum += imuBuf[i];
          sum2 += imuBuf[i+11];
        }
        sum = sum & 0xff;
        sum2 = sum2 & 0xff;
        if ((sum == imuBuf[11]) && (sum2 == imuBuf[22])){

          //receive from sensor
          a[0] = (float)(short)((imuBuf[4]<<8)|imuBuf[3])/32768*16*g;
          a[1] = (float)(short)((imuBuf[6]<<8)|imuBuf[5])/32768*16*g;
          a[2] = (float)(short)((imuBuf[8]<<8)|imuBuf[7])/32768*16*g;

          w[0] = (float)(short)((imuBuf[15]<<8)|imuBuf[14])/32768*2000 *3.141592/180; //pi
          w[1] = (float)(short)((imuBuf[17]<<8)|imuBuf[16])/32768*2000 *3.141592/180; //theta
          w[2] = (float)(short)((imuBuf[19]<<8)|imuBuf[18])/32768*2000 *3.141592/180; //psi

          an[0] = (float)(short)((imuBuf[26]<<8)|imuBuf[25])/32768 * 3.141592;
          an[1] = (float)(short)((imuBuf[28]<<8)|imuBuf[27])/32768 * 3.141592;
          an[2] = (float)(short)((imuBuf[30]<<8)|imuBuf[29])/32768 * 3.141592;

          //ekf
          euler_to_quaternion(an[0], an[1], an[2]);
          returns = ekf.step(q); //wxyz
          quaternion_to_euler(ekf.getX(0), ekf.getX(1), ekf.getX(2), ekf.getX(3)); //wxyz
          an[0] = pi;
          an[1] = theta;
          an[2] = psi;

          //calculate the location
          volatile double* S_pointer = calculation(a, w, an);

          S[0]= S_pointer[0]*100;
          S[1]= S_pointer[1]*100;
          S[2]= S_pointer[2]*100;
          
          //send the location data through UART2.
          HAL_UART_Transmit(&huart2,(uint8_t*)&S,sizeof(S),100);
          HAL_UART_Transmit(&huart2,(uint8_t*)"\n",1,100);
        }
      }
    }     
  } 
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

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

void delay_ms(unsigned short i)
{
   unsigned short k;
   while(i--)
   for (k=0;k<100;k++);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
