/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

I2C_LCD_HandleTypeDef lcd1;
uint32_t last_button_press_time = 0;
uint8_t mode=1;

const uint8_t seven_seg_patterns[12] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// Keypad character map corresponding to your layout
const char keypad_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void writeByte(uint8_t number)
 {
     if (number > 0xff) return; // Ensure number is within the valid range

          // Write the number to the GPIO pins
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (number & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (number & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (number & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (number & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (number & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (number & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (number & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (number & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
 }

/**
  * @brief  Displays a number (0-9) on the 7-segment display.
  * @param  number: The number to display.
  * @retval None
  */
void display_7seg(uint8_t number)
{
    if (number > 11) return; // Ensure number is within the valid range

    uint8_t pattern = seven_seg_patterns[number];

    // Write the pattern to the GPIO pins for the 7-segment display
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (pattern & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (pattern & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (pattern & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (pattern & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (pattern & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (pattern & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (pattern & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  Scans the 4x4 keypad to find which key is pressed.
  * @retval The character of the pressed key, or '\0' (null character) if no key is pressed.
  */
char scan_keypad(void)
{
    // Array of row pins for easier iteration
    GPIO_TypeDef* row_ports[] = {GPIOB, GPIOB, GPIOB, GPIOB};
    uint16_t row_pins[] = {GPIO_PIN_1, GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13};

    for (int row = 0; row < 4; row++)
    {
        // Drive all rows HIGH first
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

        // Drive the current row LOW
        HAL_GPIO_WritePin(row_ports[row], row_pins[row], GPIO_PIN_RESET);

        // Check columns for a LOW signal.
        // A LOW signal means a key in this row has been pressed.
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) return keypad_map[row][0];
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) return keypad_map[row][1];
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) return keypad_map[row][2];
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) return keypad_map[row][3];
    }

    // If we get here, no key was pressed
    return 99;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{



	 if ((HAL_GetTick() - last_button_press_time) > 500){

		 last_button_press_time = HAL_GetTick();
		 mode=mode+1;
	     if(mode>3){
			        mode=1;
	     }

		lcd_gotoxy(&lcd1, 9, 1);
		lcd_putchar(&lcd1, mode + '0');



	}

	//HAL_Delay(10);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

    lcd1.hi2c = &hi2c1;
    lcd1.address = 0x4E;
    lcd_init(&lcd1);
    lcd_clear(&lcd1);
    lcd_puts(&lcd1, "STM32 I2C LCD");
    //lcd_gotoxy(&lcd1, 9, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint8_t counter=0;
    char pressed_key;
  while (1)
  {
	  if(mode==1){
		  // Running light from left to right (assuming 8 LEDs)
		 	    for (int i = 0; i < 9; i++)
		 	    {
		 	      writeByte(1 << i); // Shift a single bit (LED) from left to right
		 	      HAL_Delay(100);    // Adjust delay for desired speed
		 	    }
	  }
      if(mode==2){
    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,0);
    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,0);
    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,0);
    	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,0);

    	    display_7seg(counter);
    	    HAL_Delay(500);
    	    counter++;
    	    if(counter>9){
    	    		  counter=0;
    	    	  }
      }
      if(mode==3){
    	  pressed_key=scan_keypad();
    	 	 	  	  if(pressed_key!= 99){
    	 	 	  		 // pressed_key=pressed_key- 0x30;
    	 	 	  		lcd_gotoxy(&lcd1, 0, 1);
    	 	 	  		lcd_putchar(&lcd1, pressed_key);
    	 	 	  	  }
    	 	 	  	  HAL_Delay(10);

      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, seg_f_Pin|seg_e_Pin|seg_g_Pin|led_2_Pin
                          |led_3_Pin|led_4_Pin|led_7_Pin|digit_4_Pin
                          |digit_3_Pin|digit_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, seg_a_Pin|seg_b_Pin|seg_c_Pin|LD2_Pin
                          |led_0_Pin|led_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, seg_d_Pin|row_1_Pin|row_4_Pin|row_3_Pin
                          |row_2_Pin|led_6_Pin|led_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(digit_1_GPIO_Port, digit_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : seg_f_Pin seg_e_Pin seg_g_Pin led_2_Pin
                           led_3_Pin led_4_Pin led_7_Pin digit_4_Pin
                           digit_3_Pin digit_2_Pin */
  GPIO_InitStruct.Pin = seg_f_Pin|seg_e_Pin|seg_g_Pin|led_2_Pin
                          |led_3_Pin|led_4_Pin|led_7_Pin|digit_4_Pin
                          |digit_3_Pin|digit_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : seg_a_Pin seg_b_Pin seg_c_Pin LD2_Pin
                           led_0_Pin led_1_Pin */
  GPIO_InitStruct.Pin = seg_a_Pin|seg_b_Pin|seg_c_Pin|LD2_Pin
                          |led_0_Pin|led_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : seg_d_Pin row_1_Pin row_4_Pin row_3_Pin
                           row_2_Pin led_6_Pin led_5_Pin */
  GPIO_InitStruct.Pin = seg_d_Pin|row_1_Pin|row_4_Pin|row_3_Pin
                          |row_2_Pin|led_6_Pin|led_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : col_2_Pin col_3_Pin col_4_Pin */
  GPIO_InitStruct.Pin = col_2_Pin|col_3_Pin|col_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : col_1_Pin */
  GPIO_InitStruct.Pin = col_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(col_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : digit_1_Pin */
  GPIO_InitStruct.Pin = digit_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(digit_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
