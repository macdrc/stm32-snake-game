
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdlib.h>
#include <stdbool.h>

ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SNAKE_MAX_LENGTH 100
#define SNAKE_START_LENGTH 5
#define FOOD_SYMBOL 0xFF
#define SNAKE_SYMBOL 0xFF
#define EMPTY_SYMBOL 0x00

#define GAME_AREA_X_MIN 2
#define GAME_AREA_Y_MIN 2
#define GAME_AREA_X_MAX 126
#define GAME_AREA_Y_MAX 62
#define GRID_CELL_SIZE 4
#define GRID_WIDTH ((GAME_AREA_X_MAX - GAME_AREA_X_MIN) / GRID_CELL_SIZE)
#define GRID_HEIGHT ((GAME_AREA_Y_MAX - GAME_AREA_Y_MIN) / GRID_CELL_SIZE)

volatile int snake_direction_x = 0;
volatile int snake_direction_y = 0;


typedef struct {
    uint8_t x;
    uint8_t y;
} Point;

typedef struct {
    Point body[SNAKE_MAX_LENGTH];
    uint16_t length;
    Point direction;
} Snake;

typedef struct {
    Point position;
    bool isEaten;
} Food;

/* Function declarations */
void initializeGame(Snake* snake, Food* food);
void resetGame(Snake* snake, Food* food);
void updateGame(Snake* snake, Food* food);
void renderGame(const Snake* snake, const Food* food);
bool checkCollision(const Snake* snake);
void generateFood(Food* food, const Snake* snake);
void moveSnake(Snake* snake);
bool checkFoodCollision(Snake* snake, Food* food);
bool checkSelfCollision(const Snake* snake);


void updateGame(Snake* snake, Food* food);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  ssd1306_Init();
  Snake snake;
  Food food;
  initializeGame(&snake, &food);

  while (1) {
          updateGame(&snake, &food);
          if (checkCollision(&snake)) {
              // Game over logic here
              ssd1306_Fill(Black);
              ssd1306_SetCursor(10, 26);
              ssd1306_WriteString("Game Over!", Font_11x18, White);
              ssd1306_UpdateScreen();
              HAL_Delay(2000); // Wait 5 seconds before resetting
              resetGame(&snake, &food); // Reset the game
              continue;
          }
          renderGame(&snake, &food);
          HAL_Delay(200); // Game speed
      }
}


void initializeGame(Snake* snake, Food* food) {
    // Initialize the snake in the middle of the screen
    snake->length = SNAKE_START_LENGTH;
    for (int i = 0; i < snake->length; i++) {
        snake->body[i].x = SSD1306_WIDTH / 2;
        snake->body[i].y = SSD1306_HEIGHT / 2 + i;
    }
    snake->direction.x = 0;
    snake->direction.y = -1; // Moving upwards

    // Initialize the food
    generateFood(food, snake);
}

void resetGame(Snake* snake, Food* food) {
    // Reinitialize the snake in the middle of the screen
    snake->length = SNAKE_START_LENGTH;
    for (int i = 0; i < snake->length; i++) {
        snake->body[i].x = 62;
        snake->body[i].y = 30 + i;
    }
    snake->direction.x = 0;
    snake->direction.y = -1; // Moving upwards

    // Reinitialize the food
    generateFood(food, snake);
}


void updateGame(Snake* snake, Food* food) {
    // Update direction based on global variables
    if (snake_direction_x != 0 || snake_direction_y != 0) {
        snake->direction.x = snake_direction_x;
        snake->direction.y = snake_direction_y;

        // Reset direction variables
        snake_direction_x = 0;
        snake_direction_y = 0;
    }

    moveSnake(snake);
    if (checkFoodCollision(snake, food)) {
        generateFood(food, snake);
    }
}


void renderGame(const Snake* snake, const Food* food) {
    ssd1306_Fill(Black);
    ssd1306_DrawRectangle(0,0,127,63,White);

    // Render the food as a 2x2 rectangle
    ssd1306_FillRectangle(food->position.x, food->position.y, food->position.x + 3, food->position.y + 3, White);

    // Render each segment of the snake as a 2x2 rectangle
    for (int i = 0; i < snake->length; i++) {
        ssd1306_FillRectangle(snake->body[i].x, snake->body[i].y, snake->body[i].x + 3, snake->body[i].y + 3, White);
    }
    ssd1306_UpdateScreen();
}


bool checkCollision(const Snake* snake) {
    // Check for collision with walls
    Point head = snake->body[0];
    if (head.x >= SSD1306_WIDTH || head.x < 0 || head.y >= SSD1306_HEIGHT || head.y < 0) {
        return true;
    }
    // Check for collision with self
    return checkSelfCollision(snake);
}

void generateFood(Food* food, const Snake* snake) {
    bool collision;
    do {
        collision = false;

        // Generate food in grid coordinates within game area
        food->position.x = GAME_AREA_X_MIN + (rand() % GRID_WIDTH) * GRID_CELL_SIZE;
        food->position.y = GAME_AREA_Y_MIN + (rand() % GRID_HEIGHT) * GRID_CELL_SIZE;

        // Check collision with snake
        // ...
    } while (collision);

    food->isEaten = false;
}


void moveSnake(Snake* snake) {
	Point newHead = {
	        .x = snake->body[0].x + (snake->direction.x * GRID_CELL_SIZE),
	        .y = snake->body[0].y + (snake->direction.y * GRID_CELL_SIZE)
	    };

	    // Restrict new head position within game area
	    if (newHead.x < GAME_AREA_X_MIN || newHead.x >= GAME_AREA_X_MAX ||
	        newHead.y < GAME_AREA_Y_MIN || newHead.y >= GAME_AREA_Y_MAX) {
	        // Handle collision or reset game
	        // ...
	    }
    // Shift the body
    for (int i = snake->length - 1; i > 0; i--) {
        snake->body[i] = snake->body[i - 1];
    }
    // Set new head
    snake->body[0] = newHead;
}

bool checkFoodCollision(Snake* snake, Food* food) {
    if (snake->body[0].x == food->position.x && snake->body[0].y == food->position.y) {
        food->isEaten = true;
        snake->length++;
        return true;
    }
    return false;
}

bool checkSelfCollision(const Snake* snake) {
    Point head = snake->body[0];
    for (int i = 1; i < snake->length; i++) {
        if (head.x == snake->body[i].x && head.y == snake->body[i].y) {
            return true;
        }
    }
    return false;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00000000;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void EXTI2_3_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

       snake_direction_x = 1; // Right
       snake_direction_y = 0;
    }
}


void EXTI4_15_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
        snake_direction_x = 0;
        snake_direction_y = -1; // Up
    }

    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        snake_direction_x = -1;
        snake_direction_y = 0; // Left
    }

    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
        snake_direction_x = 0;
        snake_direction_y = 1; // Down
    }
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
