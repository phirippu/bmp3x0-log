/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bmp3.h"
#include <stdio.h>
#include "retarget.h"
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

/* USER CODE BEGIN PV */
static uint8_t sensor_fifo[512 + 4];
volatile uint8_t tick = 0;
static struct bmp3_data result[75];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void User_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void bmm150_user_delay_us(uint32_t period_us, void *intf_ptr);

/*!
 * @brief This function is for writing the sensor's registers through I2C bus.
 */
int8_t bmm150_user_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief This function is for reading the sensor's registers through I2C bus.
 */
int8_t bmm150_user_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*!
 * @brief       Function to analyze the sensor data
 *
 * @param[in]   data   Structure instance of bmp3_data(compensated temp & press values)
 *
 * @return      Error code
 * @retval      0   Success
 */
static int8_t analyze_sensor_data(const struct bmp3_data *data);

/*!
 * @brief       Function to calculate the CRC of the trimming parameters.
 *
 * @param[in]   seed   CRC of each register
 * @param[in]   data   register data.
 *
 * @return      calculated CRC
 */
static int8_t cal_crc(uint8_t seed, uint8_t data);

/*!
 * @brief Function to validate the trimming parameters
 *
 * @param [in] dev Structure instance of bmp3_dev structure.
 *
 * @return      Error code
 * @retval      0   Success
 *
 */
static int8_t validate_trimming_param(struct bmp3_dev *dev);

/*!
 * @brief       Self-test API for the BMP38X
 */
int8_t bmp3_selftest_check(struct bmp3_dev *dev);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t dev_okay = 0;
    struct bmp3_dev sensor;
    struct bmp3_fifo fifo_struct;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
    RetargetInit(&huart1);
    HAL_TIM_RegisterCallback(&htim17, HAL_TIM_PERIOD_ELAPSED_CB_ID, User_TIM_PeriodElapsedCallback);
    HAL_TIM_Base_Start_IT(&htim17);
    HAL_NVIC_SetPriority(TIM17_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM17_IRQn);
    HAL_TIM_Base_Start(&htim16);
    HAL_I2C_MspInit(&hi2c1);
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x77 << 1, 2, 100) == HAL_OK) {
        dev_okay = 1;
    }

    sensor.delay_us = bmm150_user_delay_us;
    sensor.read = bmm150_user_i2c_reg_read;
    sensor.write = bmm150_user_i2c_reg_write;
    sensor.intf = BMP3_I2C_INTF;
    sensor.intf_ptr = &hi2c1;
    sensor.fifo = &fifo_struct;
    sensor.fifo->data.buffer = sensor_fifo;

    if (bmp3_selftest_check(&sensor) == BMP3_SENSOR_OK) {
        printf("Sensor is OK\n");
        sensor.settings.press_en = BMP3_ENABLE;
        sensor.settings.temp_en = BMP3_ENABLE;

        /* Select the output data rate and over sampling settings for pressure and temperature */
        sensor.settings.odr_filter.press_os = BMP3_OVERSAMPLING_16X;
        sensor.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
        sensor.settings.odr_filter.odr = BMP3_ODR_25_HZ;

        sensor.fifo->settings.mode = 1;
        sensor.fifo->settings.temp_en = 1;
        sensor.fifo->settings.press_en = 1;
        sensor.fifo->settings.time_en = 1;
        sensor.fifo->settings.stop_on_full_en = 0;
        sensor.fifo->settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;
        sensor.fifo->data.req_frames = 75;

        /* Assign the settings which needs to be set in the sensor */

        if (bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR, &sensor) == BMP3_SENSOR_OK) {
//            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED1_Pin);
            printf("Sensor settings set\n");
            sensor.settings.op_mode = BMP3_MODE_NORMAL;
            if (bmp3_set_op_mode(&sensor) == BMP3_SENSOR_OK) {
                printf("Sensor operational mode set\n");
                if (bmp3_set_fifo_settings(BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_TIME_EN | BMP3_SEL_FIFO_TEMP_EN
                                           | BMP3_SEL_FIFO_PRESS_EN | BMP3_SEL_FIFO_STOP_ON_FULL_EN, &sensor) == BMP3_SENSOR_OK) {
                    printf("Sensor fifo is set\n");
                };
                HAL_Delay(100);
//                bmp3_set_fifo_settings(BMP3_SEL_FIFO_MODE |
//                                       BMP3_SEL_FIFO_PRESS_EN |
//                                       BMP3_SEL_FIFO_TEMP_EN |
//                                       BMP3_SEL_FIFO_TIME_EN, &sensor);
            }
        }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        if (tick) {
            tick = 0;
            if (dev_okay == 0) {
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            } else {
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                bmp3_get_fifo_data(&sensor);
                bmp3_extract_fifo_data(result, &sensor);
//            printf("%d bytes downloaded. Frames: %d Time: %ld\n", sensor.fifo->data.byte_count,
//                   sensor.fifo->data.parsed_frames, sensor.fifo->data.sensor_time);
                for (int i = 0; i < sensor.fifo->data.parsed_frames; ++i) {
                    printf("%d %lld %lld\n", i, result[i].pressure, result[i].temperature);
//                    printf("%lf %lf\n", result[i].pressure, result[i].temperature);
                }
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            }
        }
    }

#pragma clang diagnostic pop
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void bmm150_user_delay_us(uint32_t period_us, void *intf_ptr) {
    __HAL_TIM_SET_COUNTER(&htim16, 0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim16) <
           period_us);  // wait for the counter to reach the us input in the parameter
    /* Wait for a period amount of microseconds. */
}

// typedef BMP3_INTF_RET_TYPE (*bmp3_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr);

int8_t bmm150_user_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {

    /* Write to registers using I2C. Return 0 for a successful execution. */
    if (HAL_I2C_Mem_Write(intf_ptr, 0x77 << 1, reg_addr, 1, reg_data, length, 1000) == HAL_OK) {
        return BMP3_OK;
    };
    return BMP3_E_COMM_FAIL;
}

int8_t bmm150_user_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {

    /* Read from registers using I2C. Return 0 for a successful execution. */
    if (HAL_I2C_Mem_Read(intf_ptr, 0x77 << 1, reg_addr, 1, reg_data, length, 1000) == HAL_OK) {
        return BMP3_OK;
    };
    return BMP3_E_COMM_FAIL;
}


/*!
 * @brief  Function to analyze the sensor data
 */
static int8_t analyze_sensor_data(const struct bmp3_data *sens_data) {
    int8_t rslt = BMP3_SENSOR_OK;

    if ((sens_data->temperature < BMP3_MIN_TEMPERATURE) || (sens_data->temperature > BMP3_MAX_TEMPERATURE)) {
        rslt = BMP3_IMPLAUSIBLE_TEMPERATURE;
    }

    if (rslt == BMP3_SENSOR_OK) {
        if ((sens_data->pressure / 100 < BMP3_MIN_PRESSURE) ||
            (sens_data->pressure / 100 > BMP3_MAX_PRESSURE)) {
            rslt = BMP3_IMPLAUSIBLE_PRESSURE;
        }
    }

    return rslt;
}

/*
 * @brief Function to verify the trimming parameters
 * */
static int8_t validate_trimming_param(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;

    rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
    if (rslt == BMP3_SENSOR_OK) {
        for (i = 0; i < 21; i++) {
            crc = (uint8_t) cal_crc(crc, trim_param[i]);
        }

        crc = (crc ^ 0xFF);
        rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
        if (stored_crc != crc) {
            rslt = BMP3_TRIMMING_DATA_OUT_OF_BOUND;
        }
    }

    return rslt;

}

/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data) {
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if ((seed & 0x80) ^ (data & 0x80)) {
            var2 = 1;
        } else {
            var2 = 0;
        }

        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t) (poly * var2);
    }

    return (int8_t) seed;
}

int8_t bmp3_selftest_check(struct bmp3_dev *dev) {
    int8_t rslt;

    /* Variable used to select the sensor component */
    uint8_t sensor_comp;

    /* Variable used to store the compensated data */
    struct bmp3_data data = {0};

    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Reset the sensor */
    rslt = bmp3_soft_reset(dev);
    if (rslt == BMP3_SENSOR_OK) {
        rslt = bmp3_init(dev);

        if (rslt == BMP3_E_COMM_FAIL || rslt == BMP3_E_DEV_NOT_FOUND) {
            rslt = BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE;
        }

        if (rslt == BMP3_SENSOR_OK) {
            rslt = validate_trimming_param(dev);
        }

        if (rslt == BMP3_SENSOR_OK) {
            /* Select the pressure and temperature sensor to be enabled */
            dev->settings.press_en = BMP3_ENABLE;
            dev->settings.temp_en = BMP3_ENABLE;

            /* Select the output data rate and over sampling settings for pressure and temperature */
            dev->settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
            dev->settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
            dev->settings.odr_filter.odr = BMP3_ODR_25_HZ;

            /* Assign the settings which needs to be set in the sensor */
            settings_sel =
                    BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
            rslt = bmp3_set_sensor_settings(settings_sel, dev);
            if (rslt == BMP3_SENSOR_OK) {
                dev->settings.op_mode = BMP3_MODE_NORMAL;
                rslt = bmp3_set_op_mode(dev);
                if (rslt == BMP3_SENSOR_OK) {
                    dev->delay_us(40000, dev->intf_ptr);

                    /* Sensor component selection */
                    sensor_comp = BMP3_PRESS | BMP3_TEMP;

                    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
                    rslt = bmp3_get_sensor_data(sensor_comp, &data, dev);
                }
            }
        }

        if (rslt == BMP3_SENSOR_OK) {
            rslt = analyze_sensor_data(&data);

            /* Set the power mode to sleep mode */
            if (rslt == BMP3_SENSOR_OK) {
                dev->settings.op_mode = BMP3_MODE_SLEEP;
                rslt = bmp3_set_op_mode(dev);
            }
        }
    }

    return rslt;
}

/** @}*/

void User_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim17) {
        tick = 1;
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
    while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
