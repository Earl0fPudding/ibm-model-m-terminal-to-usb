/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_hid.h"
#include "usb_device.h"
#include "dictionary.c"

Dictionary dictionary; // volatile?

#include "STM32L4_stdlib/timer/timer.c"
#include "STM32L4_stdlib/lcd/lcd.c"
#include "list.c"
#include "ps2_keyboard.c"

#define PRESS_REPORT_SIZE 8
#define KEYPRESS_BUFFER_SIZE 10

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len) {
    /* USER CODE BEGIN 6 */
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
    /* USER CODE END 6 */
}

static uint8_t cur_keymodifier;

void add_modifier_key(uint8_t hidcode) {
    cur_keymodifier |= hidcode;
}

void remove_modifier_key(uint8_t hidcode) {
    cur_keymodifier &= ~(hidcode);
}

uint8_t is_modkey(uint8_t scancode) {
    switch (scancode) { // do hidcode instead
        case 0x12:
        case 0x14:
        case 0x11:
        case 0x19:
        case 0x39:
        case 0x58:
        case 0x59:
            return 1;
    }
    return 0;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();
    disableInterrupts();


    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();


    GPIOA->MODER &= ~(0b11 << 0 * 2); // debug beeper init
    GPIOA->MODER |= (0b01 << 0 * 2);
    GPIOA->PUPDR &= ~((0b11 << 0 * 2));
    GPIOA->ODR &= ~(1 << 0);

    MX_USB_DEVICE_Init(0);



    //HAL_Delay(5000);
    List hid_list;
    init_list(&hid_list);
    init_list(&scancode_list);
    ps2_begin();
    initTimer6();
    lcd_init(LCD_DISP_ON);
    char buff[8];
    uint8_t scancode = 0;
    lcd_puts("testt");
    //lcd_puts("start");


    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    //HAL_GPIO_WritePin(USB_Enable_GPIO_Port, USB_Enable_Pin, SET);

    init_dictionary(&dictionary);
    lcd_clrscr();

    cur_keymodifier = 0;
    uint8_t press_report[PRESS_REPORT_SIZE];
    uint8_t cur_report_index;


    uint16_t cur_x = 0;
    uint8_t next_key_remove = 0;
    uint32_t prev_tick = HAL_GetTick();


    char received_buffer[1];
    received_buffer[0] = '\0';
    uint32_t received_length = 0;
    uint8_t map_mode = 0;

    wait_ms(2000);

    enableInterrupts();

    while (1) {
        if (map_mode == 1) {
            CDC_Receive_FS(&received_buffer, &received_length);
            if (received_buffer[0] != '\0') {
                if (received_buffer[0] == 'f') {

                }
                if (received_buffer[0] == 'p') {
//                    GPIOA->ODR |= (1 << 0);


                    MX_USB_DEVICE_DeInit(1);
                    MX_USB_DEVICE_Init(0);

                    map_mode = 0;
                    continue;
                }
//                CDC_Transmit_FS(received_buffer, sizeof(received_buffer));
                received_buffer[0] = '\0';
            }

        } else {
            if (HAL_GetTick() - prev_tick > 1500) {
                lcd_gotoxy(0, 0);
                lcd_puts("                ");
                cur_x = 0;
                prev_tick = HAL_GetTick();
            }


            memset(press_report, 0, sizeof(press_report));
            cur_report_index = 2;

            press_report[0] = 1; // normal key mode
            press_report[1] = cur_keymodifier;

            // scancode = get_scan_code();
            scancode = pop_first_from_list(&scancode_list);
            if (scancode != 0) {
                // press_report[cur_report_index] = get_hidcode(dictionary, scancode);
/*            GPIOA->ODR|=1<<0;
            HAL_Delay(20);
            GPIOA->ODR&=~(1<<0);*/
                if (scancode == 0x5f) {
                    GPIOA->ODR &= ~(1 << 0);
                }

                if (is_modkey(scancode)) {
                    if (next_key_remove == 1) {
                        remove_modifier_key(get_hidcode(dictionary, scancode));
                        next_key_remove = 0;
                    } else {
                        add_modifier_key(get_hidcode(dictionary, scancode));
                    }
                } else if (scancode == 0xf0) {
                    next_key_remove = 1;
                } else {
                    if (next_key_remove == 1) {
                        remove_from_list(&hid_list, get_hidcode(dictionary, scancode));
                        next_key_remove = 0;
                    } else {
                        add_to_list(&hid_list, get_hidcode(dictionary, scancode));
                    }
                }

                /*  lcd_clrscr();
                  sprintf(buff, "%x ", scancode);
                  lcd_puts(buff);*/

                sprintf(buff, "%x ", scancode);
                lcd_gotoxy(cur_x, 0);
                lcd_puts(buff);
                lcd_gotoxy(0, 1);
                sprintf(buff, "%x ", cur_byte);
                lcd_puts(buff);
                sprintf(buff, "%d ", cur_bit);
                lcd_puts(buff);
                cur_x += 3;


                scancode = 0;


            }
            if (is_in_list(hid_list, 42) && cur_keymodifier == 0x22) {
                MX_USB_DEVICE_DeInit(0);
                MX_USB_DEVICE_Init(1);
                map_mode = 1;
                continue;
            }
            for (uint8_t i = 0; i < hid_list.amount; ++i) {
                press_report[cur_report_index] = hid_list.items[i];
                cur_report_index++;

            }
            USBD_HID_SendReport(&hUsbDeviceFS, press_report, PRESS_REPORT_SIZE);
            /* uint8_t press_report2[PRESS_REPORT_SIZE] = {0};
             press_report2[0] = 1;
             press_report2[1] = cur_keymodifier;
             USBD_HID_SendReport(&hUsbDeviceFS, press_report2, 5);*/

            HAL_Delay(1);


        }
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
