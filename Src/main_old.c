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
#include "usb_device.h"
#include "dictionary.c"
#include "ps2_keyboard.c"
#include "STM32L4_stdlib/timer/timer.c"
#include "STM32L4_stdlib/lcd/lcd.c"

#define PRESS_REPORT_SIZE 5
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

typedef struct Keypress {
    uint8_t hidcode;
    uint32_t tick;
    uint8_t first_press_done; // maybe uint16 or 32
} Keypress;

static Keypress key_buffer[KEYPRESS_BUFFER_SIZE];
static uint8_t key_pressed_mask[KEYPRESS_BUFFER_SIZE];
static uint8_t cur_hid_keys[KEYPRESS_BUFFER_SIZE];
static uint8_t cur_keymodifier;
static uint8_t last_modkey=0;
// automatic remove curbit and cutbyte when no press for some millisecs
// automatic remove modkey from cur_keymodifier when (f0 is pressed without an modkey innerhalb von 0,5 secs) or the same key is pressed again

void init_key_buffer(void) {
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        key_buffer[i].hidcode = 0;
        key_pressed_mask[i] = 0;
        key_buffer[i].first_press_done = 0;
        cur_hid_keys[i] = 0;
    }
    cur_keymodifier = 0;
}

void add_key(uint8_t hidcode) {
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (key_buffer[i].hidcode == hidcode) {
            key_pressed_mask[i] = 1;
            key_buffer[i].tick = HAL_GetTick();

            return;
        }
    }
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (key_buffer[i].hidcode == 0) {
            key_buffer[i].hidcode = hidcode;
            key_buffer[i].tick = HAL_GetTick();
            return;
        }
    }
}

void add_to_cur_hid_keys(uint8_t hidcode) {
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (cur_hid_keys[i] == hidcode) { return; }
    }
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (cur_hid_keys[i] == 0) {
            cur_hid_keys[i] = hidcode;
        }
    }
}

void add_modifier_key(uint8_t hidcode) {
    cur_keymodifier |= hidcode;
}

void remove_modifier_key(uint8_t hidcode) {
    cur_keymodifier &= ~(hidcode);
}

void remove_from_cur_hid_keys(uint8_t hidcode) {
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (cur_hid_keys[i] == hidcode) {
            cur_hid_keys[i] = 0;
//            return;
        }
    }
}

uint8_t count_cur_hid_keys() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (key_buffer[i].hidcode != 0) {
            count++;
        }
    }
    return count;
}

void send_hid_keys_usb(void) {
    char str_buff[10];
    uint8_t press_report[PRESS_REPORT_SIZE] = {0};
    uint8_t cur_report_index = 2;

    press_report[0] = 1; // win left
    press_report[1] = cur_keymodifier;

    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (key_buffer[i].hidcode != 0) {
            if (HAL_GetTick() - key_buffer[i].tick < 500 && key_buffer[i].first_press_done == 0) {
                add_to_cur_hid_keys(key_buffer[i].hidcode);
                key_buffer[i].first_press_done = 1;
            } else {
                if (HAL_GetTick() - key_buffer[i].tick < 200) {
                    //  press_report[cur_report_index] = key_buffer[i].hidcode;
                    //  cur_report_index++;
                } else {
                    remove_from_cur_hid_keys(key_buffer[i].hidcode);
                    key_buffer[i].hidcode = 0;
                    key_pressed_mask[i] = 0;
                    key_buffer[i].first_press_done = 0;
                }
            }
        }
    }

    // lcd_clrscr();
    for (uint8_t i = 0; i < KEYPRESS_BUFFER_SIZE; ++i) {
        if (cur_hid_keys[i] != 0) {
            //sprintf(str_buff, "%d ", cur_hid_keys[i]);
            //lcd_puts(str_buff);
            press_report[cur_report_index] = cur_hid_keys[i];
            cur_report_index++;


        }

    }
    //lcd_clrscr();
    //sprintf(str_buff, "%d ", count_cur_hid_keys());
    //lcd_puts(str_buff);

    USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);
    HAL_Delay(10);
}

uint8_t is_modkey(uint8_t scancode) {
    switch (scancode) {
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
    uint8_t press_report[PRESS_REPORT_SIZE] = {0};

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

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();

    //HAL_Delay(5000);
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

    Dictionary dictionary;
    init_dictionary(&dictionary);
    lcd_clrscr();

    init_key_buffer();


    uint16_t cur_x = 0, resetbit_tmp = 0, prev_bit = 255;
    uint8_t next_mod_remove = 0;
    uint32_t prev_tick=HAL_GetTick();
    while (1) {
        send_hid_keys_usb();
        if (resetbit_tmp == 10) {
            disableInterrupts();
            if (cur_bit == prev_bit) {
                cur_bit = 0;
                cur_byte = 0;
                prev_bit = 255;
            } else {
                prev_bit = cur_bit;
            }
            enableInterrupts();
            resetbit_tmp = 0;
        }
        //  resetbit_tmp++;
        if (HAL_GetTick()-prev_tick>700) {
            lcd_gotoxy(0, 0);
            lcd_puts("                ");
            cur_x = 0;
            prev_tick=HAL_GetTick();
        }

        scancode = get_scan_code();
        if (scancode != 0) {
            if (is_modkey(scancode)) {
                if (next_mod_remove == 1) {
                    remove_modifier_key(get_hidcode(dictionary, scancode));
                    next_mod_remove=0;
                } else {
                    if(scancode==14){
                        add_key(get_hidcode(dictionary, scancode));
                    }else {
                        add_modifier_key(get_hidcode(dictionary, scancode));
                    }
                }
            } else if (scancode == 0xf0) {
                next_mod_remove=1;
            } else {
                add_key(get_hidcode(dictionary, scancode));
            }
          /*  lcd_clrscr();
            sprintf(buff, "%x ", scancode);
            lcd_puts(buff);*/

            sprintf(buff, "%x  ", scancode);
            lcd_gotoxy(cur_x, 0);
            lcd_puts(buff);
            lcd_gotoxy(0, 1);
            sprintf(buff, "%x ", cur_byte);
            lcd_puts(buff);
            sprintf(buff, "%d ", cur_bit);
            lcd_puts(buff);
            cur_x += 3;



            //HAL_Delay(500);
            //lcd_clrscr();
/*
            press_report[0] = 1; // win left
            press_report[1] = 0;
            press_report[2] = get_hidcode(dictionary, scancode);  // send 'd'
            press_report[3] = 0;  // send '0'
            press_report[4] = 0;  // send 'm'
            press_report[5] = 0;  // send 'e'
            press_report[6] = 0;  // send 'n'


            USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);

            HAL_Delay(5);

            press_report[0] = 1; // win left
            press_report[1] = 0;
            press_report[2] = 0;  // send 'd'
            press_report[3] = 0;  // send '0'
            press_report[4] = 0;  // send 'm'
            press_report[5] = 0;  // send 'e'
            press_report[6] = 0;  // send 'n'


            USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);

            HAL_Delay(5);

            press_report[0] = 1; // win left
            press_report[1] = 0;
            press_report[2] = 0;  // send 'd'
            press_report[3] = 0;  // send '0'
            press_report[4] = 0;  // send 'm'
            press_report[5] = 0;  // send 'e'
            press_report[6] = 0;  // send 'n'


            USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);
            HAL_Delay(5);

            press_report[0] = 1; // win left
            press_report[1] = 0;
            press_report[2] = 0;  // send 'd'
            press_report[3] = 0;  // send '0'
            press_report[4] = 0;  // send 'm'
            press_report[5] = 0;  // send 'e'
            press_report[6] = 0;  // send 'n'


            USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);

*/
            scancode = 0;


        }


        /* press_report[0]=1;
         press_report[1]=0x04;
         USBD_HID_SendReport(&hUsbDeviceFS, press_report, 2);

         HAL_Delay(5);*/




        /* USER CODE END WHILE */

        /*
        press_report[0] = 2; // win left
        press_report[1] = 0xE9;
        press_report[2] = 0;  // send 'd'
        press_report[3] = 0;  // send '0'
        press_report[4] = 0;  // send 'm'
        press_report[5] = 0;  // send 'e'
        press_report[6] = 0;  // send 'n'



        USBD_HID_SendReport(&hUsbDeviceFS, press_report, 2);

        HAL_Delay(5);

        press_report[0] = 1; // win left
        press_report[1] = 0;
        press_report[2] = 0x04;  // send 'd'
        press_report[3] = 0;  // send '0'
        press_report[4] = 0;  // send 'm'
        press_report[5] = 0;  // send 'e'
        press_report[6] = 0;  // send 'n'



        USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);

        HAL_Delay(5);

        press_report[0] = 1; // win left
        press_report[1] = 0;
        press_report[2] = 0;  // send 'd'
        press_report[3] = 0;  // send '0'
        press_report[4] = 0;  // send 'm'
        press_report[5] = 0;  // send 'e'
        press_report[6] = 0;  // send 'n'



        USBD_HID_SendReport(&hUsbDeviceFS, press_report, 5);

        HAL_Delay(5);


        //press_report[2] = 0;  // send button release
        press_report[0] = 1;
        press_report[1] = 0;
        press_report[2] = 0;
        press_report[3] = 0;
        press_report[4] = 0;
        press_report[5] = 0;
        press_report[6] = 0;

        USBD_HID_SendReport(&hUsbDeviceFS, press_report, PRESS_REPORT_SIZE);

        HAL_Delay(5);

        //press_report[2] = 0;  // send button release
        press_report[0] = 1;
        press_report[1] = 0;
        press_report[2] = 0;
        press_report[3] = 0;
        press_report[4] = 0;
        press_report[5] = 0;
        press_report[6] = 0;

        USBD_HID_SendReport(&hUsbDeviceFS, press_report, PRESS_REPORT_SIZE);

        HAL_Delay(5);

        //press_report[2] = 0;  // send button release
        press_report[0] = 1;
        press_report[1] = 0;
        press_report[2] = 0;
        press_report[3] = 0;
        press_report[4] = 0;
        press_report[5] = 0;
        press_report[6] = 0;

        USBD_HID_SendReport(&hUsbDeviceFS, press_report, PRESS_REPORT_SIZE);

        HAL_Delay(5);

        //press_report[2] = 0;  // send button release
        press_report[0] = 1;
        press_report[1] = 0;
        press_report[2] = 0;
        press_report[3] = 0;
        press_report[4] = 0;
        press_report[5] = 0;
        press_report[6] = 0;

        USBD_HID_SendReport(&hUsbDeviceFS, press_report, PRESS_REPORT_SIZE);
*/
        HAL_Delay(5);



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
