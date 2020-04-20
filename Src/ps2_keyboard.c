#define BUFFER_SIZE 45

#include "external_interrupts.c"

static uint8_t send_message = 0xFE;

static volatile uint8_t cur_bit = 0;
static volatile uint8_t cur_byte = 0;
static volatile uint8_t val = 0;
static volatile uint8_t send_mode = 0, resend = 0, is_break_mode = 0;
static List scancode_list; // volatile?

uint8_t generate_odd_parity_bit(uint8_t message) {
    uint8_t ones = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        ones += (message & (1 << i)) != 0;
    }
    return (ones % 2 == 0);
}


void EXTI1_IRQHandler(void) {
    EXTI->PR1 |= EXTI_PR1_PIF1; // immediately disable the interrupt

    if (send_mode) {
        if (cur_bit == 0) {

        }
        if (cur_bit > 0 && cur_bit < 9) {
            if ((send_message & (1 << (cur_bit - 1))) != 0 && (GPIOA->ODR & (1 << 1)) != 0) {
                GPIOA->ODR &= ~(1 << 1);
            } else if ((send_message & (1 << (cur_bit - 1))) == 0 && (GPIOA->ODR & (1 << 1)) == 0) {
                GPIOA->ODR |= (1 << 1);
            }
        }
        if (cur_bit == 9) {
            if (generate_odd_parity_bit(send_message) != 0 && (GPIOA->ODR & (1 << 1)) != 0) {
                GPIOA->ODR &= ~(1 << 1);
            } else if (generate_odd_parity_bit(send_message) == 0 && (GPIOA->ODR & (1 << 1)) == 0) {
                GPIOA->ODR |= (1 << 1);
            }
        }
        if (cur_bit == 10) {
            GPIOA->ODR &= ~(1 << 1);
        }
        if (cur_bit == 11) {
            if ((GPIOC->IDR & (1 << 3)) == 0) {
                send_mode = 0;
                cur_bit = 0;
                return;
            } else{
                disableInterrupts();
                GPIOC->ODR |= (1 << 2);
                for (uint8_t i = 0; i < 255; ++i) {
                    __NOP();
                }
                GPIOA->ODR |= (1 << 1);
                for (uint8_t i = 0; i < 255; ++i) {
                    __NOP();
                }
                enableInterrupts();
                GPIOC->ODR &= ~(1 << 2);

                send_mode = 1;
                cur_bit = 0;
                return;
            }
        }

    } else {
        val = GPIOC->IDR;

        val = (val & (1 << 3)) != 0;

        if (cur_bit == 0) { // START BIT
            if (val == 1) { // restart, bit has to be low
                resend = 1;

                cur_byte = 0;
                cur_bit = 0;
                return;
            }
        }
        if (cur_bit > 0 && cur_bit < 9) { // DATA BITS
            cur_byte |= val << (cur_bit - 1);
        }
        if (cur_bit == 9) { // PARITY BIT
            uint8_t ones = 0;
            for (uint8_t i = 0; i < 8; ++i) {
                ones += (cur_byte & (1 << i)) != 0;
            }
            if (ones % 2 == val) {
                resend = 1;

                cur_byte = 0;
                //cur_bit = 0;
                //return;
            }
        }
        if (cur_bit == 10) { // STOP BIT
            if (val == 1) {
                add_to_list(&scancode_list, cur_byte);
            } else {
                resend = 1;
            }

            if (cur_byte == 0xf0) {
                is_break_mode = 1;
            }
            if(resend){
                GPIOA->ODR |= (1 << 0);
            }

            if (resend || cur_byte == 0xfe || (is_break_mode == 0 && scancode_is_in_dictionary(dictionary, cur_byte))) {
                send_mode = 1;
                resend = 0;


                disableInterrupts();
                if (is_break_mode == 0) {
                    send_message = 0xF8;
                } else {
                    send_message = 0xFE;
                }

                GPIOC->ODR |= (1 << 2);
                for (uint8_t i = 0; i < 255; ++i) {
                    __NOP();
                }
                GPIOA->ODR |= (1 << 1);
                for (uint8_t i = 0; i < 255; ++i) {
                    __NOP();
                }
                enableInterrupts();
                GPIOC->ODR &= ~(1 << 2);
                // lcd_puts("lol");

            }

            cur_byte = 0;
            cur_bit = 0;
            return;
        }
    }


    cur_bit++;

}

void ps2_begin() {
    GPIOC->MODER &= ~(0b11 << 3 * 2); // set pc3 as input pin
    GPIOC->PUPDR &= ~((0b11 << 3 * 2)); // set pc3 to no pull up or pull down
    GPIOC->PUPDR |= ((0b01 << 3 * 2)); // set pc3 to pull up
    //   GPIOC->ODR |= 0b1 << 3; // set pc0 high

    initEXTI1();
}
