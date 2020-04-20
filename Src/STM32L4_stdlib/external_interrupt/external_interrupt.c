void disableInterrupts(void){
    __disable_irq();
}

void enableInterrupts(void){
    __enable_irq();
}

// initialize the EXTI0 interrupt on pc0
void initEXTI0(void){
    disableInterrupts();
    GPIOC->MODER &= ~(0b11 << 0 * 2); // set pc0 as input pin
    GPIOC->PUPDR &= ~((0b11 << 0 * 2)); // set pc0 to no pull up or pull down

    RCC->APB2ENR|= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] |= 0b010 << SYSCFG_EXTICR1_EXTI0_Pos; // set pc0 as EXTI0 interrupt pin
    EXTI->IMR1 |= EXTI_IMR1_IM0; // enable EXTI0 interrupt
    EXTI->RTSR1 |=EXTI_RTSR1_RT0; // trigger on rising edge at pc0
    NVIC_EnableIRQ(EXTI0_IRQn); // enable the interrupt vector
    enableInterrupts();
}

void EXTI0_IRQHandler(void){
    EXTI->PR1 |= EXTI_PR1_PIF0; // immediately disable the interrupt
    wait_ms(5); // wait 5 ms for bouncing
    if(GPIOC->IDR & GPIO_IDR_ID0) { // check if button is still pressed
        // do the actual interrupt stuff here
        //setTimeAndDate(19, 28, 30, 14, 6, 9, 19);
        GPIOB->ODR ^= (1 << 0); // set PB0 to high
        //disableTimer1();
    }
}