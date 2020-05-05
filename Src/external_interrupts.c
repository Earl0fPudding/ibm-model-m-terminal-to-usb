void disableInterrupts(void){
    __disable_irq();
}

void enableInterrupts(void){
    __enable_irq();
}

// initialize the EXTI0 interrupt on pc0
void initEXTI1(void){
    //disableInterrupts();
    GPIOC->MODER &= ~(0b11 << 1 * 2); // set pc0 as input pin
    GPIOC->PUPDR &= ~((0b11 << 1 * 2)); // set pc0 to no pull up or pull down
    GPIOC->PUPDR |= ((0b01 << 1 * 2)); // set pc1 to no pull up or pull down
    GPIOC->OTYPER |= (1 << 1);

   // GPIOC->ODR |= 0b1 << 1; // set pc0 high

    RCC->APB2ENR|= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] |= 0b010 << SYSCFG_EXTICR1_EXTI1_Pos; // set pc0 as EXTI0 interrupt pin
    EXTI->IMR1 |= EXTI_IMR1_IM1; // enable EXTI0 interrupt
    EXTI->RTSR1 |= EXTI_FTSR1_FT1; // trigger on falling edge at pc0
    NVIC_EnableIRQ(EXTI1_IRQn); // enable the interrupt vector
    //enableInterrupts();
}