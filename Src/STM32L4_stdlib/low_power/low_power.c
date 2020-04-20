void enableStop2(void){
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    PWR->CR1 |=PWR_CR1_LPR | 0b010 << PWR_CR1_LPMS_Pos;
}

void waitForInterrupt(void){
    __WFI();
}