void initRng(void) {
    RCC->CRRCR|=RCC_CRRCR_HSI48ON;
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    RNG->CR |= RNG_CR_RNGEN;
}

uint8_t areClocksSuccessfullyConfigured(void){
    return !((RNG->SR & RNG_SR_CECS) >> 1);
}

uint32_t getRandomNumber(void){
    while (!(RNG->SR & RNG_SR_DRDY) || (RNG->SR & (RNG_SR_SEIS) || (RNG->SR & (RNG_SR_CEIS))));
    return RNG->DR;
}

uint32_t getRandomBinaryDigits(uint8_t digits){
    uint8_t maxpos = digits; //(uint8_t) roundf(__log2f(maxInc));
    uint32_t bitmask= 0;
    for (uint8_t i = 0; i < maxpos; ++i) {
        bitmask|=1 << i;
    }
    return getRandomNumber() & bitmask;
}

char getRandomAsciiChar(){
    return getRandomBinaryDigits(8);
}

// returns a true random number between the first parameter inclusive and the second parameter inclusive
uint32_t getRandomNumberBetween(uint32_t minInc, uint32_t maxInc){
    if(minInc > maxInc) { return 0;} // prevents microcontroller from crashing
    uint32_t randy=minInc;
    for (uint32_t i = 0; i < maxInc-minInc; ++i) {
        randy+=getRandomBinaryDigits(1);
    }
    return randy;
}