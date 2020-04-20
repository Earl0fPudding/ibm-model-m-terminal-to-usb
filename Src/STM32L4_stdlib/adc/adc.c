#include "adc.h"

void initAdc1(void){
    RCC->AHB2ENR|=RCC_AHB2ENR_ADCEN; // enable adc for the bus
    RCC->CCIPR|= 0b11 << RCC_CCIPR_ADCSEL_Pos; // system clock as adc clock
    ADC1_COMMON->CCR|= ADC_CCR_VREFEN; // enable reference voltage

    ADC1->CFGR &= ~ADC_CFGR_RES_Pos; // 12 bit resolution
    ADC1->SMPR1 |=  0b111 << ADC_SMPR1_SMP6_Pos;
    ADC1->SMPR2 |=  0b111 << ADC_SMPR2_SMP17_Pos;

    // calibrate the adc
    ADC1->CR &= ~ADC_CR_DEEPPWD; // adc not in low power mode
    ADC1->CR |= ADC_CR_ADVREGEN;
    ADC1->CR &= ~ADC_CR_ADEN;
    ADC1->CR &= ~ADC_CR_ADCALDIF; // single ended input
    ADC1->CR |=ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    // enable temperature sensor
    //ADC1_COMMON->CCR |= ADC_CCR_TSEN;

    ADC1_COMMON->CCR&= ~ADC_CCR_VREFEN; // enable reference voltage


    //set input
    ADC1->SQR1 &= ~ADC_SQR1_L; // 1 conversion
    //ADC1->SQR1 |= 0 << ADC_SQR1_SQ1; // temp sensor on sequence 1
   // ADC1->SQR1 |= 17 << ADC_SQR1_SQ1; // temp sensor on sequence 1
    ADC1->SQR1 |= 6 << ADC_SQR1_SQ1; // pa1

    // enable the adc
    ADC1->ISR |=ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->ISR |=ADC_ISR_ADRDY;
}

uint16_t getAdc1Conversion(void){
    ADC1->CR |=ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC));
    return ADC1->DR;
}

float getJunctionTemperature(){
    return (( (TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1) ) * (getAdc1Conversion() - TS_CAL1) + 30)/10.0;
}

