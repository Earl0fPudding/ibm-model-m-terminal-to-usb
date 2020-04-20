#include "rtc.h"

void initRtc(void){
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // enable power for apb1
//    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN; // necessary? apparently not
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RCC->CSR|= RCC_CSR_LSION; // enable LSI
    while (!(RCC->CSR&RCC_CSR_LSIRDY)); // wait for LSI to start
    RCC->BDCR|=0b10 << RCC_BDCR_RTCSEL_Pos; // set LSI as rtc clock
    RCC->BDCR|=RCC_BDCR_RTCEN; // enable RTC
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection

    RTC->ISR|=RTC_ISR_INIT; // set RTC to init mode
    while (!(RTC->ISR & RTC_ISR_INITF)); // wait for RTC to get into init mode

    RTC->PRER=0x007F00FF; // set div a to 127 and div s to 255
    RTC->ISR&=~RTC_ISR_INIT; // tell RTC to leave init mode

    RTC->WPR = 0xFF; // enable the RTC registers' write protection

    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

void stopAlarmA(void){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->ISR &=~RTC_ISR_ALRAF; // turn alarm off
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

void enableAlarmA(void){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection
    RTC->CR |= RTC_CR_ALRAE; // enable alarm A
    RTC->CR |= RTC_CR_ALRAIE; // enable alarm A interrupt in RTC register
    EXTI->IMR1 |= EXTI_IMR1_IM18; // enable alarm interrupt in EXTI register
    EXTI->RTSR1 |=EXTI_RTSR1_RT18; // goto interrupt at rising edge
    NVIC_EnableIRQ(RTC_Alarm_IRQn); // enable the interrupt vector
    //NVIC_SetPriority(RTC_Alarm_IRQn, 0); // optional
    RTC->WPR = 0xFF; // enable write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

void disableAlarmA(void){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection
    RTC->CR &= ~RTC_CR_ALRAE; // unset enable bit for alarm A
    while (!(RTC->ISR & RTC_ISR_ALRAWF)); // wait for bit to be set
    RTC->WPR = 0xFF; // enable write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

uint8_t alarmAenabled(){
    return RTC->CR & RTC_CR_ALRAE; // return 1 if alarm A is enabled else 0
}
void setAlarmA(uint8_t daysMatter, uint8_t domOrdow, uint8_t day, uint8_t hoursMatter, uint8_t hour, uint8_t minutesMatter, uint8_t minutes, uint8_t secondsMatter, uint8_t seconds){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection
    uint8_t enabled=alarmAenabled();
    RTC->CR &= ~RTC_CR_ALRAE;
    while (!(RTC->ISR & RTC_ISR_ALRAWF));
    RTC->ALRMAR = !daysMatter << RTC_ALRMAR_MSK4_Pos |
                  !hoursMatter << RTC_ALRMAR_MSK3_Pos |
                  !minutesMatter << RTC_ALRMAR_MSK2_Pos |
                  !secondsMatter << RTC_ALRMAR_MSK1_Pos |
                  domOrdow << RTC_ALRMAR_WDSEL_Pos |
                  day/10 << RTC_ALRMAR_DT_Pos| day % 10 << RTC_ALRMAR_DU_Pos |
                  hour/10 << RTC_ALRMAR_HT_Pos| hour % 10 << RTC_ALRMAR_HU_Pos |
                  minutes/10 << RTC_ALRMAR_MNT_Pos| minutes % 10 << RTC_ALRMAR_MNU_Pos |
                  seconds/10 << RTC_ALRMAR_ST_Pos| seconds % 10 << RTC_ALRMAR_SU_Pos;
    RTC->CR |= enabled << RTC_CR_ALRAE_Pos;
    RTC->WPR = 0xFF; // enable write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

uint8_t alarmAwentoff(void){
    if(RTC->ISR & RTC_ISR_ALRAF){ return 1;}
    return 0;
}

void stopAlarmB(void){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->ISR &=~RTC_ISR_ALRBF; // turn alarm off
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

void enableAlarmB(void){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection
    RTC->CR |= RTC_CR_ALRBE;
/*    RTC->CR |= RTC_CR_ALRAIE;
    EXTI->IMR1 |= EXTI_IMR1_IM18;
    EXTI->RTSR1 |=EXTI_RTSR1_RT18;
    NVIC_EnableIRQ(RTC_Alarm_IRQn);
    NVIC_SetPriority(RTC_Alarm_IRQn, 0); */
    RTC->WPR = 0xFF; // enable write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

void disableAlarmB(void){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection
    RTC->CR &= ~RTC_CR_ALRBE;
    while (!(RTC->ISR & RTC_ISR_ALRBWF));
    RTC->WPR = 0xFF; // enable write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

uint8_t alarmBenabled(){
    return RTC->CR & RTC_CR_ALRBE;
}

void setAlarmB(uint8_t daysMatter, uint8_t domOrdow, uint8_t day, uint8_t hoursMatter, uint8_t hour, uint8_t minutesMatter, uint8_t minutes, uint8_t secondsMatter, uint8_t seconds){
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection
    uint8_t enabled=alarmBenabled();
    RTC->CR &= ~RTC_CR_ALRBE;
    while (!(RTC->ISR & RTC_ISR_ALRBWF));
    RTC->ALRMBR = !daysMatter << RTC_ALRMBR_MSK4_Pos |
                  !hoursMatter << RTC_ALRMBR_MSK3_Pos |
                  !minutesMatter << RTC_ALRMBR_MSK2_Pos |
                  !secondsMatter << RTC_ALRMBR_MSK1_Pos |
                  domOrdow << RTC_ALRMBR_WDSEL_Pos |
                  day/10 << RTC_ALRMBR_DT_Pos| day % 10 << RTC_ALRMBR_DU_Pos |
                  hour/10 << RTC_ALRMBR_HT_Pos| hour % 10 << RTC_ALRMBR_HU_Pos |
                  minutes/10 << RTC_ALRMBR_MNT_Pos| minutes % 10 << RTC_ALRMBR_MNU_Pos |
                  seconds/10 << RTC_ALRMBR_ST_Pos| seconds % 10 << RTC_ALRMBR_SU_Pos;
    RTC->CR |= enabled << RTC_CR_ALRBE_Pos;
    RTC->WPR = 0xFF; // enable write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}

uint8_t alarmBwentoff(void){
    if(RTC->ISR & RTC_ISR_ALRBF){ return 1;}
    return 0;
}

uint8_t getCurrentSeconds(void){
    return ((RTC->TR & RTC_TR_ST) >> RTC_TR_ST_Pos)*10+(RTC->TR & RTC_TR_SU) >> RTC_TR_SU_Pos;
}

uint8_t getCurrentMinutes(void){
    return ((RTC->TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos)*10+((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos);
}

uint8_t getCurrentHours(void){
    return ((RTC->TR & RTC_TR_HT) >> RTC_TR_HT_Pos)*10+((RTC->TR & RTC_TR_HU) >> RTC_TR_HU_Pos);
}

uint8_t getCurrentDay(void){
    return ((RTC->DR & RTC_DR_DT) >> RTC_DR_DT_Pos)*10+((RTC->DR & RTC_DR_DU) >> RTC_DR_DU_Pos);
}

uint8_t getCurrentMonth(void){
    return ((RTC->DR & RTC_DR_MT) >> RTC_DR_MT_Pos)*10+((RTC->DR & RTC_DR_MU) >> RTC_DR_MU_Pos);
}

uint8_t getCurrentYear(void){
    return ((RTC->DR & RTC_DR_YT) >> RTC_DR_YT_Pos)*10+((RTC->DR & RTC_DR_YU) >> RTC_DR_YU_Pos);
}

uint16_t getCurrentYearFull(void){
    return getCurrentYear()+2000;
}

uint8_t getCurrentWeekdayNum(void){
    return (RTC->DR & RTC_DR_WDU) >> RTC_DR_WDU_Pos;
}

char* getCurrentWeekdayFull(void){
    return Fullweekdays[getCurrentWeekdayNum()-1];
}
char* getCurrentWeekdayShort(void){
    return Shortweekdays[getCurrentWeekdayNum()-1];
}

// DONT FORGET TO FREE MEMORY
char* getCurrentDate(uint8_t fullYear, uint8_t includeWeekday, char delimiter){
    char *date=(char *)malloc(15);
    char buffer[4];
    uint8_t offset=0;
    if(includeWeekday){
        offset=5;
        date[0]=getCurrentWeekdayShort()[0];
        date[1]=getCurrentWeekdayShort()[1];
        date[2]=getCurrentWeekdayShort()[2];
        date[3]=',';
        date[4]=' ';
    }
    uint16_t  tmp=getCurrentDay();
    sprintf(buffer, "%d", tmp);
    if(tmp<10){
        date[offset+0]='0';
        date[offset+1]=buffer[0];
    } else{
        date[offset+0]=buffer[0];
        date[offset+1]=buffer[1];
    }
    date[offset+2]=delimiter;
    tmp=getCurrentMonth();
    sprintf(buffer, "%d", tmp);
    if(tmp<10){
        date[offset+3]='0';
        date[offset+4]=buffer[0];
    } else{
        date[offset+3]=buffer[0];
        date[offset+4]=buffer[1];
    }
    date[offset+5]=delimiter;
    if(fullYear) {
        tmp = getCurrentYearFull();
        sprintf(buffer, "%d", tmp);
        date[offset+6] = buffer[0];
        date[offset+7] = buffer[1];
        date[offset+8] = buffer[2];
        date[offset+9] = buffer[3];
        date[offset+10] = '\0';
    } else {
        tmp=getCurrentYear();
        sprintf(buffer, "%d", tmp);
        if(tmp<10){
            date[offset+6]='0';
            date[offset+7]=buffer[0];
        }
        else{
            date[offset+6]=buffer[0];
            date[offset+7]=buffer[1];
        }
        date[offset+8] = '\0';
    }
    return date;
}

// DONT FORGET TO FREE MEMORY
char* getCurrentTime(void){
    char *time=(char *)malloc(8);
    char buffer[2];
    char* delimiter=":";
    uint8_t tmp=getCurrentHours();
    sprintf(buffer, "%d", tmp);
    if(tmp<10){
        time[0]='0';
        time[1]=buffer[0];
    } else{
        time[0]=buffer[0];
        time[1]=buffer[1];
    }
    time[2]=':';
    tmp=getCurrentMinutes();
    sprintf(buffer, "%d", tmp);
    if(tmp<10){
        time[3]='0';
        time[4]=buffer[0];
    } else{
        time[3]=buffer[0];
        time[4]=buffer[1];
    }
    time[5]=':';
    tmp=getCurrentSeconds();
    sprintf(buffer, "%d", tmp);
    if(tmp<10){
        time[6]='0';
        time[7]=buffer[0];
    } else{
        time[6]=buffer[0];
        time[7]=buffer[1];
    }
    time[8]='\0';
    return time;
}

void configRtcRefIn(void){
    // 50Hz PB15
    GPIOB->MODER &= ~((0b11 << 15 * 2));
    GPIOB->MODER |= (0b10 << 15 * 2); // set pb0 as output pin
    GPIOB->OTYPER &= ~((1 << 15)); // set every pin to push pull on gpiob
    GPIOB->OSPEEDR |= (0b11 << 15 * 2); // set every pin to highest speed on gpiob
    GPIOB->PUPDR &= ~((0b11 << 15 * 2)); // set every pin to no pull up or pull down in gpiob
    GPIOB->AFR[1] |= ~(0b1111 << GPIO_AFRH_AFSEL15_Pos);
}

void configRtcRefOut(void){
    // rtc out PB2
    GPIOB->MODER &= ~((0b11 << 2 * 2));
    GPIOB->MODER |= (0b10 << 2 * 2); // set pb0 as output pin
    GPIOB->OTYPER &= ~((1 << 2)); // set every pin to push pull on gpiob
    GPIOB->OSPEEDR |= (0b11 << 2 * 2); // set every pin to highest speed on gpiob
    GPIOB->PUPDR &= ~((0b11 << 2 * 2)); // set every pin to no pull up or pull down in gpiob
    GPIOB->AFR[0] |= ~(0b1111 << GPIO_AFRL_AFSEL2_Pos);
    RTC->CR|=RTC_CR_COE;
    RTC->OR |= RTC_OR_OUT_RMP;
}

void setTimeAndDate(uint8_t hour, uint8_t minute, uint8_t second, uint8_t dayOfMonth, uint8_t dayOfWeek, uint8_t month, uint8_t year) {
    PWR->CR1 |= PWR_CR1_DBP; // write access to rtc and backup registers enabled
    RTC->WPR = 0xCA; // first step to disable RTC registers' write protection
    RTC->WPR = 0x53; // final step to disable RTC registers' write protection

    RTC->ISR |= RTC_ISR_INIT; // set RTC to init mode
    while (!(RTC->ISR & RTC_ISR_INITF)); // wait for RTC to get into init mode

    RTC->DR = dayOfWeek << RTC_DR_WDU_Pos |
              dayOfMonth / 10 << RTC_DR_DT_Pos | dayOfMonth % 10 << RTC_DR_DU_Pos |
              month / 10 << RTC_DR_MT_Pos | month % 10 << RTC_DR_MU_Pos |
              year / 10 << RTC_DR_YT_Pos | year % 10 << RTC_DR_YU_Pos;
    RTC->TR = second / 10 << RTC_TR_ST_Pos | second % 10 << RTC_TR_SU_Pos |
              minute / 10 << RTC_TR_MNT_Pos | minute % 10 << RTC_TR_MNU_Pos |
              hour / 10 << RTC_TR_HT_Pos | hour % 10 << RTC_TR_HU_Pos;

    RTC->ISR &= ~RTC_ISR_INIT; // tell RTC to leave init mode

    RTC->WPR = 0xFF; // enable the RTC registers' write protection
    PWR->CR1 &= ~PWR_CR1_DBP; // write access to rtc and backup registers disabled
}