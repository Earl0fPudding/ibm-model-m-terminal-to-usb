#include "usart.h"
#include "../timer/timer.c"
#include <math.h>
#include <string.h>

void serialSendData(uint8_t data) {
    USART1->TDR = data;
    while (!(USART1->ISR & USART_ISR_TC));
}

void serialSTX(void) {
    serialSendData(2);
}

void serialETX(void) {
    serialSendData(3);
}

void serialNewLine(void) {
    serialSendData(10); // new line
    serialSendData(13); // carriage return
}

void serialSendBulkData(uint8_t *collection) {
    for (int i = 0; i < strlen(collection); ++i) {
        serialSendData(collection[i]);
    }
}

void serialPrint(char *string) {
    serialSTX();
    serialSendBulkData(string);
    serialETX();
}

void serialPrintLine(char *string) {
    serialSTX();
    serialSendBulkData(string);
    serialNewLine();
    serialETX();
}

void initUsart1(uint32_t baud, uint64_t clockspeed) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_AHB2ENR_GPIOAEN;
    USART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8 bit
    USART1->BRR = (uint16_t) round(clockspeed / baud);
    USART1->CR2 &= ~(0b11 << USART_CR2_STOP_Pos);
    USART1->CR1 |= USART_CR1_UE;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;

    GPIOA->MODER &= ~((0b11 << 9 * 2));
    GPIOA->MODER |= (0b10 << 9 * 2); // set pb0 as output pin
    GPIOA->OTYPER &= ~((1 << 9)); // set every pin to push pull on gpiob
    GPIOA->OSPEEDR |= (0b11 << 9 * 2); // set every pin to highest speed on gpiob
    GPIOA->PUPDR &= ~((0b11 << 9 * 2)); // set every pin to no pull up or pull down in gpiob
    GPIOA->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL9_Pos);

    GPIOA->MODER &= ~((0b11 << 10 * 2));
    GPIOA->MODER |= (0b10 << 10 * 2); // set pb0 as output pin
    GPIOA->OTYPER &= ~((1 << 10)); // set every pin to push pull on gpiob
    GPIOA->OSPEEDR |= (0b11 << 10 * 2); // set every pin to highest speed on gpiob
    GPIOA->PUPDR &= ~((0b11 << 10 * 2)); // set every pin to no pull up or pull down in gpiob
    GPIOA->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL10_Pos);

}

char serialReadKey(uint8_t showKey) {
    while (!(USART1->ISR & USART_ISR_RXNE));
    char recchar = USART1->RDR;
    char out[] = {recchar};
    if (showKey && recchar != 13 && recchar != 10) {
        serialSendData(recchar);
    }
    return recchar;
}

uint8_t serialCheckLine(char *requestedString) {
    char ret[50] = "";
    while (1) {
        char newChar = serialReadKey(1);
        if (newChar == 13 || newChar == 10) { break; }
        char catChar[] = {newChar, '\0'};
        strcat(ret, catChar);
    }
    return strcmp(ret, requestedString) == 0;
}

char *serialReadLine() {
    char ret[50] = "";
    while (1) {
        char newChar = serialReadKey(1);
        if (newChar == 13 || newChar == 10) { break; }
        char catChar[] = {newChar, '\0'};
        strcat(ret, catChar);
    }
    return ret;
}

const char *serialReadLineWithReturn(char *message) {
    serialPrint(message);
    static char ret[50] = "";
    while (1) {
        char newChar = serialReadKey(1);
        if (newChar == 13 || newChar == 10) { break; }
        if (newChar == 8 || newChar==127) {
            ret[strlen(ret)-1]='\0';
            serialNewLine(); serialPrint(message);
            serialPrint(ret);
        } else {
            char catChar[] = {newChar, '\0'};
            strcat(ret, catChar);
        }
    }
    return ret;
}

uint8_t serialCheckLineWithReturn(char *message, char *options[]) {
    serialPrint(message);
    char ret[50] = "";
    while (1) {
        char newChar = serialReadKey(1);
        if (newChar == 13 || newChar == 10) { break; }
        if (newChar == 8 || newChar==127) {
            ret[strlen(ret)-1]='\0';
            serialNewLine(); serialPrint(message);
            serialPrint(ret);
        } else {
            char catChar[] = {newChar, '\0'};
            strcat(ret, catChar);
        }
    }
    for (int i = 0; i < sizeof(options)/sizeof(options[0]); ++i) {
        if(strcmp(ret, options[i])==0){return i+1;}
    }
    return 0;
}