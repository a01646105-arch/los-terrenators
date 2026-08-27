#include "MKL25Z4.h"
#include <stdio.h>

/* Global variable to store incoming byte from interrupt */
volatile char rx_flag = 0;

/* Declaración de funciones */
void UART0_init(void);
void UART0_putc(char c);
char UART0_getc(void);
int UART0_has_data(void);
void UART0_puts(const char *str);

void peripherals_init(void);
void delay_ms(uint32_t ms);
uint16_t ADC0_read(void);
char keypad_scan(void);

void showMenu(void);
void processCommand(char option);

void optionLED(void);
void optionADC(void);
void optionKeypad(void);
void optionButtons(void);

void UART0_IRQHandler(void);

/* -------------------------------------------------
 * FUNCIONES AUXILIARES Y PERIFÉRICOS
 * ------------------------------------------------- */

void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3000; j++) __NOP();
}

int UART0_has_data(void)
{
    return (rx_flag != 0) ? 1 : 0;
}

void peripherals_init(void)
{
    /* Habilitar relojes para PORTA, PORTB, PORTC, PORTD, PORTE y ADC0 */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK |
                  SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    /* Configuración LED RGB (PTB18, PTB19, PTD1) */
    PORTB->PCR[18] = PORT_PCR_MUX(1);
    PORTB->PCR[19] = PORT_PCR_MUX(1);
    PORTD->PCR[1]  = PORT_PCR_MUX(1);
    PTB->PDDR |= (1 << 18) | (1 << 19);
    PTD->PDDR |= (1 << 1);
    PTB->PSOR = (1 << 18) | (1 << 19); /* Apagar LEDs (Active LOW) */
    PTD->PSOR = (1 << 1);

    /* Configuración Botones (PTC5, PTC12) con Pull-Up */
    PORTC->PCR[5]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTC->PDDR &= ~((1 << 5) | (1 << 12));

    /* Configuración ADC0 (PTB0 - ADC0_SE8) */
    PORTB->PCR[0] = PORT_PCR_MUX(0);
    ADC0->CFG1 = ADC_CFG1_MODE(2) | ADC_CFG1_ADICLK(0) | ADC_CFG1_ADIV(1); /* 12-bit */
    ADC0->SC1[0] = ADC_SC1_ADCH(31);

    /* --- Configuración Teclado Matricial --- */
    /* Filas: PTE0-PTE3 (Salidas en HIGH) */
    for (int i = 0; i < 4; i++) {
        PORTE->PCR[i] = PORT_PCR_MUX(1);
        PTE->PDDR |= (1 << i);
        PTE->PSOR = (1 << i);
    }

    /* Columnas: PTC0-PTC3 (Entradas con Pull-Up) */
    for (int i = 0; i < 4; i++) {
        PORTC->PCR[i] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
        PTC->PDDR &= ~(1 << i);
    }
}

uint16_t ADC0_read(void)
{
    ADC0->SC1[0] = ADC_SC1_ADCH(8); /* Canal ADC0_SE8 */
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return (uint16_t)ADC0->R[0];
}

char keypad_scan(void)
{
    const char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

    for (int row = 0; row < 4; row++) {
        PTE->PSOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
        PTE->PCOR = (1 << row);

        for (int col = 0; col < 4; col++) {
            if (!(PTC->PDIR & (1 << col))) {
                while (!(PTC->PDIR & (1 << col)));
                return keys[row][col];
            }
        }
    }
    return 0;
}

/* -------------------------------------------------
 * UART FUNCTIONS
 * ------------------------------------------------- */

void UART0_init(void)
{
    SIM->SCGC4 |= 0x0400;
    SIM->SOPT2 |= 0x04000000;

    UART0->C2 = 0x00;
    UART0->BDH = 0x00;
    UART0->BDL = 0x17; // 57600 baudios
    UART0->C4 = 0x0F;
    UART0->C1 = 0x00;

    /* 0x2C = TE (0x08) | RE (0x04) | RIE (0x20) */
    UART0->C2 = 0x2C;

    SIM->SCGC5 |= 0x0200;
    PORTA->PCR[2] = 0x0200;
    PORTA->PCR[1] = 0x0200;

    NVIC_SetPriority(UART0_IRQn, 2);
    NVIC_EnableIRQ(UART0_IRQn);
}

void UART0_putc(char c)
{
    while (!(UART0->S1 & 0x80));
    UART0->D = c;
}

char UART0_getc(void)
{
    /* Wait until an interrupt receives a byte */
    while (rx_flag == 0);

    char temp = rx_flag;
    rx_flag = 0; // Clear flag after reading
    return temp;
}

void UART0_puts(const char *str)
{
    while (*str != '\0') {
        UART0_putc(*str);
        str++;
    }
}

/* -------------------------------------------------
 * MENÚ Y SUBMENÚS
 * ------------------------------------------------- */

void showMenu(void)
{
    UART0_puts("\r\n================================\r\n");
    UART0_puts("      KL25Z UART SYSTEM\r\n");
    UART0_puts("================================\r\n");
    UART0_puts("Commands:\r\n");
    UART0_puts("L - LED control\r\n");
    UART0_puts("A - Read ADC\r\n");
    UART0_puts("K - Read keypad\r\n");
    UART0_puts("B - Button status\r\n");
    UART0_puts("================================\r\n");
    UART0_puts("Please select an option: ");
}

void optionLED(void)
{
    char sub_opt;
    UART0_puts("\r\nLED control\r\n1 - Red\r\n2 - Green\r\n3 - Blue\r\n0 - All OFF\r\n");

    while (1) {
        sub_opt = UART0_getc();

        if (sub_opt == 'Q' || sub_opt == 'q') break;

        PTB->PSOR = (1 << 18) | (1 << 19);
        PTD->PSOR = (1 << 1);

        switch (sub_opt) {
            case '1':
                PTB->PCOR = (1 << 18);
                UART0_puts("Red LED ON\r\n");
                break;
            case '2':
                PTB->PCOR = (1 << 19);
                UART0_puts("Green LED ON\r\n");
                break;
            case '3':
                PTD->PCOR = (1 << 1);
                UART0_puts("Blue LED ON\r\n");
                break;
            case '0':
                UART0_puts("All LEDs OFF\r\n");
                break;
            default:
                UART0_puts("Invalid command.\r\nPlease select an option:\r\n");
                break;
        }
    }
}

void optionADC(void)
{
    char buffer[64];
    UART0_puts("\r\n--- ADC Monitoring (Press 'Q' to quit) ---\r\n");

    while (1) {
        if (UART0_has_data()) {
            char c = UART0_getc();
            if (c == 'Q' || c == 'q') break;
        }

        uint16_t adc_val = ADC0_read();
        uint32_t mv = (adc_val * 3300) / 4095;
        uint32_t v_int = mv / 1000;
        uint32_t v_dec = (mv % 1000) / 10;

        sprintf(buffer, "ADC Value: %u\r\nVoltage: %lu.%02lu V\r\n\r\n", adc_val, v_int, v_dec);
        UART0_puts(buffer);

        delay_ms(500);
    }
}

void optionKeypad(void)
{
    char key;
    char buffer[32];
    UART0_puts("\r\n--- Keypad Scanner (Press 'Q' to quit) ---\r\n");

    while (1) {
        if (UART0_has_data()) {
            char c = UART0_getc();
            if (c == 'Q' || c == 'q') break;
        }

        key = keypad_scan();
        if (key != 0) {
            sprintf(buffer, "Press a key:\r\nKey pressed: %c\r\n\r\n", key);
            UART0_puts(buffer);
        }
    }
}

void optionButtons(void)
{
    uint8_t btn1_prev = (PTC->PDIR & (1 << 5)) ? 1 : 0;
    uint8_t btn2_prev = (PTC->PDIR & (1 << 12)) ? 1 : 0;
    UART0_puts("\r\n--- Button Monitoring (Press 'Q' to quit) ---\r\n");

    while (1) {
        if (UART0_has_data()) {
            char c = UART0_getc();
            if (c == 'Q' || c == 'q') break;
        }

        uint8_t btn1_curr = (PTC->PDIR & (1 << 5)) ? 1 : 0;
        uint8_t btn2_curr = (PTC->PDIR & (1 << 12)) ? 1 : 0;

        if (btn1_curr != btn1_prev) {
            btn1_prev = btn1_curr;
            UART0_puts(btn1_curr ? "Button 1: RELEASED\r\n" : "Button 1: PRESSED\r\n");
        }

        if (btn2_curr != btn2_prev) {
            btn2_prev = btn2_curr;
            UART0_puts(btn2_curr ? "Button 2: RELEASED\r\n" : "Button 2: PRESSED\r\n");
        }
    }
}

/* -------------------------------------------------
 * PROCESAMIENTO DE COMANDOS
 * ------------------------------------------------- */

void processCommand(char option)
{
    switch (option) {
        case 'L': case 'l':
            optionLED();
            showMenu();
            break;
        case 'A': case 'a':
            optionADC();
            showMenu();
            break;
        case 'K': case 'k':
            optionKeypad();
            showMenu();
            break;
        case 'B': case 'b':
            optionButtons();
            showMenu();
            break;
        case '\r': case '\n':
            break;
        default:
            UART0_puts("\r\nInvalid command.\r\nPlease select an option: ");
            break;
    }
}

/* -------------------------------------------------
 * MAIN
 * ------------------------------------------------- */

int main(void)
{
    __disable_irq();

    UART0_init();
    peripherals_init();

    __enable_irq();

    showMenu();

    /* Main infinite loop checks if the interrupt received a character */
    while (1) {
        if (UART0_has_data()) {
            char option = UART0_getc();
            processCommand(option);
        }
    }
}

/* -------------------------------------------------
 * INTERRUPT SERVICE ROUTINE
 * ------------------------------------------------- */

void UART0_IRQHandler(void)
{
    /* Check if Receive Data Register Full flag is set */
    if (UART0->S1 & UART0_S1_RDRF_MASK)
    {
        rx_flag = UART0->D; /* Store character; reading UART0->D clears the hardware flag */
    }
}
