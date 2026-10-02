#include "MKL25Z4.h"
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"

// ==========================================
// MAPEO DEL LCD (PUERTO C)
// ==========================================
#define RS (1 << 0) // PTC0
#define EN (1 << 1) // PTC1
#define D4 (1 << 2) // PTC2
#define D5 (1 << 3) // PTC3
#define D6 (1 << 4) // PTC4
#define D7 (1 << 5) // PTC5

// ==========================================
// CONFIGURACIÓN DEL DS3231 (RTC)
// ==========================================
#define DS3231_ADDR 0x68

void delay_ms(volatile uint32_t t) {
    for(; t > 0; t--) {
        for(volatile int i = 0; i < 7000; i++);
    }
}

// ==========================================
// FUNCIONES DEL LCD
// ==========================================
void LCD_Pulse(void) {
    GPIOC->PSOR = EN;
    for(volatile int i = 0; i < 100; i++);
    GPIOC->PCOR = EN;
    for(volatile int i = 0; i < 100; i++);
}

void LCD_Write4Bits(uint8_t nibble) {
    GPIOC->PCOR = D4 | D5 | D6 | D7;
    if (nibble & 0x01) GPIOC->PSOR = D4;
    if (nibble & 0x02) GPIOC->PSOR = D5;
    if (nibble & 0x04) GPIOC->PSOR = D6;
    if (nibble & 0x08) GPIOC->PSOR = D7;
    LCD_Pulse();
}

void LCD_Send(uint8_t value, uint8_t mode) {
    if (mode) GPIOC->PSOR = RS;
    else GPIOC->PCOR = RS;

    LCD_Write4Bits(value >> 4);
    LCD_Write4Bits(value & 0x0F);
    delay_ms(1);
}

void LCD_Init(void) {
    delay_ms(50);
    GPIOC->PCOR = RS | EN;

    LCD_Write4Bits(0x03);
    delay_ms(5);
    LCD_Write4Bits(0x03);
    delay_ms(1);
    LCD_Write4Bits(0x03);
    delay_ms(1);
    LCD_Write4Bits(0x02);
    delay_ms(1);

    LCD_Send(0x28, 0);
    LCD_Send(0x0C, 0);
    LCD_Send(0x01, 0);
    delay_ms(5);
    LCD_Send(0x06, 0);
}

void LCD_Print(const char *str) {
    while (*str) LCD_Send((uint8_t)*str++, 1);
}

void LCD_Command(uint8_t cmd) {
    LCD_Send(cmd, 0);
}

// ==========================================
// FUNCIONES I2C1
// ==========================================
void I2C1_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
    I2C1->C1 = 0;

    PORTE->PCR[0] = PORT_PCR_MUX(6) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTE->PCR[1] = PORT_PCR_MUX(6) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    I2C1->F = 0x14;
    I2C1->C1 = I2C_C1_IICEN_MASK;
}

void I2C_Wait(void) {
    uint32_t timeout = 0;
    while(((I2C1->S & I2C_S_IICIF_MASK) == 0) && (timeout < 50000)) {
        timeout++;
    }
    I2C1->S |= I2C_S_IICIF_MASK;
}

void I2C_Start(void) {
    I2C1->C1 |= I2C_C1_TX_MASK;
    I2C1->C1 |= I2C_C1_MST_MASK;
}

void I2C_Stop(void) {
    I2C1->C1 &= ~I2C_C1_MST_MASK;
    I2C1->C1 &= ~I2C_C1_TX_MASK;
}

void I2C_WriteByte(uint8_t data) {
    I2C1->D = data;
    I2C_Wait();
}

// ==========================================
// CONTROL DS3231 Y BCD
// ==========================================
uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

// NUEVA FUNCIÓN: Lectura en ráfaga (Evita desincronización de registros)
void DS3231_ReadTime(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *day, uint8_t *month, uint8_t *year) {
    uint8_t raw_sec, raw_min, raw_hour, raw_dow, raw_day, raw_month, raw_year;

    // 1. Apuntar al registro 0x00 (Segundos)
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x00);
    I2C_Stop();

    // Pequeña pausa para que el bus se estabilice
    for(volatile int i = 0; i < 200; i++);

    // 2. Leer los 7 registros de corrido
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 1); // Modo lectura

    I2C1->C1 &= ~I2C_C1_TX_MASK;   // Configurar para recibir
    I2C1->C1 &= ~I2C_C1_TXAK_MASK; // Mandar ACK después de cada lectura

    uint8_t dummy = I2C1->D; // Lectura de inicio
    I2C_Wait();

    raw_sec = I2C1->D;
    I2C_Wait();

    raw_min = I2C1->D;
    I2C_Wait();

    raw_hour = I2C1->D;
    I2C_Wait();

    raw_dow = I2C1->D; // Día de la semana (lo leemos pero lo ignoramos)
    I2C_Wait();

    raw_day = I2C1->D;
    I2C_Wait();

    // ¡OJO AQUÍ! Antes de leer el penúltimo dato, preparamos el NACK
    I2C1->C1 |= I2C_C1_TXAK_MASK;

    raw_month = I2C1->D;
    I2C_Wait();

    // Mandamos el STOP ANTES de leer el último dato
    I2C_Stop();
    raw_year = I2C1->D;

    // 3. Aplicar conversiones BCD y guardar en los punteros
    *sec   = bcdToDec(raw_sec);
    *min   = bcdToDec(raw_min);
    *hour  = bcdToDec(raw_hour & 0x3F); // Máscara para reloj 24h
    *day   = bcdToDec(raw_day);
    *month = bcdToDec(raw_month & 0x1F); // Máscara de mes
    *year  = bcdToDec(raw_year);
}

void DS3231_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t day, uint8_t month, uint8_t year) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x00);

    I2C_WriteByte(decToBcd(sec));
    I2C_WriteByte(decToBcd(min));
    I2C_WriteByte(decToBcd(hour));
    I2C_WriteByte(1);
    I2C_WriteByte(decToBcd(day));
    I2C_WriteByte(decToBcd(month));
    I2C_WriteByte(decToBcd(year));

    I2C_Stop();
    delay_ms(10);
}

// ==========================================
// PROGRAMA PRINCIPAL
// ==========================================
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    // Iniciar LCD
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    for(int i = 0; i <= 5; i++) {
        PORTC->PCR[i] = PORT_PCR_MUX(1);
    }
    GPIOC->PDDR |= (RS | EN | D4 | D5 | D6 | D7);

    LCD_Init();

    LCD_Command(0x80);
    LCD_Print("Iniciando...");
    delay_ms(500);

    I2C1_Init();

    // -------------------------------------------------------------
    // SI EL RELOJ TIENE HORA BASURA O 00:00:00,
    // DESCOMENTA ESTO UNA VEZ, SUBE EL CÓDIGO, LUEGO VUÉLVELO A
    // COMENTAR Y SUBE EL CÓDIGO OTRA VEZ.
    DS3231_SetTime(23, 59, 50, 9, 9, 26);
    // -------------------------------------------------------------

    char buffer[17];
    uint8_t hour, min, sec, day, month, year;

    while(1) {
        // Leemos todos los valores en un solo movimiento estable
        DS3231_ReadTime(&hour, &min, &sec, &day, &month, &year);

        // Imprimir Fecha
        LCD_Command(0x80);
        sprintf(buffer, "Date: %02d/%02d/%02d", day, month, year);
        LCD_Print(buffer);

        // Imprimir Hora
        LCD_Command(0xC0);
        sprintf(buffer, "Time: %02d:%02d:%02d", hour, min, sec);
        LCD_Print(buffer);

        delay_ms(500);
    }
}
