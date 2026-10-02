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
// CONFIGURACIÓN DEL DS3231 (RTC I2C1)
// ==========================================
#define DS3231_ADDR 0x68

// Retardo simple por software (~milisegundos)
void delay_ms(volatile uint32_t t) {
    for(; t > 0; t--) {
        for(volatile int i = 0; i < 7000; i++);
    }
}

// ==========================================
// FUNCIONES DEL LCD (PUERTO C)
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

    LCD_Write4Bits(0x03); delay_ms(5);
    LCD_Write4Bits(0x03); delay_ms(1);
    LCD_Write4Bits(0x03); delay_ms(1);
    LCD_Write4Bits(0x02); delay_ms(1);

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
// FUNCIONES I2C1 (DS3231 - PUERTO E)
// ==========================================
void I2C1_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
    I2C1->C1 = 0;

    PORTE->PCR[0] = PORT_PCR_MUX(6) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PTE0 SDA
    PORTE->PCR[1] = PORT_PCR_MUX(6) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PTE1 SCL

    I2C1->F = 0x14;
    I2C1->C1 = I2C_C1_IICEN_MASK;
}

void I2C_Wait(void) {
    uint32_t timeout = 0;
    while(((I2C1->S & I2C_S_IICIF_MASK) == 0) && (timeout < 50000)) timeout++;
    I2C1->S |= I2C_S_IICIF_MASK;
}

void I2C_Start(void) {
    I2C1->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK;
}

void I2C_Stop(void) {
    I2C1->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK);
}

void I2C_WriteByte(uint8_t data) {
    I2C1->D = data;
    I2C_Wait();
}

uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

void DS3231_ReadTime(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *day, uint8_t *month, uint8_t *year) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x00);
    I2C_Stop();

    for(volatile int i = 0; i < 200; i++);

    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 1);

    I2C1->C1 &= ~I2C_C1_TX_MASK;
    I2C1->C1 &= ~I2C_C1_TXAK_MASK;

    uint8_t dummy = I2C1->D;
    I2C_Wait();

    *sec   = bcdToDec(I2C1->D); I2C_Wait();
    *min   = bcdToDec(I2C1->D); I2C_Wait();
    *hour  = bcdToDec(I2C1->D & 0x3F); I2C_Wait();
    dummy  = I2C1->D; I2C_Wait(); // Día semana
    *day   = bcdToDec(I2C1->D); I2C_Wait();

    I2C1->C1 |= I2C_C1_TXAK_MASK;
    *month = bcdToDec(I2C1->D & 0x1F); I2C_Wait();

    I2C_Stop();
    *year  = bcdToDec(I2C1->D);
}

void DS3231_SetTime(uint8_t hour, uint8_t min, uint8_t sec, uint8_t day, uint8_t month, uint8_t year) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x00);

    I2C_WriteByte(decToBcd(sec));
    I2C_WriteByte(decToBcd(min));
    I2C_WriteByte(decToBcd(hour));
    I2C_WriteByte(1); // Día de la semana (arbitrario para el reloj)
    I2C_WriteByte(decToBcd(day));
    I2C_WriteByte(decToBcd(month));
    I2C_WriteByte(decToBcd(year));

    I2C_Stop();
    delay_ms(10);
}

// ==========================================
// FUNCIONES SPI0 (MAX7219 - PUERTO D)
// Versión Nativa: Cátodo Común (5461BS)
// ==========================================
void SPI0_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;

    // PTD0 como CS (GPIO)
    PORTD->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOD->PDDR |= (1 << 0);
    GPIOD->PSOR = (1 << 0); // CS a HIGH por defecto

    // PTD1 (SCK) y PTD2 (MOSI)
    PORTD->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;
    PORTD->PCR[2] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;

    SPI0->C1 = SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK;
    SPI0->BR = SPI_BR_SPPR(3) | SPI_BR_SPR(5);
}

void MAX7219_Write(uint8_t addr, uint8_t data) {
    GPIOD->PCOR = (1 << 0); // CS a LOW
    for(volatile int i = 0; i < 20; i++);

    while(!(SPI0->S & SPI_S_SPTEF_MASK));
    SPI0->D = addr;
    while(!(SPI0->S & SPI_S_SPRF_MASK));
    (void)SPI0->D;

    while(!(SPI0->S & SPI_S_SPTEF_MASK));
    SPI0->D = data;
    while(!(SPI0->S & SPI_S_SPRF_MASK));
    (void)SPI0->D;

    for(volatile int i = 0; i < 50; i++);
    GPIOD->PSOR = (1 << 0); // CS a HIGH
    for(volatile int i = 0; i < 50; i++);
}

void MAX7219_Init(void) {
    MAX7219_Write(0x0F, 0x00); // 1. Apagar Display Test
    delay_ms(10);
    MAX7219_Write(0x09, 0x0F); // 2. Decode Mode para dígitos 1,2,3,4 (Code B)
    MAX7219_Write(0x0B, 0x03); // 3. Scan Limit: Mostrar solo 4 dígitos (0 a 3)
    MAX7219_Write(0x0A, 0x04); // 4. Intensidad (Brillo)
    MAX7219_Write(0x0C, 0x01); // 5. Shutdown Mode: Normal Operation

    // 6. Limpiar los 4 dígitos (0x0F = apagar dígito en modo Decode)
    for(int i = 1; i <= 4; i++) {
        MAX7219_Write(i, 0x0F);
    }
    delay_ms(10);
}

void MAX7219_DisplayTime(uint8_t hour, uint8_t min) {
    uint8_t h_tens  = (hour / 10) % 10;
    uint8_t h_units = hour % 10;
    uint8_t m_tens  = (min / 10) % 10;
    uint8_t m_units = min % 10;

    MAX7219_Write(1, h_tens);
    MAX7219_Write(2, h_units | 0x80); // Sumamos 0x80 para encender el punto/dos puntos
    MAX7219_Write(3, m_tens);
    MAX7219_Write(4, m_units);
}

// ==========================================
// PROGRAMA PRINCIPAL
// ==========================================
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    delay_ms(100);

    // Habilitar LCD (Puerto C)
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    for(int i = 0; i <= 5; i++) PORTC->PCR[i] = PORT_PCR_MUX(1);
    GPIOC->PDDR |= (RS | EN | D4 | D5 | D6 | D7);

    // Inicialización de Periféricos
    LCD_Init();
    I2C1_Init();
    SPI0_Init();
    MAX7219_Init();

    LCD_Command(0x80);
    LCD_Print("Iniciando...");
    delay_ms(500);

    // -------------------------------------------------------------
    // DESCOMENTAR SOLO UNA VEZ PARA RECONFIGURAR LA HORA DEL RTC:
    // DS3231_SetTime(15, 30, 00, 30, 9, 26);
    // -------------------------------------------------------------

    char buffer[17];
    uint8_t hour, min, sec, day, month, year;

    while(1) {
        // 1. Obtener tiempo
        DS3231_ReadTime(&hour, &min, &sec, &day, &month, &year);

        // 2. LCD (Fecha y Hora)
        LCD_Command(0x80);
        sprintf(buffer, "Date: %02d/%02d/%02d", day, month, year);
        LCD_Print(buffer);

        LCD_Command(0xC0);
        sprintf(buffer, "Time: %02d:%02d:%02d", hour, min, sec);
        LCD_Print(buffer);

        // 3. Display 7 Segmentos (MAX7219)
        MAX7219_DisplayTime(hour, min);

        delay_ms(500);
    }
}
