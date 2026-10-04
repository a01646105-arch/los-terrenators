// Implementacion del extra challenge 3 de la rubrica

#include "MKL25Z4.h"
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"

// ==========================================
// PINES Y DEFINICIONES
// ==========================================
#define RS (1 << 0) // PTC0
#define EN (1 << 1) // PTC1
#define D4 (1 << 2) // PTC2
#define D5 (1 << 3) // PTC3
#define D6 (1 << 4) // PTC4
#define D7 (1 << 5) // PTC5

#define DS3231_ADDR 0x68
#define BME280_ADDR 0x76 // SDO en GND

volatile uint8_t alarm_triggered = 0;
uint8_t alarm_enabled = 0;   // Estado de la alarma (ON/OFF)
uint8_t alarm_hour = 0;      // Hora de alarma guardada
uint8_t alarm_min  = 0;      // Minuto de alarma guardado

// Límite de temperatura en grados enteros (Por defecto 30 °C)
uint8_t temp_limit_int = 30;

void delay_ms(volatile uint32_t t) {
    for(; t > 0; t--) {
        for(volatile int i = 0; i < 7000; i++);
    }
}

// ==========================================
// FUNCIONES DEL LCD
// ==========================================
void LCD_Pulse(void) {
    GPIOC->PSOR = EN; for(volatile int i=0; i<100; i++);
    GPIOC->PCOR = EN; for(volatile int i=0; i<100; i++);
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
    if (mode) GPIOC->PSOR = RS; else GPIOC->PCOR = RS;
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
    LCD_Send(0x28, 0); LCD_Send(0x0C, 0); LCD_Send(0x01, 0);
    delay_ms(5); LCD_Send(0x06, 0);
}

void LCD_Print(const char *str) {
    while (*str) LCD_Send((uint8_t)*str++, 1);
}

void LCD_Command(uint8_t cmd) {
    LCD_Send(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) delay_ms(2); // Clear/Home requieren >1.5 ms
}

// ==========================================
// NÚCLEO I2C
// ==========================================
void I2C1_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
    PORTE->PCR[0] = PORT_PCR_MUX(6) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTE->PCR[1] = PORT_PCR_MUX(6) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    I2C1->F = 0x14;
    I2C1->C1 = I2C_C1_IICEN_MASK;
}

void I2C_Wait(void) {
    uint32_t timeout = 0;
    while(((I2C1->S & I2C_S_IICIF_MASK) == 0) && (timeout < 50000)) timeout++;
    I2C1->S |= I2C_S_IICIF_MASK;
}

static void I2C_WaitIdle(void) {
    uint32_t timeout = 0;
    while((I2C1->S & I2C_S_BUSY_MASK) && (timeout < 50000)) timeout++;
}

void I2C_Start(void) {
    I2C1->S |= I2C_S_IICIF_MASK;
    I2C1->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK;
}

void I2C_Stop(void) {
    I2C1->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    I2C_WaitIdle();
}

void I2C_WriteByte(uint8_t data) { I2C1->D = data; I2C_Wait(); }

void I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 0);
    I2C_WriteByte(reg);
    I2C_WriteByte(val);
    I2C_Stop();
}

uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg) {
    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 0);
    I2C_WriteByte(reg);
    I2C_Stop();

    for(volatile int i = 0; i < 200; i++);

    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 1);
    I2C1->C1 &= ~I2C_C1_TX_MASK;
    I2C1->C1 |= I2C_C1_TXAK_MASK;

    uint8_t dummy = I2C1->D; (void)dummy;
    I2C_Wait();

    I2C_Stop();
    return I2C1->D;
}

uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

// ==========================================
// FUNCIONES DS3231 (RTC)
// ==========================================
void DS3231_ReadDateTime(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *day, uint8_t *month, uint8_t *year) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x00);
    I2C_Stop();

    for(volatile int i = 0; i < 200; i++);

    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 1);
    I2C1->C1 &= ~I2C_C1_TX_MASK;
    I2C1->C1 &= ~I2C_C1_TXAK_MASK;

    uint8_t dummy = I2C1->D; (void)dummy; I2C_Wait();
    *sec   = bcdToDec(I2C1->D & 0x7F); I2C_Wait();
    *min   = bcdToDec(I2C1->D & 0x7F); I2C_Wait();
    *hour  = bcdToDec(I2C1->D & 0x3F); I2C_Wait();
    dummy  = I2C1->D; I2C_Wait();
    *day   = bcdToDec(I2C1->D & 0x3F); I2C_Wait();

    I2C1->C1 |= I2C_C1_TXAK_MASK;
    *month = bcdToDec(I2C1->D & 0x1F); I2C_Wait();

    I2C_Stop();
    *year  = bcdToDec(I2C1->D);
}

void DS3231_SetTime(uint8_t hour, uint8_t min) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x00);
    I2C_WriteByte(0x00);
    I2C_WriteByte(decToBcd(min));
    I2C_WriteByte(decToBcd(hour));
    I2C_Stop();
    delay_ms(10);
}

void DS3231_SetDate(uint8_t day, uint8_t month, uint8_t year) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x04);
    I2C_WriteByte(decToBcd(day));
    I2C_WriteByte(decToBcd(month));
    I2C_WriteByte(decToBcd(year));
    I2C_Stop();
    delay_ms(10);
}

void DS3231_SetAlarm(uint8_t hour, uint8_t min, uint8_t enabled) {
    I2C_WriteReg(DS3231_ADDR, 0x0E, 0x04);
    delay_ms(10);

    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x07);
    I2C_WriteByte(0x00);
    I2C_WriteByte(decToBcd(min));
    I2C_WriteByte(decToBcd(hour));
    I2C_WriteByte(0x80);
    I2C_Stop();
    delay_ms(10);

    I2C_WriteReg(DS3231_ADDR, 0x0F, 0x00);
    delay_ms(10);

    if (enabled) I2C_WriteReg(DS3231_ADDR, 0x0E, 0x05);
    delay_ms(10);

    PORTD->ISFR = (1 << 4);
    alarm_triggered = 0;
}

// ==========================================
// FUNCIONES BME280 (TEMPERATURA)
// ==========================================
uint16_t bme_dig_T1;
int16_t bme_dig_T2, bme_dig_T3;

void BME280_Init(void) {
    delay_ms(50);
    bme_dig_T1 = (uint16_t)((I2C_ReadReg(BME280_ADDR, 0x89) << 8) | I2C_ReadReg(BME280_ADDR, 0x88));
    bme_dig_T2 = (int16_t)((I2C_ReadReg(BME280_ADDR, 0x8B) << 8) | I2C_ReadReg(BME280_ADDR, 0x8A));
    bme_dig_T3 = (int16_t)((I2C_ReadReg(BME280_ADDR, 0x8D) << 8) | I2C_ReadReg(BME280_ADDR, 0x8C));

    I2C_WriteReg(BME280_ADDR, 0xF2, 0x01);
    I2C_WriteReg(BME280_ADDR, 0xF5, 0x00);
    I2C_WriteReg(BME280_ADDR, 0xF4, 0x27);
    delay_ms(50);
}

uint8_t BME280_ReadTemp(int16_t *temp_x10) {
    uint32_t msb  = I2C_ReadReg(BME280_ADDR, 0xFA);
    uint32_t lsb  = I2C_ReadReg(BME280_ADDR, 0xFB);
    uint32_t xlsb = I2C_ReadReg(BME280_ADDR, 0xFC);

    int32_t adc_T = (int32_t)((msb << 12) | (lsb << 4) | (xlsb >> 4));
    if (adc_T == 0x80000 || adc_T == 0) return 0;

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_dig_T1 << 1))) * ((int32_t)bme_dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_dig_T1)) * ((adc_T >> 4) - ((int32_t)bme_dig_T1))) >> 12) * ((int32_t)bme_dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8;

    *temp_x10 = (int16_t)(T / 10);
    return 1;
}

// ==========================================
// FUNCIONES SPI0 (MAX7219 DISPLAY)
// ==========================================
void SPI0_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
    PORTD->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOD->PDDR |= (1 << 0); GPIOD->PSOR = (1 << 0);
    PORTD->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;
    PORTD->PCR[2] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;
    SPI0->C1 = SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK;
    SPI0->BR = SPI_BR_SPPR(3) | SPI_BR_SPR(5);
}

void MAX7219_Write(uint8_t addr, uint8_t data) {
    GPIOD->PCOR = (1 << 0);
    for(volatile int i=0; i<20; i++);
    while(!(SPI0->S & SPI_S_SPTEF_MASK)); SPI0->D = addr;
    while(!(SPI0->S & SPI_S_SPRF_MASK)); (void)SPI0->D;
    while(!(SPI0->S & SPI_S_SPTEF_MASK)); SPI0->D = data;
    while(!(SPI0->S & SPI_S_SPRF_MASK)); (void)SPI0->D;
    for(volatile int i=0; i<50; i++);
    GPIOD->PSOR = (1 << 0);
    for(volatile int i=0; i<50; i++);
}

void MAX7219_Init(void) {
    MAX7219_Write(0x0F, 0x00); delay_ms(10);
    MAX7219_Write(0x09, 0x0F);
    MAX7219_Write(0x0B, 0x03);
    MAX7219_Write(0x0A, 0x04);
    MAX7219_Write(0x0C, 0x01);
    for(int i = 1; i <= 4; i++) MAX7219_Write(i, 0x0F);
    delay_ms(10);
}

void MAX7219_DisplayTime(uint8_t hour, uint8_t min) {
    MAX7219_Write(1, (hour / 10) % 10);
    MAX7219_Write(2, (hour % 10) | 0x80);
    MAX7219_Write(3, (min / 10) % 10);
    MAX7219_Write(4, min % 10);
}

// ==========================================
// TECLADO MATRICIAL
// ==========================================
const char keymap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

void Keypad_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    for (int i = 0; i <= 3; i++) {
        PORTB->PCR[i] = PORT_PCR_MUX(1);
        GPIOB->PDDR |= (1 << i); GPIOB->PSOR = (1 << i);
    }
    for (int i = 8; i <= 11; i++) {
        PORTB->PCR[i] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
        GPIOB->PDDR &= ~(1 << i);
    }
}

char Keypad_Scan(void) {
    for (int row = 0; row < 4; row++) {
        GPIOB->PCOR = (1 << row);
        delay_ms(2);
        for (int col = 0; col < 4; col++) {
            if (!(GPIOB->PDIR & (1 << (col + 8)))) {
                delay_ms(10);
                while (!(GPIOB->PDIR & (1 << (col + 8))));
                delay_ms(10);
                GPIOB->PSOR = (1 << row);
                return keymap[row][col];
            }
        }
        GPIOB->PSOR = (1 << row);
    }
    return 0;
}

// ==========================================
// HARDWARE DE ALARMA
// ==========================================
void GPIO_Alarm_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK;
    PORTE->PCR[2] = PORT_PCR_MUX(1); // Buzzer
    PORTE->PCR[3] = PORT_PCR_MUX(1); // LED
    GPIOE->PDDR |= (1 << 2) | (1 << 3);
    GPIOE->PCOR = (1 << 2) | (1 << 3);

    PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOD->PDDR &= ~(1 << 4);
    PORTD->ISFR = (1 << 4);
    NVIC_ClearPendingIRQ(PORTD_IRQn);
    NVIC_EnableIRQ(PORTD_IRQn);
}

void PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1 << 4)) {
        PORTD->ISFR = (1 << 4);
        if (alarm_enabled) alarm_triggered = 1;
    }
}

// ==========================================
// MÁQUINA DE ESTADOS (ESTACIÓN DE MONITOREO)
// ==========================================
typedef enum {
    MODE_NORMAL,
    MODE_CONFIG_TIME,
    MODE_CONFIG_DATE,
    MODE_CONFIG_ALARM,
    MODE_CONFIG_ALARM_EN,
    MODE_CONFIG_TEMP,       // NUEVO: Estado para configurar límite de temperatura
    MODE_ALARM_RINGING
} SystemState;

SystemState current_mode = MODE_NORMAL;
char input_time[5] = "____";
char input_date[7] = "______";
char input_temp[3] = "__";  // NUEVO: Buffer para límite de temperatura
uint8_t input_index = 0;

static uint8_t two_digits(const char *s) { return (uint8_t)((s[0]-'0')*10 + (s[1]-'0')); }

static uint8_t days_in_month(uint8_t m, uint8_t y) {
    static const uint8_t d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (m == 2 && (y % 4) == 0) return 29;
    return d[m - 1];
}

static void Show_Message(const char *l1, const char *l2) {
    LCD_Command(0x01);
    LCD_Command(0x80); LCD_Print(l1);
    LCD_Command(0xC0); LCD_Print(l2);
    delay_ms(1000);
    LCD_Command(0x01);
}

static void Go_Normal(uint8_t *last_sec) {
    current_mode = MODE_NORMAL;
    *last_sec = 255;
    LCD_Command(0x01);
}

void Draw_Config_Screen(void) {
    char b[24];
    LCD_Command(0x01);
    LCD_Command(0x80);

    switch (current_mode) {
    case MODE_CONFIG_TIME:
        snprintf(b, sizeof(b), "TIME  %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
        LCD_Print(b);
        LCD_Command(0xC0); LCD_Print("D=OK B=Esc *=Del");
        break;
    case MODE_CONFIG_DATE:
        snprintf(b, sizeof(b), "DATE  %c%c/%c%c/%c%c", input_date[0], input_date[1], input_date[2], input_date[3], input_date[4], input_date[5]);
        LCD_Print(b);
        LCD_Command(0xC0); LCD_Print("D=OK B=Esc *=Del");
        break;
    case MODE_CONFIG_ALARM:
        snprintf(b, sizeof(b), "ALARM %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
        LCD_Print(b);
        LCD_Command(0xC0); LCD_Print("D=OK B=Esc *=Del");
        break;
    case MODE_CONFIG_ALARM_EN:
        snprintf(b, sizeof(b), "ALARM %02d:%02d", alarm_hour, alarm_min);
        LCD_Print(b);
        LCD_Command(0xC0); LCD_Print("1=ON 0=OFF B=Esc");
        break;
    case MODE_CONFIG_TEMP: // NUEVO: Pantalla de límite de temperatura
        snprintf(b, sizeof(b), "TEMP LIMIT: %c%cC", input_temp[0], input_temp[1]);
        LCD_Print(b);
        LCD_Command(0xC0); LCD_Print("D=OK B=Esc *=Del");
        break;
    default:
        break;
    }
}

// ==========================================
// PROGRAMA PRINCIPAL
// ==========================================
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    delay_ms(100);

    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    for(int i = 0; i <= 5; i++) PORTC->PCR[i] = PORT_PCR_MUX(1);
    GPIOC->PDDR |= (RS | EN | D4 | D5 | D6 | D7);

    LCD_Init();
    I2C1_Init();
    SPI0_Init();
    MAX7219_Init();
    Keypad_Init();
    GPIO_Alarm_Init();
    BME280_Init();

    LCD_Command(0x80);
    LCD_Print("Monitoring Stn");
    LCD_Command(0xC0);
    LCD_Print("A:Tm B:Dt *:Lim ");   // Ayuda actualizada
    delay_ms(1500);
    LCD_Command(0x01);

    char buffer[24];
    char tbuf[12];
    uint8_t hour = 0, min = 0, sec = 0, day = 1, month = 1, year = 0;
    int16_t temp_x10 = 0;
    uint8_t last_sec = 255;

    DS3231_SetAlarm(0, 0, 0);

    while(1) {
        char key = Keypad_Scan();

        // --- Detección de alarma (interrupción del DS3231) ---
        if (alarm_triggered && current_mode != MODE_ALARM_RINGING) {
            current_mode = MODE_ALARM_RINGING;
            LCD_Command(0x01);
            LCD_Command(0x80); LCD_Print("*** ALARM ***");
            LCD_Command(0xC0); LCD_Print("Press # to stop");
            GPIOE->PSOR = (1 << 2) | (1 << 3); // Buzzer + LED ON
        }

        switch(current_mode) {
            // ----------------------------------------------------------------
            case MODE_ALARM_RINGING:
                if (key == '#') {
                    alarm_triggered = 0;
                    I2C_WriteReg(DS3231_ADDR, 0x0F, 0x00);
                    PORTD->ISFR = (1 << 4);
                    GPIOE->PCOR = (1 << 2) | (1 << 3);
                    Go_Normal(&last_sec);
                }
                break;

            // ----------------------------------------------------------------
            case MODE_NORMAL:
                DS3231_ReadDateTime(&hour, &min, &sec, &day, &month, &year);

                if (sec != last_sec) {
                    last_sec = sec;
                    BME280_ReadTemp(&temp_x10);

                    int16_t t = temp_x10;
                    snprintf(tbuf, sizeof(tbuf), "%s%d.%d", (t < 0) ? "-" : "", abs(t) / 10, abs(t) % 10);

                    // NUEVA LÓGICA: Evaluación del límite de temperatura
                    if (t >= (temp_limit_int * 10)) {
                        // El límite fue excedido
                        LCD_Command(0x80);
                        snprintf(buffer, sizeof(buffer), "Temp: %s C      ", tbuf);
                        LCD_Print(buffer);

                        LCD_Command(0xC0);
                        LCD_Print("WARNING         ");

                        GPIOE->PSOR = (1 << 2) | (1 << 3); // Enciende Buzzer y LED
                    } else {
                        // Temperatura dentro de los límites normales
                        LCD_Command(0x80);
                        snprintf(buffer, sizeof(buffer), "%02d/%02d/%02d %02d:%02d  ", day, month, year, hour, min);
                        LCD_Print(buffer);

                        LCD_Command(0xC0);
                        snprintf(buffer, sizeof(buffer), "T:%sC Alm:%-3s     ", tbuf, alarm_enabled ? "ON" : "OFF");
                        buffer[16] = '\0';
                        LCD_Print(buffer);

                        // Si la alarma del reloj no está sonando, apaga el Buzzer y LED
                        if (!alarm_triggered && current_mode != MODE_ALARM_RINGING) {
                            GPIOE->PCOR = (1 << 2) | (1 << 3);
                        }
                    }

                    // Display de 7 segmentos HH:MM
                    MAX7219_DisplayTime(hour, min);
                }

                // Opciones del menú principal
                if (key == 'A') {
                    current_mode = MODE_CONFIG_TIME; input_index = 0;
                    sprintf(input_time, "____"); Draw_Config_Screen();
                } else if (key == 'B') {
                    current_mode = MODE_CONFIG_DATE; input_index = 0;
                    sprintf(input_date, "______"); Draw_Config_Screen();
                } else if (key == 'C') {
                    current_mode = MODE_CONFIG_ALARM; input_index = 0;
                    sprintf(input_time, "____"); Draw_Config_Screen();
                } else if (key == 'D') {
                    alarm_enabled = !alarm_enabled;
                    DS3231_SetAlarm(alarm_hour, alarm_min, alarm_enabled);
                    last_sec = 255;
                } else if (key == '*') {
                    // NUEVO: Ingresar al menú de configuración del Límite de Temperatura
                    current_mode = MODE_CONFIG_TEMP; input_index = 0;
                    sprintf(input_temp, "__"); Draw_Config_Screen();
                }
                break;

            // ----------------------------------------------------------------
            case MODE_CONFIG_TIME:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_time[input_index++] = key; Draw_Config_Screen();
                } else if (key == '*' && input_index > 0) {
                    input_time[--input_index] = '_'; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 4) {
                    uint8_t new_h = two_digits(input_time);
                    uint8_t new_m = two_digits(input_time + 2);
                    if (new_h > 23 || new_m > 59) {
                        Show_Message("INVALID TIME", "Try again");
                        input_index = 0; sprintf(input_time, "____"); Draw_Config_Screen();
                    } else {
                        DS3231_SetTime(new_h, new_m);
                        Show_Message("TIME SAVED!", "");
                        Go_Normal(&last_sec);
                    }
                } else if (key == 'B') {
                    Go_Normal(&last_sec);
                }
                break;

            // ----------------------------------------------------------------
            case MODE_CONFIG_DATE:
                if (key >= '0' && key <= '9' && input_index < 6) {
                    input_date[input_index++] = key; Draw_Config_Screen();
                } else if (key == '*' && input_index > 0) {
                    input_date[--input_index] = '_'; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 6) {
                    uint8_t new_D = two_digits(input_date);
                    uint8_t new_M = two_digits(input_date + 2);
                    uint8_t new_Y = two_digits(input_date + 4);
                    if (new_M < 1 || new_M > 12 || new_D < 1 || new_D > days_in_month(new_M, new_Y)) {
                        Show_Message("INVALID DATE", "Try again");
                        input_index = 0; sprintf(input_date, "______"); Draw_Config_Screen();
                    } else {
                        DS3231_SetDate(new_D, new_M, new_Y);
                        Show_Message("DATE SAVED!", "");
                        Go_Normal(&last_sec);
                    }
                } else if (key == 'B') {
                    Go_Normal(&last_sec);
                }
                break;

            // ----------------------------------------------------------------
            case MODE_CONFIG_ALARM:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_time[input_index++] = key; Draw_Config_Screen();
                } else if (key == '*' && input_index > 0) {
                    input_time[--input_index] = '_'; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 4) {
                    uint8_t new_h = two_digits(input_time);
                    uint8_t new_m = two_digits(input_time + 2);
                    if (new_h > 23 || new_m > 59) {
                        Show_Message("INVALID TIME", "Try again");
                        input_index = 0; sprintf(input_time, "____"); Draw_Config_Screen();
                    } else {
                        alarm_hour = new_h; alarm_min = new_m;
                        current_mode = MODE_CONFIG_ALARM_EN;
                        Draw_Config_Screen();
                    }
                } else if (key == 'B') {
                    Go_Normal(&last_sec);
                }
                break;

            // ----------------------------------------------------------------
            case MODE_CONFIG_ALARM_EN:
                if (key == '1' || key == '0') {
                    alarm_enabled = (key == '1') ? 1 : 0;
                    DS3231_SetAlarm(alarm_hour, alarm_min, alarm_enabled);
                    Show_Message("ALARM SAVED!", alarm_enabled ? "Alarm: ON" : "Alarm: OFF");
                    Go_Normal(&last_sec);
                } else if (key == 'B') {
                    Go_Normal(&last_sec);
                }
                break;

            // ----------------------------------------------------------------
            case MODE_CONFIG_TEMP:  // NUEVA LÓGICA: Configuración del límite
                if (key >= '0' && key <= '9' && input_index < 2) {
                    input_temp[input_index++] = key; Draw_Config_Screen();
                } else if (key == '*' && input_index > 0) {
                    input_temp[--input_index] = '_'; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 2) {
                    temp_limit_int = two_digits(input_temp);
                    Show_Message("LIMIT SAVED!", "");
                    Go_Normal(&last_sec);
                } else if (key == 'B') {
                    Go_Normal(&last_sec);
                }
                break;
        }
        delay_ms(10);
    }
}
