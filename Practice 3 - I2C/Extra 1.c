// Implementacion de Challenge 2 en la rubrica de la practica

#include "MKL25Z4.h"
#include <stdio.h>
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
// Si tu sensor sigue dando ceros, cambia este 0x76 por 0x77
#define BME280_ADDR 0x76

volatile uint8_t alarm_triggered = 0;
uint8_t alarm_enabled = 1; // 1 = ON, 0 = OFF

// Variables Globales BME280 (Calibración)
int32_t t_fine;
uint16_t bme_dig_T1; int16_t bme_dig_T2, bme_dig_T3;
uint16_t bme_dig_P1; int16_t bme_dig_P2, bme_dig_P3, bme_dig_P4, bme_dig_P5, bme_dig_P6, bme_dig_P7, bme_dig_P8, bme_dig_P9;
uint8_t  bme_dig_H1, bme_dig_H3;
int16_t  bme_dig_H2, bme_dig_H4, bme_dig_H5;
int8_t   bme_dig_H6;

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

void LCD_Command(uint8_t cmd) { LCD_Send(cmd, 0); }

// ==========================================
// NÚCLEO I2C (Con Timeout optimizado)
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
    while(((I2C1->S & I2C_S_IICIF_MASK) == 0) && (timeout < 1000000)) timeout++;
    I2C1->S = I2C_S_IICIF_MASK;
}

void I2C_Start(void) { I2C1->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK; }
void I2C_Stop(void)  { I2C1->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK); }
void I2C_WriteByte(uint8_t data) { I2C1->D = data; I2C_Wait(); }

void I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 0);
    I2C_WriteByte(reg);
    I2C_WriteByte(val);
    I2C_Stop();
}

void I2C_ReadBurst(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len) {
    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 0);
    I2C_WriteByte(reg);

    I2C1->C1 |= I2C_C1_RSTA_MASK;
    I2C_WriteByte((dev_addr << 1) | 1);
    I2C1->C1 &= ~I2C_C1_TX_MASK;

    if (len == 1) I2C1->C1 |= I2C_C1_TXAK_MASK;
    else I2C1->C1 &= ~I2C_C1_TXAK_MASK;

    uint8_t dummy = I2C1->D;
    for (uint8_t i = 0; i < len; i++) {
        I2C_Wait();
        if (i == len - 2) I2C1->C1 |= I2C_C1_TXAK_MASK;
        if (i == len - 1) I2C_Stop();
        data[i] = I2C1->D;
    }
}

uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

// ==========================================
// FUNCIONES DS3231 (RTC y Alarma)
// ==========================================
void DS3231_ReadTime(uint8_t *hour, uint8_t *min, uint8_t *sec) {
    uint8_t data[3];
    I2C_ReadBurst(DS3231_ADDR, 0x00, data, 3);
    *sec  = bcdToDec(data[0]);
    *min  = bcdToDec(data[1]);
    *hour = bcdToDec(data[2] & 0x3F);
}

void DS3231_ReadDate(uint8_t *date, uint8_t *month, uint8_t *year) {
    uint8_t data[3];
    I2C_ReadBurst(DS3231_ADDR, 0x04, data, 3);
    *date  = bcdToDec(data[0]);
    *month = bcdToDec(data[1] & 0x1F); // Enmascarar bit de siglo
    *year  = bcdToDec(data[2]);
}

void DS3231_SetTime(uint8_t hour, uint8_t min) {
    I2C_WriteReg(DS3231_ADDR, 0x00, 0x00);
    I2C_WriteReg(DS3231_ADDR, 0x01, decToBcd(min));
    I2C_WriteReg(DS3231_ADDR, 0x02, decToBcd(hour));
    delay_ms(10);
}

void DS3231_SetDate(uint8_t date, uint8_t month, uint8_t year) {
    I2C_WriteReg(DS3231_ADDR, 0x04, decToBcd(date));
    I2C_WriteReg(DS3231_ADDR, 0x05, decToBcd(month));
    I2C_WriteReg(DS3231_ADDR, 0x06, decToBcd(year));
    delay_ms(10);
}

void DS3231_ToggleAlarmHardware(uint8_t enable) {
    if(enable) I2C_WriteReg(DS3231_ADDR, 0x0E, 0x05); // INTCN=1, A1IE=1
    else       I2C_WriteReg(DS3231_ADDR, 0x0E, 0x04); // INTCN=1, A1IE=0
    delay_ms(1);
    I2C_WriteReg(DS3231_ADDR, 0x0F, 0x00); // Clear flags
}

void DS3231_SetAlarm(uint8_t hour, uint8_t min) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x07);
    I2C_WriteByte(0x00);
    I2C_WriteByte(decToBcd(min));
    I2C_WriteByte(decToBcd(hour));
    I2C_WriteByte(0x80);
    I2C_Stop();
    delay_ms(1);

    DS3231_ToggleAlarmHardware(alarm_enabled);
}

// ==========================================
// FUNCIONES BME280 (Temp, Presión, Humedad)
// ==========================================
void BME280_Init(void) {
    delay_ms(50);
    uint8_t calib[24];

    // Leer calibración de Temperatura y Presión
    I2C_ReadBurst(BME280_ADDR, 0x88, calib, 24);
    bme_dig_T1 = (calib[1] << 8) | calib[0];
    bme_dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    bme_dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);

    bme_dig_P1 = (calib[7] << 8) | calib[6];
    bme_dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
    bme_dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
    bme_dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
    bme_dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
    bme_dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
    bme_dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
    bme_dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
    bme_dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);

    // Leer H1
    I2C_ReadBurst(BME280_ADDR, 0xA1, &bme_dig_H1, 1);

    // Leer H2 a H6
    uint8_t calib_h[7];
    I2C_ReadBurst(BME280_ADDR, 0xE1, calib_h, 7);
    bme_dig_H2 = (int16_t)((calib_h[1] << 8) | calib_h[0]);
    bme_dig_H3 = calib_h[2];
    bme_dig_H4 = (int16_t)((calib_h[3] << 4) | (calib_h[4] & 0x0F));
    bme_dig_H5 = (int16_t)((calib_h[5] << 4) | (calib_h[4] >> 4));
    bme_dig_H6 = (int8_t)calib_h[6];

    // Configurar registros
    I2C_WriteReg(BME280_ADDR, 0xF2, 0x01); // ctrl_hum: Habilitar humedad
    I2C_WriteReg(BME280_ADDR, 0xF4, 0x27); // ctrl_meas: Temp x1, Pres x1, Normal Mode
    delay_ms(50);
}

void BME280_ReadAll(int16_t *temp_int, uint8_t *temp_frac, uint16_t *press_hpa, uint8_t *hum_int) {
    uint8_t data[8];
    I2C_ReadBurst(BME280_ADDR, 0xF7, data, 8);

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t adc_H = (data[6] << 8) | data[7];

    if (adc_T == 0x80000) { *temp_int = 0; *temp_frac = 0; *press_hpa = 0; *hum_int = 0; return; }

    // COMPENSACIÓN DE TEMPERATURA
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_dig_T1 << 1))) * ((int32_t)bme_dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_dig_T1)) * ((adc_T >> 4) - ((int32_t)bme_dig_T1))) >> 12) * ((int32_t)bme_dig_T3)) >> 14;
    t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8;

    *temp_int = T / 100;
    int32_t frac = T % 100;
    *temp_frac = (uint8_t)(frac < 0 ? -frac : frac);

    // COMPENSACIÓN DE PRESIÓN
    int32_t p_var1, p_var2;
    uint32_t p;
    p_var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    p_var2 = (((p_var1>>2) * (p_var1>>2)) >> 11 ) * ((int32_t)bme_dig_P6);
    p_var2 = p_var2 + ((p_var1*((int32_t)bme_dig_P5))<<1);
    p_var2 = (p_var2>>2)+(((int32_t)bme_dig_P4)<<16);
    p_var1 = (((bme_dig_P3 * (((p_var1>>2) * (p_var1>>2)) >> 13 )) >> 3) + ((((int32_t)bme_dig_P2) * p_var1)>>1))>>18;
    p_var1 = ((((32768+p_var1))*((int32_t)bme_dig_P1))>>15);

    if (p_var1 == 0) {
        *press_hpa = 0;
    } else {
        p = (((uint32_t)(((int32_t)1048576)-adc_P)-(p_var2>>12)))*3125;
        if (p < 0x80000000) p = (p << 1) / ((uint32_t)p_var1);
        else p = (p / (uint32_t)p_var1) * 2;
        p_var1 = (((int32_t)bme_dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
        p_var2 = (((int32_t)(p>>2)) * ((int32_t)bme_dig_P8))>>13;
        p = (uint32_t)((int32_t)p + ((p_var1 + p_var2 + bme_dig_P7) >> 4));
        *press_hpa = (uint16_t)(p / 100);
    }

    // COMPENSACIÓN DE HUMEDAD
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme_dig_H4) << 20) - (((int32_t)bme_dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bme_dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme_dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bme_dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme_dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *hum_int = (uint8_t)((v_x1_u32r >> 12) / 1024);
}

// ==========================================
// FUNCIONES SPI0 Y MAX7219
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
// TECLADO MATRICIAL (Escaneo Instantáneo y sin bloqueos)
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
    static char last_key = 0;
    char current_key = 0;

    for (int row = 0; row < 4; row++) {
        GPIOB->PCOR = (1 << row);
        for(volatile int i = 0; i < 500; i++); // Estabilización ultra rápida

        for (int col = 0; col < 4; col++) {
            if (!(GPIOB->PDIR & (1 << (col + 8)))) {
                current_key = keymap[row][col];
                break;
            }
        }
        GPIOB->PSOR = (1 << row);
        if (current_key) break;
    }

    if (current_key != 0 && last_key == 0) {
        last_key = current_key;
        delay_ms(15); // Debouncing (filtro rebote)
        return current_key; // Devuelve la tecla al instante
    }
    else if (current_key == 0) {
        last_key = 0; // Se soltó la tecla
    }

    return 0; // Evita lecturas múltiples al mantener presionado
}

// ==========================================
// ALARMA (INTERRUPCIÓN HARDWARE)
// ==========================================
void GPIO_Alarm_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK;

    PORTE->PCR[2] = PORT_PCR_MUX(1);
    PORTE->PCR[3] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << 2) | (1 << 3);
    GPIOE->PCOR = (1 << 2) | (1 << 3);

    PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOD->PDDR &= ~(1 << 4);
    NVIC_EnableIRQ(PORTD_IRQn);
}

void PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1 << 4)) {
        PORTD->ISFR = (1 << 4);
        alarm_triggered = 1;
    }
}

// ==========================================
// MÁQUINA DE ESTADOS Y PANTALLA PRINCIPAL
// ==========================================
typedef enum {
    MODE_NORMAL,
    MODE_CONFIG_TIME,
    MODE_CONFIG_DATE,
    MODE_CONFIG_ALARM,
    MODE_ALARM_RINGING
} SystemState;

SystemState current_mode = MODE_NORMAL;
char input_data[7] = "000000"; // Espacio para 6 dígitos + nulo
uint8_t input_index = 0;

void Draw_Config_Screen(void) {
    char lcd_buf[17];
    LCD_Command(0x01); delay_ms(2);

    if (current_mode == MODE_CONFIG_TIME) {
        LCD_Command(0x80); LCD_Print("SET TIME HH:MM");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c:%c%c", input_data[0], input_data[1], input_data[2], input_data[3]);
        LCD_Print(lcd_buf);
    }
    else if (current_mode == MODE_CONFIG_ALARM) {
        LCD_Command(0x80); LCD_Print("ALARM SET HH:MM");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c:%c%c", input_data[0], input_data[1], input_data[2], input_data[3]);
        LCD_Print(lcd_buf);
    }
    else if (current_mode == MODE_CONFIG_DATE) {
        LCD_Command(0x80); LCD_Print("SET DATE DDMMYY");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c/%c%c/%c%c", input_data[0], input_data[1], input_data[2], input_data[3], input_data[4], input_data[5]);
        LCD_Print(lcd_buf);
    }
}

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

    // Fuerza el encendido del reloj interno del DS3231 en caso de estar apagado
    I2C_WriteReg(DS3231_ADDR, 0x0E, 0x04);
    BME280_Init();

    LCD_Command(0x80);
    LCD_Print("System Boot...");
    delay_ms(1000);
    LCD_Command(0x01);

    char buffer[17];
    uint8_t hour, min, sec;
    uint8_t date, month, year;

    int16_t temp_int;
    uint8_t temp_frac, hum_int;
    uint16_t press_hpa;
    uint8_t last_sec = 255;

    // Control de la información en el LCD
    uint8_t display_page = 0;

    while(1) {
        char key = Keypad_Scan();

        // Chequear interrupción de alarma
        if (alarm_triggered && current_mode != MODE_ALARM_RINGING && alarm_enabled) {
            current_mode = MODE_ALARM_RINGING;
            LCD_Command(0x01);
            LCD_Command(0x80); LCD_Print("*** ALARM ***");
            LCD_Command(0xC0); LCD_Print("Press # to stop");
            GPIOE->PSOR = (1 << 2) | (1 << 3); // Enciende LED y Zumbador
        }

        switch(current_mode) {
            case MODE_ALARM_RINGING:
                if (key == '#') {
                    alarm_triggered = 0;
                    I2C_WriteReg(DS3231_ADDR, 0x0F, 0x00); // Limpia bandera alarma
                    GPIOE->PCOR = (1 << 2) | (1 << 3);     // Apaga LED y Zumbador
                    current_mode = MODE_NORMAL;
                    last_sec = 255;
                    LCD_Command(0x01);
                }
                break;

            case MODE_NORMAL:
                DS3231_ReadTime(&hour, &min, &sec);

                // 1. Refresco general cada segundo (I2C + LCD completo)
                if (sec != last_sec) {
                    last_sec = sec;
                    DS3231_ReadDate(&date, &month, &year);
                    BME280_ReadAll(&temp_int, &temp_frac, &press_hpa, &hum_int);

                    LCD_Command(0x80);
                    sprintf(buffer, "%02d/%02d/%02d %02d:%02d", date, month, year, hour, min);
                    LCD_Print(buffer);

                    LCD_Command(0xC0);
                    if (display_page == 0) {
                        sprintf(buffer, "T:%d.%02dC Alm:%s ", temp_int, temp_frac, alarm_enabled ? "ON " : "OFF");
                    } else {
                        sprintf(buffer, "H:%d%% P:%dhPa ", hum_int, press_hpa);
                    }
                    LCD_Print(buffer);

                    MAX7219_DisplayTime(hour, min);
                }

                // 2. Interacciones de Teclado
                if (key == 'A') {
                    current_mode = MODE_CONFIG_TIME; input_index = 0;
                    sprintf(input_data, "0000"); Draw_Config_Screen();
                } else if (key == 'C') {
                    current_mode = MODE_CONFIG_ALARM; input_index = 0;
                    sprintf(input_data, "0000"); Draw_Config_Screen();
                } else if (key == '*') {
                    current_mode = MODE_CONFIG_DATE; input_index = 0;
                    sprintf(input_data, "000000"); Draw_Config_Screen();
                } else if (key == '#') {
                    alarm_enabled = !alarm_enabled;
                    DS3231_ToggleAlarmHardware(alarm_enabled);
                    last_sec = 255; // Forzar refresco para mostrar ON/OFF
                } else if (key == 'D') {
                    // REFRESCO ULTRARRÁPIDO: Actualiza LCD sin tocar I2C
                    display_page = !display_page;
                    LCD_Command(0xC0); // Mueve cursor directo a la segunda línea

                    if (display_page == 0) {
                        sprintf(buffer, "T:%d.%02dC Alm:%s ", temp_int, temp_frac, alarm_enabled ? "ON " : "OFF");
                    } else {
                        sprintf(buffer, "H:%d%% P:%dhPa ", hum_int, press_hpa);
                    }
                    LCD_Print(buffer);
                }
                break;

            case MODE_CONFIG_ALARM:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_data[input_index++] = key; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 4) {
                    uint8_t new_h = (input_data[0]-'0')*10 + (input_data[1]-'0');
                    uint8_t new_m = (input_data[2]-'0')*10 + (input_data[3]-'0');
                    DS3231_SetAlarm(new_h, new_m);
                    current_mode = MODE_NORMAL; last_sec = 255;
                    LCD_Command(0x01); LCD_Command(0x80); LCD_Print("ALARM SAVED!");
                    delay_ms(1000); LCD_Command(0x01);
                } else if (key == 'B') {
                    current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01);
                }
                break;

            case MODE_CONFIG_TIME:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_data[input_index++] = key; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 4) {
                    uint8_t new_h = (input_data[0]-'0')*10 + (input_data[1]-'0');
                    uint8_t new_m = (input_data[2]-'0')*10 + (input_data[3]-'0');
                    DS3231_SetTime(new_h, new_m);
                    current_mode = MODE_NORMAL; last_sec = 255;
                    LCD_Command(0x01); LCD_Command(0x80); LCD_Print("TIME SAVED!");
                    delay_ms(1000); LCD_Command(0x01);
                } else if (key == 'B') {
                    current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01);
                }
                break;

            case MODE_CONFIG_DATE:
                if (key >= '0' && key <= '9' && input_index < 6) {
                    input_data[input_index++] = key; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 6) {
                    uint8_t new_d = (input_data[0]-'0')*10 + (input_data[1]-'0');
                    uint8_t new_mo = (input_data[2]-'0')*10 + (input_data[3]-'0');
                    uint8_t new_y = (input_data[4]-'0')*10 + (input_data[5]-'0');
                    DS3231_SetDate(new_d, new_mo, new_y);
                    current_mode = MODE_NORMAL; last_sec = 255;
                    LCD_Command(0x01); LCD_Command(0x80); LCD_Print("DATE SAVED!");
                    delay_ms(1000); LCD_Command(0x01);
                } else if (key == 'B') {
                    current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01);
                }
                break;
        }
        delay_ms(10);
    }
}
