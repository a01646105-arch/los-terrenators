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
#define BME280_ADDR 0x76 // Mantenlo en 0x76, es el correcto para tu setup

volatile uint8_t alarm_triggered = 0;

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
// NÚCLEO I2C CORREGIDO Y MEJORADO
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
    I2C1->S = I2C_S_IICIF_MASK; // CORRECCIÓN: Asignación directa para limpiar solo esta bandera
}

void I2C_Start(void) { I2C1->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK; }
void I2C_Stop(void) { I2C1->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK); }
void I2C_WriteByte(uint8_t data) { I2C1->D = data; I2C_Wait(); }

void I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 0);
    I2C_WriteByte(reg);
    I2C_WriteByte(val);
    I2C_Stop();
}

// NUEVA FUNCIÓN: LECTURA EN RÁFAGA (Soluciona el problema del BME280)
void I2C_ReadBurst(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len) {
    I2C_Start();
    I2C_WriteByte((dev_addr << 1) | 0);
    I2C_WriteByte(reg);

    I2C1->C1 |= I2C_C1_RSTA_MASK; // Repeated Start
    I2C_WriteByte((dev_addr << 1) | 1);

    I2C1->C1 &= ~I2C_C1_TX_MASK; // Modo RX

    // Configurar ACK/NACK inicial
    if (len == 1) I2C1->C1 |= I2C_C1_TXAK_MASK; 
    else I2C1->C1 &= ~I2C_C1_TXAK_MASK; 

    uint8_t dummy = I2C1->D; // Dummy read
    for (uint8_t i = 0; i < len; i++) {
        I2C_Wait();
        if (i == len - 2) I2C1->C1 |= I2C_C1_TXAK_MASK; // NACK antes del último
        if (i == len - 1) I2C_Stop();                   // STOP en el último
        data[i] = I2C1->D;
    }
}

uint8_t bcdToDec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

// ==========================================
// FUNCIONES DS3231 (Adaptadas a Ráfaga)
// ==========================================
void DS3231_ReadTime(uint8_t *hour, uint8_t *min, uint8_t *sec) {
    uint8_t data[3];
    I2C_ReadBurst(DS3231_ADDR, 0x00, data, 3);
    *sec  = bcdToDec(data[0]);
    *min  = bcdToDec(data[1]);
    *hour = bcdToDec(data[2] & 0x3F);
}

void DS3231_SetTime(uint8_t hour, uint8_t min) {
    I2C_WriteReg(DS3231_ADDR, 0x00, 0x00); 
    I2C_WriteReg(DS3231_ADDR, 0x01, decToBcd(min));
    I2C_WriteReg(DS3231_ADDR, 0x02, decToBcd(hour));
    delay_ms(10);
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
    
    I2C_WriteReg(DS3231_ADDR, 0x0E, 0x05); 
    delay_ms(1);
    I2C_WriteReg(DS3231_ADDR, 0x0F, 0x00); // Clear flag
}

// ==========================================
// FUNCIONES BME280 (Adaptadas a Ráfaga)
// ==========================================
uint16_t bme_dig_T1;
int16_t bme_dig_T2, bme_dig_T3;

void BME280_Init(void) {
    delay_ms(50); 
    uint8_t calib[6];
    
    // Leer los 6 bytes de calibración en un solo jalón
    I2C_ReadBurst(BME280_ADDR, 0x88, calib, 6);
    
    bme_dig_T1 = (calib[1] << 8) | calib[0];
    bme_dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    bme_dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);
    
    I2C_WriteReg(BME280_ADDR, 0xF2, 0x01); 
    I2C_WriteReg(BME280_ADDR, 0xF5, 0x00); 
    I2C_WriteReg(BME280_ADDR, 0xF4, 0x27); 
    delay_ms(50); 
}

void BME280_ReadTemp(int16_t *temp_int, uint8_t *temp_frac) {
    uint8_t data[3];
    
    // Leer los 3 bytes del ADC en un solo jalón
    I2C_ReadBurst(BME280_ADDR, 0xFA, data, 3);
    
    int32_t adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    
    // Si el ADC devuelve 0x80000, la medición está apagada/falló
    if (adc_T == 0x80000) {
        *temp_int = 0; *temp_frac = 0; return;
    }
    
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_dig_T1 << 1))) * ((int32_t)bme_dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_dig_T1)) * ((adc_T >> 4) - ((int32_t)bme_dig_T1))) >> 12) * ((int32_t)bme_dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8; 
    
    *temp_int = T / 100;
    int32_t frac = T % 100;
    if (frac < 0) frac = -frac; 
    *temp_frac = (uint8_t)frac;
}

// ==========================================
// FUNCIONES SPI0 Y DISPLAY MAX7219
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
// TECLADO (PTB8 - PTB11)
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
// ALARMA (INTERRUPCIÓN)
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
// PROGRAMA PRINCIPAL
// ==========================================
typedef enum { MODE_NORMAL, MODE_CONFIG_TIME, MODE_CONFIG_ALARM, MODE_ALARM_RINGING } SystemState;
SystemState current_mode = MODE_NORMAL;
char input_time[5] = "0000"; 
uint8_t input_index = 0;

void Draw_Config_Screen(void) {
    char lcd_buf[17];
    LCD_Command(0x01); delay_ms(2);
    if (current_mode == MODE_CONFIG_TIME) {
        LCD_Command(0x80); LCD_Print("SET TIME HH:MM");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
        LCD_Print(lcd_buf);
    } else if (current_mode == MODE_CONFIG_ALARM) {
        LCD_Command(0x80); LCD_Print("ALARM SET HH:MM");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
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
    BME280_Init(); 

    LCD_Command(0x80);
    LCD_Print("System Boot...");
    delay_ms(1000);
    LCD_Command(0x01);

    char buffer[17];
    uint8_t hour, min, sec;
    int16_t temp_int;
    uint8_t temp_frac;
    uint8_t last_sec = 255; 

    while(1) {
        char key = Keypad_Scan();

        if (alarm_triggered && current_mode != MODE_ALARM_RINGING) {
            current_mode = MODE_ALARM_RINGING;
            LCD_Command(0x01);
            LCD_Command(0x80); LCD_Print("*** ALARM ***");
            LCD_Command(0xC0); LCD_Print("Press # to stop");
            GPIOE->PSOR = (1 << 2) | (1 << 3);
        }

        switch(current_mode) {
            case MODE_ALARM_RINGING:
                if (key == '#') {
                    alarm_triggered = 0;
                    I2C_WriteReg(DS3231_ADDR, 0x0F, 0x00); // Clear alarm
                    GPIOE->PCOR = (1 << 2) | (1 << 3); 
                    current_mode = MODE_NORMAL;
                    last_sec = 255; LCD_Command(0x01);
                }
                break;

            case MODE_NORMAL:
                DS3231_ReadTime(&hour, &min, &sec);
                if (sec != last_sec) {
                    last_sec = sec;
                    BME280_ReadTemp(&temp_int, &temp_frac);

                    LCD_Command(0x80); 
                    sprintf(buffer, "Time: %02d:%02d:%02d", hour, min, sec); 
                    LCD_Print(buffer);
                    
                    LCD_Command(0xC0); 
                    sprintf(buffer, "Temp: %d.%02d C   ", temp_int, temp_frac); 
                    LCD_Print(buffer);
                    
                    MAX7219_DisplayTime(hour, min);
                }

                if (key == 'A') { 
                    current_mode = MODE_CONFIG_TIME; input_index = 0; 
                    sprintf(input_time, "0000"); Draw_Config_Screen();
                } else if (key == 'C') { 
                    current_mode = MODE_CONFIG_ALARM; input_index = 0; 
                    sprintf(input_time, "0000"); Draw_Config_Screen();
                }
                break;

            case MODE_CONFIG_ALARM:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_time[input_index++] = key; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 4) { 
                    uint8_t new_h = (input_time[0]-'0')*10 + (input_time[1]-'0');
                    uint8_t new_m = (input_time[2]-'0')*10 + (input_time[3]-'0');
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
                    input_time[input_index++] = key; Draw_Config_Screen();
                } else if (key == 'D' && input_index == 4) { 
                    uint8_t new_h = (input_time[0]-'0')*10 + (input_time[1]-'0');
                    uint8_t new_m = (input_time[2]-'0')*10 + (input_time[3]-'0');
                    DS3231_SetTime(new_h, new_m); 
                    current_mode = MODE_NORMAL; last_sec = 255;
                    LCD_Command(0x01); LCD_Command(0x80); LCD_Print("TIME SAVED!");
                    delay_ms(1000); LCD_Command(0x01);
                } else if (key == 'B') { 
                    current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01); 
                }
                break;
        }
        delay_ms(10); 
    }
}
