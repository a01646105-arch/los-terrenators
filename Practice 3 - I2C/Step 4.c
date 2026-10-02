// Prueba Step 4

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

volatile uint8_t alarm_triggered = 0; // Bandera de interrupción

// Retardo simple
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

void I2C_Start(void) { I2C1->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK; }
void I2C_Stop(void) { I2C1->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK); }
void I2C_WriteByte(uint8_t data) { I2C1->D = data; I2C_Wait(); }

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

    uint8_t dummy = I2C1->D; I2C_Wait();
    *sec   = bcdToDec(I2C1->D); I2C_Wait();
    *min   = bcdToDec(I2C1->D); I2C_Wait();
    *hour  = bcdToDec(I2C1->D & 0x3F); I2C_Wait();
    dummy  = I2C1->D; I2C_Wait(); 
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
    I2C_WriteByte(1); 
    I2C_WriteByte(decToBcd(day));
    I2C_WriteByte(decToBcd(month));
    I2C_WriteByte(decToBcd(year));
    I2C_Stop();
    delay_ms(10);
}

// ---------------- ALARMA RTC ----------------
void DS3231_ClearAlarmFlag(void) {
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x0F); // Registro de Status
    I2C_WriteByte(0x00); // Limpiar flag de alarma (A1F) para que el pin INT vuelva a HIGH
    I2C_Stop();
}

void DS3231_SetAlarm(uint8_t hour, uint8_t min) {
    // 1. Configurar los registros de la Alarma 1 (0x07 a 0x0A)
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x07); 
    I2C_WriteByte(0x00);               // Segundos (00) con A1M1 = 0
    I2C_WriteByte(decToBcd(min));      // Minutos con A1M2 = 0
    I2C_WriteByte(decToBcd(hour));     // Horas con A1M3 = 0
    I2C_WriteByte(0x80);               // A1M4 = 1 (Disparar al coincidir H, M y S)
    I2C_Stop();
    delay_ms(1);

    // 2. Activar las interrupciones de la Alarma en el registro de Control (0x0E)
    I2C_Start();
    I2C_WriteByte((DS3231_ADDR << 1) | 0);
    I2C_WriteByte(0x0E); 
    I2C_WriteByte(0x05); // INTCN = 1 (Salida de interrupción), A1IE = 1 (Activar Alarma 1)
    I2C_Stop();
    delay_ms(1);

    // 3. Asegurar que la bandera (Flag) esté limpia
    DS3231_ClearAlarmFlag();
}

// ==========================================
// FUNCIONES SPI0 (MAX7219 - PUERTO D)
// ==========================================
void SPI0_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;

    PORTD->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOD->PDDR |= (1 << 0);
    GPIOD->PSOR = (1 << 0);

    PORTD->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;
    PORTD->PCR[2] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK;

    SPI0->C1 = SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK;
    SPI0->BR = SPI_BR_SPPR(3) | SPI_BR_SPR(5);
}

void MAX7219_Write(uint8_t addr, uint8_t data) {
    GPIOD->PCOR = (1 << 0);
    for(volatile int i = 0; i < 20; i++);
    while(!(SPI0->S & SPI_S_SPTEF_MASK)); SPI0->D = addr;
    while(!(SPI0->S & SPI_S_SPRF_MASK)); (void)SPI0->D;
    while(!(SPI0->S & SPI_S_SPTEF_MASK)); SPI0->D = data;
    while(!(SPI0->S & SPI_S_SPRF_MASK)); (void)SPI0->D;
    for(volatile int i = 0; i < 50; i++);
    GPIOD->PSOR = (1 << 0);
    for(volatile int i = 0; i < 50; i++);
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
// CONFIGURACIÓN DEL KEYPAD (PUERTO B)
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
        GPIOB->PDDR |= (1 << i);
        GPIOB->PSOR = (1 << i);
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
// HARDWARE DE ALARMA E INTERRUPCIÓN
// ==========================================
void GPIO_Alarm_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK;

    // PTE2 (Buzzer) y PTE3 (LED) como Salidas
    PORTE->PCR[2] = PORT_PCR_MUX(1);
    PORTE->PCR[3] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << 2) | (1 << 3);
    GPIOE->PCOR = (1 << 2) | (1 << 3); // Apagados por defecto

    // PTD4 como Entrada de Interrupción desde el RTC (Flanco de Bajada)
    PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; 
    GPIOD->PDDR &= ~(1 << 4);

    // Habilitar Interrupciones del Puerto D en el NVIC
    NVIC_EnableIRQ(PORTD_IRQn);
}

// Vector de Interrupción para el Puerto D
void PORTD_IRQHandler(void) {
    // Verificar si la interrupción vino de PTD4
    if (PORTD->ISFR & (1 << 4)) {
        PORTD->ISFR = (1 << 4); // Limpiar la bandera en el microcontrolador
        alarm_triggered = 1;    // Avisarle al loop principal que sonó la alarma
    }
}

// ==========================================
// MÁQUINA DE ESTADOS
// ==========================================
typedef enum {
    MODE_NORMAL,
    MODE_CONFIG_TIME,
    MODE_CONFIG_DATE,
    MODE_CONFIG_ALARM,
    MODE_ALARM_RINGING
} SystemState;

SystemState current_mode = MODE_NORMAL;
char input_time[5] = "0000"; 
char input_date[7] = "000000"; 
uint8_t input_index = 0;

void Draw_Config_Screen(void) {
    char lcd_buf[17];
    LCD_Command(0x01); 
    delay_ms(2);
    
    if (current_mode == MODE_CONFIG_TIME) {
        LCD_Command(0x80); LCD_Print("SET TIME HH:MM");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
        LCD_Print(lcd_buf);
    } 
    else if (current_mode == MODE_CONFIG_DATE) {
        LCD_Command(0x80); LCD_Print("SET DATE DDMMYY");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c/%c%c/%c%c", input_date[0], input_date[1], input_date[2], input_date[3], input_date[4], input_date[5]);
        LCD_Print(lcd_buf);
    }
    else if (current_mode == MODE_CONFIG_ALARM) {
        LCD_Command(0x80); LCD_Print("ALARM SET HH:MM");
        LCD_Command(0xC0); sprintf(lcd_buf, "-> %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
        LCD_Print(lcd_buf);
    }
}

// ==========================================
// PROGRAMA PRINCIPAL
// ==========================================
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    delay_ms(100);

    // Inicializaciones
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    for(int i = 0; i <= 5; i++) PORTC->PCR[i] = PORT_PCR_MUX(1);
    GPIOC->PDDR |= (RS | EN | D4 | D5 | D6 | D7);

    LCD_Init();
    I2C1_Init();
    SPI0_Init();
    MAX7219_Init();
    Keypad_Init();
    GPIO_Alarm_Init(); // <-- Inicializar hardware de alarma

    LCD_Command(0x80);
    LCD_Print("System Boot...");
    delay_ms(1000);
    LCD_Command(0x01);

    char buffer[17];
    uint8_t hour, min, sec, day, month, year;
    uint8_t last_sec = 255; 

    while(1) {
        char key = Keypad_Scan();

        // -------------------------------------------------------------
        // DISPARADOR DE LA ALARMA (Se activa por la interrupción de PTD4)
        // -------------------------------------------------------------
        if (alarm_triggered && current_mode != MODE_ALARM_RINGING) {
            current_mode = MODE_ALARM_RINGING;
            LCD_Command(0x01);
            LCD_Command(0x80); LCD_Print("*** ALARM ***");
            LCD_Command(0xC0); LCD_Print("Press # to stop");
            
            // Activar Buzzer (PTE2) y LED (PTE3)
            GPIOE->PSOR = (1 << 2) | (1 << 3);
        }

        switch(current_mode) {
            // ---------------------------------------------------------
            // MODO ALARMA SONANDO
            // ---------------------------------------------------------
            case MODE_ALARM_RINGING:
                if (key == '#') {
                    alarm_triggered = 0;
                    DS3231_ClearAlarmFlag(); // Resetear el RTC para que libere el pin INT
                    GPIOE->PCOR = (1 << 2) | (1 << 3); // Apagar Buzzer y LED
                    current_mode = MODE_NORMAL;
                    last_sec = 255;
                    LCD_Command(0x01);
                }
                break;

            // ---------------------------------------------------------
            // MODO NORMAL
            // ---------------------------------------------------------
            case MODE_NORMAL:
                DS3231_ReadTime(&hour, &min, &sec, &day, &month, &year);
                if (sec != last_sec) {
                    last_sec = sec;
                    LCD_Command(0x80); sprintf(buffer, "Date: %02d/%02d/%02d", day, month, year); LCD_Print(buffer);
                    LCD_Command(0xC0); sprintf(buffer, "Time: %02d:%02d:%02d", hour, min, sec); LCD_Print(buffer);
                    MAX7219_DisplayTime(hour, min);
                }

                if (key == 'A') { // Entrar a configurar Hora general
                    current_mode = MODE_CONFIG_TIME;
                    input_index = 0; sprintf(input_time, "0000"); 
                    Draw_Config_Screen();
                }
                else if (key == 'C') { // Entrar a configurar ALARMA
                    current_mode = MODE_CONFIG_ALARM;
                    input_index = 0; sprintf(input_time, "0000"); 
                    Draw_Config_Screen();
                }
                break;

            // ---------------------------------------------------------
            // CONFIGURAR ALARMA
            // ---------------------------------------------------------
            case MODE_CONFIG_ALARM:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_time[input_index++] = key;
                    Draw_Config_Screen();
                } 
                else if (key == 'D' && input_index == 4) { // Guardar Alarma
                    uint8_t new_h = (input_time[0] - '0') * 10 + (input_time[1] - '0');
                    uint8_t new_m = (input_time[2] - '0') * 10 + (input_time[3] - '0');
                    
                    DS3231_SetAlarm(new_h, new_m); // Mandar I2C
                    
                    current_mode = MODE_NORMAL; last_sec = 255;
                    LCD_Command(0x01); LCD_Command(0x80); LCD_Print("ALARM SAVED!");
                    delay_ms(1000); LCD_Command(0x01);
                }
                else if (key == 'B') { // Cancelar
                    current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01);
                }
                break;

            // ---------------------------------------------------------
            // CONFIGURAR HORA
            // ---------------------------------------------------------
            case MODE_CONFIG_TIME:
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_time[input_index++] = key; Draw_Config_Screen();
                } 
                else if (key == 'A' && input_index == 4) { 
                    current_mode = MODE_CONFIG_DATE; input_index = 0;
                    sprintf(input_date, "000000"); Draw_Config_Screen();
                }
                else if (key == 'B') { current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01); }
                break;

            // ---------------------------------------------------------
            // CONFIGURAR FECHA
            // ---------------------------------------------------------
            case MODE_CONFIG_DATE:
                if (key >= '0' && key <= '9' && input_index < 6) {
                    input_date[input_index++] = key; Draw_Config_Screen();
                } 
                else if (key == 'D' && input_index == 6) { 
                    uint8_t new_h = (input_time[0] - '0') * 10 + (input_time[1] - '0');
                    uint8_t new_m = (input_time[2] - '0') * 10 + (input_time[3] - '0');
                    uint8_t new_D = (input_date[0] - '0') * 10 + (input_date[1] - '0');
                    uint8_t new_M = (input_date[2] - '0') * 10 + (input_date[3] - '0');
                    uint8_t new_Y = (input_date[4] - '0') * 10 + (input_date[5] - '0');
                    DS3231_SetTime(new_h, new_m, 0, 1, new_M, new_Y); 
                    
                    current_mode = MODE_NORMAL; last_sec = 255;
                    LCD_Command(0x01); LCD_Command(0x80); LCD_Print("TIME SAVED!");
                    delay_ms(1000); LCD_Command(0x01);
                }
                else if (key == 'B') { current_mode = MODE_NORMAL; last_sec = 255; LCD_Command(0x01); }
                break;
        }
        delay_ms(10); 
    }
}
