// Prueba step 3 - Keypad pins fixed

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

    LCD_Send(0x28, 0); // 4 bits, 2 lineas
    LCD_Send(0x0C, 0); // Display ON, Cursor OFF
    LCD_Send(0x01, 0); // Clear display
    delay_ms(5);
    LCD_Send(0x06, 0); // Increment cursor
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
    I2C_WriteByte(1); // Día de la semana (arbitrario)
    I2C_WriteByte(decToBcd(day));
    I2C_WriteByte(decToBcd(month));
    I2C_WriteByte(decToBcd(year));

    I2C_Stop();
    delay_ms(10);
}

// ==========================================
// FUNCIONES SPI0 (MAX7219 - PUERTO D)
// Configurado para Cátodo Común (5461BS)
// ==========================================
void SPI0_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;

    PORTD->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOD->PDDR |= (1 << 0);
    GPIOD->PSOR = (1 << 0); // CS HIGH

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
    MAX7219_Write(0x0F, 0x00); // Apagar Display Test
    delay_ms(10);
    MAX7219_Write(0x09, 0x0F); // Decode Mode para 4 dígitos (Code B)
    MAX7219_Write(0x0B, 0x03); // Scan Limit 4 dígitos
    MAX7219_Write(0x0A, 0x04); // Brillo
    MAX7219_Write(0x0C, 0x01); // Normal Operation

    for(int i = 1; i <= 4; i++) MAX7219_Write(i, 0x0F);
    delay_ms(10);
}

void MAX7219_DisplayTime(uint8_t hour, uint8_t min) {
    MAX7219_Write(1, (hour / 10) % 10);
    MAX7219_Write(2, (hour % 10) | 0x80); // Punto decimal encendido
    MAX7219_Write(3, (min / 10) % 10);
    MAX7219_Write(4, min % 10);
}

// ==========================================
// CONFIGURACIÓN DEL KEYPAD (PUERTO B)
// Filas (Outputs): PTB0, PTB1, PTB2, PTB3
// Columnas (Inputs): PTB8, PTB9, PTB10, PTB11
// ==========================================
const char keymap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

void Keypad_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Filas como salidas (PTB0 a PTB3)
    for (int i = 0; i <= 3; i++) {
        PORTB->PCR[i] = PORT_PCR_MUX(1);
        GPIOB->PDDR |= (1 << i);
        GPIOB->PSOR = (1 << i);
    }
    // Columnas como entradas con Pull-Up (PTB8 a PTB11)
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
            // Evaluar desde PTB8 hasta PTB11
            if (!(GPIOB->PDIR & (1 << (col + 8)))) {
                while (!(GPIOB->PDIR & (1 << (col + 8)))); // Antirrebote al soltar
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
// MÁQUINA DE ESTADOS (STATE MACHINE)
// ==========================================
typedef enum {
    MODE_NORMAL,
    MODE_CONFIG_TIME,
    MODE_CONFIG_DATE
} SystemState;

SystemState current_mode = MODE_NORMAL;
char input_time[5] = "0000"; 
char input_date[7] = "000000"; 
uint8_t input_index = 0;

void Draw_Config_Screen(void) {
    char lcd_buf[17];
    LCD_Command(0x01); // Limpiar pantalla
    delay_ms(2);
    
    if (current_mode == MODE_CONFIG_TIME) {
        LCD_Command(0x80);
        LCD_Print("SET TIME HH:MM");
        LCD_Command(0xC0);
        sprintf(lcd_buf, "-> %c%c:%c%c", input_time[0], input_time[1], input_time[2], input_time[3]);
        LCD_Print(lcd_buf);
    } 
    else if (current_mode == MODE_CONFIG_DATE) {
        LCD_Command(0x80);
        LCD_Print("SET DATE DDMMYY");
        LCD_Command(0xC0);
        sprintf(lcd_buf, "-> %c%c/%c%c/%c%c", input_date[0], input_date[1], input_date[2], input_date[3], input_date[4], input_date[5]);
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

    // Habilitar LCD (Puerto C)
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    for(int i = 0; i <= 5; i++) PORTC->PCR[i] = PORT_PCR_MUX(1);
    GPIOC->PDDR |= (RS | EN | D4 | D5 | D6 | D7);

    // Inicialización de Periféricos
    LCD_Init();
    I2C1_Init();
    SPI0_Init();
    MAX7219_Init();
    Keypad_Init();

    LCD_Command(0x80);
    LCD_Print("System Boot...");
    delay_ms(1000);
    LCD_Command(0x01);

    char buffer[17];
    uint8_t hour, min, sec, day, month, year;
    uint8_t last_sec = 255; // Para actualizar LCD solo cuando cambie el segundo

    while(1) {
        char key = Keypad_Scan();

        switch(current_mode) {
            // ---------------------------------------------------------
            // MODO NORMAL: Reloj corriendo
            // ---------------------------------------------------------
            case MODE_NORMAL:
                DS3231_ReadTime(&hour, &min, &sec, &day, &month, &year);

                // Solo redibujar si el segundo cambió (evita parpadeo y libera CPU)
                if (sec != last_sec) {
                    last_sec = sec;
                    
                    LCD_Command(0x80);
                    sprintf(buffer, "Date: %02d/%02d/%02d", day, month, year);
                    LCD_Print(buffer);

                    LCD_Command(0xC0);
                    sprintf(buffer, "Time: %02d:%02d:%02d", hour, min, sec);
                    LCD_Print(buffer);

                    MAX7219_DisplayTime(hour, min);
                }

                // Presionar 'A' para entrar a Configuración
                if (key == 'A') {
                    current_mode = MODE_CONFIG_TIME;
                    input_index = 0;
                    sprintf(input_time, "0000"); // Reiniciar buffer
                    Draw_Config_Screen();
                }
                break;

            // ---------------------------------------------------------
            // MODO CONFIGURACIÓN: Ingresar Hora (HH:MM)
            // ---------------------------------------------------------
            case MODE_CONFIG_TIME:
                // Ingresar números
                if (key >= '0' && key <= '9' && input_index < 4) {
                    input_time[input_index++] = key;
                    Draw_Config_Screen();
                } 
                // Botón 'A': Siguiente (Pasar a configurar Fecha)
                else if (key == 'A' && input_index == 4) { 
                    current_mode = MODE_CONFIG_DATE;
                    input_index = 0;
                    sprintf(input_date, "000000"); // Reiniciar buffer
                    Draw_Config_Screen();
                }
                // Botón 'B': Cancelar y volver al inicio
                else if (key == 'B') {
                    current_mode = MODE_NORMAL;
                    last_sec = 255; // Forzar actualización de pantalla
                    LCD_Command(0x01);
                }
                break;

            // ---------------------------------------------------------
            // MODO CONFIGURACIÓN: Ingresar Fecha (DD/MM/YY)
            // ---------------------------------------------------------
            case MODE_CONFIG_DATE:
                // Ingresar números
                if (key >= '0' && key <= '9' && input_index < 6) {
                    input_date[input_index++] = key;
                    Draw_Config_Screen();
                } 
                // Botón 'D': Confirmar y Guardar
                else if (key == 'D' && input_index == 6) { 
                    uint8_t new_h = (input_time[0] - '0') * 10 + (input_time[1] - '0');
                    uint8_t new_m = (input_time[2] - '0') * 10 + (input_time[3] - '0');
                    
                    uint8_t new_D = (input_date[0] - '0') * 10 + (input_date[1] - '0');
                    uint8_t new_M = (input_date[2] - '0') * 10 + (input_date[3] - '0');
                    uint8_t new_Y = (input_date[4] - '0') * 10 + (input_date[5] - '0');

                    // Guardar en el RTC (segundos a 0)
                    DS3231_SetTime(new_h, new_m, 0, 1, new_M, new_Y); 

                    // Volver a modo normal
                    current_mode = MODE_NORMAL;
                    last_sec = 255; // Forzar redibujado de pantalla
                    LCD_Command(0x01); 
                    LCD_Command(0x80);
                    LCD_Print("SAVED!");
                    delay_ms(1000);
                    LCD_Command(0x01);
                }
                // Botón 'B': Cancelar y volver al inicio
                else if (key == 'B') {
                    current_mode = MODE_NORMAL;
                    last_sec = 255;
                    LCD_Command(0x01);
                }
                break;
        }
        
        delay_ms(10); // Pequeña pausa para no saturar el bus
    }
}
