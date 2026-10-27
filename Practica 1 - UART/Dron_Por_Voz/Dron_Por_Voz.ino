#include <WiFi.h>
#include <WiFiUdp.h>
#include "arduinoFFT.h" // Librería para la transformada de Fourier

// Configuración de red del Tello
const char* ssid = "TELLO-XXXXXX"; // Pon la red de tu dron
const char* password = "";         // Tello no tiene contraseña por defecto
const char* telloIP = "192.168.10.1";
const int telloPort = 8889;

WiFiUDP udp;

// Variables Globales compartidas (protegidas por un Mutex en FreeRTOS)
float frecuencia_detectada = 0.0;
float pid_output = 0.0;

// Configuración PID
float Kp = 1.0, Ki = 0.0, Kd = 0.1;
float error_previo = 0, integral = 0;
float setpoint_frecuencia = 1000.0; // Frecuencia objetivo en Hz

// --- TAREA 1: Procesamiento de Audio y Fourier ---
void TaskAudioFFT(void *pvParameters) {
  while(true) {
    // 1. Leer datos del micrófono vía I2S (Falta configurar I2S aquí)
    // 2. Llenar arreglos vReal y vImag con las muestras de audio
    
    // arduinoFFT FFT = arduinoFFT(vReal, vImag, muestras, frecuencia_muestreo);
    // FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    // FFT.Compute(FFT_FORWARD);
    // FFT.ComplexToMagnitude();
    
    // 3. Obtener el pico mayor (frecuencia de tu voz)
    // frecuencia_detectada = FFT.MajorPeak();
    
    vTaskDelay(20 / portTICK_PERIOD_MS); // Ejecutar cada 20ms
  }
}

// --- TAREA 2: Controlador PID ---
void TaskPID(void *pvParameters) {
  while(true) {
    // Calcular error (diferencia entre la voz que entra y la esperada)
    float error = setpoint_frecuencia - frecuencia_detectada;
    
    // Ecuación PID
    integral += error;
    float derivativo = error - error_previo;
    pid_output = (Kp * error) + (Ki * integral) + (Kd * derivativo);
    
    // Limitar salida para comandos RC del Tello (-100 a 100)
    if(pid_output > 100) pid_output = 100;
    if(pid_output < -100) pid_output = -100;
    
    error_previo = error;
    
    vTaskDelay(50 / portTICK_PERIOD_MS); // Bucle de control a 20Hz
  }
}

// --- TAREA 3: Control del Tello vía UDP ---
void TaskTelloUDP(void *pvParameters) {
  // Inicializar SDK del Tello
  udp.beginPacket(telloIP, telloPort);
  udp.printf("command"); // Pone al Tello en modo SDK
  udp.endPacket();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  // Despegar
  udp.beginPacket(telloIP, telloPort);
  udp.printf("takeoff");
  udp.endPacket();
  vTaskDelay(5000 / portTICK_PERIOD_MS); // Esperar a que suba
  
  while(true) {
    // Construir el comando RC usando la salida del PID
    // Ejemplo: Mover hacia adelante/atrás basado en la voz
    int velocidad_pitch = (int)pid_output; 
    
    char comandoRC[50];
    sprintf(comandoRC, "rc 0 %d 0 0", velocidad_pitch); // rc lr fb ud yv
    
    udp.beginPacket(telloIP, telloPort);
    udp.printf(comandoRC);
    udp.endPacket();
    
    vTaskDelay(100 / portTICK_PERIOD_MS); // Tello recomienda comandos RC al menos cada 100-200ms
  }
}

void setup() {
  Serial.begin(115200);
  
  // Conectar ESP32 a la red del Tello
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado al Tello");
  
  // Crear las tareas de FreeRTOS
  xTaskCreatePinnedToCore(TaskAudioFFT, "AudioFFT", 4096, NULL, 3, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(TaskPID, "PID_Control", 2048, NULL, 2, NULL, 1);   // Core 1
  xTaskCreatePinnedToCore(TaskTelloUDP, "Tello_UDP", 2048, NULL, 3, NULL, 1); // Core 1
}

void loop() {
  // En FreeRTOS, el loop puede quedarse vacío
  vTaskDelete(NULL); 
}