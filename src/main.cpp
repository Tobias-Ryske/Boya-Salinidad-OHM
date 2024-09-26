
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pines para tu configuración
const int pwmPin = 26;       // Pin que genera la onda cuadrada
const int adcPin = 27;       // Pin de lectura del sensor de conductividad
const float Vcc = 1.65;      // Voltaje de referencia máximo
const int pwmFreq = 1000;    // Frecuencia de la señal PWM
const int pwmChannel = 0;    // Canal del PWM
const int pwmResolution = 8; // Resolución del PWM
const float dutyCycle = 0.5; // Ciclo útil de la onda cuadrada (50%)
const float R_serie = 10000.0; // Resistencia de 10kΩ en serie
const float umbralVoltaje = 0.01; // Umbral para descartar mediciones de ruido
const float K = 1800.0;    // Constante de la celda, ajusta según tu sistema

// Configuración del sensor de temperatura DS18B20
const int tempPin = 25;      // Pin del sensor de temperatura DS18B20
OneWire oneWire(tempPin);
DallasTemperature sensors(&oneWire);

// Variables globales
float voltage, temperature, R_liquido, conductividad_mS_cm;
float T_ref = 25.0;
float alpha = 0.02;

// Enumeración para facilitar la identificación de salmueras
enum Salmuera {
  SALMUERA_50,
  SALMUERA_80,
  SALMUERA_100,
  SALMUERA_150,
  SALMUERA_300,
  SALMUERA_400,
  SALMUERA_DESCONOCIDA
};

// Función para determinar la salmuera basada en la conductividad
Salmuera determinarSalmuera(float conductividad_mS_cm) {
  if (conductividad_mS_cm >= 70.0 && conductividad_mS_cm < 90.0) {
    return SALMUERA_50;
  } else if (conductividad_mS_cm >= 90.0 && conductividad_mS_cm < 115.15) {
    return SALMUERA_80;
  } else if (conductividad_mS_cm >= 115.15 && conductividad_mS_cm < 145.75) {
    return SALMUERA_100;
  } else if (conductividad_mS_cm >= 145.75 && conductividad_mS_cm < 197.45) {
    return SALMUERA_150;
  } else if (conductividad_mS_cm >= 197.45 && conductividad_mS_cm < 232.0) {
    return SALMUERA_300;
  } else if (conductividad_mS_cm >= 232.0 && conductividad_mS_cm <= 255.0) {
    return SALMUERA_400;
  } else {
    return SALMUERA_DESCONOCIDA;
  }
}

// Función para la configuración del PWM
void setupPWM() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  ledcWrite(pwmChannel, 128);  // 50% ciclo útil
}

// Función para aplicar la corrección dinámica sumando en lugar de restar
float corregirConductividad(float conductividad_mS_cm) {
  // Aplicar correcciones en función de la conductividad
  if (conductividad_mS_cm >= 82.0 && conductividad_mS_cm <= 93.0) { // Cerca de 50 g/l
    return conductividad_mS_cm - 6.0;  // + 1.1 Sumar 1.1 ms/cm según tu tabla
  } else if (conductividad_mS_cm >= 93.0 && conductividad_mS_cm <= 100.0) { // Cerca de 80 g/l
    return conductividad_mS_cm + 12.3;  // Sumar 12.3 ms/cm según el error en 80 g/l
  } else if (conductividad_mS_cm >= 99.0 && conductividad_mS_cm <= 110.0) { // Cerca de 100 g/l
    return conductividad_mS_cm + 22.0;  // Sumar 26.9 ms/cm para 100 g/l
  } else if (conductividad_mS_cm >= 110.0 && conductividad_mS_cm <= 119.0) { // Cerca de 150 g/l
    return conductividad_mS_cm + 50.0;  // Sumar 57.5 ms/cm según tu tabla
  } else if (conductividad_mS_cm >= 120.0 && conductividad_mS_cm <= 125.0) { // Cerca de 300 g/l
    return conductividad_mS_cm + 110.2;  // Sumar 110.2 ms/cm según el error de 300 g/l
  } else if (conductividad_mS_cm >= 125.0 && conductividad_mS_cm <= 135.0) { // Cerca de 400 g/l
    return conductividad_mS_cm + 117.1;  // Sumar 117.1 ms/cm según tu tabla
  } else {
    // Si no se encuentra en ninguno de los rangos definidos, no se aplica corrección
    return conductividad_mS_cm;
  }
} 

// Tarea para leer el ADC
void TaskLeerADC(void *pvParameters) {
  while (1) {
    int adcValue = analogRead(adcPin);
    voltage = (adcValue / 2047.5) * Vcc * dutyCycle;
    vTaskDelay(pdMS_TO_TICKS(1000));  // Leer cada 1 segundo
  }
}

// Tarea para leer la temperatura
void TaskLeerTemperatura(void *pvParameters) {
  while (1) {
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Leer cada 1 segundo
  }
}

// Tarea para realizar la compensación de temperatura y cálculo de conductividad
void TaskCalculoConductividad(void *pvParameters) {
  while (1) {
    // Compensación de temperatura
    float compensacionTemperatura = 1 + alpha * (temperature - T_ref);
    float voltageCompensado = voltage / compensacionTemperatura;

    // Calcular la resistencia del líquido
    if (voltageCompensado < umbralVoltaje) {
      R_liquido = INFINITY;
    } else {
      R_liquido = (Vcc - voltageCompensado) / voltageCompensado * R_serie;
    }

    // Calcular la conductividad
    if (R_liquido != INFINITY) {
      float conductividad_S_m = (1.0 / R_liquido) * K;
      conductividad_mS_cm = conductividad_S_m * 1e6 / 1000.0;

      // Aplicar corrección dinámica sumando los valores
      conductividad_mS_cm = corregirConductividad(conductividad_mS_cm);
    } else {
      conductividad_mS_cm = 0.0;  // Si está fuera del agua, la conductividad es cero
    }

    // Determinar la salmuera basada en la conductividad corregida
    Salmuera salmuera = determinarSalmuera(conductividad_mS_cm);

    // Verificar si la salmuera está dentro de los rangos definidos (50 g/l a 400 g/l)
    if (salmuera != SALMUERA_DESCONOCIDA) {
      // Imprimir resultados
      Serial.print("Temperatura: ");
      Serial.print(temperature, 1);
      Serial.println(" °C");
      Serial.print("Conductividad: ");
      Serial.print(conductividad_mS_cm, 1);
      Serial.println(" mS/cm");

      Serial.print("Salmuera asignada: ");
      switch(salmuera) {
        case SALMUERA_50:
          Serial.println("50 g/l");
          break;
        case SALMUERA_80:
          Serial.println("80 g/l");
          break;
        case SALMUERA_100:
          Serial.println("100 g/l");
          break;
        case SALMUERA_150:
          Serial.println("150 g/l");
          break;
        case SALMUERA_300:
          Serial.println("300 g/l");
          break;
        case SALMUERA_400:
          Serial.println("400 g/l");
          break;
        default:
          Serial.println("Salmuera desconocida");
          break;
      }
    }
    // Si la salmuera es desconocida (<50 g/l o >400 g/l), no se hace nada

    vTaskDelay(pdMS_TO_TICKS(1000));  // Calcular cada 1 segundo
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(tempPin, INPUT_PULLUP);
  sensors.begin();
  setupPWM();

  // Crear las tareas de FreeRTOS
  xTaskCreatePinnedToCore(TaskLeerADC, "Leer ADC", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskLeerTemperatura, "Leer Temperatura", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCalculoConductividad, "Calcular Conductividad", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // El loop principal queda vacío ya que todo está gestionado por FreeRTOS
}

