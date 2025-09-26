#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "U8glib.h"

// Конфигурация пинов
#define RELAY_PIN PIN_PD6    // PD6
#define VOLTAGE_SENSOR_PIN 0 // A0

// Конфигурация делителя напряжения (12k + 3k)
#define R1 5100.0 // верхний резистор
#define R2 2000.0 // нижний резистор
#define DIVIDER_RATIO (R2 / (R1 + R2))
#define REFERENCE_VOLTAGE 5.0 // опорное напряжение АЦП

// Пороговое напряжение
#define VOLTAGE_THRESHOLD 11.0

// Тайминги
#define MEASUREMENT_INTERVAL 1000 // измерение каждую секунду (мс)
#define RELAY_ON_TIME 300000      // 5 минут (мс)
#define SLEEP_TIME 300000         // 5 минут (мс)

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

unsigned long lastMeasurementTime = 0;
unsigned long relayStartTime = 0;
unsigned long sleepStartTime = 0;
bool relayActive = false;
bool displayActive = true;

u8g_uint_t deltaX = 28;
u8g_uint_t deltaY = 24;

// Пользовательская функция задержки с возможностью прерывания
void delayWithCheck(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        // Можно добавить проверку кнопки или другого прерывания
        delay(100);
    }
}

// Чтение напряжения
float readVoltage()
{
    int adcValue = analogRead(VOLTAGE_SENSOR_PIN);
    float voltage = (adcValue * REFERENCE_VOLTAGE) / 1024.0;
    voltage = voltage / DIVIDER_RATIO; // учитываем делитель
    return voltage;
}

// Отображение напряжения на экране с обратным отсчетом
void displayVoltage(float voltage, unsigned long timeLeft)
{
    u8g.firstPage();
    do
    {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(deltaX + 10, deltaY + 10);
        u8g.print("V: ");
        u8g.print(voltage, 1);

        u8g.setPrintPos(deltaX + 10, deltaY + 20);
        u8g.print("Relay: ");
        u8g.print(relayActive ? "ON" : "OFF");

        u8g.setPrintPos(deltaX + 10, deltaY + 30);
        u8g.print("Mode: Act");

        // Отображение обратного отсчета
        u8g.setPrintPos(deltaX + 10, deltaY + 40);
        u8g.print("off:");

        // Преобразование миллисекунд в минуты и секунды
        unsigned long secondsLeft = timeLeft / 1000;
        unsigned long minutes = secondsLeft / 60;
        unsigned long seconds = secondsLeft % 60;

        // u8g.setPrintPos(deltaX+10, deltaY+50);
        if (minutes > 0)
        {
            u8g.print(minutes);
            u8g.print("m ");
        }
        u8g.print(seconds);
        u8g.print("s");

    } while (u8g.nextPage());
}

// Функция для сна на 5 минут с использованием watchdog таймера
void sleepFor5MinutesSimple()
{
    // Простой цикл задержки (менее эффективно по энергии)
    unsigned long sleepStart = millis();
    while (millis() - sleepStart < SLEEP_TIME)
    {
        // Минимальная активность
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sleep_mode();
        sleep_disable();
        delay(100);
    }
}

void setup(void)
{
    // Настройка пинов
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    // Настройка АЦП
    analogReference(DEFAULT);

    // Инициализация дисплея
    u8g.setFont(u8g_font_6x10);

    lastMeasurementTime = millis();
    sleepStartTime = millis();
    delay(100);
}

void loop(void)
{
    unsigned long currentTime = millis();

    if (displayActive)
    {
        // Расчет оставшегося времени до сна
        unsigned long timeUntilSleep = SLEEP_TIME - (currentTime - sleepStartTime);

        // Проверка, нужно ли производить измерение
        if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL)
        {
            float voltage = readVoltage();
            displayVoltage(voltage, timeUntilSleep);
            lastMeasurementTime = currentTime;

            // Проверка порога напряжения
            if (voltage > VOLTAGE_THRESHOLD && !relayActive)
            {
                digitalWrite(RELAY_PIN, HIGH);
                relayActive = true;
                relayStartTime = currentTime;
            }
        }

        // Проверка времени работы реле
        if (relayActive && (currentTime - relayStartTime >= RELAY_ON_TIME))
        {
            digitalWrite(RELAY_PIN, LOW);
            relayActive = false;
        }

        // Проверка времени активности (5 минут)
        if (currentTime - sleepStartTime >= SLEEP_TIME)
        {
            displayActive = false;

            // Отображение сообщения о переходе в сон
            u8g.firstPage();
            do
            {
                u8g.setFont(u8g_font_6x10);
                u8g.setPrintPos(deltaX + 10, deltaY + 20);
                u8g.print("GO Sleep");
                u8g.setPrintPos(deltaX + 10, deltaY + 30);
                u8g.print("mode...");
            } while (u8g.nextPage());

            delay(1000); // Пауза для отображения сообщения

            // Выключение дисплея
            u8g.firstPage();
            do
            {
            } while (u8g.nextPage());

            // Выключение реле перед сном
            digitalWrite(RELAY_PIN, LOW);
            relayActive = false;

            // Задержка для завершения операций
            delay(1000);

            // Переход в сон на 5 минут
            sleepFor5MinutesSimple();

            // Проснулись
            displayActive = true;
            sleepStartTime = millis();
            lastMeasurementTime = millis();
        }
    }

    delay(100); // Небольшая задержка для стабильности

}
