#include "encoder.h"

#include "driver/gpio.h"

constexpr float CPR = 1024.0f;   // 512 PPR × 2逓倍
constexpr float DT  = 0.01f;     // 10 ms

constexpr int RPM_AVG_SIZE = 3;  // 移動平均点数

static int encoder_pin_a;
static int encoder_pin_b;

static volatile int32_t encoder_count = 0;

static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;


// 移動平均用
static float rpm_buffer[RPM_AVG_SIZE] = {};
static int rpm_buffer_index = 0;
static int rpm_sample_count = 0;


void ARDUINO_ISR_ATTR Encoder_ISR()
{
    bool a = gpio_get_level((gpio_num_t)encoder_pin_a);
    bool b = gpio_get_level((gpio_num_t)encoder_pin_b);

    portENTER_CRITICAL_ISR(&encoder_mux);

    if (a != b)
    {
        encoder_count++;
    }
    else
    {
        encoder_count--;
    }

    portEXIT_CRITICAL_ISR(&encoder_mux);
}


void Encoder_begin(int pin_a, int pin_b)
{
    encoder_pin_a = pin_a;
    encoder_pin_b = pin_b;

    pinMode(encoder_pin_a, INPUT);
    pinMode(encoder_pin_b, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(encoder_pin_a),
        Encoder_ISR,
        CHANGE
    );
}


int32_t Encoder_getCount()
{
    int32_t count;

    portENTER_CRITICAL(&encoder_mux);

    count = encoder_count;
    encoder_count = 0;

    portEXIT_CRITICAL(&encoder_mux);

    return count;
}


float Encoder_getRPM()
{
    int32_t count = Encoder_getCount();

    // 今回の10ms区間のRPM
    float rpm =
        ((float)count / CPR) * (60.0f / DT);


    // リングバッファへ格納
    rpm_buffer[rpm_buffer_index] = rpm;

    rpm_buffer_index++;

    if (rpm_buffer_index >= RPM_AVG_SIZE)
    {
        rpm_buffer_index = 0;
    }


    // 起動直後は実際に取得済みのデータ数だけで平均する
    if (rpm_sample_count < RPM_AVG_SIZE)
    {
        rpm_sample_count++;
    }


    // 移動平均
    float sum = 0.0f;

    for (int i = 0; i < rpm_sample_count; i++)
    {
        sum += rpm_buffer[i];
    }

    return sum / rpm_sample_count;
}