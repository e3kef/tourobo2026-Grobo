#include <Arduino.h>

// ============================================================
// Potentiometer
// ============================================================

constexpr int POT_GET1_PIN = 32;
constexpr int POT_GET2_PIN = 33;

// ============================================================
// Motor pins
// ============================================================

// GET1
constexpr int GET1_PWM_PIN = 14;
constexpr int GET1_DIR_PIN = 13;

// GET2
constexpr int GET2_PWM_PIN = 26;
constexpr int GET2_DIR_PIN = 25;

// ============================================================
// PWM
// ============================================================

constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 10;

constexpr int GET1_PWM_CHANNEL = 0;
constexpr int GET2_PWM_CHANNEL = 1;

// 現在の本実装と同じ上限
// 10bit: 0～1023
constexpr int TEST_DUTY = 102;

// 1回の動作時間
constexpr uint32_t PULSE_MS = 500;

// ============================================================
// Pot limits
// ============================================================

constexpr int GET1_OPEN_LIMIT  = 1480;
constexpr int GET1_CLOSE_LIMIT = 1960;

constexpr int GET2_OPEN_LIMIT  = 2220;
constexpr int GET2_CLOSE_LIMIT = 1725;

// ADC平均回数
constexpr int ADC_AVG_COUNT = 4;

// ============================================================
// Direction
// ============================================================

enum class MoveDirection
{
    OPEN,
    CLOSE
};

// 現在の本実装の MOTOR_SIGN を反映すると
// GET1 / GET2 ともに
//
// DIR HIGH = OPEN
// DIR LOW  = CLOSE
//
// になる。

// ============================================================
// Function prototype
// ============================================================

int readPotAverage(int pin);

bool isLimitBlocked(
    int position,
    int open_limit,
    int close_limit,
    MoveDirection direction
);

void pulseMotor(
    const char *name,
    int pwm_channel,
    int dir_pin,
    int pot_pin,
    int open_limit,
    int close_limit,
    MoveDirection direction
);

void pulseBoth(
    MoveDirection direction
);

void stopAllMotors();

void printPotValues();

void printMenu();

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    // --------------------------------------------------------
    // ADC
    // --------------------------------------------------------

    analogReadResolution(12);

    pinMode(POT_GET1_PIN, INPUT);
    pinMode(POT_GET2_PIN, INPUT);

    // --------------------------------------------------------
    // DIR
    // --------------------------------------------------------

    pinMode(GET1_DIR_PIN, OUTPUT);
    pinMode(GET2_DIR_PIN, OUTPUT);

    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    ledcSetup(
        GET1_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        GET2_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        GET1_PWM_PIN,
        GET1_PWM_CHANNEL
    );

    ledcAttachPin(
        GET2_PWM_PIN,
        GET2_PWM_CHANNEL
    );

    // 起動時は必ず停止
    stopAllMotors();

    Serial.println();
    Serial.println("==============================");
    Serial.println(" GET PID-less motor test");
    Serial.println("==============================");

    printPotValues();
    printMenu();
}

// ============================================================
// Loop
// ============================================================

void loop()
{
    if (!Serial.available())
    {
        delay(10);
        return;
    }

    char command = Serial.read();

    // 改行は無視
    if (command == '\n' ||
        command == '\r')
    {
        return;
    }

    switch (command)
    {
        // ----------------------------------------------------
        // GET1
        // ----------------------------------------------------

        case '1':
            pulseMotor(
                "GET1",
                GET1_PWM_CHANNEL,
                GET1_DIR_PIN,
                POT_GET1_PIN,
                GET1_OPEN_LIMIT,
                GET1_CLOSE_LIMIT,
                MoveDirection::OPEN
            );
            break;

        case '2':
            pulseMotor(
                "GET1",
                GET1_PWM_CHANNEL,
                GET1_DIR_PIN,
                POT_GET1_PIN,
                GET1_OPEN_LIMIT,
                GET1_CLOSE_LIMIT,
                MoveDirection::CLOSE
            );
            break;

        // ----------------------------------------------------
        // GET2
        // ----------------------------------------------------

        case '3':
            pulseMotor(
                "GET2",
                GET2_PWM_CHANNEL,
                GET2_DIR_PIN,
                POT_GET2_PIN,
                GET2_OPEN_LIMIT,
                GET2_CLOSE_LIMIT,
                MoveDirection::OPEN
            );
            break;

        case '4':
            pulseMotor(
                "GET2",
                GET2_PWM_CHANNEL,
                GET2_DIR_PIN,
                POT_GET2_PIN,
                GET2_OPEN_LIMIT,
                GET2_CLOSE_LIMIT,
                MoveDirection::CLOSE
            );
            break;

        // ----------------------------------------------------
        // Both
        // ----------------------------------------------------

        case '5':
            pulseBoth(
                MoveDirection::OPEN
            );
            break;

        case '6':
            pulseBoth(
                MoveDirection::CLOSE
            );
            break;

        // ----------------------------------------------------
        // Stop
        // ----------------------------------------------------

        case '0':
            stopAllMotors();

            Serial.println(
                "STOP"
            );
            break;

        // ----------------------------------------------------
        // Pot print
        // ----------------------------------------------------

        case 'p':
        case 'P':
            printPotValues();
            break;

        default:
            Serial.println(
                "Unknown command"
            );

            printMenu();
            break;
    }
}

// ============================================================
// Read potentiometer
// ============================================================

int readPotAverage(int pin)
{
    uint32_t sum = 0;

    for (int i = 0;
         i < ADC_AVG_COUNT;
         i++)
    {
        sum += analogRead(pin);
    }

    return static_cast<int>(
        sum / ADC_AVG_COUNT
    );
}

// ============================================================
// Limit
// ============================================================

bool isLimitBlocked(
    int position,
    int open_limit,
    int close_limit,
    MoveDirection direction
)
{
    if (direction ==
        MoveDirection::OPEN)
    {
        // OPEN側でADCが小さくなる場合
        if (open_limit <
            close_limit)
        {
            return position <=
                   open_limit;
        }

        // OPEN側でADCが大きくなる場合
        return position >=
               open_limit;
    }

    // CLOSE
    if (close_limit <
        open_limit)
    {
        return position <=
               close_limit;
    }

    return position >=
           close_limit;
}

// ============================================================
// Single motor pulse
// ============================================================

void pulseMotor(
    const char *name,
    int pwm_channel,
    int dir_pin,
    int pot_pin,
    int open_limit,
    int close_limit,
    MoveDirection direction
)
{
    int before =
        readPotAverage(
            pot_pin
        );

    Serial.println();
    Serial.printf(
        "%s %s\n",
        name,
        direction ==
                MoveDirection::OPEN
            ? "OPEN"
            : "CLOSE"
    );

    Serial.printf(
        "before = %d\n",
        before
    );

    // --------------------------------------------------------
    // Hard limit
    // --------------------------------------------------------

    if (isLimitBlocked(
            before,
            open_limit,
            close_limit,
            direction))
    {
        Serial.println(
            "LIMIT BLOCKED"
        );

        ledcWrite(
            pwm_channel,
            0
        );

        return;
    }

    // --------------------------------------------------------
    // Direction
    // 現在の本実装の方向定義
    // --------------------------------------------------------

    if (direction ==
        MoveDirection::OPEN)
    {
        digitalWrite(
            dir_pin,
            HIGH
        );
    }
    else
    {
        digitalWrite(
            dir_pin,
            LOW
        );
    }

    // --------------------------------------------------------
    // Drive
    // --------------------------------------------------------

    ledcWrite(
        pwm_channel,
        TEST_DUTY
    );

    delay(PULSE_MS);

    ledcWrite(
        pwm_channel,
        0
    );

    delay(50);

    // --------------------------------------------------------
    // Result
    // --------------------------------------------------------

    int after =
        readPotAverage(
            pot_pin
        );

    int delta =
        after - before;

    Serial.printf(
        "after  = %d\n",
        after
    );

    Serial.printf(
        "delta  = %+d\n",
        delta
    );

    // --------------------------------------------------------
    // Direction check
    // --------------------------------------------------------

    bool expected_increase;

    if (direction ==
        MoveDirection::OPEN)
    {
        expected_increase =
            open_limit >
            close_limit;
    }
    else
    {
        expected_increase =
            close_limit >
            open_limit;
    }

    if (abs(delta) < 5)
    {
        Serial.println(
            "RESULT: almost no movement"
        );
    }
    else if (
        (expected_increase &&
         delta > 0) ||
        (!expected_increase &&
         delta < 0))
    {
        Serial.println(
            "RESULT: direction OK"
        );
    }
    else
    {
        Serial.println(
            "RESULT: !!! DIRECTION REVERSED !!!"
        );
    }
}

// ============================================================
// Both motors
// ============================================================

void pulseBoth(
    MoveDirection direction
)
{
    int get1_before =
        readPotAverage(
            POT_GET1_PIN
        );

    int get2_before =
        readPotAverage(
            POT_GET2_PIN
        );

    bool get1_block =
        isLimitBlocked(
            get1_before,
            GET1_OPEN_LIMIT,
            GET1_CLOSE_LIMIT,
            direction
        );

    bool get2_block =
        isLimitBlocked(
            get2_before,
            GET2_OPEN_LIMIT,
            GET2_CLOSE_LIMIT,
            direction
        );

    Serial.println();

    Serial.printf(
        "BOTH %s\n",
        direction ==
                MoveDirection::OPEN
            ? "OPEN"
            : "CLOSE"
    );

    Serial.printf(
        "GET1 before=%d block=%d\n",
        get1_before,
        get1_block
    );

    Serial.printf(
        "GET2 before=%d block=%d\n",
        get2_before,
        get2_block
    );

    // Direction
    int dir_level =
        direction ==
                MoveDirection::OPEN
            ? HIGH
            : LOW;

    digitalWrite(
        GET1_DIR_PIN,
        dir_level
    );

    digitalWrite(
        GET2_DIR_PIN,
        dir_level
    );

    // リミットに入っていない方だけ動かす
    if (!get1_block)
    {
        ledcWrite(
            GET1_PWM_CHANNEL,
            TEST_DUTY
        );
    }

    if (!get2_block)
    {
        ledcWrite(
            GET2_PWM_CHANNEL,
            TEST_DUTY
        );
    }

    delay(PULSE_MS);

    stopAllMotors();

    delay(50);

    int get1_after =
        readPotAverage(
            POT_GET1_PIN
        );

    int get2_after =
        readPotAverage(
            POT_GET2_PIN
        );

    Serial.printf(
        "GET1 after=%d delta=%+d\n",
        get1_after,
        get1_after -
            get1_before
    );

    Serial.printf(
        "GET2 after=%d delta=%+d\n",
        get2_after,
        get2_after -
            get2_before
    );
}

// ============================================================
// Stop
// ============================================================

void stopAllMotors()
{
    ledcWrite(
        GET1_PWM_CHANNEL,
        0
    );

    ledcWrite(
        GET2_PWM_CHANNEL,
        0
    );
}

// ============================================================
// Debug
// ============================================================

void printPotValues()
{
    int get1 =
        readPotAverage(
            POT_GET1_PIN
        );

    int get2 =
        readPotAverage(
            POT_GET2_PIN
        );

    Serial.printf(
        "POT GET1=%d  GET2=%d\n",
        get1,
        get2
    );
}

void printMenu()
{
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  1 : GET1 OPEN");
    Serial.println("  2 : GET1 CLOSE");
    Serial.println("  3 : GET2 OPEN");
    Serial.println("  4 : GET2 CLOSE");
    Serial.println("  5 : BOTH OPEN");
    Serial.println("  6 : BOTH CLOSE");
    Serial.println("  0 : STOP");
    Serial.println("  p : print potentiometers");
    Serial.println();
}