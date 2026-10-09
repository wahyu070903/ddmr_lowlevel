#include "wheel.h"
#include <PID_v1.h>

Wheel* wheel_instance = nullptr;

double r_setpoint = 0;
double r_input = 0;
double r_output = 0;

double l_setpoint = 0;
double l_input = 0;
double l_output = 0;

double r_Kp = 1;
double r_Ki = 0;
double r_Kd = 0;

double l_Kp = 1;
double l_Ki = 0;
double l_Kd = 0;

PID r_wheel_pid(
    &r_input,
    &r_output,
    &r_setpoint,
    r_Kp,
    r_Ki,
    r_Kd,
    DIRECT
);

PID l_wheel_pid(
    &l_input,
    &l_output,
    &l_setpoint,
    l_Kp,
    l_Ki,
    l_Kd,
    DIRECT
);

float gearbox = 30.0;

void left_encoder_isr()
{
    if (wheel_instance != nullptr)wheel_instance->left_encoder_update();
}

void right_encoder_isr()
{
    if (wheel_instance != nullptr) wheel_instance->right_encoder_update();
}

void Wheel::left_encoder_update()
{
    bool A = digitalRead(ENCODER_LEFT_A);
    bool B = digitalRead(ENCODER_LEFT_B);

    if (A == B)
        left_ticks++;
    else
        left_ticks--;
}

void Wheel::right_encoder_update()
{
    bool A = digitalRead(ENCODER_RIGHT_A);
    bool B = digitalRead(ENCODER_RIGHT_B);

    if (A == B)
        right_ticks--;
    else
        right_ticks++;
}
void Wheel::init_wheel()
{
    pinMode(DRIVER_AIN1, OUTPUT);
    pinMode(DRIVER_AIN2, OUTPUT);
    pinMode(DRIVER_PWMA, OUTPUT);

    pinMode(DRIVER_BIN1, OUTPUT);
    pinMode(DRIVER_BIN2, OUTPUT);
    pinMode(DRIVER_PWMB, OUTPUT);

    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    wheel_instance = this;

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_LEFT_A),
        left_encoder_isr,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_RIGHT_A),
        right_encoder_isr,
        CHANGE
    );

    l_wheel_pid.SetMode(AUTOMATIC);
    r_wheel_pid.SetMode(AUTOMATIC);

    l_wheel_pid.SetOutputLimits(-255, 255);
    r_wheel_pid.SetOutputLimits(-255, 255);

    l_wheel_pid.SetSampleTime(20);
    r_wheel_pid.SetSampleTime(20);
}

long Wheel::encoder_get_left()
{
    noInterrupts();

    long value = left_ticks;

    interrupts();

    return value;
}

long Wheel::encoder_get_right()
{
    noInterrupts();

    long value = right_ticks;

    interrupts();

    return value;
}

void Wheel::encoder_reset()
{
    noInterrupts();

    left_ticks = 0;
    right_ticks = 0;

    interrupts();
}

void Wheel::update_speed()
{
    unsigned long now = millis();

    if (now - last_speed_time < 20)
        return;

    long current_left = encoder_get_left();
    long current_right = encoder_get_right();

    long delta_left = current_left - last_left_ticks;
    long delta_right = current_right - last_right_ticks;

    float dt = (now - last_speed_time) / 1000.0f;

    float left_tick_per_second =
        delta_left / dt;

    float right_tick_per_second =
        delta_right / dt;

    // 20 PPR × 2 edge × 30 gearbox
    float ticks_per_wheel_rev = 20.0f * 2.0f * gearbox;

    left_speed =
        left_tick_per_second * 60.0f / ticks_per_wheel_rev;

    right_speed =
        right_tick_per_second * 60.0f / ticks_per_wheel_rev;

    last_left_ticks = current_left;
    last_right_ticks = current_right;

    last_speed_time = now;
}

float Wheel::r_get_speed()
{
    return right_speed;
}


float Wheel::l_get_speed()
{
    return left_speed;
}

void Wheel::left_motor_drive(int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0)
    {
        digitalWrite(DRIVER_BIN1, LOW);
        digitalWrite(DRIVER_BIN2, HIGH);
        analogWrite(DRIVER_PWMB, pwm);
    }
    else if (pwm < 0)
    {
        digitalWrite(DRIVER_BIN1, HIGH);
        digitalWrite(DRIVER_BIN2, LOW);
        analogWrite(DRIVER_PWMB, -pwm);
    }
    else
    {
        digitalWrite(DRIVER_BIN1, LOW);
        digitalWrite(DRIVER_BIN2, LOW);
        analogWrite(DRIVER_PWMB, 0);
    }
}


void Wheel::right_motor_drive(int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0)
    {
        digitalWrite(DRIVER_AIN1, LOW);
        digitalWrite(DRIVER_AIN2, HIGH);
        analogWrite(DRIVER_PWMA, pwm);
    }
    else if (pwm < 0)
    {
        digitalWrite(DRIVER_AIN1, HIGH);
        digitalWrite(DRIVER_AIN2, LOW);
        analogWrite(DRIVER_PWMA, -pwm);
    }
    else
    {
        digitalWrite(DRIVER_AIN1, LOW);
        digitalWrite(DRIVER_AIN2, LOW);
        analogWrite(DRIVER_PWMA, 0);
    }
}

void Wheel::r_wheel_move(float speed)
{
    r_setpoint = speed;      
    r_input = r_get_speed();

    r_wheel_pid.Compute();

    right_motor_drive( (int)r_output);
}

void Wheel::l_wheel_move(float speed)
{
    l_setpoint = speed;
    l_input = l_get_speed();

    l_wheel_pid.Compute();

    left_motor_drive((int)l_output);
}