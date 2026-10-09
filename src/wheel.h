#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>

#define DRIVER_AIN1 9
#define DRIVER_AIN2 10
#define DRIVER_PWMA 11

#define DRIVER_BIN1 8
#define DRIVER_BIN2 7
#define DRIVER_PWMB 6

#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 4

#define ENCODER_RIGHT_A 3
#define ENCODER_RIGHT_B 5


class Wheel{
    private:
        volatile long left_ticks = 0;
        volatile long right_ticks = 0;
        volatile long last_left_ticks = 0;
        volatile long last_right_ticks = 0;

        unsigned long last_speed_time = 0;

        float left_speed = 0;
        float right_speed = 0;

        void right_motor_drive(int);
        void left_motor_drive(int);

    public:
        void init_wheel();

        long encoder_get_left();
        long encoder_get_right();

        void encoder_reset();
        void left_encoder_update();
        void right_encoder_update();

        void r_wheel_move(float);
        void l_wheel_move(float);

        void update_speed();
        float r_get_speed();
        float l_get_speed();
};


void left_encoder_isr();
void right_encoder_isr();

#endif