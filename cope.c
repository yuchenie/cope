#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/time.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define LED 25

#define NUM_INPUTS 3
const uint input_pins[NUM_INPUTS] = {13, 14, 15};
const uint period_us = 50;

#define PWM_TEST 16
uint slice_test;

#define PWM_OUT 6
uint slice_out;

#define ENABLE 7
#define nBRAKE 8

#define HALL_TEST 5
#define THROTTLE_ADC 26

volatile float motor_rpm = 0.0f;
const float RATED_MOTOR_RPM = 3000.0f;
const float RATED_MOTOR_VOLTAGE = 48.0f;
const float MAX_VOLTAGE_AT_STALL = 10.0f;
volatile float throttle = 0.0f;

const int PWM_FREQ = 5000;
const int WRAPVAL = SYS_CLK_HZ / PWM_FREQ - 1;

// if the motor is stationary, then the interrupt needs to be called
// periodically to avoid the interrupt from never being called
// and the motor_rpm from never being updated
struct repeating_timer timer;

void print_uint(unsigned int value)
{
    char buffer[32]; // buffer to hold the string representation of the number
    sprintf(buffer, "%u\n", value);
    uart_puts(UART_ID, buffer);
}

void print_float(float value)
{
    char buffer[32]; // buffer to hold the string representation of the number
    sprintf(buffer, "%f\n", value);
    uart_puts(UART_ID, buffer);
}

// time of last irq
volatile uint32_t irq_prev_time = 0;

void irq_handler(uint gpio, uint32_t events)
{
    int irq_current_time = time_us_64();
    // time between steps in microseconds
    int step_period = irq_current_time - irq_prev_time;
    irq_prev_time = irq_current_time;

    // step/us * 1 elec. rev/6 steps * 1 mech. rev/4 elec. rev * 1e6 us/s * 60 s/min
    // = 2.5e6 rpm
    motor_rpm = 2.5e6f / step_period; // in RPM

    // reset the timer to call this function again in 100ms if no step is detected
    cancel_repeating_timer(&timer);
    add_repeating_timer_ms(100, (repeating_timer_callback_t)irq_handler, NULL, &timer);
}

// Convert a duty cycle percentage (0 to 1) to a level value (0 to PWM_WRAP)
int duty_cycle_to_level(float duty_cycle)
{
    if (duty_cycle < 0.0f)
        duty_cycle = 0.0f;
    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;
    return (int)(duty_cycle * WRAPVAL + 1);
}

int adc_deadzone(int adc_value)
{
    const int DEADZONE = 100; // adc units

    if (adc_value < DEADZONE)
        return 0;
    if (adc_value > 4095 - DEADZONE)
        return 4095;
    adc_value = (adc_value - DEADZONE) * 4095 / (4095 - DEADZONE);
    return adc_value;
}

void initialize()
{
    // Turn on onboard LED
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    // Test PWM output to imitate hall sensors:
    gpio_set_function(HALL_TEST, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(HALL_TEST);
    float test_rpm = 1500.0f;
    int step_period_clks = (int)((312.5e6f / test_rpm / 256.0 * 2) - 1);
    // set clock divider to 256
    pwm_set_clkdiv(slice_num, 256.0f);
    pwm_set_wrap(slice_num, step_period_clks);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, step_period_clks / 2);
    pwm_set_enabled(slice_num, true);

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(0, GPIO_FUNC_UART); // TX
    gpio_set_function(1, GPIO_FUNC_UART); // RX

    gpio_set_function(PWM_TEST, GPIO_FUNC_PWM);
    slice_test = pwm_gpio_to_slice_num(PWM_TEST);
    pwm_set_wrap(slice_test, WRAPVAL);
    pwm_set_chan_level(slice_test, PWM_CHAN_A, duty_cycle_to_level(0.25));
    pwm_set_enabled(slice_test, true);

    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);
    slice_out = pwm_gpio_to_slice_num(PWM_OUT);
    pwm_set_wrap(slice_out, WRAPVAL);
    pwm_set_chan_level(slice_out, PWM_CHAN_A, 0);
    pwm_set_enabled(slice_out, true);

    // set enable and nBRAKE pins to HIGH
    gpio_init(ENABLE);
    gpio_set_dir(ENABLE, GPIO_OUT);
    gpio_put(ENABLE, 1);

    gpio_init(nBRAKE);
    gpio_set_dir(nBRAKE, GPIO_OUT);
    gpio_put(nBRAKE, 1);

    for (int i = 0; i < NUM_INPUTS; i++)
    {
        gpio_init(input_pins[i]);
        gpio_set_dir(input_pins[i], GPIO_IN);
    }

    // configure all pins to same irq
    gpio_set_irq_enabled_with_callback(input_pins[0], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &irq_handler);
    gpio_set_irq_enabled(input_pins[1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(input_pins[2], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // add a timer that will call the irq_handler if no step has been detected for 100ms
    add_repeating_timer_ms(100, (repeating_timer_callback_t)irq_handler, NULL, &timer);

    // configure adc for throttle input
    gpio_init(THROTTLE_ADC);
    gpio_set_dir(THROTTLE_ADC, GPIO_IN);
    adc_init();
    adc_gpio_init(THROTTLE_ADC);
    adc_select_input(0);
}

int main()
{
    stdio_init_all();
    initialize();

    while (true)
    {
        throttle = (float)adc_deadzone(adc_read()) / 4095.0f;

        // Calculate duty cycle
        float duty = throttle * (motor_rpm / RATED_MOTOR_RPM + MAX_VOLTAGE_AT_STALL / RATED_MOTOR_VOLTAGE);
        print_float(duty);
        // Write float to PWM output
        pwm_set_chan_level(slice_out, PWM_CHAN_A, duty_cycle_to_level(duty));
    }
}