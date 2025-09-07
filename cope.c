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
#define LED 25

// UART to Pi. Handles sending sensor data.
#define UART_ID uart0
#define BAUD_RATE 115200

// Hall sensor inputs.
#define NUM_INPUTS 3
const uint input_pins[NUM_INPUTS] = {13, 14, 15};

// V/Hz linear mapping values.
const float vphz = 0.016;
const float b = 4;

// Velocity calculation parameters.
volatile uint64_t prev_us = 0;
volatile uint64_t current_us = 0;
volatile float velocity_rpm = 0;
volatile uint pulses = 0;

// Throttle input.
volatile float throttle = 0.0;
#define THROTTLE_SENSOR 28

// PWM GPIO.
#define WRAPVAL 5000
// #define CLKDIV 41.67f // 3000 RPM
#define CLKDIV 83.33f // 1500 RPM

#define PWM_TEST 16
uint slice_test;

#define PWM_OUT 18
uint slice_out;

void print_uint(unsigned int value) {
    char buffer[32];  // buffer to hold the string representation of the number
    sprintf(buffer, "%u\n", value); 
    uart_puts(UART_ID, buffer);  
}

void print_float(float value) {
    char buffer[32];  // buffer to hold the string representation of the number
    sprintf(buffer, "%f\n", value); 
    uart_puts(UART_ID, buffer);  
}

void on_pwm_wrap() {
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_TEST));
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
}

void irq_handler(uint gpio, uint32_t events) {    
    // debounce not necessary because only 1 bit changes at a time.
    pulses++;
}

void initialize() {
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);
    
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(0, GPIO_FUNC_UART);  // TX
    gpio_set_function(1, GPIO_FUNC_UART);  // RX

    gpio_set_function(PWM_TEST, GPIO_FUNC_PWM);
    slice_test = pwm_gpio_to_slice_num(PWM_TEST);
    pwm_clear_irq(slice_test);
    pwm_set_irq_enabled(slice_test, true);
    pwm_set_wrap(slice_test, WRAPVAL);
    pwm_set_clkdiv(slice_test, CLKDIV);
    pwm_set_chan_level(slice_test, PWM_CHAN_A, 2500);

    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);
    slice_out = pwm_gpio_to_slice_num(PWM_OUT);
    pwm_clear_irq(slice_out);
    pwm_set_irq_enabled(slice_out, true);
    pwm_set_wrap(slice_out, WRAPVAL);
    pwm_set_clkdiv(slice_out, CLKDIV);
    pwm_set_chan_level(slice_out, PWM_CHAN_A, 0);

    pwm_set_mask_enabled((1u << slice_test) | (1u << slice_out));
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    for (int i = 0; i < NUM_INPUTS; i++) {
        gpio_init(input_pins[i]);
        gpio_set_dir(input_pins[i], GPIO_IN);
        gpio_pull_down(input_pins[i]); // or gpio_pull_up() depending on your wiring -- pls ask kelvin later
    }

    // configure all pins to same irq
    gpio_set_irq_enabled_with_callback(input_pins[0], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &irq_handler);
    gpio_set_irq_enabled(input_pins[1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(input_pins[2], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    adc_init();
    adc_gpio_init(THROTTLE_SENSOR);
    adc_select_input(2);
    throttle = adc_read() / 4095;
}

uint8_t read() {
    uint8_t value = 0;

    for (int i = 0; i < NUM_INPUTS; i++) {
        value <<= 1; 
        value |= gpio_get(input_pins[i]); 
    }

    return value;
}

float read_throttle() {
    adc_select_input(2);
    throttle = adc_read() / 4095;
}

float constrain(float value, float min, float max) {
    if (value > max) { 
        return max; 
    } else if (value < min) {
        return min;
    } else {
        return value;
    }
}

int main() {
    stdio_init_all();
    initialize();

    while (true) {
        uint8_t input_value = read();
        // print_uint8(pulses);

        read_throttle();
        current_us = time_us_64();
        velocity_rpm = ((float)pulses/24) / ((float) (current_us - prev_us) / 60 * 1e-6);
        float max_voltage = velocity_rpm * vphz + b;
        float raw_duty_cycle = (throttle * max_voltage) / 48.0;
        float duty_cycle = constrain(raw_duty_cycle, 0.0, 1.0);
        pwm_set_chan_level(slice_out, PWM_CHAN_A, (int)(duty_cycle*WRAPVAL));

        print_float(velocity_rpm);
        // print_float(throttle);

        prev_us = current_us;
        pulses = 0;

        sleep_ms(50);
    }
}