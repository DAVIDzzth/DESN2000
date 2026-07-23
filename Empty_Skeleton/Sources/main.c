#include "../Headers/stm32f103xe.h"
#include "../LCD/ili9341.h"
#include "../LCD/lcd_hw.h"
#include "../LCD/lcd_grph.h"

/*
 * APE home automation hardware test.
 *
 * R1302 controls one blind at a time.
 * SW1 selects which blind is controlled:
 *   SW1 low  -> Blind 1
 *   SW1 high -> Blind 2
 *
 * SW0 controls the Smart Plug state shown on the LCD.
 * PB0 starts a three-ring doorbell sequence.
 * PB2-PB5 control LED8-LED11 through the I2C GPIO expander.
 * The PA2 light sensor controls LED0-LED7:
 *   bright environment -> LED0-LED7 off
 *   dark environment   -> LED0-LED7 on
 *
 * Clock assumption: SystemInit() leaves the MCU at the reset-default
 * 8 MHz HSI clock.
 */

/* PI4IOE5V9554A I2C expander addresses and registers. */
#define EXPANDER_WRITE_ADDRESS       0x70U
#define EXPANDER_READ_ADDRESS        0x71U
#define EXPANDER_INPUT_REG           0x00U
#define EXPANDER_OUTPUT_REG          0x01U
#define EXPANDER_POLARITY_REG        0x02U
#define EXPANDER_CONFIG_REG          0x03U
#define I2C_TIMEOUT_COUNT            100000U
#define ADC_TIMEOUT_COUNT            100000U
#define EXPANDER_RETRY_MS            500U
#define I2C_ERROR_MASK               (I2C_SR1_BERR | I2C_SR1_ARLO | \
                                      I2C_SR1_AF | I2C_SR1_OVR | \
                                      I2C_SR1_TIMEOUT)

/* Direct GPIO inputs. */
#define PB0_MASK                     (1U << 0)
#define SW0_MASK                     (1U << 6)
#define SW1_MASK                     (1U << 7)

/* ADC and blind settings. */
#define R1302_ADC_CHANNEL            1U
#define LIGHT_SENSOR_ADC_CHANNEL     2U
#define BLIND_REVERSED               0U

/*
 * Ambient-light thresholds.
 * Values below 150 are treated as dark conditions.
 * Values above 160 are treated as bright conditions.
 * The gap between the thresholds prevents LED flicker.
 */
#define LIGHT_DARK_THRESHOLD         110U
#define LIGHT_BRIGHT_THRESHOLD       120U
#define LED0_7_MASK                  0x00FFU

/* Speaker and doorbell settings. */
#define DAC_MIDPOINT                 2048U
#define DOORBELL_AMPLITUDE           1400U
#define TONE_2000_HZ_ARR             249U
#define TONE_1700_HZ_ARR             293U
#define DOORBELL_HIGH_TONE_MS        200U
#define DOORBELL_LOW_TONE_MS         200U
#define DOORBELL_GAP_MS              500U
#define DOORBELL_REPEAT_COUNT        3U
#define BUTTON_DEBOUNCE_MS           30U

/* LCD layout settings. */
#define BAR_X0                       20U
#define BAR_X1                       220U
#define BAR_HEIGHT                   22U
#define BAR1_Y0                      60U
#define BAR2_Y0                      126U

/* Doorbell states. */
#define DOORBELL_IDLE                0U
#define DOORBELL_HIGH_TONE           1U
#define DOORBELL_LOW_TONE            2U
#define DOORBELL_GAP                 3U

/* Variables shared with the TIM2 interrupt handler. */
static volatile unsigned short g_dac_amplitude = DOORBELL_AMPLITUDE;
static volatile unsigned char g_dac_phase = 0U;
static volatile unsigned char g_audio_enabled = 0U;

/* Doorbell state machine variables. */
static unsigned char g_doorbell_state = DOORBELL_IDLE;
static unsigned char g_doorbell_repeats_completed = 0U;
static unsigned short g_doorbell_state_start = 0U;

/* Function declarations. */
static void short_delay(volatile unsigned int count);

static void gpio_inputs_init(void);
static void gpio_led0_7_init(void);
static void led0_7_set(unsigned char turn_on);
static unsigned char update_ambient_lighting(unsigned int light_adc);
static unsigned char pb0_pressed_event(unsigned short now);
static unsigned char sw0_is_on(void);
static unsigned char sw1_selects_blind2(void);

static void timebase_init(void);
static unsigned short time_ms(void);
static unsigned short elapsed_ms(unsigned short start, unsigned short now);

static void i2c1_init(void);
static void i2c1_bus_recover(void);
static unsigned char i2c_wait_sr1_set(unsigned int mask);
static unsigned char i2c_wait_sr2_clear(unsigned int mask);
static unsigned char i2c1_write_register(unsigned char reg, unsigned char value);
static unsigned char i2c1_read_register(unsigned char reg, unsigned char *value);
static unsigned char expander_init(void);
static unsigned char update_pb2_to_led8_control(unsigned char *button_bits);

static void adc1_init(void);
static unsigned short adc1_read_channel(unsigned char channel);
static unsigned char adc_to_percent(unsigned int adc_value);

static void dac1_speaker_init(void);
static void tim2_audio_init(void);
static void audio_set_frequency_arr(unsigned short arr_value);
static void audio_start(void);
static void audio_stop(void);

static void doorbell_start(unsigned short now);
static void doorbell_update(unsigned short now);
static unsigned char doorbell_is_active(void);

static void lcd_draw_layout(void);
static void lcd_draw_progress_bar(unsigned short y0, unsigned char percent);
static void lcd_draw_percent(unsigned short y, unsigned char percent);
static void lcd_draw_selected_blind(unsigned char blind2_selected);
static void lcd_draw_smart_plug(unsigned char is_on);
static void lcd_draw_doorbell_status(unsigned char is_active);
static void lcd_draw_ambient_status(unsigned char is_night);
static void lcd_draw_light_adc(unsigned short adc_value);
static void lcd_draw_button_bits(unsigned char button_bits,
                                 unsigned char expander_ok);
static void value_to_4digits(unsigned short value, unsigned char *buffer);
static void percent_to_string(unsigned char percent, unsigned char *buffer);

int main(void)
{
    unsigned short adc_raw;
    unsigned int adc_filtered;
    unsigned short light_raw;
    unsigned int light_filtered;
    unsigned char ambient_is_night;
    unsigned char previous_ambient_is_night;
    unsigned char knob_percent;
    unsigned char blind1_percent;
    unsigned char blind2_percent;
    unsigned char previous_blind1_percent;
    unsigned char previous_blind2_percent;
    unsigned char blind2_selected;
    unsigned char previous_blind2_selected;
    unsigned char smart_plug_on;
    unsigned char previous_smart_plug_on;
    unsigned char doorbell_active;
    unsigned char previous_doorbell_active;
    unsigned char button_bits;
    unsigned char previous_button_bits;
    unsigned char expander_ok;
    unsigned char previous_expander_ok;
    unsigned short expander_retry_time;
    unsigned short previous_light_display;
    unsigned short now;

    adc_filtered = 0U;
    light_filtered = 0U;
    ambient_is_night = 0U;
    previous_ambient_is_night = 255U;
    blind1_percent = 0U;
    blind2_percent = 0U;
    previous_blind1_percent = 255U;
    previous_blind2_percent = 255U;
    previous_blind2_selected = 255U;
    previous_smart_plug_on = 255U;
    previous_doorbell_active = 255U;
    button_bits = 0U;
    previous_button_bits = 255U;
    expander_ok = 0U;
    previous_expander_ok = 255U;
    expander_retry_time = 0U;
    previous_light_display = 0xFFFFU;

    /*
     * Initialize all direct MCU peripherals first.
     * These features must continue to work even if the I2C expander is absent
     * or the I2C bus is temporarily stuck.
     */
    gpio_inputs_init();
    gpio_led0_7_init();
    adc1_init();
    dac1_speaker_init();
    tim2_audio_init();
    timebase_init();

    /*
     * Initialize the LCD before I2C1. The LCD driver changes GPIOB settings,
     * so I2C1 is configured afterwards to guarantee that PB6 and PB7 finish
     * in alternate-function open-drain mode.
     */
    lcd_init();
    lcd_draw_layout();

    /* Read all direct inputs and draw the initial live state immediately. */
    adc_filtered = adc1_read_channel(R1302_ADC_CHANNEL);
    light_filtered = adc1_read_channel(LIGHT_SENSOR_ADC_CHANNEL);
    knob_percent = adc_to_percent(adc_filtered);
    blind1_percent = knob_percent;
    blind2_percent = knob_percent;
    ambient_is_night = update_ambient_lighting(light_filtered);
    blind2_selected = sw1_selects_blind2();
    smart_plug_on = sw0_is_on();
    doorbell_active = doorbell_is_active();

    lcd_draw_progress_bar(BAR1_Y0, blind1_percent);
    lcd_draw_percent(40U, blind1_percent);
    lcd_draw_progress_bar(BAR2_Y0, blind2_percent);
    lcd_draw_percent(106U, blind2_percent);
    lcd_draw_selected_blind(blind2_selected);
    lcd_draw_smart_plug(smart_plug_on);
    lcd_draw_doorbell_status(doorbell_active);
    lcd_draw_ambient_status(ambient_is_night);
    lcd_draw_light_adc((unsigned short)light_filtered);

    previous_blind1_percent = blind1_percent;
    previous_blind2_percent = blind2_percent;
    previous_blind2_selected = blind2_selected;
    previous_smart_plug_on = smart_plug_on;
    previous_doorbell_active = doorbell_active;
    previous_ambient_is_night = ambient_is_night;
    previous_light_display = (unsigned short)light_filtered;

    /*
     * Configure the expander last. Every I2C operation has a timeout, so an
     * I2C fault can no longer freeze the rest of the application.
     */
    i2c1_init();
    expander_ok = expander_init();
    expander_retry_time = time_ms();
    lcd_draw_button_bits(button_bits, expander_ok);
    previous_button_bits = button_bits;
    previous_expander_ok = expander_ok;

    while (1) {
        now = time_ms();

        /* Read direct switches first; these do not depend on I2C. */
        blind2_selected = sw1_selects_blind2();
        smart_plug_on = sw0_is_on();

        /* Read and smooth the single physical potentiometer R1302. */
        adc_raw = adc1_read_channel(R1302_ADC_CHANNEL);
        adc_filtered = ((adc_filtered * 7U) + adc_raw) / 8U;
        knob_percent = adc_to_percent(adc_filtered);

        /*
         * Read and smooth the PA2 ambient-light sensor.
         * Dark conditions turn LED0-LED7 on; bright conditions turn them off.
         */
        light_raw = adc1_read_channel(LIGHT_SENSOR_ADC_CHANNEL);
        light_filtered = ((light_filtered * 15U) + light_raw) / 16U;
        ambient_is_night = update_ambient_lighting(light_filtered);

        /* SW1 selects which stored blind position R1302 updates. */
        if (blind2_selected != 0U) {
            blind2_percent = knob_percent;
        } else {
            blind1_percent = knob_percent;
        }

        /* Start one complete three-ring sequence on a PB0 press edge. */
        if (pb0_pressed_event(now) != 0U) {
            if (doorbell_is_active() == 0U) {
                doorbell_start(now);
            }
        }

        /* Keep the doorbell sequence non-blocking. */
        doorbell_update(now);
        doorbell_active = doorbell_is_active();

        /* Update only LCD regions whose values changed. */
        if (blind1_percent != previous_blind1_percent) {
            lcd_draw_progress_bar(BAR1_Y0, blind1_percent);
            lcd_draw_percent(40U, blind1_percent);
            previous_blind1_percent = blind1_percent;
        }

        if (blind2_percent != previous_blind2_percent) {
            lcd_draw_progress_bar(BAR2_Y0, blind2_percent);
            lcd_draw_percent(106U, blind2_percent);
            previous_blind2_percent = blind2_percent;
        }

        if (blind2_selected != previous_blind2_selected) {
            lcd_draw_selected_blind(blind2_selected);
            previous_blind2_selected = blind2_selected;
        }

        if (smart_plug_on != previous_smart_plug_on) {
            lcd_draw_smart_plug(smart_plug_on);
            previous_smart_plug_on = smart_plug_on;
        }

        if (doorbell_active != previous_doorbell_active) {
            lcd_draw_doorbell_status(doorbell_active);
            previous_doorbell_active = doorbell_active;
        }

        if (ambient_is_night != previous_ambient_is_night) {
            lcd_draw_ambient_status(ambient_is_night);
            previous_ambient_is_night = ambient_is_night;
        }

        /*
         * Show live sensor and button values for threshold calibration
         * and GPIO-expander diagnosis.
         */
        if ((unsigned short)light_filtered != previous_light_display) {
            lcd_draw_light_adc((unsigned short)light_filtered);
            previous_light_display = (unsigned short)light_filtered;
        }

        /*
         * Service PB2-PB5 and LED8-LED11 only after every direct input and LCD
         * state has been updated. A failed I2C transaction therefore cannot
         * stop the potentiometer, switches, PB0, light sensor or LCD display.
         */
        if (expander_ok != 0U) {
            expander_ok = update_pb2_to_led8_control(&button_bits);
        } else if (elapsed_ms(expander_retry_time, now) >= EXPANDER_RETRY_MS) {
            i2c1_bus_recover();
            expander_ok = expander_init();
            expander_retry_time = now;
        }

        if ((button_bits != previous_button_bits) ||
            (expander_ok != previous_expander_ok)) {
            lcd_draw_button_bits(button_bits, expander_ok);
            previous_button_bits = button_bits;
            previous_expander_ok = expander_ok;
        }

        short_delay(1500U);
    }
}

static void short_delay(volatile unsigned int count)
{
    while (count > 0U) {
        count--;
    }
}

static void gpio_inputs_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPEEN;

    /*
     * PE0 is PB0. The schematic provides an external pull-down resistor,
     * so PB0 is configured as a floating input and reads high when pressed.
     */
    GPIOE->CRL &= ~(0xFU << 0);
    GPIOE->CRL |=  (0x4U << 0);

    /* PC6 is SW0 and PC7 is SW1. Both switches select 3.3 V or ground. */
    GPIOC->CRL &= ~((0xFU << 24) | (0xFU << 28));
    GPIOC->CRL |=  ((0x4U << 24) | (0x4U << 28));
}

static void gpio_led0_7_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;

    /*
     * PF0-PF7 drive LED0-LED7.
     * Each pin is configured as a 2 MHz general-purpose push-pull output.
     */
    GPIOF->CRL = 0x22222222U;

    /* Start with all automatic-lighting LEDs off. */
    GPIOF->BSRR = ((unsigned int)LED0_7_MASK << 16);
}

static void led0_7_set(unsigned char turn_on)
{
    if (turn_on != 0U) {
        /* Set PF0-PF7 high to turn LED0-LED7 on. */
        GPIOF->BSRR = LED0_7_MASK;
    } else {
        /* Reset PF0-PF7 low to turn LED0-LED7 off. */
        GPIOF->BSRR = ((unsigned int)LED0_7_MASK << 16);
    }
}

static unsigned char update_ambient_lighting(unsigned int light_adc)
{
    static unsigned char is_night = 0U;

    /*
     * Hysteresis prevents rapid toggling when the ADC value is close
     * to the day/night boundary.
     */
    if (is_night == 0U) {
        if (light_adc < LIGHT_DARK_THRESHOLD) {
            is_night = 1U;
        }
    } else {
        if (light_adc > LIGHT_BRIGHT_THRESHOLD) {
            is_night = 0U;
        }
    }

    led0_7_set(is_night);
    return is_night;
}

static unsigned char pb0_pressed_event(unsigned short now)
{
    static unsigned char last_raw_state = 0U;
    static unsigned char stable_state = 0U;
    static unsigned short change_time = 0U;
    unsigned char raw_state;
    unsigned char pressed_event;

    raw_state = ((GPIOE->IDR & PB0_MASK) != 0U) ? 1U : 0U;
    pressed_event = 0U;

    if (raw_state != last_raw_state) {
        last_raw_state = raw_state;
        change_time = now;
    }

    if ((raw_state != stable_state) &&
        (elapsed_ms(change_time, now) >= BUTTON_DEBOUNCE_MS)) {
        stable_state = raw_state;

        if (stable_state != 0U) {
            pressed_event = 1U;
        }
    }

    return pressed_event;
}

static unsigned char sw0_is_on(void)
{
    return ((GPIOC->IDR & SW0_MASK) != 0U) ? 1U : 0U;
}

static unsigned char sw1_selects_blind2(void)
{
    return ((GPIOC->IDR & SW1_MASK) != 0U) ? 1U : 0U;
}

static void timebase_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* 8 MHz divided by 8000 gives one timer count per millisecond. */
    TIM3->PSC = 7999U;
    TIM3->ARR = 0xFFFFU;
    TIM3->CNT = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->CR1 = TIM_CR1_CEN;
}

static unsigned short time_ms(void)
{
    return (unsigned short)TIM3->CNT;
}

static unsigned short elapsed_ms(unsigned short start, unsigned short now)
{
    /* Unsigned subtraction handles the 16-bit timer wrap correctly. */
    return (unsigned short)(now - start);
}

static void i2c1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* PB6 is SCL and PB7 is SDA, alternate-function open-drain, 50 MHz. */
    GPIOB->CRL &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB->CRL |=  ((0xFU << 24) | (0xFU << 28));

    /* Configure standard-mode I2C for an 8 MHz APB1 clock. */
    I2C1->CR1 = 0U;
    I2C1->CR2 = 8U;
    I2C1->CCR = 40U;
    I2C1->TRISE = 9U;
    I2C1->CR1 = I2C_CR1_PE;
}

static void i2c1_bus_recover(void)
{
    unsigned int pulse;

    /*
     * Disable I2C1 and temporarily use PB6/PB7 as open-drain GPIO outputs.
     * Releasing both pins lets the board's 4.7 kOhm pull-ups bring them high.
     */
    I2C1->CR1 &= ~I2C_CR1_PE;

    GPIOB->CRL &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB->CRL |=  ((0x6U << 24) | (0x6U << 28));
    GPIOB->BSRR = (1U << 6) | (1U << 7);
    short_delay(100U);

    /*
     * Nine SCL pulses release a slave that may be waiting for the remainder
     * of an interrupted byte.
     */
    for (pulse = 0U; pulse < 9U; pulse++) {
        GPIOB->BRR = (1U << 6);
        short_delay(100U);
        GPIOB->BSRR = (1U << 6);
        short_delay(100U);
    }

    /* Generate a manual STOP condition: SDA low, SCL high, then SDA high. */
    GPIOB->BRR = (1U << 7);
    short_delay(100U);
    GPIOB->BSRR = (1U << 6);
    short_delay(100U);
    GPIOB->BSRR = (1U << 7);
    short_delay(100U);

    i2c1_init();
}

static unsigned char i2c_wait_sr1_set(unsigned int mask)
{
    unsigned int timeout;

    timeout = I2C_TIMEOUT_COUNT;
    while (((I2C1->SR1 & mask) == 0U) && (timeout > 0U)) {
        if ((I2C1->SR1 & I2C_ERROR_MASK) != 0U) {
            return 0U;
        }

        timeout--;
    }

    return (timeout > 0U) ? 1U : 0U;
}

static unsigned char i2c_wait_sr2_clear(unsigned int mask)
{
    unsigned int timeout;

    timeout = I2C_TIMEOUT_COUNT;
    while (((I2C1->SR2 & mask) != 0U) && (timeout > 0U)) {
        timeout--;
    }

    return (timeout > 0U) ? 1U : 0U;
}

static unsigned char i2c1_write_register(unsigned char reg, unsigned char value)
{
    volatile unsigned int dummy;

    I2C1->SR1 &= ~I2C_ERROR_MASK;

    if (i2c_wait_sr2_clear(I2C_SR2_BUSY) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (i2c_wait_sr1_set(I2C_SR1_SB) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->DR = EXPANDER_WRITE_ADDRESS;
    if (i2c_wait_sr1_set(I2C_SR1_ADDR) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;

    if (i2c_wait_sr1_set(I2C_SR1_TXE) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }
    I2C1->DR = reg;

    if (i2c_wait_sr1_set(I2C_SR1_TXE) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }
    I2C1->DR = value;

    if (i2c_wait_sr1_set(I2C_SR1_BTF) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1U;
}

static unsigned char i2c1_read_register(unsigned char reg, unsigned char *value)
{
    volatile unsigned int dummy;

    I2C1->SR1 &= ~I2C_ERROR_MASK;

    if (value == 0) {
        return 0U;
    }

    if (i2c_wait_sr2_clear(I2C_SR2_BUSY) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (i2c_wait_sr1_set(I2C_SR1_SB) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->DR = EXPANDER_WRITE_ADDRESS;
    if (i2c_wait_sr1_set(I2C_SR1_ADDR) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;

    if (i2c_wait_sr1_set(I2C_SR1_TXE) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }
    I2C1->DR = reg;

    if (i2c_wait_sr1_set(I2C_SR1_BTF) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (i2c_wait_sr1_set(I2C_SR1_SB) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    I2C1->DR = EXPANDER_READ_ADDRESS;
    if (i2c_wait_sr1_set(I2C_SR1_ADDR) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    /* Perform the STM32F1 single-byte receive sequence atomically. */
    I2C1->CR1 &= ~I2C_CR1_ACK;
    __disable_irq();
    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;
    I2C1->CR1 |= I2C_CR1_STOP;
    __enable_irq();

    if (i2c_wait_sr1_set(I2C_SR1_RXNE) == 0U) {
        i2c1_bus_recover();
        return 0U;
    }

    *value = (unsigned char)I2C1->DR;
    return 1U;
}

static unsigned char expander_init(void)
{
    unsigned char success;

    /*
     * This is the same register sequence as the earlier standalone test:
     * output register 0x00 first, then configuration register 0xF0.
     */
    success = i2c1_write_register(EXPANDER_OUTPUT_REG, 0x00U);

    if (success != 0U) {
        success = i2c1_write_register(EXPANDER_CONFIG_REG, 0xF0U);
    }

    return success;
}

static unsigned char update_pb2_to_led8_control(unsigned char *button_bits)
{
    unsigned char expander_input;
    unsigned char new_button_bits;

    if (button_bits == 0) {
        return 0U;
    }

    if (i2c1_read_register(EXPANDER_INPUT_REG, &expander_input) == 0U) {
        return 0U;
    }

    /*
     * Expander bits 7-4 are PB5, PB4, PB3 and PB2.
     * Shifting them right maps PB2-PB5 directly to LED8-LED11.
     */
    new_button_bits = (unsigned char)((expander_input >> 4) & 0x0FU);

    if (i2c1_write_register(EXPANDER_OUTPUT_REG, new_button_bits) == 0U) {
        return 0U;
    }

    *button_bits = new_button_bits;
    return 1U;
}

static void adc1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    /*
     * PA1 is R1302 on ADC1 channel 1.
     * PA2 is the ambient-light sensor on ADC1 channel 2.
     */
    GPIOA->CRL &= ~((0xFU << 4) | (0xFU << 8));

    /* ADC clock is PCLK2 divided by 2. */
    RCC->CFGR &= ~(3U << 14);

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = R1302_ADC_CHANNEL;

    /* ADC channels 1 and 2 use a 55.5-cycle sample time. */
    ADC1->SMPR2 &= ~((7U << 3) | (7U << 6));
    ADC1->SMPR2 |=  ((5U << 3) | (5U << 6));

    ADC1->CR2 |= ADC_CR2_ADON;
    short_delay(1000U);

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {
    }
}

static unsigned short adc1_read_channel(unsigned char channel)
{
    static unsigned short last_r1302_value = 0U;
    static unsigned short last_light_value = 4095U;
    unsigned int timeout;
    unsigned short value;

    /* Select one ADC channel for the next regular conversion. */
    ADC1->SQR3 = (unsigned int)(channel & 0x1FU);

    /* A second ADON write starts one software-triggered conversion. */
    ADC1->CR2 |= ADC_CR2_ADON;

    timeout = ADC_TIMEOUT_COUNT;
    while (((ADC1->SR & ADC_SR_EOC) == 0U) && (timeout > 0U)) {
        timeout--;
    }

    /*
     * Never block the whole application if an ADC conversion fails.
     * Return the most recent valid value for that channel.
     */
    if (timeout == 0U) {
        if (channel == LIGHT_SENSOR_ADC_CHANNEL) {
            return last_light_value;
        }

        return last_r1302_value;
    }

    value = (unsigned short)(ADC1->DR & 0x0FFFU);

    if (channel == LIGHT_SENSOR_ADC_CHANNEL) {
        last_light_value = value;
    } else {
        last_r1302_value = value;
    }

    return value;
}

static unsigned char adc_to_percent(unsigned int adc_value)
{
    unsigned int percent;

    if (adc_value > 4095U) {
        adc_value = 4095U;
    }

    percent = ((adc_value * 100U) + 2047U) / 4095U;

    if (BLIND_REVERSED != 0U) {
        percent = 100U - percent;
    }

    return (unsigned char)percent;
}

static void dac1_speaker_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* PA4 is DAC channel 1 and is configured in analog mode. */
    GPIOA->CRL &= ~(0xFU << 16);

    DAC->CR = DAC_CR_EN1;
    DAC->DHR12R1 = DAC_MIDPOINT;
}

static void tim2_audio_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* 8 MHz divided by 8 gives a 1 MHz timer counter. */
    TIM2->PSC = 7U;
    TIM2->ARR = TONE_2000_HZ_ARR;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);

    /* Keep TIM2 stopped until the doorbell is active. */
    TIM2->CR1 = 0U;
}

static void audio_set_frequency_arr(unsigned short arr_value)
{
    TIM2->ARR = arr_value;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
}

static void audio_start(void)
{
    g_dac_phase = 0U;
    g_audio_enabled = 1U;
    TIM2->CNT = 0U;
    TIM2->SR = 0U;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void audio_stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    g_audio_enabled = 0U;
    g_dac_phase = 0U;
    DAC->DHR12R1 = DAC_MIDPOINT;
}

static void doorbell_start(unsigned short now)
{
    g_doorbell_state = DOORBELL_HIGH_TONE;
    g_doorbell_repeats_completed = 0U;
    g_doorbell_state_start = now;

    audio_set_frequency_arr(TONE_2000_HZ_ARR);
    audio_start();
}

static void doorbell_update(unsigned short now)
{
    unsigned short state_elapsed;

    state_elapsed = elapsed_ms(g_doorbell_state_start, now);

    if (g_doorbell_state == DOORBELL_HIGH_TONE) {
        if (state_elapsed >= DOORBELL_HIGH_TONE_MS) {
            audio_set_frequency_arr(TONE_1700_HZ_ARR);
            g_doorbell_state = DOORBELL_LOW_TONE;
            g_doorbell_state_start = now;
        }
    } else if (g_doorbell_state == DOORBELL_LOW_TONE) {
        if (state_elapsed >= DOORBELL_LOW_TONE_MS) {
            audio_stop();
            g_doorbell_repeats_completed++;

            if (g_doorbell_repeats_completed >= DOORBELL_REPEAT_COUNT) {
                g_doorbell_state = DOORBELL_IDLE;
            } else {
                g_doorbell_state = DOORBELL_GAP;
                g_doorbell_state_start = now;
            }
        }
    } else if (g_doorbell_state == DOORBELL_GAP) {
        if (state_elapsed >= DOORBELL_GAP_MS) {
            audio_set_frequency_arr(TONE_2000_HZ_ARR);
            audio_start();
            g_doorbell_state = DOORBELL_HIGH_TONE;
            g_doorbell_state_start = now;
        }
    } else {
        g_doorbell_state = DOORBELL_IDLE;
    }
}

static unsigned char doorbell_is_active(void)
{
    return (g_doorbell_state == DOORBELL_IDLE) ? 0U : 1U;
}

void TIM2_IRQHandler(void)
{
    unsigned short amplitude;

    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR &= ~TIM_SR_UIF;

        if (g_audio_enabled != 0U) {
            amplitude = g_dac_amplitude;

            if (g_dac_phase == 0U) {
                DAC->DHR12R1 = DAC_MIDPOINT + amplitude;
                g_dac_phase = 1U;
            } else {
                DAC->DHR12R1 = DAC_MIDPOINT - amplitude;
                g_dac_phase = 0U;
            }
        } else {
            DAC->DHR12R1 = DAC_MIDPOINT;
        }
    }
}

static void lcd_draw_layout(void)
{
    lcd_fillScreen(BLACK);
    lcd_fontColor(WHITE, BLACK);

    lcd_putString(54U, 10U, (unsigned char *)"HOME CONTROL HUB");

    lcd_putString(20U, 40U, (unsigned char *)"Blind 1");
    lcd_drawRect(BAR_X0, BAR1_Y0, BAR_X1, BAR1_Y0 + BAR_HEIGHT, WHITE);

    lcd_putString(20U, 106U, (unsigned char *)"Blind 2");
    lcd_drawRect(BAR_X0, BAR2_Y0, BAR_X1, BAR2_Y0 + BAR_HEIGHT, WHITE);

    lcd_putString(20U, 174U, (unsigned char *)"R1302 controls:");
    lcd_putString(20U, 206U, (unsigned char *)"Smart Plug:");
    lcd_putString(20U, 236U, (unsigned char *)"Doorbell:");
    lcd_putString(20U, 266U, (unsigned char *)"Ambient:");

    lcd_fontColor(LIGHT_GRAY, BLACK);
    lcd_putString(20U, 290U, (unsigned char *)"Light ADC:");
    lcd_putString(20U, 306U, (unsigned char *)"PB2-5:");

    lcd_fontColor(WHITE, BLACK);
}

static void lcd_draw_progress_bar(unsigned short y0, unsigned char percent)
{
    unsigned short inner_x0;
    unsigned short inner_x1;
    unsigned short fill_x1;
    unsigned int fill_width;

    inner_x0 = BAR_X0 + 2U;
    inner_x1 = BAR_X1 - 2U;

    lcd_fillRect(inner_x0, y0 + 2U, inner_x1, y0 + BAR_HEIGHT - 2U, DARK_GRAY);

    fill_width = ((unsigned int)(inner_x1 - inner_x0 + 1U) * percent) / 100U;

    if (fill_width > 0U) {
        fill_x1 = (unsigned short)(inner_x0 + fill_width - 1U);
        lcd_fillRect(inner_x0, y0 + 2U, fill_x1, y0 + BAR_HEIGHT - 2U, CYAN);
    }
}

static void lcd_draw_percent(unsigned short y, unsigned char percent)
{
    unsigned char text[5];

    percent_to_string(percent, text);

    lcd_fillRect(172U, y, 225U, y + 10U, BLACK);
    lcd_fontColor(YELLOW, BLACK);
    lcd_putString(178U, y, text);
    lcd_fontColor(WHITE, BLACK);
}

static void lcd_draw_selected_blind(unsigned char blind2_selected)
{
    lcd_fillRect(126U, 168U, 225U, 188U, BLACK);
    lcd_fontColor(CYAN, BLACK);

    if (blind2_selected != 0U) {
        lcd_putString(132U, 174U, (unsigned char *)"BLIND 2");
    } else {
        lcd_putString(132U, 174U, (unsigned char *)"BLIND 1");
    }

    lcd_fontColor(WHITE, BLACK);
}

static void lcd_draw_smart_plug(unsigned char is_on)
{
    lcd_fillRect(105U, 200U, 225U, 220U, BLACK);

    if (is_on != 0U) {
        lcd_fontColor(GREEN, BLACK);
        lcd_putString(120U, 206U, (unsigned char *)"ON");
    } else {
        lcd_fontColor(RED, BLACK);
        lcd_putString(120U, 206U, (unsigned char *)"OFF");
    }

    lcd_fontColor(WHITE, BLACK);
}

static void lcd_draw_doorbell_status(unsigned char is_active)
{
    lcd_fillRect(105U, 230U, 225U, 250U, BLACK);

    if (is_active != 0U) {
        lcd_fontColor(YELLOW, BLACK);
        lcd_putString(120U, 236U, (unsigned char *)"RINGING");
    } else {
        lcd_fontColor(GREEN, BLACK);
        lcd_putString(120U, 236U, (unsigned char *)"READY");
    }

    lcd_fontColor(WHITE, BLACK);
}


static void lcd_draw_ambient_status(unsigned char is_night)
{
    lcd_fillRect(86U, 260U, 225U, 280U, BLACK);

    if (is_night != 0U) {
        lcd_fontColor(YELLOW, BLACK);
        lcd_putString(92U, 266U, (unsigned char *)"NIGHT - ON");
    } else {
        lcd_fontColor(CYAN, BLACK);
        lcd_putString(92U, 266U, (unsigned char *)"DAY - OFF");
    }

    lcd_fontColor(WHITE, BLACK);
}

static void lcd_draw_light_adc(unsigned short adc_value)
{
    unsigned char buffer[5];

    value_to_4digits(adc_value, buffer);
    lcd_fillRect(92U, 286U, 150U, 302U, BLACK);
    lcd_fontColor(WHITE, BLACK);
    lcd_putString(92U, 290U, buffer);
}

static void lcd_draw_button_bits(unsigned char button_bits,
                                 unsigned char expander_ok)
{
    unsigned char buffer[5];

    lcd_fillRect(78U, 302U, 160U, 319U, BLACK);

    if (expander_ok == 0U) {
        lcd_fontColor(RED, BLACK);
        lcd_putString(78U, 306U, (unsigned char *)"I2C ERR");
    } else {
        buffer[0] = ((button_bits & 0x01U) != 0U) ? '1' : '0';
        buffer[1] = ((button_bits & 0x02U) != 0U) ? '1' : '0';
        buffer[2] = ((button_bits & 0x04U) != 0U) ? '1' : '0';
        buffer[3] = ((button_bits & 0x08U) != 0U) ? '1' : '0';
        buffer[4] = '\0';

        lcd_fontColor(WHITE, BLACK);
        lcd_putString(78U, 306U, buffer);
    }

    lcd_fontColor(WHITE, BLACK);
}

static void value_to_4digits(unsigned short value, unsigned char *buffer)
{
    if (value > 4095U) {
        value = 4095U;
    }

    buffer[0] = (unsigned char)('0' + ((value / 1000U) % 10U));
    buffer[1] = (unsigned char)('0' + ((value / 100U) % 10U));
    buffer[2] = (unsigned char)('0' + ((value / 10U) % 10U));
    buffer[3] = (unsigned char)('0' + (value % 10U));
    buffer[4] = '\0';
}

static void percent_to_string(unsigned char percent, unsigned char *buffer)
{
    if (percent >= 100U) {
        buffer[0] = '1';
        buffer[1] = '0';
        buffer[2] = '0';
        buffer[3] = '%';
        buffer[4] = '\0';
    } else if (percent >= 10U) {
        buffer[0] = (unsigned char)('0' + (percent / 10U));
        buffer[1] = (unsigned char)('0' + (percent % 10U));
        buffer[2] = '%';
        buffer[3] = ' ';
        buffer[4] = '\0';
    } else {
        buffer[0] = (unsigned char)('0' + percent);
        buffer[1] = '%';
        buffer[2] = ' ';
        buffer[3] = ' ';
        buffer[4] = '\0';
    }
}