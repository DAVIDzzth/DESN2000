#include "../Headers/stm32f103xe.h"
#include "../LCD/ili9341.h"
#include "../LCD/lcd_hw.h"
#include "../LCD/lcd_grph.h"

#define EXPANDER_WRITE_ADDRESS   0x70U
#define EXPANDER_READ_ADDRESS    0x71U

#define EXPANDER_INPUT_REG       0x00U
#define EXPANDER_OUTPUT_REG      0x01U
#define EXPANDER_CONFIG_REG      0x03U

#define DAC_MIDPOINT             2048U
#define TONE_HALF_PERIOD_ARR     293U   /* 1 MHz / 294 / 2 = about 1700.7 Hz */

static volatile unsigned short g_dac_amplitude = 0U;
static volatile unsigned char g_dac_phase = 0U;

static void short_delay(volatile unsigned int count)
{
    while (count > 0U) {
        count--;
    }
}

static void i2c1_init(void)
{
    /* Enable GPIOB and I2C1 clocks. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* Reset I2C1 before configuration. */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /*
     * PB6 = I2C1_SCL, PB7 = I2C1_SDA.
     * MODE = 11: output up to 50 MHz
     * CNF  = 11: alternate-function open-drain
     * Therefore each 4-bit pin configuration is 0xF.
     */
    GPIOB->CRL &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB->CRL |=  ((0xFU << 24) | (0xFU << 28));

    I2C1->CR1 = 0U;

    /* APB1 is 8 MHz because SystemInit() is currently empty. */
    I2C1->CR2 = 8U;

    /* Standard-mode I2C at 100 kHz: CCR = 8 MHz / (2 x 100 kHz) = 40. */
    I2C1->CCR = 40U;
    I2C1->TRISE = 9U;       /* PCLK1 in MHz + 1 */

    I2C1->CR1 = I2C_CR1_PE;
}

static void i2c1_write_register(unsigned char reg, unsigned char value)
{
    volatile unsigned int dummy;

    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    }

    I2C1->CR1 |= I2C_CR1_START;
    while ((I2C1->SR1 & I2C_SR1_SB) == 0U) {
    }

    I2C1->DR = EXPANDER_WRITE_ADDRESS;
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0U) {
    }

    /* Reading SR1 followed by SR2 clears ADDR. */
    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;

    while ((I2C1->SR1 & I2C_SR1_TXE) == 0U) {
    }
    I2C1->DR = reg;

    while ((I2C1->SR1 & I2C_SR1_TXE) == 0U) {
    }
    I2C1->DR = value;

    while ((I2C1->SR1 & I2C_SR1_BTF) == 0U) {
    }

    I2C1->CR1 |= I2C_CR1_STOP;
}

static unsigned char i2c1_read_register(unsigned char reg)
{
    volatile unsigned int dummy;
    unsigned char value;

    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    }

    /* First transaction: send the register address. */
    I2C1->CR1 |= I2C_CR1_START;
    while ((I2C1->SR1 & I2C_SR1_SB) == 0U) {
    }

    I2C1->DR = EXPANDER_WRITE_ADDRESS;
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0U) {
    }

    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;

    while ((I2C1->SR1 & I2C_SR1_TXE) == 0U) {
    }
    I2C1->DR = reg;

    while ((I2C1->SR1 & I2C_SR1_BTF) == 0U) {
    }

    /* Repeated START: read one byte from that register. */
    I2C1->CR1 |= I2C_CR1_START;
    while ((I2C1->SR1 & I2C_SR1_SB) == 0U) {
    }

    I2C1->DR = EXPANDER_READ_ADDRESS;
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0U) {
    }

    /* Single-byte receive sequence for STM32F1. */
    I2C1->CR1 &= ~I2C_CR1_ACK;

    __disable_irq();
    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;
    I2C1->CR1 |= I2C_CR1_STOP;
    __enable_irq();

    while ((I2C1->SR1 & I2C_SR1_RXNE) == 0U) {
    }

    value = (unsigned char)I2C1->DR;
    return value;
}

static void adc1_r1302_init(void)
{
    /* Enable GPIOA and ADC1 clocks. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    /* PA1 is ADC1 channel 1. Analog mode = CNF 00, MODE 00. */
    GPIOA->CRL &= ~(0xFU << 4);

    /* ADC clock = PCLK2 / 2 = 4 MHz at the default 8 MHz clock. */
    RCC->CFGR &= ~(3U << 14);

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;

    /* One regular conversion, first conversion is channel 1. */
    ADC1->SQR1 = 0U;
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = 1U;

    /* Channel 1 sample time = 55.5 ADC cycles. */
    ADC1->SMPR2 &= ~(7U << 3);
    ADC1->SMPR2 |=  (5U << 3);

    ADC1->CR2 |= ADC_CR2_ADON;
    short_delay(1000U);

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {
    }
}

static unsigned short adc1_read_r1302(void)
{
    /* A second write of ADON starts a conversion on STM32F1. */
    ADC1->CR2 |= ADC_CR2_ADON;

    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
    }

    return (unsigned short)(ADC1->DR & 0x0FFFU);
}

static void dac1_speaker_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* PA4 = DAC channel 1. Analog mode = CNF 00, MODE 00. */
    GPIOA->CRL &= ~(0xFU << 16);

    DAC->CR = DAC_CR_EN1;
    DAC->DHR12R1 = DAC_MIDPOINT;
}

static void tim2_tone_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /*
     * TIM2 clock = 8 MHz.
     * Prescaler 7 gives a 1 MHz timer count.
     * Update every 294 counts; two updates make one square-wave period.
     */
    TIM2->PSC = 7U;
    TIM2->ARR = TONE_HALF_PERIOD_ARR;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
}

void TIM2_IRQHandler(void)
{
    unsigned short amplitude;

    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR &= ~TIM_SR_UIF;

        amplitude = g_dac_amplitude;

        if (g_dac_phase == 0U) {
            DAC->DHR12R1 = DAC_MIDPOINT + amplitude;
            g_dac_phase = 1U;
        } else {
            DAC->DHR12R1 = DAC_MIDPOINT - amplitude;
            g_dac_phase = 0U;
        }
    }
}

void delay (void);

int main(void) {
	unsigned char expander_input;
    unsigned char led_value;
    unsigned char previous_led_value;
    unsigned short adc_value;
    unsigned int filtered_adc;

    previous_led_value = 0xFFU;
    filtered_adc = 0U;

    /* 初始化 I2C、R1302 ADC 和扬声器 DAC */
    i2c1_init();
    adc1_r1302_init();
    dac1_speaker_init();

    /*
     * 先将 LED 输出寄存器清零，避免启动时
     * LED8-LED11 短暂全部亮起。
     */
    i2c1_write_register(EXPANDER_OUTPUT_REG, 0x00U);

    /*
     * 扩展器配置：
     * bit 7-4 = 输入，对应 PB5-PB2
     * bit 3-0 = 输出，对应 LED11-LED8
     */
    i2c1_write_register(EXPANDER_CONFIG_REG, 0xF0U);

    /* 启动约 1.7 kHz 的固定频率声音 */
    tim2_tone_init();

    while (1) {
        /*
         * 读取 PB2-PB5：
         * bit 4 = PB2
         * bit 5 = PB3
         * bit 6 = PB4
         * bit 7 = PB5
         *
         * 右移四位后：
         * bit 0 -> LED8
         * bit 1 -> LED9
         * bit 2 -> LED10
         * bit 3 -> LED11
         */
        expander_input =
            i2c1_read_register(EXPANDER_INPUT_REG);

        led_value =
            (unsigned char)((expander_input >> 4) & 0x0FU);

        if (led_value != previous_led_value) {
            i2c1_write_register(
                EXPANDER_OUTPUT_REG,
                led_value
            );

            previous_led_value = led_value;
        }

        /* 读取 R1302，ADC 范围为 0-4095 */
        adc_value = adc1_read_r1302();

        /*
         * 简单低通滤波，减少旋钮 ADC 数值抖动，
         * 避免扬声器音量出现沙沙变化。
         */
        filtered_adc =
            ((filtered_adc * 7U) + adc_value) / 8U;

        /*
         * 将 ADC 0-4095 映射成 DAC 振幅 0-2047。
         * 只改变振幅，因此音量改变但频率不改变。
         */
        g_dac_amplitude =
            (unsigned short)(filtered_adc >> 1);

        short_delay(2000U);
    }
}

