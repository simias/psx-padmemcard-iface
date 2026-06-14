#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof((_arr)[0]))

#define BAUD_RATE 115200UL
#define USART_BAUD_RATE(_br) ((F_CPU * 4 + (_br) / 2) / (_br))

#ifdef TODO_SPI
static void spi_init(void)
{
#ifdef USE_ALT_PIN
    /* Use PE0 */
    PORTMUX.TWISPIROUTEA &= ~PORTMUX_SPI0_gm;
    PORTMUX.TWISPIROUTEA |= PORTMUX_SPI0_1_bm;

    /* Set MOSI pin direction to output */
    PORTE.DIR |= PIN0_bm;
#else
    /* Set MOSI pin direction to output */
    PORTA.DIR |= PIN4_bm;
#endif

    SPI0.CTRLA = SPI_DORD_bm /* LSB is transmitted first */
                 | SPI_ENABLE_bm /* Enable module */
                 | SPI_MASTER_bm /* SPI module in Master mode */
                 | SPI_PRESC_DIV4_gc; /* System Clock divided by 4 */

    /* Enable double buffering, MODE0 and disable multi-host */
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_MODE_0_bm | SPI_SSD_bm;
}

static void spi_wait_bsy(void)
{
    for (;;) {
        if (SPI0.INTFLAGS & SPI_DREIF_bm) {
            break;
        }
    }
}

static void spi_send_encoded()
{
    /* We need at least 50us of zero to reset */
    for (unsigned i = 0; i < 15; i++) {
        spi_wait_bsy();
        SPI0.DATA = 0;
    }

    for (unsigned i = 0; i < ARRAY_SIZE(led_spi_encoded); i++) {
        /* Wait for data register empty */
        spi_wait_bsy();
        /* SPI0.DATA = led_spi_encoded[i] | 0xff; */
        SPI0.DATA = led_spi_encoded[i];
    }
}

static void spi_send_leds()
{
    /* We need to output the LEDs very fast and without interruption since the
     * protocol uses fixed timings to detect ones and zeroes (it's not real
     * SPI). This chip is not fast enough to do both the encoding and sending at
     * the same time without breaks between bytes, so we need to precompute
     * everything. */
    uint16_t bitpos = 0;

    for (unsigned i = 0; i < ARRAY_SIZE(led_spi_encoded); i++) {
        led_spi_encoded[i] = 0x00;
    }

    for (unsigned led = 0; led < NLEDS; led++) {
        uint32_t c = 0;
        c |= leds[led].g;
        c <<= 8;
        c |= leds[led].r;
        c <<= 8;
        c |= leds[led].b;

        for (unsigned b = 0; b < 24; b++) {
            unsigned high = (c >> (23 - b)) & 1;

            /* It's hard to get clean timing with the Atmel because the SPI
             * tends to insert small pauses between bytes which mess the timings
             * up. Theoretically we could do with only 3 bit per LED bit: 100 or
             * 110, but it doesn't work well in my experiments.
             *
             * Instead I double the SPI clock and do this: 100000 for 0, 111100
             * for 1. That means that the 0 pulses are a bit below spec */
            led_spi_set_bit(bitpos++, 1);
            led_spi_set_bit(bitpos++, high);
            led_spi_set_bit(bitpos++, high);
            led_spi_set_bit(bitpos++, high);
            led_spi_set_bit(bitpos++, 0);
            led_spi_set_bit(bitpos++, 0);
        }
    }

    spi_send_encoded();
}

#endif

static int uart_putchar(int c)
{
    while (!(USART2.STATUS & USART_DREIF_bm)) {
        ;
    }

    USART2.TXDATAL = c;

    return c;
}

static int uart_putchar_stream(char c, FILE *stream)
{
    if (c == '\n') {
        uart_putchar('\r');
    }

    return uart_putchar(c);
}

static FILE uart_stream =
    FDEV_SETUP_STREAM(uart_putchar_stream, NULL, _FDEV_SETUP_WRITE);

static void uart_init(void)
{
    USART2.BAUD = USART_BAUD_RATE(BAUD_RATE);
    USART2.CTRLB = USART_TXEN_bm;

    /* PIN F1 TX*/
    PORTF.DIR |= PIN0_bm;

    stdout = &uart_stream;
}

static void pwm_init(void)
{
    /* LED on PC0: use PWM */
    PORTMUX.TCAROUTEA &= ~PORTMUX_TCA0_gm;
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;

    /* Period */
    TCA0.SINGLE.PER = 0x1FF;

    /* Duty cycle */
    TCA0.SINGLE.CMP0 = 0x10;

    /* Set divider */
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    PORTC.DIR |= PIN0_bm;
}

static void init(void)
{
    cli();

    /* Watchdog setup */
    /* wdt_enable(WDTO_2S); */

    /* Per datasheet:
   *
   * The 40-pin version of the ATmega4809 is using the die of the 48-pin
   * ATmega4809 but offers fewer connected pads. For this reason, the pins
   * PB[5:0] and PC[7:6] must be disabled (INPUT_DISABLE) or enable pull-ups
   * (PULLUPEN).
   */
    PORTB.PIN0CTRL &= ~PORT_ISC_gm;
    PORTB.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN1CTRL &= ~PORT_ISC_gm;
    PORTB.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN2CTRL &= ~PORT_ISC_gm;
    PORTB.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN3CTRL &= ~PORT_ISC_gm;
    PORTB.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN4CTRL &= ~PORT_ISC_gm;
    PORTB.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN5CTRL &= ~PORT_ISC_gm;
    PORTB.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN6CTRL &= ~PORT_ISC_gm;
    PORTC.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN7CTRL &= ~PORT_ISC_gm;
    PORTC.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;

    /* Enable prescaler with PDIV = 0 (DIV 2). With fuse2/osccfg set to 0x01
     * that will give us a 8MHz CPU clock.
     *
     * According to the datasheet, with our 3V5 power supply the chip cannot
     * safely run at 16MHz but we could run at 10MHz if we wanted and stay
     * within the safe range.
     */
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0x1);

    uart_init();
    puts("Starting up...");

    pwm_init();
}

const uint16_t anim_delay_ms = 33;
const uint8_t anim_lut[256] = {
    0x8,  0x8,  0x8,  0x9,  0x9,  0x9,  0x9,  0xa,  0xa,  0xa,  0xb,  0xb,
    0xb,  0xc,  0xc,  0xc,  0xd,  0xd,  0xd,  0xe,  0xe,  0xe,  0xf,  0xf,
    0x10, 0x10, 0x11, 0x11, 0x11, 0x12, 0x13, 0x13, 0x14, 0x14, 0x15, 0x15,
    0x16, 0x17, 0x17, 0x18, 0x18, 0x19, 0x1a, 0x1b, 0x1b, 0x1c, 0x1d, 0x1e,
    0x1f, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x2a,
    0x2b, 0x2c, 0x2d, 0x2f, 0x30, 0x31, 0x33, 0x34, 0x36, 0x37, 0x39, 0x3a,
    0x3c, 0x3e, 0x3f, 0x41, 0x43, 0x45, 0x47, 0x49, 0x4b, 0x4d, 0x4f, 0x51,
    0x54, 0x56, 0x59, 0x5b, 0x5e, 0x60, 0x63, 0x64, 0x64, 0x64, 0x64, 0x64,
    0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x63, 0x61,
    0x5f, 0x5d, 0x5b, 0x59, 0x57, 0x55, 0x54, 0x52, 0x50, 0x4e, 0x4d, 0x4b,
    0x4a, 0x48, 0x47, 0x45, 0x44, 0x42, 0x41, 0x40, 0x3e, 0x3d, 0x3c, 0x3a,
    0x39, 0x38, 0x37, 0x36, 0x35, 0x34, 0x32, 0x31, 0x30, 0x2f, 0x2e, 0x2d,
    0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28, 0x27, 0x26, 0x26, 0x25, 0x24, 0x23,
    0x23, 0x22, 0x21, 0x21, 0x20, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 0x1c, 0x1b,
    0x1b, 0x1a, 0x1a, 0x19, 0x19, 0x18, 0x18, 0x17, 0x17, 0x16, 0x16, 0x15,
    0x15, 0x14, 0x14, 0x14, 0x13, 0x13, 0x12, 0x12, 0x12, 0x11, 0x11, 0x11,
    0x10, 0x10, 0x10, 0xf,  0xf,  0xf,  0xe,  0xe,  0xe,  0xd,  0xd,  0xd,
    0xd,  0xc,  0xc,  0xc,  0xc,  0xb,  0xb,  0xb,  0xb,  0xa,  0xa,  0xa,
    0xa,  0xa,  0x9,  0x9,  0x9,  0x9,  0x9,  0x8,  0x8,  0x8,  0x8,  0x8,
    0x8,  0x7,  0x7,  0x7,  0x7,  0x7,  0x7,  0x7,  0x6,  0x6,  0x6,  0x6,
    0x6,  0x6,  0x6,  0x6,  0x5,  0x5,  0x5,  0x5,  0x5,  0x5,  0x5,  0x5,
    0x5,  0x5,  0x4,  0x4,
};

int main(void)
{
    unsigned i;
    init();

    for (i = 0;; i++) {
        unsigned luti;

        printf("loop %u\n", i);

        for (luti = 0; luti < ARRAY_SIZE(anim_lut); luti++) {
            TCA0.SINGLE.CMP0 = anim_lut[luti];
            _delay_ms(anim_delay_ms);
            wdt_reset();
        }
    }

    return 0;
}
