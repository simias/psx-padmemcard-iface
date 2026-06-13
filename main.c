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

static int uart_putchar_stream(char c, FILE* stream)
{
    if (c == '\n') {
        uart_putchar('\r');
    }

    return uart_putchar(c);
}

static FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar_stream, NULL, _FDEV_SETUP_WRITE);

static void uart_init(void)
{
    USART2.BAUD = USART_BAUD_RATE(BAUD_RATE);
    USART2.CTRLB = USART_TXEN_bm;

    /* PIN F1 TX*/
    PORTF.DIR |= PIN0_bm;

    stdout = &uart_stream;
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
     * that will give us a 8MHz CPU clock. */
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0x1);

    uart_init();
    puts("Starting up...");
}

int main(void)
{
    unsigned i;
    init();

    PORTC.DIR |= PIN0_bm;
    PORTC.OUTSET |= PIN0_bm;

    for (i = 0;; i++) {
        wdt_reset();
        printf("loop %u\n", i);
        PORTC.OUTCLR = PIN0_bm;
        _delay_ms(500);
        PORTC.OUTSET = PIN0_bm;
        _delay_ms(500);
    }

    return 0;
}
