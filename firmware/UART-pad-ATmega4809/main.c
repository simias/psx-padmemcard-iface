#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof((_arr)[0]))

#define BAUD_RATE 115200
#define USART_BAUD_RATE(_br) ((F_CPU * 4 + (_br) / 2) / (_br))

enum padmem_slot {
    SLOT_1 = 1,
    SLOT_2 = 2,
};

static void spi_init(void)
{
    SPI0.CTRLB = SPI_MODE_3_gc | SPI_BUFEN_bm | SPI_BUFWR_bm;

    /* Enable SS IRQ (to reset the logic) and RX IRQ */
    SPI0.INTCTRL = SPI_RXCIE_bm;

    /* LSB first, CLK_PER / 16 */
    SPI0.CTRLA = 0;

    /* MISO pin activated in the SPI IRQ handler */
    PORTA.DIRCLR = PIN5_bm;

    /* Enable IRQ on /SS posedge to reset the state machine */
    PORTA.PIN7CTRL = PORT_ISC_RISING_gc;
}

/* Must be a power of two */
#define TX_FIFO_LEN 0x40
#define TX_FIFO_IMASK ((TX_FIFO_LEN << 1) - 1)

static volatile uint8_t uart_tx_fifo[TX_FIFO_LEN];
static volatile uint8_t uart_tx_wi = 0;
static volatile uint8_t uart_tx_ri = 0;

static bool uart_can_write(void)
{
    return USART2.STATUS & USART_DREIF_bm;
}

ISR_N(USART2_DRE_vect_num)
static void uart_tx_irq(void)
{
    uint8_t ri = uart_tx_ri;
    uint8_t wi = uart_tx_wi;

    if (ri != wi && uart_can_write()) {
        USART2.TXDATAL = uart_tx_fifo[(ri++) & (TX_FIFO_LEN - 1)];
        uart_tx_ri = ri & TX_FIFO_IMASK;
    }

    if (ri == wi) {
        /* FIFO empty, disable IRQ */
        USART2.CTRLA &= ~USART_DREIE_bm;
    }
}

static void uart_putchar(int c)
{
    uint8_t sr = SREG;
    uint8_t ri;
    uint8_t wi;

    cli();

    ri = uart_tx_ri;
    wi = uart_tx_wi;

    if (ri == wi && uart_can_write()) {
        /* FIFO is empty and UART can accept a byte, just use that */
        USART2.TXDATAL = c;
        goto done;
    }

    /* UART is busy, attempt to store in the buffer */
    while ((wi ^ TX_FIFO_LEN) == ri) {
        /* Buffer full! */
        if (!(sr & CPU_I_bm)) {
            /* Interrupts were disabled when we were called, we can't sleep, so
             * we give up. */
            goto done;
        }

        /* Wait for UART to empty */
        sei();
        sleep_cpu();
        cli();

        ri = uart_tx_ri;
        wi = uart_tx_wi;
    }

    uart_tx_fifo[(wi++) & (TX_FIFO_LEN - 1)] = c;

    uart_tx_wi = wi & TX_FIFO_IMASK;

    USART2.CTRLA |= USART_DREIE_bm;

done:
    SREG = sr;
}

static int uart_putchar_stream(char c, FILE *stream)
{
    (void)stream;

    if (c == '\n') {
        uart_putchar('\r');
    }

    uart_putchar(c);

    return c;
}

static FILE uart_stream =
    FDEV_SETUP_STREAM(uart_putchar_stream, NULL, _FDEV_SETUP_WRITE);

static void uart_tx_frame(uint8_t cmd, const uint8_t *dat, uint8_t len)
{
    uint8_t csum = cmd;
    uint8_t i;

    uart_putchar(0xa6);
    uart_putchar(cmd);

    for (i = 0; i < len; i++) {
        uint8_t b = dat[i];
        csum += b;
        uart_putchar(b);
        if (b == 0xa7) {
            /* Escape */
            uart_putchar(b);
        }
    }

    uart_putchar(0xa7);
    uart_putchar((len + 1) & 0x7f);
    uart_putchar(csum ^ 0xff);
}

static void uart_tx_nack(void)
{
    uart_tx_frame('!', NULL, 0);
}

enum uart_rx_state {
    RX_START,
    WAIT_FOR_DATA,
    WAIT_FOR_ESCAPE,
    WAIT_FOR_CSUM,
    RX_DONE,
};

static volatile enum uart_rx_state uart_st = RX_START;
static volatile uint8_t rx_buf[256];
static volatile uint8_t rx_buf_len;

ISR_N(USART2_RXC_vect_num)
static void uart_rx_irq(void)
{
    uint8_t b = USART2.RXDATAL;
    enum uart_rx_state nstate = RX_START;
    /* How many bytes to receive - 1*/
    static uint8_t csum;
    static uint8_t wi;

    switch (uart_st) {
    case RX_START:
        if (b == 0xa5) {
            wi = 0;
            csum = 0;
            nstate = WAIT_FOR_DATA;
        }
        break;

    case WAIT_FOR_DATA:
        if (b == 0xa7) {
            nstate = WAIT_FOR_ESCAPE;
        } else {
            rx_buf[wi++] = b;
            csum += b;
            nstate = WAIT_FOR_DATA;

            if (wi == 0) {
                /* Overflow */
                nstate = RX_START;
            }
        }

        break;

    case WAIT_FOR_ESCAPE:
        if (b == 0xa7) {
            /* 0xa7 0xa7 -> we want an 0xa7 */
            rx_buf[wi++] = b;
            csum += b;
            nstate = WAIT_FOR_DATA;

            if (wi == 0) {
                /* Overflow */
                nstate = RX_START;
            }
        } else if ((b >> 4) == 0xb) {
            /* 0-RLE
             *
             * We could use a more aggressive encoding allowing for even longer
             * runs, but eventually the decoding takes too long and we miss the
             * next byte. Plus the gains become really small.
             */
            uint8_t nzero = (b & 0xf) + 3;
            uint8_t wi_s = wi;

            nstate = WAIT_FOR_DATA;

            while (nzero--) {
                rx_buf[wi++] = 0;
            }

            if (wi < wi_s) {
                /* Overflow */
                nstate = RX_START;
            }
        } else if (b == (wi & 0x7f)) {
            /* End of data */
            nstate = WAIT_FOR_CSUM;
        } else {
            /* Spurious/corrupted data */
            uart_tx_nack();
            nstate = RX_START;
        }

        break;

    case WAIT_FOR_CSUM:
        if ((csum ^ 0xff) == b) {
            rx_buf_len = wi;
            nstate = RX_DONE;
        } else {
            uart_tx_nack();
        }
        break;

    case RX_DONE:
        if (wi > 0) {
            nstate = RX_DONE;
        } else {
            nstate = RX_START;
        }
        break;
    }

    if (nstate != RX_START && nstate != RX_DONE) {
        TCA0.SINGLE.CMP0BUF = 0x20;
    }

    uart_st = nstate;
}

static void uart_init(void)
{
    /* Add pull-up on RX pin for when nothing is connected */
    PORTF.PIN1CTRL = PORT_PULLUPEN_bm;

    /* PIN F1 TX*/
    PORTF.DIR |= PIN0_bm;

    USART2.BAUD = USART_BAUD_RATE(BAUD_RATE);
    USART2.CTRLA = USART_RXCIE_bm;
    USART2.CTRLB = USART_TXEN_bm | USART_RXEN_bm;

    stdout = &uart_stream;
}

static void pwm_init(void)
{
    /* LED on PC0: use PWM */
    PORTC.DIR |= PIN0_bm;

    PORTMUX.TCAROUTEA &= ~PORTMUX_TCA0_gm;
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;

    /* Period */
    TCA0.SINGLE.PER = 0xff;

    /* Duty cycle */
    TCA0.SINGLE.CMP0 = 0x10;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    /* Set divider */
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;
}

static void rtc_waitbsy(void)
{
    while (RTC.STATUS & (RTC_CTRLABUSY_bm | RTC_CNTBUSY_bm | RTC_PERBUSY_bm |
                         RTC_CMPBUSY_bm)) {
        ;
    }
}

static volatile uint8_t nticks_10ms = 0;

ISR_N(RTC_CNT_vect_num)
static void rtc_irq(void)
{
    uint8_t pwm_cmp;

    /* ACK */
    RTC.INTFLAGS = RTC_OVF_bm;

    nticks_10ms++;

    pwm_cmp = TCA0.SINGLE.CMP0BUFL;

    if (pwm_cmp > 5) {
        pwm_cmp -= 2;

        TCA0.SINGLE.CMP0BUFL = pwm_cmp;
        TCA0.SINGLE.CMP0BUFH = 0;
    }
}

static void rtc_init(void)
{
    /* We want a tic every 10ms */
    rtc_waitbsy();
    RTC.PER = (32768 + (100 >> 1)) / 100;

    rtc_waitbsy();
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;

    /* Enable overflow IRQ */
    rtc_waitbsy();
    RTC.INTCTRL = RTC_OVF_bm;

    /* Enable RTC with no prescaling */
    rtc_waitbsy();
    RTC.CTRLA = RTC_RTCEN_bm | RTC_PRESCALER_DIV1_gc;

    rtc_waitbsy();
}

static volatile uint8_t spi_rx_seq = 0;

ISR_N(PORTA_PORT_vect_num)
static void porta_irq(void)
{
    uint8_t flags = PORTA.INTFLAGS;

    /* ACK */
    PORTA.INTFLAGS = flags;

    if (flags & PORT_INT_7_bm) {
        /* /SS rising edge (deselect). Make sure we don't drive anything on MISO
         * until we know we're being talked to. */
        PORTA.DIRCLR = PIN5_bm;

        /* Reset SPI */
        SPI0.CTRLA = 0;
        _delay_us(1);
        SPI0.CTRLA = SPI_DORD_bm | SPI_PRESC_DIV16_gc | SPI_ENABLE_bm;


        spi_rx_seq = 0;

        /* We have two words in the TX buffer, so we need to prepare the first
         * two responses. Then in spi0_irq we are offet by one response every
         * time. */
        SPI0.DATA = 0xff;
        SPI0.DATA = 0x41;
    }
}

ISR_N(SPI0_INT_vect_num)
static void spi0_irq(void)
{
    uint8_t rx = SPI0.DATA;
    bool dsr = false;

    switch (spi_rx_seq) {
    case 0:
        dsr = (rx == 0x01);
        SPI0.DATA = 0x5a;
        break;
    case 1:
        dsr = (rx == 0x42);
        /* Buttons 1 */
        SPI0.DATA = 0xff;
        break;
    case 2:
        dsr = true;
        /* Buttons 2 */
        SPI0.DATA = ~(1 << 5);
        break;
    case 3:
        dsr = true;
    }

    if (dsr) {
        /* PAD interaction: activate MISO */
        PORTA.DIRSET = PIN5_bm;
        spi_rx_seq += 1;

        TCA0.SINGLE.CMP0BUFL = 0x15;
        TCA0.SINGLE.CMP0BUFH = 0;

        /* Simulate the delay of a real controller */
        _delay_us(8);
        PORTD.DIRSET = PIN0_bm;
        _delay_us(4);
        PORTD.DIRCLR = PIN0_bm;
    } else {
        /* Don't deactivate anything here or we'll lose the last byte */
        spi_rx_seq = 0xff;
    }
}

static void init(void)
{
    cli();

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

    /* Enable prescaler with PDIV = (DIV 4). With fuse2/osccfg set to 0x01
     * that will give us a 4MHz CPU clock.
     *
     * According to the datasheet, with our 3V5 power supply the chip cannot
     * safely run at 16MHz but we could run at 10MHz if we wanted and stay
     * within the safe range. But then we have to take into account that we want
     * the SPI to run at 250kHz...
     */
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, CLKCTRL_PEN_bm | CLKCTRL_PDIV_4X_gc);

    uart_init();

    pwm_init();

    spi_init();

    rtc_init();

    /* PS1 Reset: pull to zero to trigger reset, hi-Z the rest of the time */
    PORTC.DIRCLR = PIN1_bm;
    PORTC.OUTCLR = PIN1_bm;

    /* /DSR signal to tell the console that we have data to send: pull to 0 for
     * > 2us to signal, otherwise hi-z */
    PORTD.DIRCLR = PIN0_bm;
    PORTD.OUTCLR = PIN0_bm;

    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    /* Make sure the peripherals see stable values before we attempt to start a
     * command */
    _delay_ms(200);

    sei();

    puts("Starting up...");
}

int main(void)
{
    init();

    for (;;) {
        cli();
        if (uart_st != RX_DONE) {
            /* This may look race-y but it isn't: the instruction immediately
             * after SEI is guaranteed to be executed before any IRQ is handled.
             * So it's not possible for an IRQ to sneak in-between.
             */
            sei();
            sleep_cpu();
            continue;
        }
        sei();

        switch (rx_buf[0]) {
        case '?':
            /* Query interface version */
            {
                uint8_t version[] = { 1, 0, 0 };

                uart_tx_frame('?', version, ARRAY_SIZE(version));
            }
            break;
        case 'R':
            /* Reset the PlayStation */
            {
                PORTC.DIRSET = PIN1_bm;
                _delay_ms(100);
                PORTC.DIRCLR = PIN1_bm;

                uart_tx_frame('R', NULL, 0);
            }
            break;
        default:
            printf("Unknown command %x\n", rx_buf[0]);
            uart_tx_nack();
        }

        /* We're ready to accept the next command */
        uart_st = RX_START;
    }
}
