#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define nop()  __asm__ __volatile__("nop")

#define WEAK_HIGH_CLOCK 1

#define NUM_BYTES 20
#define NUM_BITS 160

volatile uint8_t buffer[NUM_BYTES];
uint8_t buffer2[NUM_BYTES];
volatile uint8_t bufidx = NUM_BITS;

inline void delay_loop(uint8_t _count) {
    __asm__ volatile (
        "1: dec %0" "\n\t"
        "nop" "\n\t"
        "brne 1b"
        : "=r" (_count)
        : "0" (_count)
    );
}

#define DELAY_US(v) delay_loop((v)<<2);

#define CLOCK_PIN  2
#define DATA_PIN 3

// Clock on PD1
inline void set_clock() {
#if WEAK_HIGH_CLOCK
    // DDRD &= ~(1 << 1);
    pinMode(CLOCK_PIN, INPUT);
#endif
    // PORTD |= 1 << 1;
    digitalWrite(CLOCK_PIN, HIGH);
}

inline void clear_clock() {
    // PORTD &= ~(1 << 1);
    digitalWrite(CLOCK_PIN, LOW);
#if WEAK_HIGH_CLOCK
    // DDRD |= 1 << 1;
    pinMode(CLOCK_PIN, OUTPUT);
#endif
}

// Data on PD0
inline void set_data() {
    pinMode(DATA_PIN, INPUT);
    digitalWrite(DATA_PIN, HIGH);
}

inline void clear_data() {
    digitalWrite(DATA_PIN, LOW);
    pinMode(DATA_PIN, OUTPUT);
}

inline uint8_t read_data() {
    return (PIND >> 0) & 1;
}

void stop_timer1() {
    TCCR1B &= ~0x07; // Stop timer
    TIMSK1 &= ~_BV(OCIE1A);
    nop();
    TCNT1 = 0x0000;
    TIFR1 = _BV(OCF1A);
}

void start_timer1() {
    TIMSK1 |= (1 << OCIE1A); // Enable compare interrupt
    OCR1A = 272;
    TCCR1A = 0x00; // set CTC mode
    TCCR1B = 0x08;
    TCCR1B |= 1; // Start timer
}

ISR(TIMER1_COMPA_vect) {
    if (bufidx < NUM_BITS) {
        clear_clock();
        DELAY_US(1);
        set_clock();
        uint8_t v = read_data();
        if (!v)
            // LED_ON;
            digitalWrite(LED_BUILTIN, HIGH);
        buffer[bufidx >> 3] |= (v << (bufidx & 7));
        bufidx++;
    } else {
        clear_clock();
        bufidx |= 1;
    }
}

// test by-the-book read
// total scan time about 4ms
void read_keyboard_using_timer() {
    bufidx = 0;
    start_timer1();
    while (bufidx <= NUM_BITS) {
        /* wait */
    }
    stop_timer1();
    digitalWrite(LED_BUILTIN, LOW);
}

// inline void delay_us_64(uint8_t v) {
//     while (v--) {
//         DELAY_US(0);
//     }
// }

// test slowest read
void read_keyboard_slow() {
    for (uint8_t i = 0; i < NUM_BITS; ++i) {
        clear_clock();
        DELAY_US(34); // holding clock low for longer than about 34us causes a reset
        set_clock();
        uint8_t v = read_data();
        if (!v)
            digitalWrite(LED_BUILTIN, HIGH);
        buffer[i >> 3] |= (v << (i & 7));

        //_delay_ms(3); // can delay massively here; with clock high it won't reset
    }
    clear_clock();
    digitalWrite(LED_BUILTIN, LOW);
}

// test fastest read
// total scan time about 565us (with debug LED output removed)
void read_keyboard_fast() {
    for (uint8_t i = 0; i < NUM_BITS; ++i) {
        cli();
        clear_clock();
        nop(); // some delay is needed here
        nop();
        nop();
        set_clock();
        sei();
        uint8_t v = read_data();
        if (!v)
            digitalWrite(LED_BUILTIN, HIGH);
        buffer[i >> 3] |= (v << (i & 7)); // compiler's output for this line takes about 2us :-(
    }
    clear_clock();
    digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
    Serial.println("Initialize Serial Monitor");
    Serial.begin(9600);
    delay(500);
    // write your initialization code here
    CPU_PRESCALE(0);

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);

    for (uint8_t i = 0; i < NUM_BYTES; ++i) {
        buffer[i] = 0xFF;
    }
    for (uint8_t i = 0; i < NUM_BYTES; ++i) {
        buffer2[i] = 0xFF;
    }

    set_data();
    clear_clock();

    _delay_ms(1000);

    Serial.println("wyse");

    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    uint8_t changed = 0;
    for (uint8_t i = 0; i < NUM_BYTES; ++i) {
        buffer[i] = 0;
    }
    //read_keyboard_using_timer();
    //read_keyboard_slow();
    read_keyboard_fast();
    for (uint8_t i = 0; i < NUM_BYTES; ++i) {
        if (buffer[i] != buffer2[i]) {
            changed = 1;
            buffer2[i] = buffer[i];
        }
    }
    if (changed) {
        for (uint8_t i = 0; i < NUM_BYTES; ++i) {
            Serial.print(~buffer2[i], HEX);
            Serial.print(' ');
        }
        Serial.println();
        for (uint8_t i = 0; i < NUM_BITS; ++i) {
            uint8_t v = (buffer2[i >> 3] >> (i & 7)) & 1;
            if (!v) {
                Serial.print(i,HEX);
                Serial.print(' ');
            }
        }
        Serial.println();
    }
    _delay_ms(5);
}
