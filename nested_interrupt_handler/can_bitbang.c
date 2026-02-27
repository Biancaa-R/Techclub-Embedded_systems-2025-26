/* Bitbanged CAN for LPC2148 (standard 11-bit ID frames)
 *
 * Pins:
 *   CAN_TX -> P0.16  (output, drives transceiver TXD)
 *   CAN_RX -> P0.17  (input, reads transceiver RXD)
 *
 * Timing:
 *   Uses oversampling factor OS = 8
 *   Timer tick = bitrate * OS
 *
 * Notes:
 *   - You must provide a stable CPU clock and Timer0 setup function set_timer_freq().
 *   - Attach TX pin to transceiver TXD, RX pin to transceiver RXD.
 *   - For best reliability: pick lower CAN bitrate (125kbps) the first time.
 *
 * Limitations:
 *   - This implementation focuses on correct bit framing, stuffing, CRC,
 *     and a reliable blocking transmit. RX is oversampled and parsed but
 *     will need careful tuning to be robust in noisy environments.
 *
 * Compile with: arm-none-eabi-gcc/Keil; adapt register macros if using a different header.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* LPC214x GPIO registers (addresses from user manual) */
#define IO0DIR   (*(volatile uint32_t *)0xE0028008)
#define IO0SET   (*(volatile uint32_t *)0xE0028004)
#define IO0CLR   (*(volatile uint32_t *)0xE0028000)
#define IO0PIN   (*(volatile uint32_t *)0xE002800C)

#define CAN_TX_PIN   (1u << 16)   // P0.16
#define CAN_RX_PIN   (1u << 17)   // P0.17

/* Timer0 registers (simple wrappers) */
#define T0IR   (*(volatile uint32_t *)0xE0004000)
#define T0TCR  (*(volatile uint32_t *)0xE0004004)
#define T0TC   (*(volatile uint32_t *)0xE0004008)
#define T0PR   (*(volatile uint32_t *)0xE000400C)
#define T0MCR  (*(volatile uint32_t *)0xE0004014)
#define T0MR0  (*(volatile uint32_t *)0xE0004018)

/* Oversampling factor */
#define OS 8

/* CAN timing default (choose a bitrate) */
static uint32_t can_bitrate = 125000; // default 125 kbps
static uint32_t timer_tick_hz;        // bitrate * OS
static uint32_t timer_match_value;    // timer cycles for ISR (depends on peripheral PCLK)

/* Simple delay using timer: waits for n timer ticks (tick frequency = timer_tick_hz) */
static void timer0_wait_ticks(uint32_t ticks) {
    T0TC = 0;
    T0IR = 0xFF;
    T0MR0 = ticks - 1;
    T0MCR = (1<<0); // on match, reset TC (no interrupt needed here)
    T0TCR = 1;      // enable timer
    // busy-wait until MR0 match resets timer (TC becomes 0 then stops? MR resets TC)
    // We poll the IR flag for match (MR0).
    while ((T0IR & 1u) == 0) { /* spin */ }
    T0IR = 1; // clear match
    T0TCR = 0; // disable timer
}

/* Helper to set TX pin high/low */
static inline void can_pin_tx_hi(void) { IO0SET = CAN_TX_PIN; }
static inline void can_pin_tx_lo(void) { IO0CLR = CAN_TX_PIN; }
static inline inline bool can_pin_rx_read(void) { return (IO0PIN & CAN_RX_PIN) != 0; }

/* CRC-15 (polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1)
   polynomial binary: 1 0010 0010 1101 1 -> 0x4599 (15-bit)
   We'll implement bit-by-bit CRC (LSB-first or MSB-first? CAN transmits LSB first for fields
   where required. Standard CRC is computed over bit stream shown in ISO 11898.)
*/
static uint16_t crc15_compute(const uint8_t *bits, uint32_t bitlen) {
    // bits here is a bit array LSB-first or explicit 0/1 values per element.
    // For simplicity we'll expect bits[] to be array of bytes with values 0/1 representing
    // the bit stream in transmit order (MSB->LSB as seen on bus). We'll compute CRC per spec:
    // Shift register is 15 bits, initialized to all 1s (0x7FFF).
    const uint16_t poly = 0x4599; // CRC-15 poly
    uint16_t crc = 0x0000;        // Many implementations init to 0. CAN initializes to 0.
    // According to CAN spec, CRC register is reset to 0 before the first CRC bit.
    for (uint32_t i = 0; i < bitlen; ++i) {
        uint8_t bit = bits[i] & 1;
        // input bit XOR top bit
        uint16_t msb = ((crc >> 14) & 1); // top of 15-bit reg
        uint16_t newbit = msb ^ bit;
        crc = (crc << 1) & 0x7FFF; // keep 15 bits
        if (newbit) crc ^= poly;
    }
    return crc & 0x7FFF;
}

/* Build a standard (11-bit) CAN frame bitstream (without NRZ encoded edges, actual bits)
   We'll produce an array of bits (0 or 1) in transmit order (send left-to-right).
   Frame format (simplified, base frame):
     SOF (0)
     Identifier (11 bits, MSB first)
     RTR (0 for data frame)
     IDE (0)
     r0 (0)
     DLC (4 bits MSB first)
     Data field (0..8 bytes, MSB-first per byte)
     CRC sequence (15 bits)
     CRC delimiter (1)
     ACK slot (1) and ACK delimiter (1)  (transmitter sends recessive for ACK slot)
     EOF (7 recessive bits)
*/
#define MAX_FRAME_BITS 2000
static uint8_t tx_bits[MAX_FRAME_BITS];
static uint32_t tx_bitlen;

static void build_can_base_frame(uint16_t id11, uint8_t dlc, const uint8_t data[]) {
    uint32_t p = 0;
    tx_bitlen = 0;
    // helper to push bit (0 or 1)
    #define PUSH(b) do { tx_bits[p++] = (b) ? 1 : 0; } while(0)

    // SOF (dominant = 0)
    PUSH(0);

    // Identifier 11 bits MSB first (b10 .. b0)
    for (int i = 10; i >= 0; --i) PUSH((id11 >> i) & 1);

    // RTR (0 = data)
    PUSH(0);

    // IDE (0 = standard)
    PUSH(0);

    // r0
    PUSH(0);

    // DLC 4 bits MSB first
    for (int i = 3; i >= 0; --i) PUSH((dlc >> i) & 1);

    // Data bytes: each byte MSB first
    for (int b = 0; b < dlc; ++b) {
        for (int i = 7; i >= 0; --i) PUSH((data[b] >> i) & 1);
    }

    // Compute CRC over all bits from Identifier up to Data (but not SOF and not CRC)
    // According to spec, CRC is computed starting at the start of the frame after SOF,
    // up to the end of data field (i.e., includes the control bits).
    // We'll collect those bits into a buffer for CRC calculation.
    // For simplicity, let's compute CRC over bits from Identifier (first ID bit) onward.
    uint32_t crc_input_len = 11 + 1 + 1 + 1 + 4 + dlc * 8; // id + rtr + ide + r0 + dlc + data
    uint8_t crc_buffer[2048];
    uint32_t idx = 0;
    // Identifier
    for (int i = 10; i >= 0; --i) crc_buffer[idx++] = (id11 >> i) & 1;
    crc_buffer[idx++] = 0; // RTR
    crc_buffer[idx++] = 0; // IDE
    crc_buffer[idx++] = 0; // r0
    for (int i = 3; i >= 0; --i) crc_buffer[idx++] = (dlc >> i) & 1;
    for (int b = 0; b < dlc; ++b)
        for (int i = 7; i >= 0; --i) crc_buffer[idx++] = (data[b] >> i) & 1;

    uint16_t crc = crc15_compute(crc_buffer, crc_input_len);

    // append CRC (15 bits MSB first)
    for (int i = 14; i >= 0; --i) PUSH((crc >> i) & 1);

    // CRC delimiter (recessive = 1)
    PUSH(1);

    // ACK slot (transmitter leaves it recessive; receiver pulls dominant if OK)
    PUSH(1);

    // ACK delimiter
    PUSH(1);

    // EOF (7 recessive bits)
    for (int i = 0; i < 7; ++i) PUSH(1);

    tx_bitlen = p;

    #undef PUSH
}

/* Bit stuffing: CAN requires stuffing a complement bit after 5 consecutive identical bits.
   We will create a stuffed bitstream (in-place into out_bits).
*/
static uint8_t stuffed_bits[MAX_FRAME_BITS * 2];
static uint32_t stuffed_len;

static void perform_bit_stuffing(const uint8_t *in_bits, uint32_t in_len) {
    uint32_t outp = 0;
    int consecutive = 1;
    uint8_t last = in_bits[0];
    stuffed_bits[outp++] = last;
    for (uint32_t i = 1; i < in_len; ++i) {
        uint8_t b = in_bits[i];
        if (b == last) {
            consecutive++;
            stuffed_bits[outp++] = b;
            if (consecutive == 5) {
                // insert complement bit
                stuffed_bits[outp++] = !b;
                consecutive = 0; // stuffed bit breaks the run
                last = !b;
            } else {
                last = b;
            }
        } else {
            stuffed_bits[outp++] = b;
            last = b;
            consecutive = 1;
        }
    }
    stuffed_len = outp;
}

/* Blocking transmit of stuffed_bits[] at given bitrate.
   We'll drive the TX pin active for a full bit time, then set for next bit.
   Dominant = 0 (drive low), recessive = 1 (release high).
*/
static void drive_bus_recessive(void) {
    // For recessive on transceiver, set TXD high (transceiver will put recessive on bus)
    can_pin_tx_hi();
}
static void drive_bus_dominant(void) {
    // drive dominant: pull TXD low
    can_pin_tx_lo();
}

/* Simple arbitration check: while transmitting dominant, sample RX. If we transmit recessive but
   observe dominant on bus, we lost arbitration.
   Note: robust arbitration requires continuous sampling at bit times; this simple check
   checks after the bit is driven.
*/
static bool check_arbitration_loss(uint8_t bit_sent) {
    bool bus = can_pin_rx_read(); // 1 = recessive, 0 = dominant
    if (bit_sent == 1 && bus == 0) {
        // we sent recessive but bus is dominant -> we lost arbitration
        return true;
    }
    return false;
}

/* Transmit the stuffed bit stream (blocking). Returns true if transmitted and ack received (simple).
   WARNING: This routine is timing-critical. It uses timer0_wait_ticks() to wait for exactly one bit period
   (timer tick = OS * bitrate; we will wait OS ticks for one bit).
*/
static bool can_transmit_blocking(void) {
    // Precondition: timer configured so that timer0_wait_ticks( OS ) equals 1 bit-time
    // We'll disable interrupts to avoid jitter during TX; caller must consider system needs.
    // (Alternatively, set a high priority for this routine's timer and mask less important IRQs)
    // For safety, we won't touch NVIC in this snippet; the user may choose in production to disable IRQs.
    // Drive bus initial state to recessive
    drive_bus_recessive();

    // Send each stuffed bit; each bit period = OS timer ticks, but our wait utility expects ticks param
    for (uint32_t i = 0; i < stuffed_len; ++i) {
        uint8_t bit = stuffed_bits[i];
        if (bit == 0) drive_bus_dominant();
        else drive_bus_recessive();

        // Small settle time optional here (manufacturer recommends sample point after some TQ)
        timer0_wait_ticks(OS); // wait one bit time

        // simple arbitration check:
        if (check_arbitration_loss(bit)) {
            // lost arbitration
            drive_bus_recessive();
            return false;
        }
    }

    // After sending EOF, the ACK slot will be driven by receiver dominant if CRC OK.
    // We left bus recessive during ACK slot, then sample the ACK after bit-time.
    // Wait one bit for ACK slot, then sample.
    timer0_wait_ticks(OS);
    bool ack_seen = (can_pin_rx_read() == 0); // 0 = dominant -> ACK present

    // drive recessive to release bus
    drive_bus_recessive();
    return ack_seen;
}

/* Public API: send standard frame (blocking). Returns true on success (ACK received) */
bool can_send(uint16_t id11, uint8_t dlc, const uint8_t *data) {
    if (dlc > 8) return false;

    build_can_base_frame(id11, dlc, data);
    perform_bit_stuffing(tx_bits, tx_bitlen);
    bool ack = can_transmit_blocking();
    return ack;
}

/* --- Initialization & timer setup helpers --- */

/* User must supply a function to convert required tick frequency to timer match value.
   For LPC2148 Timer0, if PCLK = CCLK (assume), and CPU clk known, you set prescaler to create
   a timer tick frequency of 'timer_tick_hz'. Here we implement a simple config where we set
   T0PR so that T0TC increments at 1MHz or other. This part must be adapted based on your
   system PCLK settings. For a portable example we assume PCLK = 15 MHz and compute match.
*/
void can_gpio_init(void) {
    // Set P0.16 output, P0.17 input
    IO0DIR |= CAN_TX_PIN;     // output
    IO0DIR &= ~CAN_RX_PIN;    // input
    // default set TX high (recessive)
    can_pin_tx_hi();
}

/* Basic timer configuration: we want Timer0 to increment every CPU cycle or every PCLK cycle.
   For reliability adapt this to your clock. This helper assumes PCLK frequency passed in param.
*/
void can_timer_init(uint32_t pclk_hz) {
    // timer tick frequency = bitrate * OS
    timer_tick_hz = can_bitrate * OS;
    // we will set PR so that T0TC increments at timer_tick_hz frequency by computing
    // prescaler = (pclk_hz / timer_tick_hz) - 1 ; then MR0 = 1 (or simply use wait by MR)
    // But here timer0_wait_ticks() uses MR0 and resets; we implement MR0 = ticks-1 and PR = 0
    // To make small MR0 values match, we need T0TC to increment at pclk_hz; so to have ticks correspond
    // to bit-subdivisions we must set PR to (pclk_hz / timer_tick_hz) - 1.
    uint32_t prescaler = (pclk_hz / timer_tick_hz) - 1;
    if (prescaler > 0xFFFFF) prescaler = 0xFFFFF;
    T0PR = prescaler;
    // Other timer regs will be set inside timer0_wait_ticks
}

/* Example usage:
 *
 *   can_gpio_init();
 *   can_timer_init(PCLK_HZ);  // e.g., PCLK_HZ = 15000000 or 60000000 depending on CCLK/PCLK
 *
 *   uint8_t dat[2] = {0x12, 0x34};
 *   bool ok = can_send(0x123, 2, dat);
 */

