#if !defined(ESP8266)
  #error This code is designed to run on ESP8266 and ESP8266-based boards!
#endif

#include <Arduino.h>
#include <string>

#include <EEPROM.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

#include <Ticker.h>
Ticker Timer1;

// Timing
#define bit_length 1280      // 1.28 ms
#define time_agc_pulse 10000      // 10 ms
#define time_agc_stablize 90000   // 90 ms
#define time_preamb_up 2*bit_length
#define time_preamb_dn 2*bit_length
#define time_softsync_up 3.5*bit_length
#define time_softsync_dn 0.5*bit_length
#define time_interframe 25*bit_length

// cli vars
String cmd = "";
String dbgdsp = "";

// Receiving vars
#define max_bytes 12
volatile bool frame_buffer[max_bytes * 8];
volatile uint8_t frame_received[max_bytes];
volatile uint8_t bit_cnt;
volatile uint8_t preamble;
volatile bool skip_next_edge;
volatile bool in_frame;
volatile bool last_bit;
volatile unsigned long last_uS = 0;
volatile unsigned long current_uS;
volatile unsigned long elapsed_uS;

// Transmitting vars
volatile uint8_t tx_frame[max_bytes];
volatile bool tx_frame_buffer[max_bytes * 8];
volatile uint8_t tx_state = 0;
volatile bool tx_start = false;
volatile uint8_t tx_preamb = 0;
volatile uint8_t tx_bit_cnt = 0;
volatile bool tx_next_bit = 0;
volatile uint8_t tx_repeat = 0;
volatile bool tx_done = true;

int RxPin = D2;  // D2
int TxPin = D1;  // D1
int LED = D3;


void IRAM_ATTR handleTimerInterrupt() {
  uint32_t tx_len = 0;

  switch (tx_state) {
    case 0 :
      break;
    case 1 :  // Send AGC pulse
      GPOS = (1 << LED);
      tx_len = time_agc_pulse;
      tx_state = 2;
      break;
    case 2 :  // AGC stabilization time
      GPOC = (1 << LED);
      tx_len = time_agc_stablize;
      tx_preamb = 2;
      tx_state = 11;
      break;
    case 11 : // Preamble (HW Sync) UP
      GPOS = (1 << LED);
      tx_len = time_preamb_up;
      tx_state = 12;
      break;
    case 12 : // Preamble (HW Sync) DN
      GPOC = (1 << LED);
      tx_len = time_preamb_dn;
      if (--tx_preamb > 0) tx_state = 11; else tx_state = 21;
      break;
    case 21 : // Soft sync UP
      GPOS = (1 << LED);
      tx_len = time_softsync_up;
      tx_state = 22;
      break;
    case 22 : // Soft sync DN
      GPOC = (1 << LED);
      tx_bit_cnt = 0; 
      tx_len = time_softsync_dn;
      tx_state = 31;
      break;
    case 31 : // Send Data
      tx_next_bit = !tx_frame_buffer[tx_bit_cnt++];
      if (tx_next_bit) GPOS = (1 << LED); else GPOC = (1 << LED);
      tx_len = 0.5 * bit_length;
      tx_state = 32;
      break;
    case 32 : // Send Data
      if (!tx_next_bit) GPOS = (1 << LED); else GPOC = (1 << LED);
      tx_len = 0.5 * bit_length;
      if (tx_bit_cnt < 56) tx_state = 31; else tx_state = 41;
      break;
    case 41 : // Interframe
      GPOC = (1 << LED);
      tx_len = time_interframe;
      if (tx_repeat-- > 0) { tx_preamb = 7; tx_state = 11; } else { tx_state = 51; }
      break;
    case 51 : // Done
      tx_len = 0;
      tx_done = true;
      break;
  }

  if (tx_len > 0) {
    timer1_write(5 * tx_len);  // Clock run at 5 MHz (80 / 16) -> tx_len in µs
  }
}

void process_tx() {
    tx_frame[0] = 0xA7; tx_frame[1] = 0x82; tx_frame[2] = 0x82; tx_frame[3] = 0xd5;
    tx_frame[4] = 0xf4; tx_frame[5] = 0xe7; tx_frame[6] = 0xee;

    for (uint8_t idx = 0; idx < 56; idx++) {
      tx_frame_buffer[idx] = tx_frame[idx / 8] & (1 << (7 - (idx % 8)));
    }

  tx_state = 1;
  tx_repeat = 2;
  timer1_write(1000);
}

void IRAM_ATTR handlePinInterrupt() {
  current_uS = micros();
  elapsed_uS = current_uS - last_uS;
  last_uS = current_uS;

  if (elapsed_uS > 20 * bit_length) { // Longer than 25.6ms (20 bits) (interframe is about 32ms (25 bits)) - reset everything
    bit_cnt = 0;
    preamble = 0;
    in_frame = false;
  }
  else if (elapsed_uS > 4 * bit_length) {  // 6ms..25ms : Error discard egde
  }
  else if (elapsed_uS > 3 * bit_length) {  // 3 to 4 bit length : catch 4.5 to 4.8ms of last preamble pulse (3.5 bit length) (Software Sync)
    skip_next_edge = 0;
    last_bit = false;
    in_frame = true;
  }
  else if (elapsed_uS > 1.5 * bit_length) { // 1.5 to 3 bit length : catch 2.56 ms preamble pulses (2 bit length) (Hardware sync)
    preamble ++;
  }
  else if (in_frame && (elapsed_uS > 0.75 * bit_length)) { // .75 to 1.5 bit length : catch 1.28 ms edge (1 bit length) (next bit flipped in manchester coding)
    last_bit = !last_bit;
    frame_buffer[bit_cnt++] = last_bit;
  }
  else if (in_frame && (elapsed_uS > 0.25 * bit_length)) { // .25 to 0.75 bit length : catch 0.64 ms edge (1/2 bit length) (next bit same in manchester coding)
    if (!skip_next_edge) { frame_buffer[bit_cnt++] = last_bit; skip_next_edge = true; }
    else skip_next_edge = false;
  }
  else { // less than .25 bit length - ignore (bounce or interference)
  }

  // Limit frame length to buffer length
  if (bit_cnt >= max_bytes * 8) bit_cnt--;  
}


void start_rx() {
    pinMode(RxPin,INPUT);
    RxPin = digitalPinToInterrupt(RxPin);
    ELECHOUSE_cc1101.SetRx();
    attachInterrupt(RxPin, handlePinInterrupt, CHANGE);
}

void disable_rx() {
    detachInterrupt(RxPin);
}


bool process_rx() {
  if ((bit_cnt >= 56) && (bit_cnt < sizeof(frame_buffer))) {
    // Disable Interrupt during stream processing
    detachInterrupt(RxPin);
    // clear received bytes buffer - not using memset as array is volatile
    for (unsigned int idx = 0; idx < sizeof(frame_received); idx++) frame_received[idx] = 0;
    // fill received bytes buffer from serial bit stream
    for (uint8_t idx = 0; idx < bit_cnt; idx++) {
      frame_received[idx / 8] |= frame_buffer[idx] << (7 - (idx % 8));
    }
    // remove RTS obfuscasion
    uint8_t last_byte = 0;
    uint8_t new_byte = 0;
    for (int idx = 0; idx < bit_cnt / 8; idx++) {
      new_byte = frame_received[idx] ^ last_byte;
      last_byte = frame_received[idx];
      frame_received[idx] = new_byte;
    }
    // calculate check digit
    uint8_t check_digit = 0;
    for (uint8_t idx = 0; idx < 7; idx++) {
      check_digit = check_digit ^ frame_received[idx] ^ (frame_received[idx] >> 4);
    }
    check_digit &= 0x0f;               // check digit is only low nibble
    frame_received[1] &= 0xf0;         // remove it from frame
    frame_received[1] |= check_digit;  // add calculated check to frame (should be zero for good frame)

    // Debug information of received frame on serial
    char txt[255];
    sprintf(txt, "Frame received : %i bit %i preamble pulses -", bit_cnt, preamble);
    dbgdsp = txt;
    for (uint8_t idx = 0; idx < 7; idx++) { sprintf(txt, " %02X", frame_received[idx]); dbgdsp += txt; }

    // restore interrupt for next frame and return
    bit_cnt = 0;
    attachInterrupt(RxPin, handlePinInterrupt, CHANGE);
    return((check_digit == 0));
  }
  return (false);
}


void process_serial() {

  if (dbgdsp.length() > 0) {
      for (unsigned int idx = 0; idx < 2+cmd.length(); idx++) Serial.print("\b \b");
      Serial.println(dbgdsp);
      dbgdsp = "";
      Serial.print("> ");
      Serial.print(cmd);
  }

  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch)
    {
    case '\r':
        Serial.println();
        Serial.println(cmd);
        Serial.print("> ");
        cmd = "";
        process_tx();
        break;
    case '\n': break;
    case '\b':
        if (cmd.length() > 0) {
            cmd.remove(cmd.length()-1);
            Serial.print("\b \b");
        }
        break;
    default:
        cmd += ch;
        Serial.print(ch);
        break;
    }
  }
}


void setup() {
    // Init serial port
    Serial.begin(115200);  
    Serial.print("\nRTS Rx/Tx Started\n> ");

    // Init CC1101 RF chip
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(433.42);

    // Set Digital I/O
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // setup TX timer
    timer1_attachInterrupt(handleTimerInterrupt);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);

    // Start receiving
    start_rx();
}



void loop() {

    process_serial();

    process_rx();

}