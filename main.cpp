#if !defined(ESP8266)
  #error This code is designed to run on ESP8266 and ESP8266-based boards!
#endif

#include <Arduino.h>
#include <string>
//#include <LittleFS.h>
#include <EEPROM.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

#include <Ticker.h>
Ticker Timer1;

// Hardware I/O
int rx_pin = D2;  // D2
int tx_pin = D1;  // D1
int rx_led = D3;
int tx_led = D4;

// Timing
#define FREQ 433.42
#define bit_length 1280                // 1.28 ms
#define time_agc_pulse 10000           // 10 ms
#define time_agc_pulse_new 6500        // 6.5 ms
#define time_agc_stablize 90000        // 90 ms
#define time_agc_stablize_new 10000    // 10 ms
#define time_preamb_up 2*bit_length
#define time_preamb_dn 2*bit_length
#define time_softsync_up 3.5*bit_length
#define time_softsync_dn 0.5*bit_length
#define time_interframe 25*bit_length

// Receiving vars
#define max_bytes 12
volatile bool rx_frame_buffer[max_bytes * 8];
volatile uint8_t rx_frame_received[max_bytes];
volatile bool rx_done = false;
volatile uint8_t rx_bit_cnt = 0;
volatile uint8_t rx_preamble_cnt = 0;
volatile uint8_t rx_state = 0;
volatile bool rx_skip_next_edge = false;
volatile bool rx_in_frame = false;
volatile bool rx_last_bit = false;
volatile unsigned long rx_last_uS = 0;
volatile unsigned long rx_current_uS = 0;
volatile unsigned long rx_elapsed_uS = 0;

// Transmitting vars
volatile uint8_t tx_frame_tosend[max_bytes];
volatile bool tx_frame_buffer[max_bytes * 8];
volatile bool tx_start = false;
volatile bool tx_done = true;
volatile bool tx_newproto = false;
volatile bool tx_newtiming = false;
volatile uint8_t tx_state = 0;
volatile uint8_t tx_preamb = 0;
volatile uint8_t tx_bit_tosend = 56;
volatile uint8_t tx_bit_cnt = 0;
volatile bool tx_next_bit = 0;
volatile uint8_t tx_repeat = 0;
volatile bool tx_cc1101mode = false;

// cli vars
String cmd = "";
String dbgdsp = "";

// Remote control storage
struct RC {
  bool newprotocol = false;  // false:56 bit legacy; true:80 bit new
  bool newtiming = false;    // false:10+90ms AGC 2+7+7 preamble; true:7+10ms AGC 12+6+6 preamble
  uint32_t remoteid = 0;
  uint16_t channelid = 0;
  uint8_t actionid = 0;
  uint16_t rollingcode = 0;
  uint8_t encryptionkey = 0xa7;
  bool updated = false;
};

RC rx_rc;
RC tx_rc;

void IRAM_ATTR handlePinInterrupt();

void IRAM_ATTR handleTimerInterrupt() {
  uint32_t tx_len = 0;

  switch (tx_state) {
    case 0 :
      break;
    case 1 :  // Send AGC pulse
      GPOS = (1 << tx_pin);
      if (tx_newtiming) tx_len = time_agc_pulse_new; else tx_len = time_agc_pulse;
      tx_state = 2;
      break;
    case 2 :  // AGC stabilization time
      GPOC = (1 << tx_pin);
      if (tx_newtiming) tx_len = time_agc_stablize_new; else tx_len = time_agc_stablize;
      if (tx_newtiming) tx_preamb = 12; else tx_preamb = 2;
      tx_state = 11;
      break;
    case 11 : // Preamble (HW Sync) UP
      GPOS = (1 << tx_pin);
      tx_len = time_preamb_up;
      tx_state = 12;
      break;
    case 12 : // Preamble (HW Sync) DN
      GPOC = (1 << tx_pin);
      tx_len = time_preamb_dn;
      if (--tx_preamb > 0) tx_state = 11; else tx_state = 21;
      break;
    case 21 : // Soft sync UP
      GPOS = (1 << tx_pin);
      tx_len = time_softsync_up;
      tx_state = 22;
      break;
    case 22 : // Soft sync DN
      GPOC = (1 << tx_pin);
      tx_bit_cnt = 0; 
      tx_len = time_softsync_dn;
      tx_state = 31;
      break;
    case 31 : // Send Data
      tx_next_bit = !tx_frame_buffer[tx_bit_cnt++];
      if (tx_next_bit) GPOS = (1 << tx_pin); else GPOC = (1 << tx_pin);
      tx_len = 0.5 * bit_length;
      tx_state = 32;
      break;
    case 32 : // Send Data
      if (!tx_next_bit) GPOS = (1 << tx_pin); else GPOC = (1 << tx_pin);
      tx_len = 0.5 * bit_length;
      if (tx_bit_cnt < tx_bit_tosend) tx_state = 31; else tx_state = 41;
      break;
    case 41 : // Interframe
      GPOC = (1 << tx_pin);
      tx_len = time_interframe;
      if (tx_repeat-- > 0) {
        if (tx_newtiming) tx_preamb = 6; else tx_preamb = 7;
        tx_state = 11;
      }
      else { tx_state = 51; }
      break;
    case 51 : // Done
      tx_len = 0;
      tx_done = true;
      break;
  }

  if (tx_len > 0) {
    timer1_write(5 * tx_len);  // Timer clock run at 5 MHz (80 / 16) -> tx_len in µs
  }
}

void cc1101_rx() {
    detachInterrupt(digitalPinToInterrupt(rx_pin));
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(FREQ);
    ELECHOUSE_cc1101.SetRx();
    tx_cc1101mode = false;
    rx_state = 0;
    GPOS = (1 << tx_led); // TX LED OFF
    attachInterrupt(digitalPinToInterrupt(rx_pin), handlePinInterrupt, CHANGE);
}

void cc1101_tx() {
    detachInterrupt(digitalPinToInterrupt(rx_pin));
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(FREQ);
    ELECHOUSE_cc1101.SetTx();
    GPOC = (1 << tx_led); // TX LED ON
    tx_cc1101mode = true;
}

void process_tx() {
  // wait until frame sent, then switch back to RX mode
  if (tx_done && tx_cc1101mode) cc1101_rx();
}

void start_tx() {
  // Serialize frame to bit buffer
  for (uint8_t idx = 0; idx < tx_bit_tosend; idx++) {
    tx_frame_buffer[idx] = tx_frame_tosend[idx / 8] & (1 << (7 - (idx % 8)));
  }
  // switch radio tx on, initialize tx state machine and launch timer
  tx_done = false;
  tx_state = 1;
  cc1101_tx();
  timer1_write(100);
}

void send_frame(bool new_proto) {
  if (new_proto)
  {
    tx_newproto = true;
    tx_newtiming = true;
    tx_bit_tosend = 80;
  }
  else {
    tx_newproto = false;
    tx_newtiming = false;
    tx_bit_tosend = 56;
  }

  tx_frame_tosend[0] = 0xA7; tx_frame_tosend[1] = 0x82; tx_frame_tosend[2] = 0x82; tx_frame_tosend[3] = 0xd5;
  tx_frame_tosend[4] = 0xf4; tx_frame_tosend[5] = 0xe7; tx_frame_tosend[6] = 0xee;

  tx_frame_tosend[7] = 0xaa; tx_frame_tosend[8] = 0xaa; tx_frame_tosend[9] = 0x00;

  tx_repeat = 2;
  start_tx();
}

void IRAM_ATTR handlePinInterrupt() {
  rx_current_uS = micros();
  rx_elapsed_uS = rx_current_uS - rx_last_uS;
  rx_last_uS = rx_current_uS;

  if (rx_elapsed_uS > 20 * bit_length) { // Longer than 25.6ms (20 bits) (interframe is about 32ms (25 bits)) - reset everything
    rx_bit_cnt = 0;
    rx_preamble_cnt = 0;
    rx_in_frame = false;
    rx_state = 0;
    GPOS = (1 << rx_led); // RX LED OFF
  }
  else if (rx_elapsed_uS > 4 * bit_length) {  // 6ms..25ms : Error discard egde
  }
  else if ((rx_elapsed_uS > 3 * bit_length) && (rx_preamble_cnt > 0)) {  // 3 to 4 bit length : catch 4.5 to 4.8ms of last preamble pulse (3.5 bit length) (Software Sync)
    rx_state = 2;
    rx_bit_cnt = 0;
    rx_skip_next_edge = 0;
    rx_last_bit = false;
    rx_in_frame = true;
  }
  else if (rx_elapsed_uS > 1.5 * bit_length) { // 1.5 to 3 bit length : catch 2.56 ms preamble pulses (2 bit length) (Hardware sync)
    rx_preamble_cnt ++;
    rx_state = 1;
    GPOC = (1 << rx_led); // RX LED ON
  }
  else if (rx_in_frame && (rx_elapsed_uS > 0.75 * bit_length)) { // .75 to 1.5 bit length : catch 1.28 ms edge (1 bit length) (next bit flipped in manchester coding)
    rx_state = 3;
    rx_last_bit = !rx_last_bit;
    rx_frame_buffer[rx_bit_cnt++] = rx_last_bit;
  }
  else if (rx_in_frame && (rx_elapsed_uS > 0.25 * bit_length)) { // .25 to 0.75 bit length : catch 0.64 ms edge (1/2 bit length) (next bit same in manchester coding)
    rx_state = 3;
    if (!rx_skip_next_edge) { rx_frame_buffer[rx_bit_cnt++] = rx_last_bit; rx_skip_next_edge = true; }
    else rx_skip_next_edge = false;
  }
  else { // less than .25 bit length - ignore (bounce or interference)
  }

  // Limit frame length to buffer length
  if (rx_bit_cnt >= max_bytes * 8) rx_bit_cnt--;  
}



bool process_rx() {
  // Check if frame is ready (bits received and no more bit since more than 5 bits length)
  // Check is using 'rx_last_uS' wich is update by the ISR.  This is not atomic, so it can happen that the result is wrong
  // as this routine is call very often.  That's why there is 2 successive tests to be avoid false positive.
  if ((!rx_done) && (rx_state == 3) && ((micros() - rx_last_uS) > 5 * bit_length) && ((micros() - rx_last_uS) > 5 * bit_length)) {
    // Disable Interrupt during stream processing
    detachInterrupt(digitalPinToInterrupt(rx_pin));
    // init state machine for next frame and set done flag
    rx_state = 0;
    if (rx_bit_cnt & 0x01) { rx_bit_cnt &= 0xfe; Serial.println("Bit cnt error - fixing"); }
    rx_done = true;
    GPOS = (1 << rx_led); // RX LED OFF
  }
  
  // If stuck and no more receiving, reset state machine for next frame
  if ((rx_state != 0) && ((micros() - rx_last_uS) > 50 * bit_length) && ((micros() - rx_last_uS) > 55 * bit_length)) {
    rx_state = 0;
    GPOS = (1 << rx_led); // RX LED OFF
  }

  uint8_t check_digit = 0xff;

  // If frame received done and bit count is correct, process the frame data
  if ((rx_done) && ((rx_bit_cnt == 56) || (rx_bit_cnt  == 80))) {
    // clear received bytes buffer - not using memset as array is volatile
    for (unsigned int idx = 0; idx < sizeof(rx_frame_received); idx++) rx_frame_received[idx] = 0;
    // fill received bytes buffer from serial bit stream
    for (uint8_t idx = 0; idx < rx_bit_cnt; idx++) {
      rx_frame_received[idx / 8] |= rx_frame_buffer[idx] << (7 - (idx % 8));
    }
    // remove RTS obfuscasion
    uint8_t last_byte = 0;
    uint8_t new_byte = 0;
    for (int idx = 0; idx < rx_bit_cnt / 8; idx++) {
      new_byte = rx_frame_received[idx] ^ last_byte;
      last_byte = rx_frame_received[idx];
      rx_frame_received[idx] = new_byte;
    }
    // calculate check digit
    check_digit = 0;
    for (uint8_t idx = 0; idx < rx_bit_cnt / 8; idx++) {
      check_digit = check_digit ^ rx_frame_received[idx] ^ (rx_frame_received[idx] >> 4);
    }
    check_digit &= 0x0f;               // check digit is only low nibble
    rx_frame_received[1] &= 0xf0;         // remove it from frame
    rx_frame_received[1] |= check_digit;  // add calculated check to frame (should be zero for good frame)

    // Debug information of received frame on serial
    char txt[255];
    sprintf(txt, "Frame received : %i bit %i preamble pulses -", rx_bit_cnt, rx_preamble_cnt / 2);
    dbgdsp = txt;
    for (uint8_t idx = 0; idx < rx_bit_cnt / 8; idx++) { sprintf(txt, " %02X", rx_frame_received[idx]); dbgdsp += txt; }

    // update data struct if check digit ok
    if (check_digit == 0) {
      rx_rc.encryptionkey = rx_frame_received[0];
      rx_rc.actionid = (rx_frame_received[1] & 0xf0) >> 4;
      rx_rc.rollingcode = (rx_frame_received[3] + (rx_frame_received[2] << 8));  // LSB first
      rx_rc.channelid = rx_frame_received[4];
      rx_rc.remoteid = (rx_frame_received[5] + (rx_frame_received[6] << 8));  // MSB First
      rx_rc.updated = true;
    }
  }

  if (rx_done) {
    // restore interrupt for next frame and return
    rx_bit_cnt = 0;
    rx_done = false;
    rx_state = 0;
    attachInterrupt(digitalPinToInterrupt(rx_pin), handlePinInterrupt, CHANGE);
  }

  return((check_digit == 0));
}


void show_rc(RC rc) {
  char txt[10];
  const char ActTxt[] = "--      My      Up      My+Up   Dn      My+Dn   Up+Dn   --      Prog    Sun+FlagFlag    --      --      --      --      --      ";
  Serial.print("Remote ID      : "); sprintf(txt, "%04X", rc.remoteid); Serial.println(txt);
  Serial.print("Channel ID     : "); sprintf(txt, "%02X", rc.channelid); Serial.println(txt);
  Serial.print("Encryption key : "); sprintf(txt, "%02X", rc.encryptionkey); Serial.println(txt);
  Serial.print("Rolling code   : "); sprintf(txt, "%04X", rc.rollingcode); Serial.println(txt);
  Serial.print("Action code    : "); sprintf(txt, "%02X", rc.actionid); Serial.println(txt);
  Serial.print("Action         : ");
  for (uint8_t idx = 0; idx < 7; idx++) Serial.print(ActTxt[(rc.actionid & 0x0f) * 8 + idx]);
  Serial.println();
}

void exec_cmd() {
  switch (cmd[0]) {
    case 'n' :
      send_frame(true);
      break;
    case 'o' :
      send_frame(false);
      break;
    case 's' :
      show_rc(rx_rc);
      break;
    case 'w' :    
      EEPROM.put(0, rx_rc);
      EEPROM.commit();
      break;
    case 'r' :    
      EEPROM.get(0, rx_rc);
      break;

   }
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
        exec_cmd();
        Serial.print("> ");
        cmd = "";
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
    Serial.print("\nRTS Rx/Tx\n");

    EEPROM.begin(1024);

    // Init CC1101 RF chip
    Serial.print("CC1101 radio module...");
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(FREQ);
    if (ELECHOUSE_cc1101.getCC1101()) {
      Serial.println();
      Serial.print("Version   : "); Serial.println(ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION), HEX);
      Serial.print("Frequency : "); Serial.println(FREQ);
    }
    else
    {
      Serial.println(" module not found !");
    }

    Serial.print("> ");

    // Set Digital I/O and Interrupt handlers
    // RX PIN, Interrupt & CC1101 receiving mode
    pinMode(rx_pin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(rx_pin), handlePinInterrupt, CHANGE);
    cc1101_rx();

    // setup TX timer
    delay(10);
    pinMode(tx_pin, OUTPUT);
    GPOC = (1 << tx_pin);
    timer1_attachInterrupt(handleTimerInterrupt);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);

    // LED
    pinMode(rx_led, OUTPUT);
    digitalWrite(rx_led, HIGH);
    pinMode(tx_led, OUTPUT);
    digitalWrite(tx_led, HIGH);
}



void loop() {

    process_serial();

    process_rx();
    process_tx();


}