#if !defined(ESP8266)
  #error This code is designed to run on ESP8266 and ESP8266-based boards!
#endif

// Include framework & embedded hardware libs
#include <Arduino.h>
#include <string>
#include <Ticker.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Include specific hardware libs
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// Include project specific headers
#include "cred.h"

// Timer
Ticker Timer1;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);


// Some constants
#define MAX_REMOTE 40     // Maximum number of remote
#define MAX_SHUTTER 20    // Maximum number of shutter
#define MAX_AUX 10        // Maximum number of aux remote per shutter

#define SH_STATUS_CLOSED 0
#define SH_STATUS_CLOSING 1
#define SH_STATUS_OPEN 2
#define SH_STATUS_OPENING 3

#define MQTT_TOPIC "home-assistant/cover"
#define MQTT_TOPIC_SUB "home-assistant/cover/#"
#define MQTT_LWT "home-assistant/cover/availability"

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
#define max_frame_bit 80
volatile bool rx_frame_buffer[max_frame_bit + 1];
volatile uint8_t rx_frame_received[(max_frame_bit / 8) + 1];
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
volatile uint8_t tx_frame_tosend[(max_frame_bit / 8) + 1];
volatile bool tx_frame_buffer[max_frame_bit];
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
std::string line = "";
std::string dbgdsp = "";

// Network vars
bool wifi_connected = false;
bool mqtt_connected = false;
uint8_t eeprom_to_save = 0;

// Remote control storage
struct RC {
  bool newprotocol = false;  // false:56 bit legacy; true:80 bit new
  bool newtiming = false;    // false:10+90ms AGC 2+7+7 preamble; true:7+10ms AGC 12+6+6 preamble
  uint16_t remoteid = 0;
  uint8_t channelid = 0;
  uint8_t actionid = 0;
  uint16_t rollingcode = 0;
  uint8_t encryptionkey = 0xa7;
  bool updated = false;
};

RC rc[MAX_REMOTE];
RC rx_rc;
RC tx_rc;
RC tmp_rc;

// Shutter storage
struct Shutter {
  uint8_t rc_cmd_id = 0xff;     // motor command remote
  uint8_t rc_aux_id[MAX_AUX];   // aux remotes
  bool do_relay = false;        // relay aux remote to cmd remote
  uint8_t op_time = 0;          // operating time (s)
  uint8_t position = 0;         // 0..100 (close to open)
  uint8_t action = 0;           // current action
  uint8_t status = 0;           // current status
  uint8_t cmd = 0;              // cmd requested
  uint8_t remain_time = 0;      // current operation remaining time
  bool updated = false;
  bool send_position = false;
  bool send_status = false;
};

Shutter shut[MAX_SHUTTER];

// Other vars
unsigned long currentmillis = millis();
unsigned long lastmillis = currentmillis;
uint32_t uptime = 0;
uint8_t loopidx = 0;
char uptime_txt[48];

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

void do_rolling(RC *rc) {
  rc->encryptionkey = 0xA0 | ((rc->encryptionkey - 1) & 0x0f);
  rc->rollingcode += 1;
  rc->updated = true;
}

void send_frame(RC *rc, uint8_t repeat) {
  tx_newproto = rc->newprotocol;
  tx_newtiming = rc->newtiming;
  if (tx_newproto) tx_bit_tosend = 80; else tx_bit_tosend = 56;
  
  do_rolling(rc);

  tx_frame_tosend[0] = rc->encryptionkey;
  tx_frame_tosend[1] = rc->actionid << 4;
  tx_frame_tosend[2] = (rc->rollingcode & 0xff00) >> 8; tx_frame_tosend[3] = rc->rollingcode & 0xff;
  tx_frame_tosend[4] = rc->channelid;
  tx_frame_tosend[5] = rc->remoteid & 0xff; tx_frame_tosend[6] = (rc->remoteid & 0xff00) >> 8;
  tx_frame_tosend[7] = 0xa0; tx_frame_tosend[8] = 0x00; tx_frame_tosend[9] = 0x25;  // Temp - To be redefined

  uint8_t check_digit = 0;
  for (uint8_t idx = 0; idx < tx_bit_tosend / 8; idx++) {
    check_digit = check_digit ^ tx_frame_tosend[idx] ^ (tx_frame_tosend[idx] >> 4);
  }
  tx_frame_tosend[1] = (tx_frame_tosend[1] & 0xf0) + (check_digit & 0x0f);

  tx_repeat = repeat;

  // Debug information of received frame on serial
  char txt[255];
  sprintf(txt, "Sending frame  : %i bit - %i repeat -", tx_bit_tosend, tx_repeat);
  dbgdsp = txt;
  for (uint8_t idx = 0; idx < tx_bit_tosend / 8; idx++) { sprintf(txt, " %02X", tx_frame_tosend[idx]); dbgdsp += txt; }

  for (uint8_t idx = 1; idx < 9; idx++) tx_frame_tosend[idx] ^= tx_frame_tosend[idx-1];

  dbgdsp += " -";
  for (uint8_t idx = 0; idx < tx_bit_tosend / 8; idx++) { sprintf(txt, " %02X", tx_frame_tosend[idx]); dbgdsp += txt; }

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
  if (rx_bit_cnt >= max_frame_bit + 1) rx_bit_cnt--;  
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

void process_rxframe() {
  char Txt[10] = "";
  char rc_list[3 * MAX_REMOTE + 1] = "";
  char shut_list[3 * MAX_SHUTTER + 1] = "";
  if (rx_rc.updated == true) {
    for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) {
      if ((rx_rc.remoteid == rc[idx].remoteid) && (rx_rc.channelid == rc[idx].channelid)) {
        if ((rx_rc.rollingcode > rc[idx].rollingcode) && (rx_rc.rollingcode < rc[idx].rollingcode + 100)) {
          rc[idx].rollingcode = rx_rc.rollingcode;
          rc[idx].encryptionkey = rx_rc.encryptionkey;
          rc[idx].actionid = rx_rc.actionid;
          rc[idx].updated = true;
          sprintf(Txt, " %2i", idx);
          strcat(rc_list, Txt);
          for (uint8_t idx_shutter = 0; idx_shutter < MAX_SHUTTER; idx_shutter++)
            for (uint8_t idx_shutter_remote = 0; idx_shutter_remote < MAX_AUX; idx_shutter_remote++) {
              if (idx == shut[idx_shutter].rc_aux_id[idx_shutter_remote]) {
                shut[idx_shutter].action = rx_rc.actionid;
                sprintf(Txt, " %2i", idx_shutter);
                strcat(shut_list, Txt);
              }
            }
        }
      }
    }
    if (strlen(rc_list) == 0) strcpy(rc_list, " none");
    if (strlen(shut_list) == 0) strcpy(shut_list, " none");
    dbgdsp = "Remote match :" + std::string(rc_list) + " Shutter match :" + std::string(shut_list);
    rx_rc.updated = false;
  }
}

void process_shutter(bool is_second) {
  for (uint8_t idx = 0; idx < MAX_SHUTTER; idx++) {
    switch (shut[idx].action) {
      case 0:
              break;
      case 2: // Up
              if (is_second && (shut[idx].position < 100)) {
                if (shut[idx].op_time != 0) shut[idx].position += (100 / shut[idx].op_time); else shut[idx].position = 100;
                if (shut[idx].position > 100) shut[idx].position = 100;
                shut[idx].updated = true;
                shut[idx].send_position = true;
              }
              if (shut[idx].status != SH_STATUS_OPENING) {
                shut[idx].status = SH_STATUS_OPENING;
                shut[idx].send_status = true;
              }
              if (shut[idx].position == 100) {
                shut[idx].status = SH_STATUS_OPEN;
                shut[idx].send_status = true;
                shut[idx].action = 0;
              }
              break;
      case 4: // Down
              if (is_second && (shut[idx].position > 0)) {
                if (shut[idx].op_time != 0) shut[idx].position -= (100 / shut[idx].op_time); else shut[idx].position = 0;
                if (shut[idx].position > 100) shut[idx].position = 0;  // overflow on previous substraction
                shut[idx].updated = true;
                shut[idx].send_position = true;
              }
              if (shut[idx].status != SH_STATUS_CLOSING) {
                shut[idx].status = SH_STATUS_CLOSING;
                shut[idx].send_status = true;
              }
              if (shut[idx].position == 0) {
                shut[idx].status = SH_STATUS_CLOSED;
                shut[idx].send_status = true;
                shut[idx].action = 0;
              }
              break;
    }
  }
}

bool show_rc(RC rc, int idx, bool force = false, bool headspace = false) {
  if (force || (rc.remoteid != 0 && rc.remoteid != 0xffff)) {
    if (headspace) Serial.println();
    char txt[12];
    const char ActTxt[] = "--      My      Up      My+Up   Dn      My+Dn   Up+Dn   --      Prog    Sun+FlagFlag    --      --      --      --      --      ";
    Serial.print("Remote idx     : ");
    if (idx >= 0) { sprintf(txt, "%02i", idx); Serial.println(txt); }
    if (idx == -1) Serial.println("Last RX");
    if (idx == -2) Serial.println("Next TX");
    Serial.print("Remote ID      : "); sprintf(txt, "%04X", rc.remoteid); Serial.println(txt);
    Serial.print("Channel ID     : "); sprintf(txt, "%02X", rc.channelid); Serial.println(txt);
    Serial.print("Encryption key : "); sprintf(txt, "%02X", rc.encryptionkey); Serial.println(txt);
    Serial.print("Protocol       : "); if (rc.newprotocol) Serial.println("80 bits"); else Serial.println("56 bits");
    Serial.print("Timing         : "); if (rc.newtiming) Serial.println("12 - 6 preambles"); else Serial.println("2 - 7 preambles");
    Serial.print("Rolling code   : "); sprintf(txt, "%04X", rc.rollingcode); Serial.println(txt);
    Serial.print("Action code    : "); sprintf(txt, "%02X", rc.actionid); Serial.println(txt);
    Serial.print("Action         : ");
    for (uint8_t idx = 0; idx < 7; idx++) Serial.print(ActTxt[(rc.actionid & 0x0f) * 8 + idx]);
    Serial.println();
    return(true);
  }
  return(false);
}

bool show_shutter(Shutter sh, int idx, bool force = false, bool headspace = false) {
  if (force || (sh.rc_cmd_id != 0xff) || (sh.rc_aux_id[0] != 0xff)) {  // To be optimize as 1st aux could be empty
    if (headspace) Serial.println();
    char txt[12];
    Serial.print("Shutter idx    : "); sprintf(txt, "%02i", idx); Serial.println(txt);
    Serial.print("Cmd remote idx : "); sprintf(txt, "%02i", sh.rc_cmd_id); Serial.println(txt);
    Serial.print("Aux remote idx :"); for (int i = 0; i<MAX_AUX; i++) { sprintf(txt, " %02i", sh.rc_aux_id[i]); Serial.print(txt); }; Serial.println();
    Serial.print("Relay Aux->Cmd : "); if (sh.do_relay) Serial.println("Yes"); else Serial.println("No");
    Serial.print("Open/Close time: "); sprintf(txt, "%02i", sh.op_time); Serial.println(txt);
    Serial.print("Action         : "); sprintf(txt, "%02x", sh.action); Serial.println(txt);
    Serial.print("Position       : "); sprintf(txt, "%03i", sh.position); Serial.println(txt);
    Serial.print("Command        : "); sprintf(txt, "%03i", sh.cmd); Serial.println(txt);
    return(true);
  }
  return(false);
}

std::string trim(const std::string& str,
                 const std::string& whitespace = " \t")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

void data_save() {
  for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) { rc[idx].updated = false; EEPROM.put(idx*50, rc[idx]); }
  EEPROM.put(2000, rx_rc);
  EEPROM.put(2100, tx_rc);
  for (uint8_t idx = 0; idx < MAX_SHUTTER; idx++) EEPROM.put(3000+idx*50, shut[idx]);
  EEPROM.commit();
  dbgdsp = "Data saved";
}

void data_load() {
  for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) EEPROM.get(idx*50, rc[idx]);
  EEPROM.get(2000, rx_rc);
  EEPROM.get(2100, tx_rc);
  for (uint8_t idx = 0; idx < MAX_SHUTTER; idx++) EEPROM.get(3000+idx*50, shut[idx]);
  dbgdsp = "Data loaded";
}

void uptime_to_text(char const *header, char const *trailer) {
  int d = uptime/86400;
  int h = (uptime - (d*86400)) / 3600;
  int m = (uptime - (d*86400) - (h*3600)) / 60;
  int s = uptime % 60;
  sprintf(uptime_txt, "%s%4id %02ih:%02im:%02is%s", header, d, h, m, s, trailer);
}

void exec_cmd() {
  std::string cmd = "";
  std::string param1 = "";
  std::string param2 = "";
  std::string param3 = "";
  int p1 = -1;
  int p2 = -1;
  int p3 = -1;
  uint8_t idx = 0;
  line = trim(line);
  while ((idx < line.length()) & (line[idx] != ' ')) {
    cmd += line[idx];
    idx++;
  }
  while ((idx < line.length()) & (line[idx] == ' ')) idx++;
  while ((idx < line.length()) & (line[idx] != ' ')) {
    param1 += line[idx];
    idx++;
  }
  while ((idx < line.length()) & (line[idx] == ' ')) idx++;
  while ((idx < line.length()) & (line[idx] != ' ')) {
    param2 += line[idx];
    idx++;
  }
  while ((idx < line.length()) & (line[idx] == ' ')) idx++;
  while ((idx < line.length()) & (line[idx] != ' ')) {
    param3 += line[idx];
    idx++;
  }
  try { p1 = std::stoi(param1, nullptr, 0); } catch(...) {}
  try { p2 = std::stoi(param2, nullptr, 0); } catch(...) {}
  try { p3 = std::stoi(param3, nullptr, 0); } catch(...) {}
 
  if ((cmd == "show") || (cmd == "sh")) {
    bool found = false;
    if (param1 == "rx") show_rc(rx_rc, -1, true);
    if (param1 == "tx") show_rc(tx_rc, -2, true);
    if (param1 == "remote" || param1 == "rc") {
      if ((p2 >= 0) && (p2 < MAX_REMOTE)) show_rc(rc[p2], p2, true, false);
      else for (idx = 0; idx < MAX_REMOTE; idx++) { found |= show_rc(rc[idx], idx, false, found); }
    }
    if (param1 == "shutter" || param1 == "device" || param1 == "dev") {
      if ((p2 >= 0) && (p2 < MAX_SHUTTER)) show_shutter(shut[p2], p2, true, false);
      else for (idx = 0; idx < MAX_SHUTTER; idx++) { found |= show_shutter(shut[idx], idx, false, found); }
    }
    if (param1 == "uptime") {
      uptime_to_text("Uptime         : ", "");
      Serial.println(uptime_txt);
    }
  }
  if ((cmd == "copy") || (cmd == "cp")) {
    tmp_rc.encryptionkey = 0xff;
    if ((p1 >= 0) && (p1 < MAX_REMOTE)) tmp_rc = rc[p1];
    else if (param1 == "rx") tmp_rc = rx_rc;
    else if (param1 == "tx") tmp_rc = tx_rc;
    if (tmp_rc.encryptionkey != 0xff) {
      if ((p2 >= 0) && (p2 < MAX_REMOTE)) rc[p2] = tmp_rc;
      else if (param2 == "rx") rx_rc = tmp_rc;
      else if (param2 == "tx") tx_rc = tmp_rc;
    }
  }
  if ((cmd == "roll") || (cmd == "rl")) {
    if ((p1 >= 0) && (p1 < MAX_REMOTE)) do_rolling(&rc[p1]);
    else if (param1 == "rx") do_rolling(&rx_rc);
    else if (param1 == "tx") do_rolling(&tx_rc);
  }
  if ((cmd == "link") || (cmd == "ln")) {
    if ((p1 >= 0) && (p1 < MAX_REMOTE)) {
      if ((p2 >= 0) && (p2 < MAX_SHUTTER)) {
        int cmd_idx = -1;
        for (uint8_t idx = 0; idx < MAX_AUX; idx++) if (shut[p2].rc_aux_id[idx] == p1) cmd_idx = idx;
        if (cmd_idx < 0) for (uint8_t idx = 0; idx < MAX_AUX; idx++) if (shut[p2].rc_aux_id[idx] == 0xff) { cmd_idx = idx; break; }
        if (cmd_idx >= 0) shut[p2].rc_aux_id[cmd_idx] = p1;
      }
    }
  }
  if ((cmd == "unlink") || (cmd == "ul")) {
    if ((p1 >= 0) && (p1 < MAX_REMOTE)) {
      if ((p2 >= 0) && (p2 < MAX_SHUTTER)) {
        for (uint8_t idx = 0; idx < MAX_AUX; idx++) if (shut[p2].rc_aux_id[idx] == p1) shut[p2].rc_aux_id[idx] = 0xff;
      }
    }
  }
  if (cmd == "save") data_save();
  if (cmd == "load") data_load();
  if ((cmd == "edit") || (cmd == "ed")) {
    if (param1 == "rid") tx_rc.remoteid = p2;
    if (param1 == "cid") tx_rc.channelid = p2;
    if (param1 == "rc") tx_rc.rollingcode = p2;
    if (param1 == "ac") tx_rc.actionid = p2;
    if (param1 == "enc") tx_rc.encryptionkey = p2;
    if (param1 == "proto" and param2 == "56") tx_rc.newprotocol = false;
    if (param1 == "proto" and param2 == "80") tx_rc.newprotocol = true;
  }
  if ((cmd == "device") || (cmd == "dev")) {
    if ((p2 > 0) && (p2 < MAX_SHUTTER)) {
      if ((param1 == "opt") && (p3 >= 0) && (p3 <= 100)) shut[p2].op_time = p3;
      if ((param1 == "pos") && (p3 >= 0) && (p3 <= 100)) { shut[p2].position = p3; shut[p2].send_position = true; }
    }
  }
  if (cmd == "send") {
      send_frame(&tx_rc, 2);
  }
}

void process_serial() {

  if (dbgdsp.length() > 0) {
      for (unsigned int idx = 0; idx < 2+line.length(); idx++) Serial.print("\b \b");
      uptime_to_text("[", " ] ");
      Serial.print(uptime_txt);
      Serial.println(dbgdsp.c_str());
      dbgdsp = "";
      Serial.print("> ");
      Serial.print(line.c_str());
  }

  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch)
    {
    case '\r':
        Serial.println();
        exec_cmd();
        Serial.print("> ");
        line = "";
        break;
    case '\n': break;
    case '\b':
        if (line.length() > 0) {
            line.pop_back();
            Serial.print("\b \b");
        }
        break;
    default:
        if (line.length() < 80) {
          line += ch;
          Serial.print(ch);
        }
        break;
    }
  }
}

void process_mqtt() {
  if (mqtt_client.connected()) {
    for (uint8_t idx = 0; idx < MAX_SHUTTER; idx++) {
      char topic[80];
      char value[40];
      if (shut[idx].send_position) {
        sprintf(topic, "%s/%i/position", MQTT_TOPIC, idx);
        sprintf(value, "%i", shut[idx].position);
        mqtt_client.publish(topic, value, true);
        shut[idx].send_position = false;
        return;  // Send only one per loop
      } 
      if (shut[idx].send_status) {
        sprintf(topic, "%s/%i/state", MQTT_TOPIC, idx);
        switch (shut[idx].status) {
          case SH_STATUS_CLOSED:  mqtt_client.publish(topic, "closed", true); break;
          case SH_STATUS_CLOSING: mqtt_client.publish(topic, "closing", true); break;
          case SH_STATUS_OPEN:    mqtt_client.publish(topic, "open", true); break;
          case SH_STATUS_OPENING: mqtt_client.publish(topic, "opening", true); break;
        }
        shut[idx].send_status = false;
        return; // Send only one per loop
      } 
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  char value[10] = "";
  uint8_t idx = 0;
  while ((idx < length) && (idx < sizeof(value) - 2)) {
    value[idx] = (char)payload[idx];
    idx++;
  }
  value[idx] = '\0';
  dbgdsp = "Message arrived [" + std::string(topic) + "] " + std::string(value);

  //mqtt_client.publish(topic, "", false); 
}

void setup() {
  // Init serial port
  Serial.begin(115200);  
  Serial.print("\nRTS Rx/Tx\n");

  // Init CC1101 RF chip
  delay(100);
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

  EEPROM.begin(4096);
  data_load();
  
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

  // Start WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

}



void loop() {


  // every seconds processing
  currentmillis = millis();
  if ((lastmillis + 1000 < currentmillis) || (currentmillis + 1000000 < lastmillis)) {

    // time management
    uptime ++;
    lastmillis += 1000;

    process_shutter(true);

    // network management
    if (!wifi_connected) {
      if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        dbgdsp = "WiFi connected - IP ";
        dbgdsp.append(WiFi.localIP().toString().c_str());
        mqtt_client.setServer(MQTT_IP, 1883);
        mqtt_client.setCallback(mqtt_callback);
      }
    }
    else {
      if (!mqtt_connected){
        if (mqtt_client.connect("ESP8266-Closer", MQTT_USER, MQTT_PASS, MQTT_LWT, 1, true, "offline")) {
          mqtt_connected = true;
          dbgdsp = "MQTT connected";
          mqtt_client.publish(MQTT_LWT, "online", true);
          mqtt_client.subscribe(MQTT_TOPIC_SUB);
        } else {
          dbgdsp = "MQTT client connection failed";
        }
      } else if (! mqtt_client.connected()) {
        mqtt_connected = false;
        dbgdsp = "MQTT client disconnected";
      }

      if (WiFi.status() != WL_CONNECTED) {
        wifi_connected = false;
        dbgdsp = "WiFi disconnected";
      }
    }

    // data management
    bool updated = false;
    for (uint8_t idx = 0; idx < MAX_REMOTE; idx++) {
      if (rc[idx].updated) updated = true;
    }
    if (updated) {
      if (eeprom_to_save <= 0) data_save(); else eeprom_to_save--;
    } else eeprom_to_save = 60;
  }

  // every loop processing
  mqtt_client.loop();
  process_serial();

  // once per loop processing
  switch (loopidx++)
  {
  case 0:
    process_rx();
    break;
  case 1:
    process_rxframe();
    break;
  case 2:
    process_shutter(false);
    break;
  case 3:
    process_tx();
    break;
  case 4:
    process_mqtt();
    break;

  default:
    loopidx = 0;
    break;
  }

}