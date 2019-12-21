#include "nRF24L01.h"
#include "RF24.h"

static const int RF_CE_PIN = 9; // radio module chip enable pin
static const int RF_CS_PIN = 10; // radio module chip select pin
static const uint8_t RF_PKT_SIZE = 26; // radio packet payload size
static const uint8_t NUM_PKTS = 4; // number of packets in a complete transmission
static const uint8_t CH_FREQ = 86; // radio module channel frequency offset (in MHz)
const uint64_t pipes[2] = {0x000000000042, 0x000000000043}; // our radio pipe addresses

static const uint16_t CLK_PER = 10; // parallel comms clock period (in ms)
static const int SYNC_PIN = A0; // frame synchronization signal output pin
static const int CLK_PIN = 8; // clock output pin

// parallel output pins
static const int BIT7_PIN = 7;
static const int BIT6_PIN = 6;
static const int BIT5_PIN = 5;
static const int BIT4_PIN = 4;
static const int BIT3_PIN = 3;
static const int BIT2_PIN = 2;
static const int BIT1_PIN = 1;
static const int BIT0_PIN = 0;

static char maze_buff[NUM_PKTS * (RF_PKT_SIZE - 1)]; // maze data buffer
static uint8_t expected_pkt_num = 0; // expected index of next packet in the transmission

RF24 radio(RF_CE_PIN, RF_CS_PIN); // radio object. uses SPI bus + pins 9 and 10

void setup(void) {
  /* Runs once upon startup */
  
  // make the fpga comms pins outputs
  pinMode(SYNC_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(BIT7_PIN, OUTPUT);
  pinMode(BIT6_PIN, OUTPUT);
  pinMode(BIT5_PIN, OUTPUT);
  pinMode(BIT4_PIN, OUTPUT);
  pinMode(BIT3_PIN, OUTPUT);
  pinMode(BIT2_PIN, OUTPUT);
  pinMode(BIT1_PIN, OUTPUT);
  pinMode(BIT0_PIN, OUTPUT);

  radio.begin(); // begin operation of the radio module 
  
  radio.setRetries(15, 15); // set delay between retries and number of retries
  radio.setAutoAck(true); // turn on the Auto Acknowledge feature
  radio.setChannel(CH_FREQ); // set the frequency to (2400 + CH_FREQ) MHz
  radio.setPALevel(RF24_PA_MAX); // set power level
  radio.setDataRate(RF24_250KBPS); // set data rate (in kb/s)
  radio.setPayloadSize(RF_PKT_SIZE); // set packet payload to RF_PKT_SIZE

  radio.openReadingPipe(1, pipes[0]); // open a reading pipe
  radio.openWritingPipe(pipes[1]); // open a writing pipe
  
  radio.startListening(); // start listening for radio packets
}

void loop(void) {
/* Program superloop */

  if(radio.available()) { // if a new packet is available...
    unsigned char recv_buff[RF_PKT_SIZE]; // buffer for receiving data from the radio module
    radio.read(recv_buff, RF_PKT_SIZE); // read the packet
  
    uint8_t pkt_num = (uint8_t)recv_buff[RF_PKT_SIZE - 1]; // extract its index
    if(pkt_num == expected_pkt_num) { // if it matches the expected index...
      memcpy(maze_buff + pkt_num * (RF_PKT_SIZE - 1), recv_buff, RF_PKT_SIZE - 1); // copy its data into the maze buffer

      if(pkt_num == NUM_PKTS - 1) { // if it is the last packet in the frame...
        for(uint8_t byte_num = 0; byte_num < NUM_PKTS * (RF_PKT_SIZE - 1); byte_num++) { // transmit the maze to the fpga   
          // if this is the first byte assert a synchronization signal
          if(byte_num == 0) digitalWrite(SYNC_PIN, HIGH);
          else digitalWrite(SYNC_PIN, LOW);

          // output the byte on the parallel interface
          digitalWrite(BIT7_PIN, maze_buff[byte_num] & (1 << 7) ? HIGH : LOW);
          digitalWrite(BIT6_PIN, maze_buff[byte_num] & (1 << 6) ? HIGH : LOW);
          digitalWrite(BIT5_PIN, maze_buff[byte_num] & (1 << 5) ? HIGH : LOW);
          digitalWrite(BIT4_PIN, maze_buff[byte_num] & (1 << 4) ? HIGH : LOW);
          digitalWrite(BIT3_PIN, maze_buff[byte_num] & (1 << 4) ? HIGH : LOW);
          digitalWrite(BIT2_PIN, maze_buff[byte_num] & (1 << 2) ? HIGH : LOW);
          digitalWrite(BIT1_PIN, maze_buff[byte_num] & (1 << 1) ? HIGH : LOW);
          digitalWrite(BIT0_PIN, maze_buff[byte_num] & (1 << 0) ? HIGH : LOW);

          delayMicroseconds(CLK_PER / 2); // hold the clock low for half a period
          digitalWrite(CLK_PIN, HIGH); // generate rising edge
          delayMicroseconds(CLK_PER / 2); // hold the clock high for half a period
          digitalWrite(CLK_PIN, LOW); // make the clock low again
        }
      }

      expected_pkt_num = (expected_pkt_num + 1) % 4; // update expected index of the next packet
    }
    else {
      expected_pkt_num = 0; // if the packet index didn't match the expected one, abort the transmission and wait for a new one
    }
  }
}
