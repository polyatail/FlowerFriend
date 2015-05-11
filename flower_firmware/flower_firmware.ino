#define DEBUG

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleTimer.h>

static uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#ifdef DEBUG
#include <SoftwareSerial.h>
#include "printf.h"
#endif

#define RF_DATA_RATE     RF24_250KBPS       // range vs. P(on-air collision) trade-off
#define RF_LAG_TERM      (65)               // given in iterations, compensates for longer on-air time
#define RF_CHANNEL       (0xF0F0F0F0D2LL)
#define MAX_FLOWERS      (255)
#define FLOWER_TTL       (300)              // how many iterations to keep a flower in memory
#define ITERS_PER_COLOR  (128)              // how long to display each flower's color
#define TIME_PER_ITER    (20)               // time per iteration
#define MIN_SEND_DELAY   (50)               // randomly send out packets within this time range
#define MAX_SEND_DELAY   (500)              //   given in iterations

SimpleTimer timer;

uint16_t ttls[MAX_FLOWERS] = {0, };

uint16_t uniqueid = 0;
uint16_t clock = 0;
uint16_t current_color = 0;
uint16_t senddelay = 0;

uint32_t last_recvd_packet[8] = {0, };
uint32_t packet[8] = {0, };

// radio
RF24 radio(9, 10);

// LED
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 6, NEO_GRB + NEO_KHZ800);

uint32_t Wheel (byte WheelPos) {
  float brightness = 1.0;
  
  WheelPos = 255 - WheelPos;
  
  if (WheelPos < 85) {
    return strip.Color((255 - WheelPos * 3) * brightness, 0, WheelPos * 3 * brightness);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3 * brightness, (255 - WheelPos * 3) * brightness);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3 * brightness, (255 - WheelPos * 3) * brightness, 0);
  }
}

uint32_t crc_update (uint32_t crc, byte data)
{
    byte tbl_idx;
    tbl_idx = crc ^ (data >> (0 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    return crc;
}

uint32_t crc_packet (uint32_t *my_packet)
{
  uint32_t crc = ~0L;
  byte *my_byte_packet = (byte*)my_packet;
  
  for (int i = 0; i < 4 * 7; i++)
  {
    crc = crc_update(crc, *my_byte_packet++);
  }
  
  crc = ~crc;
  
  return crc;
}

void setup()
{
  #ifdef DEBUG
  mySerial.begin(9600);
  printf_begin();
  #endif

  // radio
  radio.begin();
  radio.setRetries(0, 0);
  radio.setAutoAck(0);
  radio.setPayloadSize(32);
  radio.setDataRate(RF_DATA_RATE);
  radio.openWritingPipe(RF_CHANNEL);
  radio.openReadingPipe(1, RF_CHANNEL);
  radio.startListening();
  
  #ifdef DEBUG
  radio.printDetails();
  #endif
  
  // LED
  strip.begin();
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  
  // setup timer and ID
  randomSeed(analogRead(A0) + analogRead(A1));
  uniqueid = random(0, MAX_FLOWERS);
  
  ttls[uniqueid] = FLOWER_TTL;
  
  timer.setInterval(TIME_PER_ITER, meat);
}

void loop()
{
  timer.run();
}

void meat()
{
  if (senddelay == 0)
  {
    // fill packet
    packet[1] = uniqueid;
    packet[2] = clock;
    packet[3] = current_color;
    packet[4] = ttls[current_color];
    
    // generate checksum (RF24 CRC doesn't check payload contents)
    uint32_t packet_hash = crc_packet(packet + 1);
    packet[0] = packet_hash;
    
    #ifdef DEBUG
    printf("meat() sending packet id: %u time: %u current_color: %u ttl: %u hash: %lu",
           uniqueid, clock, current_color, ttls[current_color], packet_hash);
    #endif
  
    radio.stopListening();
    bool ok = radio.write(&packet, sizeof(packet));
    radio.startListening();
    
    #ifdef DEBUG
    if (ok)
      printf(" success!\n");
    else
      printf(" failed!\n");
    #endif
    
    senddelay = random(MIN_SEND_DELAY, MAX_SEND_DELAY);
  } else {
    senddelay--;
  }
  
  if (radio.available())
  {
    radio.read(&packet, sizeof(packet));
    
    if (memcmp(&packet, &last_recvd_packet, sizeof(packet)) != 0)
    {
      memcpy(&last_recvd_packet, &packet, sizeof(packet));
      
      if (crc_packet(packet + 1) == packet[0])
      {
        #ifdef DEBUG
        printf("meat() recv'd packet my_id: %u their_id: %lu my_time: %u their_time: %lu\n", uniqueid, packet[1], clock, packet[2]);
        #endif
        
        // update the sender's TTL
        ttls[packet[1]] = FLOWER_TTL;
        
        // if the TTL of the color the sender is currently showing is greater than ours, update it
        if (ttls[packet[3]] < packet[4])
        {
          #ifdef DEBUG
          printf("meat() mesh ttl update id: %lu old_ttl: %u new_ttl: %lu\n", packet[3], ttls[packet[3]], packet[4]);
          #endif
             
          ttls[packet[3]] = packet[4];
        } 
        
        if ((abs(packet[2] - clock) > 1 || current_color != packet[3]) && packet[1] < uniqueid)
        {
          #ifdef DEBUG
          printf("meat() out-of-sync time: %lu time+lag: %lu, current_color: %lu\n", packet[2], packet[2] + RF_LAG_TERM, packet[3]);
          #endif
          
          // copy timestamp and color
          clock = packet[2] + RF_LAG_TERM;
          
          if (clock > ITERS_PER_COLOR)
          {
            clock = ITERS_PER_COLOR;
          }
            
          current_color = packet[3];
        }
      } else {
        #ifdef DEBUG
        printf("meat() invalid crc %lu != %lu\n", crc_packet(packet + 1), packet[0]);
        #endif 
      }
    } else {
      #ifdef DEBUG
      printf("meat() ignoring duplicate packet from id: %lu\n", packet[1]);
      #endif
    }
  }

  // display next color
  if (clock % ITERS_PER_COLOR == 0)
  {
    // start looking from the current color onward
    for (uint16_t i = 0; i < MAX_FLOWERS; i++)
    {
      uint16_t my_i = (i + current_color + 1) % (MAX_FLOWERS);
      
      if (ttls[my_i] > 0)
      {
        current_color = my_i;
        strip.setPixelColor(0, Wheel(my_i));
        break;
      }
    }
    
    #ifdef DEBUG
    printf("meat() changing color to: %u ttl: %u\n", current_color, ttls[current_color]);
    #endif
    
    strip.show();
  }
  
  // update TTLs
  for (uint16_t i = 0; i < MAX_FLOWERS; i++)
  {
    if (ttls[i] > 0 && i != uniqueid)
    {
      ttls[i]--;
      
      #ifdef DEBUG
      if (ttls[i] == 0)
      {
        printf("meat() goodbye, friend! id: %u\n", i); 
      }
      #endif
    }
  }
  
  // increment clock
  clock = (clock + 1) % ITERS_PER_COLOR;
}
