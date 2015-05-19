//#define DEBUG

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleTimer.h>
#include <avr/pgmspace.h>

const static PROGMEM prog_uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#ifdef DEBUG
#include <MemoryFree.h>
#include <SoftwareSerial.h>
#include "printf.h"
#endif

#define LED_PIN          (2)                // 2 for actual FlowerFriend, 6 generally on Arduino Unos
#define LED_BRIGHTNESS   (0.25)
#define RF_DATA_RATE     (RF24_250KBPS)     // range vs. P(on-air collision) trade-off
#define RF_CHANNEL       (50)
#define RF_LAG_TERM      (0)                // given in iterations, compensates for longer on-air time
#define RF_ADDR          (0xF0F0F0F0D2LL)
#define SYNC_FUZZ        (10)               // allow flowers to be this many cycles off before forcing a sync
#define MAX_FLOWERS      (255)
#define FLOWER_TTL       (500)              // how many iterations to keep a flower in memory
#define ITERS_PER_COLOR  (64)               // how long to display each flower's color
#define TIME_PER_ITER    (20)               // time per iteration
#define MIN_SEND_DELAY   (1)                // randomly send out packets within this time range
#define MAX_SEND_DELAY   (50)               //   given in iterations
#define TIME_TO_FLASH    (0)                // flash this many times upon synchronizing (given in iterations)

SimpleTimer timer;

uint32_t ttls[MAX_FLOWERS] = {0, };

uint32_t uniqueid = 0;
uint32_t clock = 0;
uint32_t current_color = 0;
uint32_t next_color = 0;
uint32_t senddelay = 0;
uint32_t flashing = 0;

uint32_t last_packet_hash = 0;
uint32_t packet[8] = {0, };

// radio
RF24 radio(9, 10);

// LED
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t Wheel (byte WheelPos) {
  WheelPos = 255 - WheelPos;
  
  if (WheelPos < 85) {
    return strip.Color((255 - WheelPos * 3) * LED_BRIGHTNESS, 0, WheelPos * 3 * LED_BRIGHTNESS);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3 * LED_BRIGHTNESS, (255 - WheelPos * 3) * LED_BRIGHTNESS);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3 * LED_BRIGHTNESS, (255 - WheelPos * 3) * LED_BRIGHTNESS, 0);
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
  mySerial.begin(115200);
  printf_begin();
  #endif

  // radio
  radio.begin();
  radio.setRetries(0, 0);
  radio.setAutoAck(0);
  radio.setPayloadSize(32);
  radio.setDataRate(RF_DATA_RATE);
  radio.setChannel(RF_CHANNEL);
  radio.openWritingPipe(RF_ADDR);
  radio.openReadingPipe(1, RF_ADDR);
  radio.startListening();
  
  #ifdef DEBUG
  radio.printDetails();
  #endif
  
  // LED
  strip.begin();
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  
  // setup timer and ID
  randomSeed(analogRead(A0));
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
    #ifdef DEBUG
    printf("free memory: %d\n", getFreeMemory());
    #endif
    
    // fill packet
    packet[1] = uniqueid;
    packet[2] = clock;
    packet[3] = current_color;
    packet[4] = ttls[current_color];
    packet[5] = next_color;
    packet[6] = ttls[next_color];
    
    // generate checksum (RF24 CRC doesn't check payload contents)
    uint32_t packet_hash = crc_packet(packet + 1);
    packet[0] = packet_hash;
    
    #ifdef DEBUG
    printf("sending packet id: %lu time: %lu current_color: %lu ttl: %lu hash: %lu",
           uniqueid, clock, current_color, ttls[current_color], packet_hash);
    #endif
  
    radio.stopListening();
    bool ok = radio.write(&packet, sizeof(packet));
    radio.startListening();
    
    #ifdef DEBUG
    if (ok)
      printf(" success\n");
    else
      printf(" failed\n");
    #endif
    
    senddelay = random(MIN_SEND_DELAY, MAX_SEND_DELAY);
  } else {
    senddelay--;
  }
  
  if (radio.available())
  {
    radio.read(&packet, sizeof(packet));
    
    if (packet[0] != last_packet_hash)
    {
      last_packet_hash = packet[0];
      
      if (crc_packet(packet + 1) == packet[0])
      {
        #ifdef DEBUG
        printf("recv'd packet my_id: %lu their_id: %lu my_time: %lu their_time: %lu\n", uniqueid, packet[1], clock, packet[2]);
        #endif
        
        // update the sender's TTL
        ttls[packet[1]] = FLOWER_TTL;
        
        // if the TTL of the color the sender is currently showing is greater than ours, update it
        if (ttls[packet[3]] < packet[4])
        {
          #ifdef DEBUG2
          printf("mesh ttl update id: %lu old_ttl: %lu new_ttl: %lu\n", packet[3], ttls[packet[3]], packet[4]);
          #endif
             
          ttls[packet[3]] = packet[4];
        }
        
        // same for their next color
        if (ttls[packet[5]] < packet[6])
        {
          #ifdef DEBUG2
          printf("mesh ttl update id: %lu old_ttl: %lu new_ttl: %lu\n", packet[5], ttls[packet[5]], packet[6]);
          #endif
             
          ttls[packet[5]] = packet[6];
        } 
        
        if (packet[1] < uniqueid)                           // only listen if sender has priority over us
        {
          uint32_t dist1 = abs((int32_t)packet[2] - (int32_t)clock);
          uint32_t dist2 = ITERS_PER_COLOR - dist1;
          
          if (dist1 > SYNC_FUZZ && dist2 > SYNC_FUZZ)       // only update if our clock is off by enough
          {
            #ifdef DEBUG2
            printf("out-of-sync time: %lu time+lag: %lu, current_color: %lu\n", packet[2], packet[2] + RF_LAG_TERM, packet[3]);
            #endif
            
            // setup our flashing light to show we're syncing
            flashing = TIME_TO_FLASH;
            
            // copy timestamp and color
            clock = packet[2] + RF_LAG_TERM;
            
            if (clock > ITERS_PER_COLOR)
            {
              clock = ITERS_PER_COLOR;
            }
              
            current_color = packet[3];
            next_color = packet[5];
          }
        }
      #ifdef DEBUG2
      } else {
        printf("invalid crc %lu != %lu\n", crc_packet(packet + 1), packet[0]);
      #endif
      }
    #ifdef DEBUG2
    } else {
      printf("ignore duplicate from id: %lu\n", packet[1]);
    #endif
    }
  }

  // display next color
  if (clock % ITERS_PER_COLOR == 0)
  {
    // start looking from the current color onward
    for (uint32_t i = 0; i < MAX_FLOWERS; i++)
    {
      uint32_t my_i = (i + current_color + 1) % MAX_FLOWERS;
      
      if (ttls[my_i] > 0)
      {
        current_color = my_i;
        strip.setPixelColor(0, Wheel(my_i));
        break;
      }
    }
    
    // it's lame to repeat code, but repeat that code to find the next color
    for (uint32_t i = 0; i < MAX_FLOWERS; i++)
    {
      uint32_t my_i = (i + current_color + 1) % MAX_FLOWERS;
      
      if (ttls[my_i] > 0)
      {
        next_color = my_i;
        break;
      }
    }
    
    #ifdef DEBUG
    printf("current_color: %lu ttl: %lu next_color: %lu ttl: %lu\n",
           current_color, ttls[current_color], next_color, ttls[next_color]);
    #endif
    
    strip.show();
  } else {
    // fade between the two colors over ITERS_PER_COLOR iterations
    uint32_t fade_color[4] = {0, };
    
    float fraction = (float)clock / (float)ITERS_PER_COLOR;
    
    for (int i = 0; i < 4; i++)
    {
      uint8_t from_color = (Wheel(current_color) >> (8 * i)) & 0xFF;
      uint8_t to_color = (Wheel(next_color) >> (8 * i)) & 0xFF;
      uint8_t merge = 0;
      
      if (from_color < to_color)
      {
        merge = (uint8_t)(from_color + fraction * (to_color - from_color));
      } else {
        merge = (uint8_t)(from_color - fraction * (from_color - to_color));
      }
      
      fade_color[i] = (uint32_t)merge << (8 * i);
    }

    strip.setPixelColor(0, fade_color[0] | fade_color[1] | fade_color[2] | fade_color[3]);
    strip.show();
  }
  
  // flash if we're flashing
  if (flashing > 0)
  {
    if (flashing % 2 == 0)
    {
      strip.setPixelColor(0, strip.Color(255 * LED_BRIGHTNESS, 255 * LED_BRIGHTNESS, 255 * LED_BRIGHTNESS));
    } else {
      strip.setPixelColor(0, strip.Color(0, 0, 0)); 
    }
    
    strip.show();
    flashing--;
  }
  
  // update TTLs
  for (uint32_t i = 0; i < MAX_FLOWERS; i++)
  {
    if (ttls[i] > 0 && i != uniqueid)
    {
      ttls[i]--;
      
      #ifdef DEBUG
      if (ttls[i] == 0)
      {
        printf("goodbye id: %lu\n", i); 
      }
      #endif
    }
  }
  
  // increment clock
  clock = (clock + 1) % ITERS_PER_COLOR;
}
