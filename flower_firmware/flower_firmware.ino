//#define DEBUG
//#define DEBUG2

#include <SPI.h>
#include <avr/pgmspace.h>

#include "RF24.h"
#include "Adafruit_NeoPixel.h"
#include "SimpleTimer.h"

const static uint32_t crc_table[16] PROGMEM = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

#ifdef DEBUG
#include "printf.h"
#endif

#define LED_PIN          (7)                // 2 for FF Rev. 2, 7 for FF Rev. 4
#define LED_BRIGHTNESS   (1)
#define RF_DATA_RATE     (RF24_250KBPS)     // range vs. P(on-air collision) trade-off
#define RF_CHANNEL       (10)               // 2.400 GHz + n MHz; WiFi operates 2.412 - 2.484 GHz; antenna is tuned to 2.450 GHz; max=2.527 GHz
#define RF_LAG_TERM      (0)                // given in iterations, compensates for longer on-air time
#define RF_ADDR          (0xF0F0F0F0D2LL)
#define PACKET_SIZE      (13)               // in bytes
#define PACKET_RESEND    (2)                // send every packet this many times--at least one will get through, right?
#define MAX_REBROADCAST  (3)
#define RESEND_DELAY     (100)
#define MAX_FLOWERS      (255)
#define FLOWER_TTL       (75)               // 10 * how many iterations to keep a flower in memory
#define FLOWER_SEEN_TTL  (75)               // 10 * how many iterations to consider a flower directly seen
#define ITERS_PER_COLOR  (64)               // how long to display each flower's color
#define TIME_PER_ITER    (20)               // time per iteration
#define MIN_SEND_DELAY   (30)               // randomly send out packets within this time range
#define MAX_SEND_DELAY   (50)               //   given in iterations

SimpleTimer timer;

uint8_t ttls[MAX_FLOWERS] = {0, };
uint8_t ttl_counter = 10;
uint8_t lowest_flower_seen = 0;

uint8_t uniqueid = 0;
uint8_t clock = 0;
uint8_t current_color = 0;
uint8_t next_color = 0;
uint8_t senddelay = 0;
uint8_t flashing = 0;
uint32_t flashing_color = 0;

uint32_t last_packet_hash = 0;
uint8_t packet[PACKET_SIZE] = {0, };

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

uint32_t crc_packet (uint8_t *my_packet)
{
  uint32_t crc = ~0L;
  
  for (int i = 0; i < PACKET_SIZE - 4; i++)
  {
    crc = crc_update(crc, *my_packet++);
  }
  
  crc = ~crc;
  
  return crc;
}

bool sync_clock (uint8_t their_clock, uint8_t their_current_color, uint8_t their_next_color)
{
  if (clock != their_clock + RF_LAG_TERM || current_color != their_current_color)
  {
    #ifdef DEBUG2
    printf("out-of-sync my_time: %u their_time+lag: %u, my_color: %u their_color: %u\n", clock, their_clock + RF_LAG_TERM, current_color, their_current_color);
    #endif

    // copy timestamp
    clock = their_clock + RF_LAG_TERM;
    
    // we should actually be on the next color by now
    if (clock >= ITERS_PER_COLOR)
    {
      clock = clock % ITERS_PER_COLOR;
      current_color = their_next_color;
    // otherwise copy the current color of our sync buddy
    } else {
      current_color = their_current_color;
    }
    
    // find the new next_color
    for (uint8_t i = 0; i < MAX_FLOWERS; i++)
    {
      uint8_t my_i = (i + current_color + 1) % MAX_FLOWERS;
      
      if (ttls[my_i] > 0)
      {
        next_color = my_i;
        break;
      }
    }
  }
  
  return 1;
}

bool send_mesh_packet()
{
  // fill packet
  //    0-3        CRC32 hash
  //    4          hops
  //    5          my color (uniqueid)
  //    6          my clock
  //    7          current color
  //    8          current color TTL
  //    9          next color
  //    10         next color TTL
  //    11-EOF     other colors and TTLs in my ttls[] array
  packet[4] = 0;
  packet[5] = uniqueid;
  packet[6] = clock;
  packet[7] = current_color;
  packet[8] = ttls[current_color];
  packet[9] = next_color;
  packet[10] = ttls[next_color];
  
  memset(packet + 11, 0, PACKET_SIZE - 11);
  
  // fill packet with 'mesh' TTLs that we've cached
  uint8_t last_color_in_packet = next_color;
  
  for (uint8_t i = 11; i < PACKET_SIZE; i += 2)
  {
    // find the next color with ttl > 0
    for (uint8_t i = 0; i < MAX_FLOWERS; i++)
    {
      uint8_t my_i = (i + last_color_in_packet + 1) % MAX_FLOWERS;
      
      if (ttls[my_i] > 0)
      {
        last_color_in_packet = my_i;
        break;
      }
    }
    
    // but don't waste time if we're repeating ourselves
    if (last_color_in_packet == current_color)
    {
      break;
    }
    
    // fill the packet until the packet is full      
    packet[i] = last_color_in_packet;
    packet[i + 1] = ttls[last_color_in_packet];
  }
  
  // generate checksum (RF24 CRC doesn't check payload contents)
  uint32_t packet_hash = crc_packet(packet + 4);
  packet[0] = (packet_hash >> 0) & 0xFF;
  packet[1] = (packet_hash >> 8) & 0xFF;
  packet[2] = (packet_hash >> 16) & 0xFF;
  packet[3] = (packet_hash >> 24) & 0xFF;
  
  #ifdef DEBUG
  printf("sending packet id: %u time: %u current_color: %u ttl: %u hash: %lu",
         uniqueid, clock, current_color, ttls[current_color], packet_hash);
  #endif

  bool ok;

  radio.stopListening();
  for (uint8_t i = 0; i < PACKET_RESEND; i++)
  {
    ok = radio.write(&packet, sizeof(packet));
    delayMicroseconds(RESEND_DELAY);
  }
  radio.startListening();
  
  #ifdef DEBUG
  if (ok)
    printf(" success\n");
  else
    printf(" failed\n");
  #endif
  
  return ok;
}

bool rebroadcast_packet()
{
  packet[4] += 1;

  // generate checksum (RF24 CRC doesn't check payload contents)
  uint32_t packet_hash = crc_packet(packet + 4);
  packet[0] = (packet_hash >> 0) & 0xFF;
  packet[1] = (packet_hash >> 8) & 0xFF;
  packet[2] = (packet_hash >> 16) & 0xFF;
  packet[3] = (packet_hash >> 24) & 0xFF;
  
  #ifdef DEBUG
  printf("rebroadcasting packet from id: %u hops: %u hash: %lu",
         packet[5], packet[4], packet_hash);
  #endif

  bool ok;

  radio.stopListening();
  for (uint8_t i = 0; i < PACKET_RESEND; i++)
  {
    ok = radio.write(&packet, sizeof(packet));
    delayMicroseconds(RESEND_DELAY);
  }
  radio.startListening();
  
  #ifdef DEBUG
  if (ok)
    printf(" success\n");
  else
    printf(" failed\n");
  #endif
  
  return ok;
}

void setup()
{
  #ifdef DEBUG
  Serial.begin(9600);
  printf_begin();
  #endif

  // radio
  radio.begin();
  radio.setRetries(0, 0);
  radio.setAutoAck(0);
  radio.setPayloadSize(PACKET_SIZE);
  radio.setDataRate(RF_DATA_RATE);
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(RF_ADDR);
  radio.setCRCLength(RF24_CRC_8);
  radio.setAddressWidth(3);
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
  
  // flash our unique color a few times so we remember it
  current_color = uniqueid;
  next_color = uniqueid;
  flashing = 30;
  flashing_color = Wheel(uniqueid);
  
  timer.setInterval(TIME_PER_ITER, meat);
}

void loop()
{
  timer.run();
}

void meat()
{
  // send our typical mesh packet
  if (senddelay == 0)
  {
    send_mesh_packet();
    senddelay = random(MIN_SEND_DELAY, MAX_SEND_DELAY);
  } else {
    senddelay--;
  }
  
  #ifdef DEBUG
  uint8_t packetid = 0;
  #endif

  // deal with incoming data
  while (radio.available() > 0)
  {
    #ifdef DEBUG
    packetid += 1;
    #endif
    
    radio.read(&packet, sizeof(packet));
    
    // unpack hash
    uint32_t packet_hash = ((uint32_t)packet[0] << 0) | \
                           ((uint32_t)packet[1] << 8) | \
                           ((uint32_t)packet[2] << 16) | \
                           ((uint32_t)packet[3] << 24);
    
    // is this a duplicate packet?
    if (packet_hash == last_packet_hash)
    {
      #ifdef DEBUG2
      printf("packet %u ignore duplicate from id: %u\n", packetid, packet[4]);
      #endif

      continue;
    }

    last_packet_hash = packet_hash;
      
    // does the hash checkout?
    if (crc_packet(packet + 4) != packet_hash)
    {
      #ifdef DEBUG2
      printf("packet %u invalid crc %lu != %lu\n",
             packetid, crc_packet(packet + 4), packet_hash);
      #endif

      continue;
    }

    // is this a rebroadcasted packet from this device?
    if (packet[5] == uniqueid)
    {
      #ifdef DEBUG2
      printf("packet %u is from self %u == %u -- ignoring\n",
             packetid, packet[5], uniqueid);
      #endif

      continue;
    }

    #ifdef DEBUG
    printf("recv'd packet %u my_id: %u their_id: %u my_time: %u their_time: %u\n", 
           packetid, uniqueid, packet[5], clock, packet[6]);
    #endif
          
    // update the sender's TTL
    ttls[packet[5]] = FLOWER_TTL;
          
    // iterate through the packet and copy higher TTLs shared by sender
    for (uint8_t i = 7; i < PACKET_SIZE; i += 2)
    {
      // don't waste time if there's nothing interesting in the packet
      if (packet[i + 1] == 0)
      {
        break;
      }
  
      // if the TTL of the color the sender is currently showing is greater than ours, update it
      if (ttls[packet[i]] < packet[i + 1])
      {
        #ifdef DEBUG2
        printf("mesh ttl update id: %u old_ttl: %u new_ttl: %u\n", packet[i], ttls[packet[i]], packet[i + 1]);
        #endif
                 
        ttls[packet[i]] = packet[i + 1];
      }
    }
          
    // is this packet from the lowest flower in the network? if so, sync
    if (packet[5] == lowest_flower_seen)
    {
      sync_clock(packet[6], packet[7], packet[9]);
    }

    // rebroadcast if we should
    if (packet[4] < MAX_REBROADCAST)
    {
      rebroadcast_packet();
    }
  }

  // display next color
  if (clock % ITERS_PER_COLOR == 0)
  {
    current_color = next_color;
    
    // find the new next_color
    for (uint8_t i = 0; i < MAX_FLOWERS; i++)
    {
      uint8_t my_i = (i + current_color + 1) % MAX_FLOWERS;
      
      if (ttls[my_i] > 0)
      {
        next_color = my_i;
        break;
      }
    }
    
    #ifdef DEBUG
    printf("current_color: %u ttl: %u next_color: %u ttl: %u\n",
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
      strip.setPixelColor(0, flashing_color);
    } else {
      strip.setPixelColor(0, strip.Color(0, 0, 0)); 
    }
    
    strip.show();
    flashing--;
  }
  
  // update TTLs
  if (ttl_counter == 0)
  {
    // keep track of the lowest flower in the network
    lowest_flower_seen = uniqueid;
    
    for (uint8_t i = 0; i < MAX_FLOWERS; i++)
    {
      if (ttls[i] > 0 && i != uniqueid)
      {
        ttls[i]--;
        
        if (i < lowest_flower_seen)
        {
          lowest_flower_seen = i;
        }

        #ifdef DEBUG
        if (ttls[i] == 0)
        {
          printf("goodbye id: %u\n", i); 
        }
        #endif
      }
    }
    
    ttl_counter = 10;
  } else {
    ttl_counter--;
  }
  
  // increment clock
  clock = (clock + 1) % ITERS_PER_COLOR;
}
