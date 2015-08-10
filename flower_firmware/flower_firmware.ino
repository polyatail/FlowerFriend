//#define DEBUG
//#define DEBUG2

#include <SPI.h>

#include "RF24.h"
#include "Adafruit_NeoPixel.h"
#include "SimpleTimer.h"

#ifdef DEBUG
#include "printf.h"
#endif

#define LED_PIN          (7)                // 2 for FF Rev. 2, 7 for FF Rev. 4
#define LED_BRIGHTNESS   (1)
#define RF_DATA_RATE     (RF24_250KBPS)     // range vs. P(on-air collision) trade-off
#define RF_CHANNEL       (10)               // 2.400 GHz + n MHz; WiFi operates 2.412 - 2.484 GHz; antenna is tuned to 2.450 GHz; max=2.527 GHz
#define RF_LAG_TERM      (0)                // given in iterations, compensates for longer on-air time
#define RF_ADDR          (0xF0F0F0F0D2LL)
#define PACKET_SIZE      (8)                // in bytes
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

uint8_t last_packet_hash = 0;
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

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
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
  //    0          CRC8 hash
  //    1          hops
  //    2          my color (uniqueid)
  //    3          my clock
  //    4          current color
  //    5          current color TTL
  //    6          next color
  //    7          next color TTL
  //    8-EOF      other colors and TTLs in my ttls[] array
  packet[1] = 0;
  packet[2] = uniqueid;
  packet[3] = clock;
  packet[4] = current_color;
  packet[5] = ttls[current_color];
  packet[6] = next_color;
  packet[7] = ttls[next_color];
  
  memset(packet + 8, 0, PACKET_SIZE - 8);
  
  // fill packet with 'mesh' TTLs that we've cached
  uint8_t last_color_in_packet = next_color;
  
  for (uint8_t i = 8; i < PACKET_SIZE; i += 2)
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
  packet[0] = CRC8(packet + 1, PACKET_SIZE - 1);
  
  #ifdef DEBUG
  printf("sending packet id: %u time: %u current_color: %u ttl: %u hash: %u",
         uniqueid, clock, current_color, ttls[current_color], packet[0]);
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
  packet[1] += 1;

  // generate checksum (RF24 CRC doesn't check payload contents)
  packet[0] = CRC8(packet + 1, PACKET_SIZE - 1);
  
  #ifdef DEBUG
  printf("rebroadcasting packet from id: %u hops: %u hash: %u",
         packet[2], packet[1], packet[0]);
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
    
    // is this a duplicate packet?
    if (packet[0] == last_packet_hash)
    {
      #ifdef DEBUG2
      printf("packet %u ignore duplicate from id: %u\n", packetid, packet[4]);
      #endif

      continue;
    }

    last_packet_hash = packet[0];
      
    // does the hash checkout?
    if (CRC8(packet + 1, PACKET_SIZE - 1) != packet[0])
    {
      #ifdef DEBUG2
      printf("packet %u invalid crc %u != %u\n",
             packetid, CRC8(packet + 1, PACKET_SIZE - 1), packet[0]);
      #endif

      continue;
    }

    // is this a rebroadcasted packet from this device?
    if (packet[2] == uniqueid)
    {
      #ifdef DEBUG2
      printf("packet %u is from self %u == %u -- ignoring\n",
             packetid, packet[2], uniqueid);
      #endif

      continue;
    }

    #ifdef DEBUG
    printf("recv'd packet %u my_id: %u their_id: %u my_time: %u their_time: %u\n", 
           packetid, uniqueid, packet[2], clock, packet[3]);
    #endif
          
    // update the sender's TTL
    ttls[packet[2]] = FLOWER_TTL;
          
    // iterate through the packet and copy higher TTLs shared by sender
    for (uint8_t i = 4; i < PACKET_SIZE; i += 2)
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
    if (packet[2] <= lowest_flower_seen)
    {
      sync_clock(packet[3], packet[4], packet[6]);
    }

    // rebroadcast if we should
    if (packet[1] < MAX_REBROADCAST)
    {
      rebroadcast_packet();
    }
  }

  // display next color
  if (clock == ITERS_PER_COLOR)
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

        if (ttls[i] == 0)
        {
          #ifdef DEBUG
          printf("goodbye id: %u\n", i);
          #endif
        } else {
          if (i < lowest_flower_seen)
          {
            lowest_flower_seen = i;
          }
        }
      }
    }
    
    ttl_counter = 10;
  } else {
    ttl_counter--;
  }
  
  // increment clock
  clock = (clock + 1) % ITERS_PER_COLOR;
}
