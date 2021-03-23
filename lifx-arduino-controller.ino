/*
 * references: 
 * https://community.lifx.com/t/sending-lan-packet-using-arduino/1460/3
 * https://lan.developer.lifx.com/docs/encoding-a-packet
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "secrets.h"

const int button = 0;
int buttonState = 0;
int status = WL_IDLE_STATUS;

char packetBuffer[256]; //buffer to hold incoming packet

WiFiUDP Udp;

// The LIFX Header structure
#pragma pack(push, 1)
typedef struct {
  /* frame header */
  uint16_t size;
  uint16_t protocol:12;
  uint8_t  addressable:1;
  uint8_t  tagged:1;
  uint8_t  origin:2;
  uint32_t source;
  /* frame address */
  uint8_t  target[8];
  uint8_t  reserved[6]; 
  uint8_t  res_required:1;
  uint8_t  ack_required:1;
  uint8_t  :6;
  uint8_t  sequence;
  /* protocol header */
  uint64_t :64;
  uint16_t type;
  uint16_t :16;
  /* variable length payload follows */
} lifx_header;
#pragma pack(pop)

// Device::SetPower Payload
#pragma pack(push, 1)
typedef struct {
  uint16_t level;
} lifx_payload_device_set_power;
#pragma pack(pop)

// Remote IP (In this case we broadcast to the entire subnet)
IPAddress broadcast_ip(255, 255, 255, 255);

unsigned int localPort = 8888;  // Port to listen for responses on
unsigned int lifxPort  = 56700; // Port used to send to LIFX devices

// Packet buffer size
#define LIFX_INCOMING_PACKET_BUFFER_LEN 300

// Payload types
#define LIFX_DEVICE_GETPOWER 20
#define LIFX_DEVICE_SETPOWER 21
#define LIFX_DEVICE_STATEPOWER 22

bool lightsOn = true;

void powerSwitch() {
  uint16_t power = 0;
  lifx_header header;
  lifx_payload_device_set_power payload;

  // Initialize both structures
  memset(&header, 0, sizeof(header));
  memset(&payload, 0, sizeof(payload));
  
  // setup the header
  header.size = sizeof(lifx_header);
  header.protocol = 1024;
  header.addressable = true;
  header.tagged = true;
  header.source = 2;
  header.ack_required = 0;
  header.res_required = 1;
  header.sequence = 1;
  header.type = LIFX_DEVICE_SETPOWER;

  if(lightsOn) {
    payload.level = 0;
    lightsOn = false;
    Serial.println("Turning off lights...");
  } else {
    payload.level = 65534;
    lightsOn = true;
    Serial.println("Turning on lights...");
  }

  // send packet
  Udp.beginPacket(broadcast_ip, 56700);
  Udp.write((char *) &header, sizeof(lifx_header));
  Udp.write((char *) &payload, sizeof(payload));
  Udp.endPacket();
  }

void setup() {
  pinMode(button, INPUT);
  Serial.begin(9600);

  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
  }
  Serial.print("Connected to ");
  Serial.print(ssid);
  Serial.println(" wifi.");
  Udp.begin(localPort);
  Serial.println("Listening on port 8888.");
}

void loop() {
  while(digitalRead(button) == HIGH) {
    powerSwitch();
    delay(500);
    break;
    }
    // Check for incoming packets
    int packetLen = Udp.parsePacket();
    byte packetBuffer[LIFX_INCOMING_PACKET_BUFFER_LEN];
    if (packetLen && packetLen < LIFX_INCOMING_PACKET_BUFFER_LEN) {
      Udp.read(packetBuffer, sizeof(packetBuffer));
      Serial.print("Received Packet type: ");
      Serial.println(((lifx_header *)packetBuffer)->type);
    }
}
