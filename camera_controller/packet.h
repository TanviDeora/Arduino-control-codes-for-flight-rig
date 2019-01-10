#ifndef PACKET_H
#define PACKET_H
typedef union {
  struct {
    uint8_t startByte;
    uint32_t cameraTime;
    float irSensorVals[4];
    uint8_t proboscisDetect[4];
    uint8_t inject;
    uint8_t stopByte;
  };
  uint8_t data[27]; 
}packet_t;

void init_packet(packet_t * packet) {
  packet->startByte = 0xFF;
  packet->stopByte = 0xAA;
}
#endif
