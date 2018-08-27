#pragma once

#include "MicroBit.h"

#define PREAMBLE          0xBC
#define PKT_COMMAND       0x01
#define PKT_RESPONSE      0x02
#define PKT_NOTIFICATION  0x04
#define MAX_PACKET_SIZE  0x0110
#define NORDIC_TAP_HEADER_LEN 0x0A

typedef enum {
  LK_WAITING,
  LK_RCV_PACKET,
} T_LINK_STATUS;

typedef enum {
  N_ACCESS_ADDRESS,
  N_CRC,
  N_CHANNEL_MAP,
  N_HOP_INTERVAL,
  N_HOP_INCREMENT,
  N_PACKET,
  N_CONNECT_REQ,
  N_PACKET_NORDIC,
  N_HIJACK_STATUS,
  N_CONN_LOST
} T_NOTIFICATION_TYPE, *PT_NOTIFICATION;

/*
 * Ideally:
 *
 * - recover_crc(aa)
 * - recover_chm(aa, crc)
 * - recover(aa, crc, chm, [hopinter])
 **/

typedef enum {
  /* STATUS = 0x00,  0x00 */
  VERSION = 0x01, /* 0x01 */
  RESET, /* 0x02 */
  LIST_AA, /* 0x03 */
#if 0
  RECOVER_AA, /* 0x04 */
  RECOVER_AA_CHM, /* 0x05 */
  RECOVER_AA_CHM_HOPINTER, /* 0x06 */
#endif
  RECOVER, /* 0x04 */
  SNIFF_CONREQ=0x07, /* 0x07 */
  ENABLE_JAMMING, /* 0x08 */
  ENABLE_HIJACKING, /* 0x09 */
  SEND_PKT, /* 0x0A */
  COLLAB_CHM, /* 0x0B */
  /*
  MODE_SET,
  MODE_GET,
  NOTIFY = 0x80,
  ERROR,
  */
  PACKET = 0x0D,
  DEBUG = 0x0E,
  VERBOSE = 0x0F
} T_OPERATION, *PT_OPERATION;

class Link
{
private:
  MicroBitSerial *m_serial;
  MicroBit *m_bit;

  /* Temporary payload buffer. */
  uint8_t m_payload[MAX_PACKET_SIZE];

  /* Temporary nordic_tap buffer. */
  uint8_t m_nordic_tap[MAX_PACKET_SIZE];

  /* Number of bytes received so far. */
  int m_nbRecvBytes;

  /* Number of bytes expected (packet payload size). */
  int m_nbExpectedBytes;

  T_LINK_STATUS m_status;

  uint8_t crc(uint8_t *data, int size, uint8_t prevCrc);
  uint8_t crc(uint8_t *data, int size);

public:

  /* Constructor. */
  Link(MicroBit *ubit);

  /* Interface. */
  bool readPacket(PT_OPERATION ptOperation, uint8_t *pData, int *pnCount, uint8_t *pubFlags);
  bool sendPacket(T_OPERATION tOperation, uint8_t *pData, int nCount, uint8_t ubFlags);
  bool sendNotification(T_NOTIFICATION_TYPE tNotification, uint8_t *pData, int nCount);

  /* Notifications. */
  bool notifyAccessAddress(uint32_t accessAddress, int channel, uint8_t rssi);
  bool notifyCrc(uint32_t accessAddress, uint32_t crc);
  bool notifyChannelMap(uint32_t accessAddress, uint8_t *chm);
  bool notifyHopInterval(uint32_t accessAddress, uint16_t hopInterval);
  bool notifyHopIncrement(uint32_t accessAddress, uint8_t hopIncrement);
  bool notifyConnectionPacket(uint8_t *pPacket, int nPacketSize);
  bool notifyHijackStatus(uint8_t status);
  bool notifyBlePacket(uint8_t *pPacket, int nPacketSize);
  bool notifyNordicTapBlePacket(
    uint8_t *pPacket,
    int nPacketSize,
    uint8_t channel,
    uint8_t rssi,
    uint8_t direction,
    uint32_t delta,
    uint16_t eventCounter);
  bool notifyConnectionLost(void);

  /* Helpers. */
  bool version(uint8_t major, uint8_t minor);
  bool debug(uint8_t *pData);
  bool verbose(uint8_t *pData);
};
