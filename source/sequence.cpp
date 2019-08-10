/**
 * This whole module needs code refactoring, I know. But since the BLE 5 support is still
 * "experimental", I'll do some cleaning later. I promise.
 **/

#include "sequence.h"

int LegacySequenceGenerator::findChannelIndex(int hopIncrement, int channel, int start)
{
  for (int i=0; i<37; i++)
    if (m_sequences[hopIncrement][(start + i)%37] == channel)
      return (start + i)%37;

  /* -1 if not found. */
  return -1;
}



int LegacySequenceGenerator::computeDistance(int hopIncrement, int firstChannel, int secondChannel)
{
  int distance;

  /* Find first channel index. */
  int fcIndex = findChannelIndex(hopIncrement, firstChannel, 0);

  /* Find second channel index. */
  int scIndex = findChannelIndex(hopIncrement, secondChannel, fcIndex);

  /* Compute distance. */
  if (scIndex > fcIndex)
    distance = (scIndex - fcIndex);
  else
    distance = (scIndex - fcIndex) + 37;

  return distance;
}

/**
 * generateLUT
 *
 * Generate a lookup table based on the distance between two channels given
 * all the possible hop increment values.
 **/

void LegacySequenceGenerator::generateLUT(uint8_t *lookupTable, int firstChannel, int secondChannel)
{
  for (int hopinc=0; hopinc<12; hopinc++)
  {
    lookupTable[hopinc] = computeDistance(hopinc, firstChannel, secondChannel);
  }
}

void LegacySequenceGenerator::generateSequence(int hopIncrement, uint8_t *sequence)
{
  int channel = 0;

  /* Generate sequence. */
  for (int i=0; i<37; i++)
  {
      /* Next sequence channel. */
      if (m_chm[channel] == 1)
        sequence[i] = channel;
      else
        sequence[i] = m_remapping[channel % m_nchannels];

      /* Increment channel. */
      channel = (channel + hopIncrement)%37;
  }
}

bool LegacySequenceGenerator::initialize(uint8_t *chm, uint32_t accessAddress)
{
  uint8_t lut[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  int i,j=0,hopinc=0, channel, count, k;
  bool duplicates = false;

  /* Copy chm. */
  for (i=0; i<37; i++)
    m_chm[i] = chm[i];

  /* Generate remapping. */
  for (i=0; i<37; i++)
  {
    if (chm[i] == 1)
    {
      m_remapping[j++] = i;
    }
  }

  /* Save number of used channels. */
  m_nchannels = j;

  /* Generate all the possible sequences. */
  for (hopinc=0; hopinc<12; hopinc++)
  {
    /* Generate sequence with the given hop increment. */
    generateSequence(hopinc+5, m_sequences[hopinc]);
  }

  /* Find two specific channels. */
  for (channel=0; channel<37; channel++)
  {
    if (m_chm[channel] == 1)
    {
      /* Check if channel is unique across all possible sequences. */
      j = 0;
      for (hopinc=0; hopinc<12; hopinc++)
      {
        count = 0;
        for (i=0; i<37; i++)
        {
          if (m_sequences[hopinc][i] == channel)
          {
            /* Exit loop if channel is found more than once. */
            count++;
            if (count > 1)
              break;
          }
        }

        if (count > 1)
          j = 1;
      }

      /**
       * If channel is unique, consider it as a potential first channel and look
       * for a second one in order to have a working lookup table.
       **/

      if (j==0)
      {
        /* Found our first candidate channel, now check if we can find a unique second channel. */
        m_firstChannel = channel;

        for (j=0; j<37; j++)
        {
          if (m_chm[j] == 1)
          {
            generateLUT(lut, m_firstChannel, j);

            /* Search doubles. */
            duplicates = false;
            for (i=0; i<11; i++)
            {
              for (k=i+1; k<12; k++)
              {
                if (lut[i] == lut[k])
                {
                    duplicates = true;
                }
              }
            }

            if (!duplicates)
            {
              /* Save second channel. */
              m_secondChannel = j;

              /* Generate our reverse LUT. */
              for (i=0; i<37; i++)
                m_rlut[i] = 0;
              for (i=0; i<12; i++)
                m_rlut[lut[i]] = (i+5);

              /* Success. */
              return true;
            }
          }
        }
      }
    }
  }


  /* We cannot recover hop increment and interval with this chm. */
  return false;

}

void LegacySequenceGenerator::updateChannelMap(uint8_t *chm)
{
  int i,j;

  /* Generate remapping. */
  j = 0;
  for (i=0; i<37; i++)
  {
    if (chm[i] == 1)
    {
      m_remapping[j++] = i;
    }
  }
  m_nchannels = j;

  /* Copy chm. */
  for (i=0; i<37; i++)
    m_chm[i] = chm[i];
}

int LegacySequenceGenerator::getFirstChannel(void)
{
  return m_firstChannel;
}

int LegacySequenceGenerator::getSecondChannel(void)
{
  return m_secondChannel;
}

int LegacySequenceGenerator::getHopIncrement(int interval)
{
  return m_rlut[interval];
}

void LegacySequenceGenerator::setHopIncrement(int increment)
{
  m_hopIncrement = increment;
}

void LegacySequenceGenerator::resetConnection(void)
{
  m_currentChannel = 0;
}

int LegacySequenceGenerator::getNextChannel(void)
{
  /* Compute next channel. */
  m_currentChannel = (m_currentChannel + m_hopIncrement)%37;

  /* Return channel number. */
  if (m_chm[m_currentChannel] == 1)
    return m_currentChannel;
  else
    return m_remapping[m_currentChannel%m_nchannels];
}

int LegacySequenceGenerator::getCurrentChannel(void)
{
  if (m_chm[m_currentChannel] == 1)
    return m_currentChannel;
  else
    return m_remapping[m_currentChannel%m_nchannels];
}


void LegacySequenceGenerator::prepareToFollow(void)
{
  /* Switch to first channel. */
  m_currentChannel = m_firstChannel;
}


/**
 * SequenceGenerator
 *
 * Generate the initial sequence, starting from channel 0 with automatic
 * remapping of unused channels.
 **/

LegacySequenceGenerator::LegacySequenceGenerator()
{
}

uint8_t LegacySequenceGenerator::get_channel(uint16_t counter){
  return (counter)%37;
}

int LegacySequenceGenerator::resolveCounter(uint32_t *measures, int count, uint8_t channel){return 0;}

/*********************************************
 * BLE5 Channel Selection Algorithm #2
 *********************************************/

Ble5SequenceGenerator::Ble5SequenceGenerator()
{
  m_nCandidates = 0;
  m_counter = 0;
}

uint16_t Ble5SequenceGenerator::channel_id(uint32_t accessAddress)
{
  return ((accessAddress & 0xffff0000)>>16) ^ (accessAddress & 0x0000ffff);
}

uint16_t Ble5SequenceGenerator::permute(uint16_t v)
{
  v = (((v & 0xaaaa) >> 1) | ((v & 0x5555) << 1));
  v = (((v & 0xcccc) >> 2) | ((v & 0x3333) << 2));
  return (((v & 0xf0f0) >> 4) | ((v & 0x0f0f) << 4));
}

uint16_t Ble5SequenceGenerator::mam(uint16_t a, uint16_t b)
{
  return (17 * a + b) % (0x10000);
}

uint16_t Ble5SequenceGenerator::unmapped_event_channel_selection(uint16_t counter, uint16_t chanid)
{
  uint16_t prne;

  prne = counter ^ chanid;
  prne = mam(permute(prne), chanid);
  prne = mam(permute(prne), chanid);
  prne = mam(permute(prne), chanid);
  return prne ^ chanid;
}

uint8_t Ble5SequenceGenerator::remap_channel(uint8_t channel)
{
  /* TODO: implement remapping. */
  return 0;
}

/**
 * get_channel()
 *
 * Compute current channel number.
 **/

uint8_t Ble5SequenceGenerator::get_channel(uint16_t counter, uint16_t chanid)
{
  //if (m_nChannels == 37)
    return unmapped_event_channel_selection(counter, chanid)%37;
  //else
  //  return remap_channel(unmapped_event_channel_selection(counter, chanid));
}

uint8_t Ble5SequenceGenerator::get_channel(uint16_t counter)
{
  return get_channel(counter, m_chanId);
}

bool Ble5SequenceGenerator::initialize(uint8_t *chm, uint32_t accessAddress)
{
  /* Copy channel map. */
  m_nChannels = 0;
  for (int i=0; i<37; i++)
  {
    m_chm[i] = chm[i];
    if (m_chm[i])
      m_nChannels++;
  }

  /* Save access address, compute channel id. */
  m_accessAddress = accessAddress;
  m_chanId = channel_id(m_accessAddress);
  m_counter = 0;

  return true;
}

void Ble5SequenceGenerator::resetConnection(void)
{
  /* Reset counter. */
  m_counter = 0;
}

void Ble5SequenceGenerator::prepareToFollow(void)
{
  /* Nothing to do here. */
}

void Ble5SequenceGenerator::updateChannelMap(uint8_t *chm)
{
  /* Not supported yet. */
}

int Ble5SequenceGenerator::getNextChannel(void)
{
  /* Increment counter */
  m_counter = (m_counter + 1)%0x10000;

  /* Return corresponding channel. */
  return get_channel(m_counter, m_chanId);
}

int Ble5SequenceGenerator::getCurrentChannel(void)
{
  /* Return corresponding channel. */
  return get_channel(m_counter, m_chanId);
}

int Ble5SequenceGenerator::getFirstChannel(void)
{
  /* This is an ugly hack. I have to admit. */
  return (int)m_counter;
}
int Ble5SequenceGenerator::getSecondChannel(void)
{
  return 0;
}
int Ble5SequenceGenerator::getHopIncrement(int interval)
{
  return 0;
}

void Ble5SequenceGenerator::setHopIncrement(int increment)
{
}

int Ble5SequenceGenerator::resolveCounter(uint32_t *measures, int count, uint8_t channel)
{
  int ncandidates = 0,k;
  int32_t candidate = 0;
  int32_t last_counter = 0;

  /* We loop over the 65535 possibilities and try to find a unique candidate. */
  for (candidate=0; candidate<65536; candidate++)
  {
    if (
      (get_channel((uint16_t)(candidate+measures[0])%65536) == get_channel(count)) &&
      (get_channel((uint16_t)(candidate+measures[1])%65536) == get_channel(count+1)) &&
      (get_channel((uint16_t)(candidate+measures[2])%65536) == get_channel(count+2)) &&
      (get_channel((uint16_t)(candidate+measures[3])%65536) == get_channel(count+3)) &&
      (get_channel((uint16_t)(candidate+measures[4])%65536) == get_channel(count+4))) {
        last_counter = candidate;
        ncandidates++;
      }
  }

  if (ncandidates == 1)
  {
    /* Compute the next value of the counter, adding the total number of hops
     * as it was measured in the last measure we made. */
    m_counter = ((last_counter + measures[4])%65536) + 1 + 12;
    measures[0] = last_counter;

    /* We are ready to synchronize now :) */
  }

  return ncandidates;
}
