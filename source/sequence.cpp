#include "sequence.h"

int SequenceGenerator::findChannelIndex(int hopIncrement, int channel, int start)
{
  for (int i=0; i<37; i++)
    if (m_sequences[hopIncrement][(start + i)%37] == channel)
      return (start + i)%37;

  /* -1 if not found. */
  return -1;
}



int SequenceGenerator::computeDistance(int hopIncrement, int firstChannel, int secondChannel)
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

void SequenceGenerator::generateLUT(uint8_t *lookupTable, int firstChannel, int secondChannel)
{
  for (int hopinc=0; hopinc<12; hopinc++)
  {
    lookupTable[hopinc] = computeDistance(hopinc, firstChannel, secondChannel);
  }
}

void SequenceGenerator::generateSequence(int hopIncrement, uint8_t *sequence)
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

bool SequenceGenerator::initialize(uint8_t *chm)
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

void SequenceGenerator::updateChannelMap(uint8_t *chm)
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

int SequenceGenerator::getFirstChannel(void)
{
  return m_firstChannel;
}

int SequenceGenerator::getSecondChannel(void)
{
  return m_secondChannel;
}

int SequenceGenerator::getHopIncrement(int interval)
{
  return m_rlut[interval];
}

void SequenceGenerator::setHopIncrement(int increment)
{
  m_hopIncrement = increment;
}

void SequenceGenerator::resetConnection(void)
{
  m_currentChannel = 0;
}

int SequenceGenerator::getNextChannel(void)
{
  /* Compute next channel. */
  m_currentChannel = (m_currentChannel + m_hopIncrement)%37;

  /* Return channel number. */
  if (m_chm[m_currentChannel] == 1)
    return m_currentChannel;
  else
    return m_remapping[m_currentChannel%m_nchannels];
}

int SequenceGenerator::getCurrentChannel(void)
{
  if (m_chm[m_currentChannel] == 1)
    return m_currentChannel;
  else
    return m_remapping[m_currentChannel%m_nchannels];
}


void SequenceGenerator::prepareToFollow(void)
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

SequenceGenerator::SequenceGenerator()
{
}
