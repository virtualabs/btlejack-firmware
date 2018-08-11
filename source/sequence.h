#pragma once

#ifndef UNITTEST
#include "MicroBit.h"
#else
#include <stdint.h>
#include <iostream>
#endif

/**
 * Sequence generator
 **/

class SequenceGenerator
{

private:

  /* Recovery parameters. */
  uint8_t m_sequences[12][37];
  uint8_t m_remapping[37];
  uint8_t m_chm[37];
  int m_rlut[37];
  int m_nchannels;
  int m_firstChannel;
  int m_secondChannel;

  /* Connection following parameters. */
  int m_hopIncrement;
  int m_currentChannel;

  void generateSequence(int hopIncrement, uint8_t *sequence);
  int computeDistance(int hopIncrement, int firstChannel, int secondChannel);
  int findChannelIndex(int hopIncrement, int channel, int start);
  void generateLUT(uint8_t *lookupTable, int firstChannel, int secondChannel);

public:

  /* Constructor. */
  SequenceGenerator();

  /* Connection recovery routines. */
  bool initialize(uint8_t *chm);
  int getFirstChannel(void);
  int getSecondChannel(void);
  int getHopIncrement(int interval);

  /* Connection following. */
  void setHopIncrement(int increment);
  void resetConnection(void);
  void prepareToFollow(void);
  void updateChannelMap(uint8_t *chm);
  int getNextChannel(void);
  int getCurrentChannel(void);

};
