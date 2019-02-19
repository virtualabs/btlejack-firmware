#pragma once

#ifndef UNITTEST
#include "MicroBit.h"
#else
#include <stdint.h>
#include <iostream>
#endif

class ISequenceGenerator
{
public:
  virtual bool initialize(uint8_t *chm, uint32_t accessAddress);
  virtual void resetConnection(void);
  virtual void prepareToFollow(void);
  virtual void updateChannelMap(uint8_t *chm);
  virtual int getNextChannel(void);
  virtual int getCurrentChannel(void);
  virtual int getFirstChannel(void);
  virtual int getSecondChannel(void);
  virtual int getHopIncrement(int interval);
  virtual void setHopIncrement(int increment);
  virtual uint8_t get_channel(uint16_t counter);
  virtual int resolveCounter(uint32_t *measures, int count, uint8_t channel);

};

/**
 * Sequence generator
 **/

class LegacySequenceGenerator : public ISequenceGenerator
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
  LegacySequenceGenerator();

  /* Connection recovery routines. */
  bool initialize(uint8_t *chm, uint32_t accessAddress);
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
  uint8_t get_channel(uint16_t counter);
  int resolveCounter(uint32_t *measures, int count, uint8_t channel);



};


class Ble5SequenceGenerator : public ISequenceGenerator
{

private:

  /* Recovery parameters. */
  uint8_t m_chm[37];
  int m_nChannels;
  uint16_t m_candidates[500];
  int m_nCandidates;
  int m_totalHops;
  int m_hopInterval;

  /* Connection following parameters. */
  uint32_t m_accessAddress;
  uint16_t m_chanId;
  uint16_t m_counter;

  /* Basic operations (PRNG) */
  uint16_t channel_id(uint32_t accessAddress);
  uint16_t permute(uint16_t v);
  uint16_t mam(uint16_t a, uint16_t b);
  uint16_t unmapped_event_channel_selection(uint16_t counter, uint16_t chanid);
  uint8_t remap_channel(uint8_t channel);
  uint8_t get_channel(uint16_t counter, uint16_t chanid);



public:

  /* Constructor. */
  Ble5SequenceGenerator();

  /* Connection recovery routines. */
  bool initialize(uint8_t *chm, uint32_t accessAddress);
  uint8_t get_channel(uint16_t counter);
  int resolveCounter(uint32_t *measures, int count, uint8_t channel);

  /* Connection following. */
  void resetConnection(void);
  void prepareToFollow(void);
  void updateChannelMap(uint8_t *chm);
  int getNextChannel(void);
  int getCurrentChannel(void);
  int getFirstChannel(void);
  int getSecondChannel(void);
  int getHopIncrement(int interval);
  void setHopIncrement(int increment);
};
