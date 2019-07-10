#pragma once

#include "MicroBit.h"

/*
This class defines a filtering policy for advertisements sniffing.
It allows to choose between two main modes : WHITELIST and BLACKLIST.
If the mode is WHITELIST, the user will be able to add rules to allow some specific frames only. 
If the mode is BLACKLIST, the user will be able to add rules to drop some specific frames.

A rule matches some specific pattern in the link layer frames and is composed of three main fields :
[payload|variable][mask|variable][position|1byte]

For example, if the following rules is provided :
payload : 0x00 0x00 0x66 0x55 0x44 0x33 0x22 0x11
mask :    0x0f 0x00 0xff 0xff 0xff 0xff 0xff 0xff
position : 0

Every ADV_IND (first byte of the rule) transmitted by 11:22:33:44:55:66 (bytes 3 to 8) will match the rule.
If multiple rules are provided, the frame matches the policy if at least one rules is matched.

If the position "IN" (0xFF) is chosen for a given rule, it searches for the pattern in the whole packet.
*/


#define POLICY_LENGTH 512
#define IN 0xFF

typedef enum FilteringMode {
	BLACKLIST,
	WHITELIST
} FilteringMode;

// Filtering Policy
class FilteringPolicy {
	private:
                // Mode of the filtering policy
		FilteringMode mode;
                // Actual length of the policy
		int length;
                // Buffer storing the policy's rules
		uint8_t rules[POLICY_LENGTH];

	public:
                // Buffer (and length) used to send the current policy using serial port
		uint8_t policy_buffer[POLICY_LENGTH+1+4];
		int policy_buffer_size;

		FilteringPolicy(FilteringMode mode);
		void reset_policy(FilteringMode mode);
		void update_buffer();
		FilteringMode getMode();
		int add_rule( uint8_t size, uint8_t *payload, uint8_t *mask, uint8_t position);
		int match_rule(uint8_t *buffer, uint8_t buffer_size, int rule_offset);
		int match_rules(uint8_t *buffer, uint8_t buffer_size);
};
