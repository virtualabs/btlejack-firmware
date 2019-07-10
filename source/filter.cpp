#include "filter.h"

FilteringPolicy::FilteringPolicy(FilteringMode mode) {
	/* Constructor */
	this->reset_policy(mode);
}
void FilteringPolicy::reset_policy(FilteringMode mode){
	/* 
	This method allows to reset the current policy. 
 	It clears the rules buffer and sets the mode according to the parameter. 
	*/
	this->mode = mode;
	this->length = 0;
	for (int i=0;i<POLICY_LENGTH;i++) this->rules[i] = 0;
	this->policy_buffer_size = 0;
}

void FilteringPolicy::update_buffer(){
	/*
	This method generates the buffer used to transmit the policy via the serial port.

	Format of the corresponding buffer :
	[mode|1byte][length|4bytes][rules|variable]
	*/
	this->policy_buffer[0] = (this->mode == BLACKLIST ? 0x00 : 0x01);
	this->policy_buffer[1] = this->length & 0xFF000000;
	this->policy_buffer[2] = this->length & 0x00FF0000;
	this->policy_buffer[3] = this->length & 0x0000FF00;
	this->policy_buffer[4] = this->length & 0x000000FF;
	memcpy(this->policy_buffer+5,this->rules, this->length);
	this->policy_buffer_size = this->length + 5;
}

FilteringMode FilteringPolicy::getMode(){
	/* getter allowing to get the current mode */
	return this->mode;
}
int FilteringPolicy::add_rule( uint8_t size, uint8_t *payload, uint8_t *mask, uint8_t position){
	/*
	This method allow to add a rule to the policy.

	The format of a rule is the following :
	[length|1byte][payload|<length> bytes][mask|<length> bytes][position|1byte]

	The format of the buffer is the following :
	[rule][rule][rule]

	So, if the two following rules are stored :

	* payload = 0x01 | mask = 0x0F | position = 0x00
	* payload = 0x66 0x55 0x44 0x33 0x22 0x11 | mask = 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF | position = 0x02

	The buffer contains :

	0x01 0x01 0x0F 0x00 0x06 0x66 0x55 0x44 0x33 0x22 0x11 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0x02

	\_____first rule__/ \____________________________second rule____________________________/

	*/
	int success = 1;
	if ((this->length + 2 + 2*size) >= POLICY_LENGTH) {
		// Not enough space in policy buffer
		success = 0;
	}
	else {
		this->rules[this->length] = size;
		memcpy(this->rules+this->length+1,payload,size);
		memcpy(this->rules+this->length+1+size,mask,size);
		this->rules[this->length+1+2*size] = position;
		this->length += 1+size*2+1;
	}
	return success;
}
int FilteringPolicy::match_rule(uint8_t *buffer, uint8_t buffer_size, int rule_offset){
	/* 
	This method checks if a specific rule (selected using rule_offset, its position in the policy buffer) matchs a
	provided frame (buffer/buffer_size).

	It returns 1 if the rule matches the frame or 0 if not.
	*/
	int error = 0;
	uint8_t size = this->rules[rule_offset];
	uint8_t position = this->rules[rule_offset+1+2*size];
	if (buffer_size < size || rule_offset < 0 || rule_offset >= this->length || (position != IN && position >= buffer_size)) {
		error = 1;
	}
	else {
		if (position == IN) {
			int found = 0;
			for (int h=0;h <= buffer_size - size;h++) {
				int j=0;
				int localError = 0;
				while (j<size && localError == 0) {
					if ((buffer[h+j] & this->rules[rule_offset+1+size+j]) == this->rules[rule_offset+1+j]) 
					{
						j++;
					}
					else {
						localError = 1;
					}
				}
				if (localError == 0) 
				{
					found = 1;
					break;
				}
			}
			if (found == 0) {
				error = 1;
			}
		}
		else {
			int j=0;
			while (j<size && error == 0) {
				if ((buffer[position+j] & this->rules[rule_offset+1+size+j]) == this->rules[rule_offset+1+j]) 
				{
					j++;
				}
				else {
					error = 1;
				}
			}
		}
	}
	return 1-error;
}
int FilteringPolicy::match_rules(uint8_t *buffer, uint8_t buffer_size){
	/*
	This method checks if at least one rule of the policy matches the provided frame (buffer/buffer_size).

	It returns 1 if the policy matches the frame, 0 if not.
	*/
	int match = 0;
	int i=0;
	while (i<this->length && match == 0) {
		uint8_t size = this->rules[i];
		match = match_rule(buffer,buffer_size,i);
		i += 1+size*2+1;
	}
	return match;
}
