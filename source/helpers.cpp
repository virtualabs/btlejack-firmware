#include "helpers.h"

/**
 * BTLE CRC reverse routine, originally written by Mike Ryan,
 * Dominic Spill and Michael Ossmann, taken from ubertooth_le.
 **/

uint32_t btle_reverse_crc(uint32_t crc, uint8_t *data, int len)
{
	uint32_t state = crc;
	uint32_t lfsr_mask = 0xb4c000;
	uint32_t ret;
    uint8_t cur;
	int i, j, top_bit;

	for (i = len - 1; i >= 0; --i) {
		cur = data[i];
		for (j = 0; j < 8; ++j) {
			top_bit = state >> 23;
			state = (state << 1) & 0xffffff;
			state |= top_bit ^ ((cur >> (7 - j)) & 1);
			if (top_bit)
				state ^= lfsr_mask;
		}
	}

	ret = 0;
	for (i = 0; i < 24; ++i)
		ret |= ((state >> i) & 1) << (23 - i);

	return ret;
}


/**
 * swap_bits
 *
 * Swap every bit of a byte.
 **/

uint8_t swap_bits(uint8_t b)
{
    uint8_t o = 0;
    int i =0;

    for (i=0; i<8; i++)
    {
        o = o << 1;
        o |= (b&(1<<i))?1:0;
    }
    return o;
}


/**
 * dewhiten()
 *
 * Dewhiten a BLE packet given its channel.
 **/

void dewhiten(uint8_t *data, int len, int channel)
{
    int i,j;
    uint8_t c;
    uint8_t lfsr = swap_bits(channel) | 2;

    for (i=0; i<len; i++)
    {
        c = swap_bits(data[i]);
        for (j=7; j>=0; j--)
        {
            if (lfsr & 0x80)
            {
                lfsr ^= 0x11;
                c ^= (1<<j);
            }
            lfsr <<= 1;
        }
        data[i] = swap_bits(c);
    }
}


/**
 * is_valid_aa()
 *
 * Check if a given access address complies with the specifications.
 **/

int is_valid_aa(uint32_t aa)
{
    uint8_t a,b,c,d;
    uint32_t bb;
    int i, t;

    /* Four different bytes. */
    a = (aa & 0xff000000)>>24;
    b = (aa & 0x00ff0000)>>16;
    c = (aa & 0x0000ff00)>>8;
    d = (aa & 0x000000ff);
    if ((a==b) && (b==c) && (c==d))
        return 0;

    /* Not the advertising access address. */
    if (aa == 0x8E89BED6)
        return 0;

    /* Check consecutive bits. */
    bb = aa;
    for (i=0; i<26; i++)
    {
        if (((bb&0x3F) == 0) || ((bb&0x3F)==0x3F))
            return 0;
        bb >>= 1;
    }

    /* Check transitions. */
    bb = aa;
    t = 0;
    a = (bb & 0x80000000)>>31;
    for (i=30; i>=0; i--)
    {
        if (((bb & (1<<i))>>i) != a)
        {
            a = ((bb & (1<<i))>>i);
            t++;
            if (t>24)
                return 0;
        }
        if ((i < 26) && (t<2))
            return 0;
    }

    return 1;
}


/**
 * chm_to_array()
 *
 * Convert a 5-byte channel map into a 37-byte array.
 **/

void chm_to_array(uint8_t *chm, uint8_t *chmArray)
{
	int i,j;

	for (i=0; i<5; i++)
	{
		for (j=0; j<8; j++)
		{
			if ((8*i + j) < 37)
			{
				if (chm[i] & (1<<j))
					chmArray[8*i + j] = 1;
				else
					chmArray[8*i + j] = 0;
			}
		}
	}
}


void array_to_chm(uint8_t *chmArray, uint8_t *chm)
{
	int i;

	for (i=0; i<5; i++)
		chm[i] = 0;

	/* Loop over the channel map array. */
	for (i=0; i<37; i++)
	{
		chm[i/8] |= (chmArray[i]==1)?(1 << (i%8)):0;
	}
}

void whiten(uint8_t *data, int len, int channel)
{
	dewhiten(data,len,channel);
}

uint32_t whiten_pattern(uint8_t *pattern,int size, int offset,int channel) {
	uint8_t *payload = (uint8_t*)malloc(sizeof(uint8_t)*(size+offset));
	for (int i=0;i<size;i++) {
		if (i >= offset) payload[i] = pattern[i-offset];
		else payload[i] = 0;
	}
	whiten(payload,size+offset,channel);
	uint32_t output = (payload[offset] | (payload[offset+1] << 8) | (payload[offset+2] << 16) | (payload[offset+3] << 24));
	return output;
}
