// message.c Navigation Message Processing
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.
#include "cmsis_os.h"
//#include <cyg/kernel/kapi.h>
#include "message.h"
#include "constants.h"
#include "ephemeris.h"
#include "time.h"
#include "tracking.h"
#include "gpio1.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

#define PREAMBLE        (0x8b << (30-8)) // look_for_preamble is 0x8b, but it's located
                                        // in the MSBits of a 30 bit word.

// GPS parity bit-vectors
// The last 6 bits of a 30bit GPS word are parity check bits.
// Each parity bit is computed from the XOR of a selection of bits from the
// 1st 24 bits of the current GPS word, and the last 2 bits of the _previous_
// GPS word.
// These parity bit-vectors are used to select which message bits will be used
// for computing each of the 6 parity check bits.
// We assume the two bits from the previous message (b29, b30) and the 24 bits
// from the current message, are packed into a 32bit word in this order:
//   < b29, b30, b1, b2, b3, ... b23, b24, X, X, X, X, X, X > (X = don't care)
// Example: if PBn = 0x40000080,
// The parity check bit "n" would be computed from the expression (b30 XOR b23).
#define PB1     0xbb1f3480
#define PB2     0x5d8f9a40
#define PB3     0xaec7cd00
#define PB4     0x5763e680
#define PB5     0x6bb1f340
#define PB6     0x8b7a89c0

/******************************************************************************
 * Globals
 ******************************************************************************/

// Right now we're declaring a message structure per channel, since the 
// messages come out of locked channels.. but you could argue they should be
// in a per satellite array.
message_t  messages[N_CHANNELS];
//cyg_flag_t  message_flag;


/******************************************************************************
 * Statics
 ******************************************************************************/

// None

/******************************************************************************
 * Count the number of bits set in the input and return (1) if this count is
 * odd (0) otherwise.
 * This is used in the navigation message parity check algorithm.
 *
 * Note, on the ARM there is probably a more efficient way to do this.
 ******************************************************************************/
static int parity( unsigned long word)
{
    int count = 0;

    while( word)
    {
        if( (long)word < 0) count++;
        word <<= 1; // Want to go this way because we typically ignore some
                   // LSBits
    }
    return( count & 1);
}


/******************************************************************************
 * Return 1 if and only if input 30bit word passes GPS parity check, otherwise
 * return 0.
 *
 * The input word is expected to be 32bits, with the 30bit word right justified.
 * The two most significant bits (b30 and b31) should contain the last two bits
 * of the _previous_ GPS word.
 ******************************************************************************/
static int ParityCheck( unsigned long word)
{
    return( !(
                (word & 0x3f) ^ // Last 6 bits are the message parity bits
               ((parity( word & PB1) << 5) | (parity( word & PB2) << 4) |
                (parity( word & PB3) << 3) | (parity( word & PB4) << 2) |
                (parity( word & PB5) << 1) |  parity( word & PB6))
             )
          );
}

/******************************************************************************
 * New satellite in a channel; clear the message. For now that means just
 * clearing the valid flags (and the current subframe for display purposes)
 *****************************************************************************/
           
void clear_messages (unsigned short ch)
{
    unsigned short i;
    
    messages[ch].frame_sync = 0;
    messages[ch].subframe = 0;
    messages[ch].set_epoch_flag = 0;
    for (i = 0; i < 5; ++i)
        messages[ch].subframes[i].valid = 0;
}
    
    
/******************************************************************************
 * Load bits into wordbuf0 and wordbuf1. Flip the incoming bit if we're sync'd
 * onto a subframe and the bits are inverted.
 *
 * Note, see coments about weird 2+ 30 bit pattern below in the words below.
 *****************************************************************************/
static void store_bit( unsigned short ch, unsigned short bit)
{
    // If we're synced on the current message and the data is inverted,
    // flip the incoming bit.
    if( messages[ch].frame_sync && messages[ch].data_inverted)
        bit ^= 1;
    
    // GPS NAV messages come MSBit 1st, so the most recent bit is the LSBit.
    messages[ch].wordbuf0 = (messages[ch].wordbuf0 << 1) | bit;
    
    // NAV words are 30 bits long and the parity check requires the upper
    // two bits to be the least two bits of the previous word. Note that we
    // use wordbuf1 to look for the preamble (TLM and HOW at the same time)
    if( messages[ch].wordbuf0 & (1 << 30))
        messages[ch].wordbuf1 = (messages[ch].wordbuf1 << 1) | 1;
    else
        messages[ch].wordbuf1 = (messages[ch].wordbuf1 << 1);
}

/******************************************************************************
 * Take the message's buffer of 2 + 30 bits (2 from the previous word) and 
 * store it in the subframe's word as 24 bits of data after completing a parity 
 * check.
 *
 *****************************************************************************/
static void store_word( unsigned long ch)
{
    // If bit 30 is set then flip bits 29 - 6 as per GPS spec.
    if( messages[ch].wordbuf0  & (1 << 30))
        messages[ch].wordbuf0 ^= 0x3fffffc0;
                
    if( ParityCheck(messages[ch].wordbuf0))
    {
        // Store the word without the 6 partiy bits and the 2 upper bits
        // in the subframes array
        // We may want to convert this to pointers for efficiency?
        messages[ch].subframes[messages[ch].subframe].word[messages[ch].wordcount]
                = (messages[ch].wordbuf0 >> 6) & 0x00ffffff;
        // Mark it as valid
        messages[ch].subframes[messages[ch].subframe].valid
                |= (1 << messages[ch].wordcount);
    }
}


/*******************************************************************************
 * This function finds the preamble, TLM and HOW in the navigation message and 
 * synchronizes to the nav message.
*******************************************************************************/

static void look_for_preamble( unsigned short ch)
{
    unsigned long   TLM, HOW;           // TLeMetry, HandOffWord
    unsigned short  current_subframe;
    unsigned short  previous_subframe;
    unsigned short  data_inverted;

    // Note: Bits are stored in wordbuf0/1 in store_bits()
    
    /* Save local copies of the wordbuf's for local checks of TLM and HOW */
    TLM = messages[ch].wordbuf1;
    HOW = messages[ch].wordbuf0;

    /* Test for inverted data. Bit 0 and 1 of HOW must be zero. */
    if( HOW & 1)        // Test for inverted data
    {
        TLM = ~TLM;
        HOW = ~HOW;

        data_inverted = 1;
    }
    else
        data_inverted = 0;

    // Flip bits 29 - 6 if the previous word's LSB is 1
    if( TLM  & (1 << 30))
        TLM ^= 0x3fffffc0;
    if( HOW  & (1 << 30))
        HOW ^= 0x3fffffc0;

    if( (TLM & 0x3fc00000) != PREAMBLE) // Test for preamble
        return;

    current_subframe = (int)((HOW >> 8) & 7);             // Subframe ID

    if( (current_subframe < 1) || (current_subframe > 5)) // subframe range test
        return;

    if( HOW & 3) // Confirm zeros
        return;

    // The advantage of previous checks is saving time on false hits.
    // They do increase time on true hits though.
    if( !ParityCheck( TLM))
        return;

    if( !ParityCheck( HOW))
        return;
    
    // Hooray! We found a valid preamble and a sane HOW word, so for now
    // we'll assume we're synced. We won't really know until we get the next
    // subframe and check that the TOW has incremented by exactly 1.
    messages[ch].frame_sync = 1;
    
    // Hand off the current satellite to the message structure
    messages[ch].prn = CH[ch].prn;
    
    // Record the current subframe number (from zero)
    messages[ch].subframe = --current_subframe;
        
    // Flag whether the bits are inverted, and if so invert wordbuf0 so we
    // don't lose the next incoming word.
    if( data_inverted)
    {
        messages[ch].data_inverted = 1;
        messages[ch].wordbuf0 = ~messages[ch].wordbuf0;
    }
    else
        messages[ch].data_inverted = 0;
    
    messages[ch].subframes[current_subframe].word[0] = TLM;
    messages[ch].subframes[current_subframe].word[1] = HOW;
        
    // We've just stored two words into the current subframe
    messages[ch].wordcount = 2;
    // Flag Words 0 and 1 as valid words
    messages[ch].subframes[current_subframe].valid = 3;

    // Extract and store the TOW from the HOW so we can easily compare it to
    // future TOWs to verify our frame sync. (TOW == bits 1-17 of 30 bit word)
    // Maybe this should be a macro. Could be done faster if we assumed 32bits.
    messages[ch].subframes[current_subframe].TOW = (HOW >> (30 - 17)) 
                                                   & ((1 << 17) - 1);

    if( current_subframe > 0)
        previous_subframe = current_subframe - 1;
    else
        previous_subframe = 4;
            
    // Even if the previous subframe had valid TLM and HOW words, kill both the
    // current and previous subframes if their TOW's are not incrementally
    // different.
    if( messages[ch].subframes[previous_subframe].valid & 3)
    {
        if( messages[ch].subframes[current_subframe].TOW 
            != (messages[ch].subframes[previous_subframe].TOW + 1))
        {
            // We're not actually synced. Kill everything and start 
            // the sync search over again.
            clear_messages( ch);
            return;
        }
        // We really do have sync since the TOW's are incremental. And
        // now that we have a valid TLM/HOW, we know the actual time of week.
        // Note that the TOW in the HOW is actually the time at the start
        // of the next subframe.
        
        // Update the gps "time_in_bits". Given that we know the current bit
        // is the last bit of the HOW word (the 60th bit of the subframe), we
        // can calculate the gps time in bit-counts (1/50 seconds).
        // Note, time_in_bits is incremented in the tracking.c lock() function.
        if( messages[ch].subframes[current_subframe].TOW)
            CH[ch].time_in_bits =
                messages[ch].subframes[current_subframe].TOW * 300 - 240;

        else // The TOW can be zero so handle this case.
            CH[ch].time_in_bits = BITS_IN_WEEK - 240;
        
        // Update the gps time in seconds (the receiver's main clock) if we
        // haven't already.
        set_time_with_tow( messages[ch].subframes[current_subframe].TOW);
        
        if(!messages[ch].set_epoch_flag)
        {
            // Finally, flag the tracking loop that the next bit (20ms epoch)
            // is the beginning of a new word. This will reset the epoch counter
            // to 0 every 30 bits to track words... but ONLY once.
			CH[ch].sync_20ms_epoch_count = 1;
			messages[ch].set_epoch_flag  = 1;
        }
    }
}

/******************************************************************************
 * Stuff incoming bits from the tracking interrupt into words and subframes in
 * the messages structure.
 ******************************************************************************/
void message_thread(void const *argument) // input 'data' not used
{
    //cyg_flag_value_t channels_with_bits;
    //cyg_flag_value_t channels_with_subframes;
    //cyg_flag_value_t which_subframe;
	unsigned int   which_subframe;
    unsigned int   channels_with_subframes;
    unsigned short ch;
    
    // There's no way that we're going to get a bit before this thread
    // is first executed, so it's ok to run the flag init here.
    //cyg_flag_init(&message_flag);

    while(1)
    {
        // Block if there are no channels with bits ready. Wake up if any bits
        // from the 12 channels (0xFFF) are set. Clear the flag on wakeup,
        // with the bits saved in channels_with_bits.
        osSignalWait(0x0002, osWaitForever);

        // OK we're awake, process the messages

        //setbits32( GPS4020_GPIO_WRITE, 0x08); // DEBUG:
        gpio1_set(1);
        // Clear the flag IPC shadow (see below)
        channels_with_subframes = 0;
        
        for(ch = 0; ch < N_CHANNELS; ++ch)
        {
            if(channels_with_bits & (1 << ch))
            {                
                // This channel has a bit to process: store the bit into 
                // wordbuf0 and wordbuf1
                store_bit(ch, CH[ch].bit);

                // If the channel isn't sync'd to the message frame,
                // look for the preamble
                if(!messages[ch].frame_sync)
                {
                    look_for_preamble(ch);
                    
                    // If we just found sync, then reset the counters
                    if(messages[ch].frame_sync)
                    {
                        messages[ch].bitcount = 0;
                    }
                }
                // Frame is sync'd, so get bits and words.
                else /* Frame is sync'd */
                {                    
                    // If we have 30 bits, that's a word so store it
                    messages[ch].bitcount++;
                    if(messages[ch].bitcount >= 30)
                    {
                        messages[ch].bitcount = 0;

                        // Store the word in the subframes array
                        store_word( ch);
                        
                        messages[ch].wordcount++;
                        if(messages[ch].wordcount >= 10)
                        {
                            // We've got 10 words so we have a subframe. Use
                            // frame_sync to restart the subframe parsing.
                            // Note that preamble will reset wordcount.
                            messages[ch].frame_sync = 0;
                            
                            // we assume in the preamble that the bit stream
                            // hasn't been inverted when we check the TLM/HOW.
                            // so if the channel IS inverted, make it non-
                            // inverted as we check the TLM/HOW. 
                            if(messages[ch].data_inverted)
                                messages[ch].wordbuf0 = ~messages[ch].wordbuf0;
                            
                            // Only send along complete, error free subframes
                            if(messages[ch].subframes[messages[ch].subframe].valid
                                 == 0x3ff)
                            {
                                // Set the flag to wake up the ephemeris thread
                                channels_with_subframes |= (1 << ch);
                                
                                // Set the subframe flag so we know which 
                                // subframe to process. We don't need a 
                                // shadow register here because we're only 
                                // going to set one subframe per channel
                                // at a time.
                                which_subframe = (1 << messages[ch].subframe);

                            }
                            else
                            {
                            	ephemeris[ch].have_subframe &= ~(1 << messages[ch].subframe);
                            	if((ephemeris[ch].have_subframe & 7) == 7)
                            		ephemeris[ch].valid = 1;
                                else
                                	ephemeris[ch].valid = 0;
                            }
                        }
                    }
                }
                channels_with_bits &= ~(1 << ch);
            }
        }
        
        // Wake up the ephemeris thread if there are subframes ready
		for( ch = 0; ch < N_CHANNELS; ++ch)
        {
            if(channels_with_subframes & (1 << ch))
            {                
                //subframes = cyg_flag_poll(&ephemeris_subframe_flags[ch],
                //                           0x01f,
                //                           CYG_FLAG_WAITMODE_OR
                //                           | CYG_FLAG_WAITMODE_CLR);
                
                // Look for subframes to process
                if(which_subframe & (1 << 0))
                    process_subframe1(ch);
                
                if(which_subframe & (1 << 1))
                    process_subframe2(ch);
                
                if(which_subframe & (1 << 2))
                    process_subframe3(ch);
 
                /* Note: Tak put it best: Almanac decoding is not supported. */
                               
                if(which_subframe & (1 << 3))
                    process_subframe4(ch);             
                           
                if(which_subframe & (1 << 4))
                    process_subframe5(ch);
                
                // We've processed all available subframes in this channel;
                // do we have enough to have a complete ephemeris?
                // (Note that SV health is checked in subframe 1.
                if(((ephemeris[ch].have_subframe & 7) == 7)
                      && (ephemeris[ch].ura < 8))               
                        ephemeris[ch].valid = 1;
                    else
                        ephemeris[ch].valid = 0;
            }
        }
        gpio1_clr(1);
        //clearbits32( GPS4020_GPIO_WRITE, 0x08); // DEBUG:
    }
}
