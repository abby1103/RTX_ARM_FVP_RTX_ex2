#ifndef __NAMURU_H
#define __NAMURU_H
//namuru base address
#include <stdint.h>
#define NAMURU_BASE       0xFF200000
//status block
#define STATUS_BASE				0x380
//control block
#define CONTROL_BASE			0x3C0
//channels
#define N_CHANNELS        12
//peripheral base address
#define NAMURU_BASE				0xFF200000

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */

typedef struct
{
  __O uint32_t prn_key;
  __O uint32_t carr_nco;
  __O uint32_t code_nco;
  __O uint32_t code_slew;
  __I uint32_t i_early;
  __I uint32_t q_early;
  __I uint32_t i_prompt;
  __I uint32_t q_prompt;
  __I uint32_t i_late;
  __I uint32_t q_late;
  __I uint32_t carr_meas;
  __I uint32_t code_meas;
  __I uint32_t epoch;
  __I uint32_t epoch_check;
  __O uint32_t epoch_load;
      uint32_t spare;
}  channel_t;

typedef struct 
{
	channel_t channels[N_CHANNELS];
} ch_block_t;

typedef struct 
{
	__I uint32_t status;
	__I uint32_t new_data;
	__I uint32_t tic_count;
	__I uint32_t accum_count;
} status_block_t;

typedef struct
{
	__O uint32_t reset;
	__O uint32_t prog_tic;
	__O uint32_t prog_accum_int;
} control_block_t;

#define ch_block      ((ch_block_t*)      (NAMURU_BASE))
#define status_block  ((status_block_t*)  (NAMURU_BASE + STATUS_BASE))
#define control_block ((control_block_t*) (NAMURU_BASE + CONTROL_BASE))

#endif
