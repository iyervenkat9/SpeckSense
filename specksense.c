/**
 * \file
 *          A Contiki application that detects multiple 
 *          concurrent channel activity in the 2.4 GHz band.
 *          The application works on low-power sensor node platforms 
 *          that feature the cc2420 radio from Texas Instruments.
 *          
 * \author
 *          Venkatraman Iyer <iyer.venkat9@gmail.com>
 */

#include "contiki.h"
#include <math.h>
#include <stdio.h> /* For printf() */
#include "string.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/rime.h"
#include "lib/random.h"
#include "sys/pt.h"
#include "sys/timer.h"
#include "interferer_settings.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "sys/rtimer.h"
#include "dev/serial-line.h"
#include "kmeans.h"


/*---------------------------------------------------------------------------*/
#if PROCESS_ID == 1
PROCESS(specksense, "SpeckSense");
AUTOSTART_PROCESSES(&specksense);
#elif PROCESS_ID == 2
PROCESS(channel_allocation, "Spectrum monitoring");
AUTOSTART_PROCESSES(&channel_allocation);
#else
#error "Choose a valid process 1. specksense on RADIO_CHANNEL,\
        2. scanning all channels."
#endif




/*---------------------------------------------------------------------------*/

//! Global variables
enum task_types {SCAN_RSSI = 1, CYCLE_RSSI, RECEIVE_PACKET_BURST};
enum interferers {WIFI = 1, BLUETOOTH, MICROWAVE, NONE};
static struct etimer et;
static uint16_t n_clusters, cidx;

//! Global variables for RSSI scan
static int  rssi_val, rle_ptr = -1, 
            step_count = 1, cond, 
            itr, itr_j, 
            n_samples, max_samples;
static unsigned rssi_levels[120];

static rtimer_clock_t sample_st, sample_end, 
                      exec_st, exec_end;

static struct record record;

static float channel_metric[16];
static int channel_arr[] = {11, 12, 13, 14, 15, 16, 17, 18, 
                            19, 20, 21, 22, 23, 24, 25, 26};

 #ifdef WITH_BOOST_CPU
        uint16_t cpu1, cpu2;
#endif /* WITH_BOOST_CPU */

/*---------------------------------------------------------------------------*/
static void rssi_sampler(int time_window_ms){
      sample_st = RTIMER_NOW();
      dint();
    max_samples = time_window_ms * 20;
     step_count = 1; rle_ptr = -1;
     record.sequence_num = 0;
     P2DIR |= BV(6);
     P2SEL &= ~BV(6);
     
     while (step_count <= NUM_RECORDS) {
#ifdef DEBUG_APP
	  printf("starting step %d:%d:%d\n",step_count, rle_ptr, record.sequence_num);
#endif
	  record.sequence_num++;
	  rle_ptr = -1;
	  record.rssi_rle[0][1] = 0;
      record.rssi_rle[0][0] = 0;
	  n_samples = max_samples;
	  watchdog_stop();
	  while ((rle_ptr < RUN_LENGTH) && (n_samples)) {
	       P2OUT |= BV(6);
	       
#ifdef WITH_BOOST_CPU
	       /* Temporarily boost CPU speed */
	       cpu1 = DCOCTL;
	       cpu2 = BCSCTL1;
	       DCOCTL = 0xff;
	       BCSCTL1 |= 0x07;
#endif /* WITH_BOOST_CPU */
		    
	       CC2420_SPI_ENABLE();
	       MY_FASTSPI_GETRSSI(rssi_val);
	       CC2420_SPI_DISABLE();

	       rssi_val -= 45; /* compensation offset */
	       n_samples = n_samples - 1;
	       
	       rssi_val = ((signed char) rssi_val);
	       cond = 0x01 & (( record.rssi_rle[rle_ptr][0] != 
                            rssi_levels[-rssi_val-1]) | 
                            (record.rssi_rle[rle_ptr][1] == 32767));
	       
	       rle_ptr = rle_ptr + cond;
	       record.rssi_rle[rle_ptr][0] = rssi_levels[-rssi_val-1];
	       record.rssi_rle[rle_ptr][1] = (record.rssi_rle[rle_ptr][1]) * 
                                         (1-cond) + 1 ;
#ifdef WITH_BOOST_CPU
	       /* Restore CPU speed */
	       DCOCTL = cpu1;
	       BCSCTL1 = cpu2;
#endif /* WITH_BOOST_CPU */
		P2OUT &= ~BV(6);
	  }
	  watchdog_start();
#ifdef DEBUG_APP
	  printf("about to write to file %s\n", filename); print_rssi_rle();	  
#endif
	  step_count++;
     }
     eint();
     sample_end = RTIMER_NOW();
     if (rle_ptr < RUN_LENGTH) rle_ptr++;
}

/*---------------------------------------------------------------------------*/
static rtimer_clock_t rtimer_diff(rtimer_clock_t a, rtimer_clock_t b) {
    if (RTIMER_CLOCK_LT(a,b)){
	return (0xFFFF - a + b);
    }else
	return (a-b);
}

static void print_rssi_rle(){
     int i;
     printf("RSSI");
     for (i = 0; i <= rle_ptr; i++)
	  printf(":%d,%d",record.rssi_rle[i][0],record.rssi_rle[i][1]);
     printf("\nrle_ptr:%d,%d,%d\n",rle_ptr, max_samples, n_samples);
}

static void init_power_levels(){
/* Populate the rssi quantization levels.
 */
#if POWER_LEVELS == 2
     for (itr = 0; itr < 120; itr++)	  
	  if (itr < 90)
	    rssi_levels[itr] = 2;
	  else
	    rssi_levels[itr] = 1;
#elif POWER_LEVELS == 4  
      for (itr = 0; itr < 120; itr++)	  
	  if (itr < 30)
	       rssi_levels[itr] = 4;
	  else if (itr >= 30 && itr < 60)
	       rssi_levels[itr] = 3;
	  else if (itr >= 60 && itr < 90)
	       rssi_levels[itr] = 2;
	  else
	       rssi_levels[itr] = 1;
#elif POWER_LEVELS == 8
      for (itr = 0; itr < 120; itr++)	  
	  if (itr < 14)
	       rssi_levels[itr] = 8;
	  else if (itr >= 12 && itr < 25)
	       rssi_levels[itr] = 7;
	  else if (itr >= 25 && itr < 38)
	       rssi_levels[itr] = 6;
	  else if (itr >= 38 && itr < 51)
	       rssi_levels[itr] = 5;
	  else if (itr >= 51 && itr < 64)
	       rssi_levels[itr] = 4;
	  else if (itr >= 64 && itr < 77)
	       rssi_levels[itr] = 3;
	  else if (itr >= 77 && itr < 90)
	       rssi_levels[itr] = 2;
	  else 
	       rssi_levels[itr] = 1;
#elif POWER_LEVELS == 16
      for (itr = 0; itr < 120; itr++)	  
	  if (itr < 6)
	       rssi_levels[itr] = 16;
	  else if (itr >= 6 && itr < 12)
	       rssi_levels[itr] = 15;	  
	  else if (itr >= 12 && itr < 18)
	       rssi_levels[itr] = 14;
	  else if (itr >= 18 && itr < 24)
	       rssi_levels[itr] = 13;
	  else if (itr >= 24 && itr < 30)
	       rssi_levels[itr] = 12;
	  else if (itr >= 30 && itr < 36)
	       rssi_levels[itr] = 11;
	  else if (itr >= 36 && itr < 42)
	       rssi_levels[itr] = 10;
	  else if (itr >= 42 && itr < 48)
	       rssi_levels[itr] = 9;
	  else if (itr >= 48 && itr < 54)
	       rssi_levels[itr] = 8;
	  else if (itr >= 54 && itr < 60)
	       rssi_levels[itr] = 7;
	  else if (itr >= 60 && itr < 66)
	       rssi_levels[itr] = 6;
	  else if (itr >= 66 && itr < 72)
	       rssi_levels[itr] = 5;
	  else if (itr >= 72 && itr < 78)
	       rssi_levels[itr] = 4;
	  else if (itr >= 78 && itr < 84)
	       rssi_levels[itr] = 3;
	  else if (itr >= 84 && itr < 90)
	       rssi_levels[itr] = 2;
	  else 
	       rssi_levels[itr] = 1;
#elif POWER_LEVELS == 120
      for (itr = 0; itr < 120; itr++)
          rssi_levels[itr] = itr;
#else
#error "Power levels should be one of the following values: 2, 4, 8, 16 or 120"	       
#endif	  

}


/*---------------------------------------------------------------------------*/
#if PROCESS_ID == 1
PROCESS_THREAD(specksense, ev, data)
{  
  
  PROCESS_BEGIN();	  
  
  init_power_levels();
	  
  cc2420_set_channel(RADIO_CHANNEL);
  while (1) {          
     record.sequence_num = 0;
     
     leds_on(LEDS_RED);
     leds_on(LEDS_GREEN);     
     
     rssi_sampler(TIME_WINDOW);  
     leds_off(LEDS_GREEN);
     leds_off(LEDS_RED);
     printf("done sampling on channel %d\n", cc2420_get_channel());
#if DEBUG_RSSI == 1     
     watchdog_stop();
     print_rssi_rle();
     watchdog_start();
#endif
     
     n_clusters = kmeans(&record, rle_ptr);
     print_interarrival(RADIO_CHANNEL, n_clusters);
     
  }
  PROCESS_END();
}
#elif PROCESS_ID == 2
PROCESS_THREAD(channel_allocation, ev, data)
{
    PROCESS_BEGIN();
    
    init_power_levels();
    
    for (itr = 0; itr < 16; itr++)
        channel_metric[itr] = 0;

    cidx = 11;
    cc2420_set_channel(cidx);
    itr = 0;
    while (itr < 3)
    {
  
	  record.sequence_num = 0;
	 
	  leds_on(LEDS_RED);
	  rssi_sampler(TIME_WINDOW);
#if DEBUG_RSSI == 1
	  print_rssi_rle();
#endif
	  watchdog_stop();
#if CHANNEL_METRIC == 1      
      printf("Channel %d:", cc2420_get_channel());
      n_clusters = kmeans(&record, rle_ptr);
      print_interarrival(cc2420_get_channel(), n_clusters);
#elif CHANNEL_METRIC == 2      
      channel_metric[cidx-11] = channel_metric[cidx-11] + 
                                channel_metric_rssi_threshold(&record, rle_ptr);
#endif                
	  channel_rate(&record, n_clusters);
	  watchdog_start();
	      
	  leds_off(LEDS_RED);	
	  cidx = (cc2420_get_channel() == 26)?11:cc2420_get_channel()+1;
	  cc2420_set_channel(cidx);
	  if (cidx == 11)
	    itr++;
    }
#if CHANNEL_METRIC == 2
    for (itr = 0; itr < 16; itr++) {
      channel_metric[itr] = channel_metric[itr]/3.0;
      printf("Channel %d: Channel metric: %ld.%03u\n",
                itr+11, (long)channel_metric[itr], 
                (unsigned)((channel_metric[itr]-
                            floor(channel_metric[itr]))*1000));
    }
    
    for (itr = 0; itr < 15; itr++)
        for (itr_j = itr+1; itr_j < 16; itr_j++)
            if (channel_metric[itr] > channel_metric[itr_j]) {
                    int tmp_channel;
                    float tmp_val = channel_metric[itr];
                    
                    channel_metric[itr] = channel_metric[itr_j];
                    channel_metric[itr_j] = tmp_val;
                    
                    tmp_channel = channel_arr[itr];
                    channel_arr[itr] = channel_arr[itr_j];
                    channel_arr[itr_j] = tmp_channel;
            }
            
    printf("Channel ordering:");
    for (itr = 15; itr >= 0; itr--) {
        printf(" %d", channel_arr[itr]);
        if (itr)
            printf(",");
    }
    
#endif        
    PROCESS_END();
}
#endif


/*---------------------------------------------------------------------------*/
