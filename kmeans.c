#include "contiki.h"
#include "kmeans.h"


#define TVARIANCE 7
#define TVARIANCE_CUMULATIVE 30

static int X[RUN_LENGTH>>1][2];
static int K[10][2];
static uint32_t burst_ts[RUN_LENGTH>>1];
static uint32_t cost;
static uint8_t num_bursts;
static int cluster_test, cluster_pref;
static uint8_t  burst_label[RUN_LENGTH>>1], 
                num_occurrences, 
                intf_class[5], 
                max_cnt, 
                intf_idx, 
                wifi_beacon;
static uint32_t relative_ts;
static uint16_t record_ptr, 
                bsize, 
                plevel, 
                j, tlist[15][2], 
                burst_interval[RUN_LENGTH>>1];

static uint32_t cost_final, prev_cost_final, 
		diffcost_between_clusters, 
		cost_runs, 
		old_cost, 
		cost_diff,
		sq_distance, min_distance,
		distance,
		mean_period,
		free_samples,
		busy_samples;		
static int  prev_K_final[10][2], K_final[10][2], 
            i, iter = 1, num_clusters_final = 0, 
            prev_num_clusters_final = 0, 
            nruns = 0, idx;

static unsigned sum_points1 = 0, sum_points2 = 0, num_points = 0;

static struct cluster_info {
     uint32_t mean_period;
     uint16_t burst_size;
     uint8_t plevel;     
     uint8_t num_occurrences;
     uint8_t intf_type;
} clusters[10];

uint16_t abs_diff(uint16_t a, uint16_t b) {
        if (a <= b) return b-a;
        else        return a-b;
}

void channel_rate(struct record *record, int n_clusters)
{
      record_ptr = 0;
      free_samples = 0;
      busy_samples = 0;
      wifi_beacon = 0;
      
      for (i = 0; i < n_clusters; i++)
	   if (clusters[i].intf_type == WIFI_BEACON)
		wifi_beacon = 1;
	   
      while(record_ptr < RUN_LENGTH)
      {	
	  if (record->rssi_rle[record_ptr][0] == 1)
	      free_samples = free_samples + record->rssi_rle[record_ptr][1];
	  else
	      busy_samples = busy_samples + record->rssi_rle[record_ptr][1];
	  
	  record_ptr++;
      }

      printf("WiFi-%d: Clusters-%d: Cost-%ld: fs-%lu: ts-%lu: ",
			    wifi_beacon, n_clusters, prev_cost_final,
			    free_samples, (free_samples+busy_samples));
}

void update_tlist(uint16_t ts_diff) {
    uint8_t i, match_found = 0;
    
    for (i = 0; i < 15; i++) {
        if (!tlist[i][0] && !match_found) {
            tlist[i][0] = ts_diff;
            tlist[i][1] = 1;
            break;
        }
        if ((ts_diff % tlist[i][0] < TVARIANCE_CUMULATIVE) &&
           (ts_diff > (tlist[i][0] >> 1))){
            match_found = 1;
            tlist[i][1]++;
        }
        else if ( (tlist[i][0] - (ts_diff % tlist[i][0])) 
                    < TVARIANCE_CUMULATIVE) {
            match_found = 1;
            tlist[i][1]++;
        }
    }    
}

void add_to_tlist(uint16_t ts_diff) {
    uint8_t i;
    
    for (i = 0; i < 15; i++) {
        if (!tlist[i][0]) {
            tlist[i][0] = ts_diff;
            tlist[i][1] = 1;
            break;
        }
        
        if (ts_diff % tlist[i][0] < TVARIANCE) {
            if (abs_diff(ts_diff, tlist[i][0]) 
                    < (tlist[i][0] >> 1)) {
                tlist[i][0] = (tlist[i][0] + ts_diff) >> 1;
                break;
            }
        }
        else if ( (tlist[i][0] - (ts_diff % tlist[i][0])) 
                    < TVARIANCE) {
            if (abs_diff(ts_diff, tlist[i][0]) 
                    < (tlist[i][0] >> 1)) {
                tlist[i][0] = (tlist[i][0] + ts_diff) >> 1;            
                break;
            }
        }
    }    
}

void check_periodic(uint16_t itr_cnt, int cluster_id) {
    uint32_t sq_term = 0, sum_term = 0, mean_t;
    uint16_t prev_burst = 0, number_samples, confidence = 0, pos = 0;   
    uint16_t sum_diff = 0;
    uint8_t j = 0, n_sources = 0;
    
    /* Calculate for every burst, the interburst_separation 
     * from its previous burst instance    
     */
    for (i = 0; i < 15; i++) {
        tlist[i][0] = 0;
        tlist[i][1] = 0;
    }

    num_occurrences = 0; 
    for (i = 0; i < num_bursts; i++)
    {
        if (burst_label[i] == cluster_id)
        {
            if (num_occurrences) {               
                burst_interval[num_occurrences-1] = (uint16_t) (burst_ts[i] 
                                                    - burst_ts[prev_burst]);
                add_to_tlist((uint16_t) (burst_ts[i] - burst_ts[prev_burst]));
            }
              
            prev_burst = i;
            num_occurrences++;
        }
    }
    
    for (i = 0; i < num_occurrences-1; i++) {
        sum_diff = burst_interval[i];
        sum_term += sum_diff;
        update_tlist(sum_diff);
        for (j = i+1; j < num_occurrences-1; j++) {
            sum_diff += burst_interval[j];
            update_tlist(sum_diff);
        }
    }
    
    if (num_occurrences > 2) {
        mean_t = (uint32_t) sum_term/(num_occurrences - 1);
        printf("Channel %d: Kperiod(%d,%d): mean: %lu:",
               itr_cnt, clusters[cluster_id].burst_size, 
                clusters[cluster_id].plevel, mean_t);
        for (i = 0; i < 15; i++)
            if (!tlist[i][0])
                break;
            else {
                uint16_t tmp_confidence;
                number_samples = (free_samples + busy_samples)/tlist[i][0];
                tmp_confidence = (200*tlist[i][1])
                                    / (number_samples*(number_samples+1));
                if (confidence < tmp_confidence && tlist[i][0] < 5000) {
                    pos = i;
                    confidence = tmp_confidence;                
                }            
            }
        
        if ((clusters[cluster_id].burst_size >= 2) && 
            (clusters[cluster_id].burst_size < 43))
                n_sources = ((confidence << 1) + 100) / 200;
        
        printf("Period:%d:cnt:%d:max confidence:%d:Beacon:%s\n", 
            tlist[pos][0], tlist[pos][1], 
            confidence, (n_sources)? "yes":"no");
   }
    
}


void print_interarrival(uint16_t itr_cnt, int num_clusters) {
    uint16_t prev_burst = 0;   
    
    /* Label every burst according to the 
     * cluster to which it belongs
     */
    for (i = 0; i < num_bursts; i++)
    {
        sq_distance = 65535;
        for (j = 0; j < num_clusters; j++)
        {
            distance = ((clusters[j].burst_size - X[i][0]) * 
                        (clusters[j].burst_size - X[i][0])) +
                        ((clusters[j].plevel - X[i][1]) * 
                        (clusters[j].plevel - X[i][1]));
                    
            if ((j == 0) || (distance < sq_distance))
            {
                sq_distance = distance;
                burst_label[i] = j;
            }
        }
    }
    
    //! Calculate for every burst, the interburst_separation from its previous burst instance
    for (j = 0; j < num_clusters; j++)
    {
        mean_period = 0;
        num_occurrences = 0;  max_cnt = 0; intf_idx = 0;
        printf("K(%d,%d): ",clusters[j].burst_size, clusters[j].plevel);

        for (i = 0; i < num_bursts; i++)
        {
            if (burst_label[i] == j)
            {
                if (num_occurrences) {
                    if (num_occurrences > 1)                                          
                        printf(",%lu", 
                               (uint32_t) (burst_ts[i] - burst_ts[prev_burst]));
                    else
                        printf("%lu", 
                               (uint32_t) (burst_ts[i] - burst_ts[prev_burst]));
                }
                prev_burst = i;
                num_occurrences++;
            }
        }
        printf("\n");
        check_periodic(itr_cnt, j);
    }
        
}


int kmeans(struct record *record, int rle_ptr){    
      /* Initialize all values */
      cost = 0;
      cluster_test = 0;
      cluster_pref = -1;
      relative_ts = 0;
      record_ptr = 0; bsize = 0; plevel = 0; j = 0, num_bursts = 0;

      cost_final = 65535; 
      prev_cost_final = 65535;
      diffcost_between_clusters = 65535;
      cost_runs = 65535;
      iter = 1; 
      num_clusters_final = 0;
      prev_num_clusters_final = 0;
      nruns = 0;
      old_cost = 65535; cost_diff = 0;
      
     for (i = 0; i < 10; i++)
     {
        prev_K_final[i][0] = 0;
	prev_K_final[i][1] = 0;
	K_final[i][0] = 0;
	K_final[i][1] = 0;
     }
   
   
      free_samples = 0; busy_samples = 0;
      if (rle_ptr == RUN_LENGTH) rle_ptr--;
      /* Populate feature vectors for the clustering process */
      while(record_ptr < rle_ptr)
      {
        /* Process burst size and power level for a 
        * continuous stretch of RSSI readings 
        * above -90 dBm ( > power level 1)
        */
        if (record->rssi_rle[record_ptr][0] <= POWER_LEVELS) {                    
            while ((record->rssi_rle[record_ptr][0] > 1) &&
                    (record_ptr < rle_ptr))
            {
                bsize += record->rssi_rle[record_ptr][1];
                busy_samples += record->rssi_rle[record_ptr][1];
                plevel += record->rssi_rle[record_ptr][0] * record->rssi_rle[record_ptr][1];
                relative_ts += record->rssi_rle[record_ptr][1];
                record_ptr++;
            }
            
            if (bsize > 0)
            {
                /* Populate feature matrix 
                * 0 - bsize, 
                * 1 - power level normalized by 16
                */
                X[num_bursts][0] = bsize;
                X[num_bursts][1] = (plevel << 4) / bsize;
                burst_ts[num_bursts] = relative_ts - bsize;
                num_bursts++;
                
                bsize = 0; 
                plevel = 0; 
            }
            /* We now see a burst of RSSI values 
            * below -90 dBm (power level 1). 
            * Update timestamps and free samples
            */
            if (record_ptr < rle_ptr) {
                relative_ts += record->rssi_rle[record_ptr][1];
                free_samples += record->rssi_rle[record_ptr][1];
                record_ptr++;
            }
        } else
            record_ptr++;
      }
      
    cluster_test = 0;
    while ((cluster_test < 10) && (diffcost_between_clusters > 3))
    {
        cost_runs = 65535;
        nruns = 0;
        cluster_test++;
        if (cluster_test > 1)	  
        {
            for (i = 0; i < num_clusters_final; i++)
            {
                prev_num_clusters_final = num_clusters_final;
                prev_K_final[i][0] = K_final[i][0];
                prev_K_final[i][1] = K_final[i][1];
                prev_cost_final = cost_final;
            }
        }

        while (nruns < 5)
        {
            old_cost = 65535; cost_diff = 0;	   
            iter = 1;
            
            if (old_cost > cost)
                cost_diff = old_cost - cost;
            else
                cost_diff = cost - old_cost;

            /* Initialize clusters
            */
            for (i = 0; i < 10; i++)
            {
                K[i][0] = 0;
                K[i][1] = 0;
                clusters[i].burst_size = 0;
                clusters[i].plevel = 0;
                clusters[i].mean_period = 0;
                clusters[i].num_occurrences = 0;
                clusters[i].intf_type = UNKNOWN;
            }
            /* We randomly initialize clusters to points in X */
            if (cluster_pref >= 0)
            {		
                K[0][0] = X[cluster_pref][0];
                K[0][1] = X[cluster_pref][1];
                for (i = 1; i < cluster_test; i++)
                {
                    idx = random_rand() % num_bursts;
                    K[i][0] = X[idx][0];
                    K[i][1] = X[idx][1];
                }
            }
            else
            {
                for (i = 0; i < cluster_test; i++)
                {
                    idx = random_rand() % num_bursts;
                    K[i][0] = X[idx][0];
                    K[i][1] = X[idx][1];
                }
            }

            /* Repeat until desired number of iterations 
            * OR the cost_diff is smaller than a 
            * randomly chosen value
            */
            while ((iter < 10) && (cost_diff > 3))
            {
            
                if (iter > 1)
                    old_cost = cost;
                
                /* Move centroid to the average 
                * of all related feature points
                */
                cost = 0;
                for (i = 0; i < num_bursts; i++)
                {
                    sq_distance = 65535;
                    for (j = 0; j < cluster_test; j++)
                    {
                        distance = ((K[j][0] - X[i][0])*(K[j][0] - X[i][0])) +
                            ((K[j][1] - X[i][1])*(K[j][1] - X[i][1]));
                                
                        if ((j == 0) || (distance < sq_distance)){
                            sq_distance = distance;
                            burst_label[i] = j;
                        }
                    }
                    cost = cost + sq_distance;
                }
                
                for (j = 0; j < cluster_test; j++)
                {
                    sum_points1 = 0; sum_points2 = 0; num_points = 0;
                    for (i = 0; i < num_bursts; i++)
                    {
                        if (burst_label[i] == j)
                        {
                            sum_points1 = sum_points1 + X[i][0];
                            sum_points2 = sum_points2 + X[i][1];
                            num_points++;
                        }
                    }
                
                    if (num_points)
                    {
                        K[j][0] = sum_points1 / num_points;
                        K[j][1] = sum_points2 / num_points;
                    }
                    else{
                        K[j][0] = 0x7FFF;
                        K[j][1] = 0x7FFF;
                    }
                }
                
                cost /= num_bursts;

                /* Added an enhancement to the clustering algorithm. 
                * Here, instead of randomly selecting cluster points in 
                * the next iteration, we explicitly choose burst features 
                * that have a large cost, i.e. they have a large distance 
                * from their cluster head in the current iteration.
                * This should hopefully solve the problem of outlier features 
                * not being classified separately.
                */
                min_distance = 0;
                for (i = 0; i < num_bursts; i++)
                {	
                    sq_distance = ( (K[burst_label[i]][0] - X[i][0]) * 
                                    (K[burst_label[i]][0] - X[i][0])) +
                                    ((K[burst_label[i]][1] - X[i][1]) * 
                                    (K[burst_label[i]][1] - X[i][1]));
                    
                    if (sq_distance > min_distance) {
                        min_distance = sq_distance;
                        cluster_pref = i;
                    }
                }
                
                if (old_cost > cost)
                    cost_diff = old_cost - cost;
                else
                    cost_diff = cost - old_cost;
                
                
                iter++;
            }
            
            if (cost_runs > cost) 
            {
                cost_runs = cost;
                num_clusters_final = 0;
                for (i = 0; i < cluster_test; i++)
                {
                    P2OUT |= BV(6);
                    if (K[i][0] != 0x7FFF && K[i][1] != 0x7FFF) {		      
                    K_final[num_clusters_final][0] = K[i][0];
                    K_final[num_clusters_final][1] = K[i][1];
                    num_clusters_final++;
                    }
                    P2OUT &= ~BV(6);
                }
            }
            nruns++;
        } /* (nruns < 5) */
        
        if (cost_final > cost_runs)
        {
            diffcost_between_clusters = cost_final - cost_runs;
            cost_final = cost_runs;
        }
        else
        {
            diffcost_between_clusters = cost_runs - cost_final;
            cost_final = cost_runs;
        }
    } /* ((cluster_test < 10) && (diffcost_between_clusters > 3)) */
    
    printf("num_clusters:%d, cost:%ld, samples: %lu (%lu,%lu)\n", 
            prev_num_clusters_final, prev_cost_final, 
            free_samples + busy_samples, 
            free_samples, busy_samples);
    for (i = 0; i < prev_num_clusters_final; i++)
    {	  
	  clusters[i].burst_size = prev_K_final[i][0];
	  clusters[i].plevel = prev_K_final[i][1];	  
	  clusters[i].mean_period = 0;
	  clusters[i].num_occurrences = 0;
    }
    return prev_num_clusters_final;
}


float channel_metric_rssi_threshold(struct record *record, int rle_ptr) {
      uint16_t free_samples = 0; busy_samples = 0;
      record_ptr = 0;
      if (rle_ptr == RUN_LENGTH) rle_ptr--;
      /* Populate feature vectors for the clustering process */
      while(record_ptr < rle_ptr)
      {
        /* Process burst size and power level for a 
        * continuous stretch of RSSI readings 
        * above -90 dBm ( > power level 1)
        */
        if (record->rssi_rle[record_ptr][0] <= POWER_LEVELS) {                    
            while ((record->rssi_rle[record_ptr][0] > 1) &&
                    (record_ptr < rle_ptr))
            {
                busy_samples += record->rssi_rle[record_ptr][1];
                record_ptr++;
            }
            
            /* We now see a burst of RSSI values 
            * below -90 dBm (power level 1). 
            * Update timestamps and free samples
            */
            if (record_ptr < rle_ptr) {
                free_samples += record->rssi_rle[record_ptr][1];
                record_ptr++;
            }
        } else
            record_ptr++;
      }
      printf("free samples = %d, busy samples = %d\n",
                free_samples, busy_samples);
      return ( ((float) (free_samples*1.0)) / (free_samples + busy_samples));
}