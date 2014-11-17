#include "contiki.h"
#include "kmeans.h"


//! Wifi Beacon - 1
// struct record rec = {{{1,1989},{2,1},{3,21},{2,2},{1,2108},{2,1},{3,21},{2,2},{1,2108},{2,1},{3,20},{2,2},{1,2108},{2,2},{3,21},{2,1},{1,2108},{2,1},{3,20},{2,1},{1,2108},{2,1},{3,22},{2,1},{1,2106},{2,1},{3,21},{2,2},
// 			{1,2107},{2,1},{3,21},{2,1},{1,2109},{2,1},{3,21},{2,1},{1,2108},{2,2},{3,21},{2,1},{1,2108},{2,1},{3,22},{2,1},{1,2108},{2,1},{3,21},{2,2},{1,2108},{2,1},{3,21},{2,1},{1,2109},{2,1},{3,19},{2,1},{1,2110},
// 			{2,2},{3,19},{2,1},{1,2108},{2,1},{3,22},{2,1},{1,2106},{2,1},{3,21},{2,2},{1,2108},{2,1},{3,21},{2,1},{1,2108},{2,2},{3,21},{2,1},{1,2108},{2,1},{3,21},{2,2},{1,2107},{2,2},{3,21},{2,1},{1,2108},{2,1},
// 			{3,22},{2,1},{1,2108},{2,1},{3,21},{2,1},{1,2108},{2,2},{3,19},{2,1},{1,2108},{2,1},{3,22},{2,1}}};

//! WiFi Beacon + Bluetooth
// struct record rec = {{{1,343},{2,7},{1,96},{2,4},{1,1044},{2,1},{4,19},{3,1},{2,1},{1,2148},{2,1},{3,1},{4,20},{3,1},{2,1},{1,1560},{2,4},{1,517},{2,1},{3,1},{4,20},{3,1},{2,1},{1,1283},{2,3},{1,827},{2,1},{3,1},
// {4,20},{3,1},{2,1},{1,711},{2,4},{1,832},{2,3},{1,466},{2,4},{1,94},{2,1},{3,1},{4,20},{3,1},{2,1},{1,61},{2,3},{1,1170},{2,4},{1,876},{2,1},{3,1},{4,21},{2,2},{1,713},{2,4},{1,1443},{2,1},
// {3,1},{4,19},{2,1},{1,2070},{2,1},{3,1},{4,20},{3,1},{2,1},{1,2114},{2,2},{4,21},{2,1},{1,877},{2,3},{1,1145},{2,4},{1,86},{2,1},{3,1},{4,18},{3,1},{2,1},{1,201},{2,4},{1,232},{2,4},{1,779},
// {2,3},{1,943},{2,1},{3,2},{4,20},{3,1},{2,1},{1,281},{2,3},{1,1407},{2,4},{1,367},{2,1},{4,21},{3,1},{2,1}}};

//! Two WiFi Beacon sources - one is an Android phone
// struct record rec = {{{1,305},{2,3},{3,19},{2,3},{1,310},{2,1},{3,1},{4,20},{3,1},{2,1},{1,73},{2,2},{3,19},{2,2},{1,407},{2,2},{3,19},{2,2},{1,405},{2,2},{3,19},{2,2},{1,405},{2,2},{3,19},{2,2},{1,404},{2,2},{3,20},
// {2,2},{1,310},{2,2},{4,21},{3,1},{2,1},{1,72},{2,2},{3,19},{2,2},{1,408},{2,2},{3,15},{2,2},{1,407},{2,2},{3,19},{2,2},{1,405},{2,2},{3,19},{2,2},{1,404},{2,2},{3,20},{2,2},{1,310},{2,1},{3,1},{4,20},
// {3,1},{2,1},{1,73},{2,2},{3,19},{2,2},{1,406},{2,1},{3,20},{2,1},{1,408},{2,1},{3,20},{2,1},{1,405},{2,2},{3,19},{2,2},{1,404},{2,2},{3,19},{2,2},{1,312},{2,1},{4,19},{3,1},{2,1},{1,75},{2,2},{3,19},
// {2,2},{1,405},{2,1},{3,20},{2,2},{1,407},{2,2},{3,19},{2,2},{1,405},{2,2}}};

//! Bluetooth with single-slot and multi-slot data transfers
// struct record rec = {{{1,2450},{2,59},{1,6},{2,3},{1,356},{2,1},{1,4264},{2,2},{1,11},{2,3},{1,6234},{2,1},{1,14},{2,4},{1,2102},{2,59},{1,6},{2,3},{1,1627},{2,1},{1,12},{2,3},{1,1443},{2,1},{1,11},
// {2,4},{1,1282},{2,58},{1,6},{2,4},{1,1257},{2,58},{1,2008},{2,2},{1,9},{2,1},{1,5},{2,1},{1,23},{2,1},{1,4},{2,3},{1,1},{2,1},{1,15},{2,3},{1,1415},{2,59},{1,6},{2,4},
// {1,4845},{2,4},{1,2461},{2,3},{1,3454},{2,1},{1,11},{2,4},{1,2553},{2,60},{1,6},{2,4},{1,3270},{2,1},{1,11},{2,3},{1,779},{2,60},{1,6},{2,4},{1,1336},{2,1},{1,11},{2,2},
// {1,5},{2,1},{1,2},{2,3},{1,6},{2,3},{1,3},{2,1},{1,5},{2,4},{1,5},{2,5},{1,9},{2,3},{1,593},{2,2},{1,11},{2,3},{1,156},{2,3},{1,2104},{2,1},{1,11},{2,3},{1,1734},{2,2}}};

//! Wifi Beacons on different channels
// struct record rec = {{{1,1514},{2,23},{1,69},{2,1},{3,22},{2,1},{1,2059},{2,23},{1,68},{2,2},{3,21},{2,2},{1,2059},{2,23},{1,68},{2,1},{3,22},{2,1},{1,2057},{2,23},{1,69},{2,1},{3,22},{2,1},{1,2059},
// {2,21},{1,70},{2,2},{3,21},{2,2},{1,2057},{2,23},{1,70},{2,1},{3,22},{2,1},{1,2057},{2,23},{1,70},{2,2},{3,21},{2,1},{1,2057},{2,23},{1,71},{2,1},{3,21},{2,2},{1,2056},{2,24},{1,70},{2,1},{3,19},{2,2},
// {1,2058},{2,23},{1,69},{2,1},{3,22},{2,1},{1,2057},{2,24},{1,68},{2,2},{3,21},{2,1},{1,2059},{2,23},{1,68},{2,2},{3,21},{2,1},{1,2059},{2,23},{1,67},{2,1},{3,21},{2,2},{1,2058},{2,21},{1,71},{2,1},{3,22},
// {2,1},{1,2058},{2,21},{1,71},{2,1},{3,21},{2,2},{1,2056},{2,23},{1,71},{2,1},{3,21},{2,2},{1,2056},{2,23},{1,71},{2,1}}};

#define TVARIANCE 7
#define TVARIANCE_CUMULATIVE 30

static int X[RUN_LENGTH>>1][2];
static int K[10][2];
static uint32_t burst_ts[RUN_LENGTH>>1];
static uint32_t cost;
static uint8_t num_bursts;
static int cluster_test, cluster_pref;
static uint8_t burst_label[RUN_LENGTH>>1], num_occurrences, intf_class[5], max_cnt, intf_idx, wifi_beacon;
static uint32_t relative_ts;
static uint16_t record_ptr, bsize, plevel, j, tlist[15][2], burst_interval[RUN_LENGTH>>1];

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
static int prev_K_final[10][2], K_final[10][2], i, iter = 1, num_clusters_final = 0, prev_num_clusters_final = 0, nruns = 0, idx;
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
        else if ( (tlist[i][0] - (ts_diff % tlist[i][0])) < TVARIANCE_CUMULATIVE) {
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
        else if ( (tlist[i][0] - (ts_diff % tlist[i][0])) < TVARIANCE) {
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
//         sq_term += (uint32_t) (sum_diff) * sum_diff;
        sum_term += sum_diff;
        update_tlist(sum_diff);
        for (j = i+1; j < num_occurrences-1; j++) {
            sum_diff += burst_interval[j];
//             printf("%d mod %d = %d\n", sum_diff, 1859, sum_diff % 1859);
            update_tlist(sum_diff);
        }
    }
    
    if (num_occurrences > 2) {
//         variance = sq_term - ((uint32_t) (sum_term*sum_term))/(num_occurrences - 1);
//         variance = variance / (num_occurrences - 2);
//         printf("num_occurrences = %d, sum_term %lu, sq_term %lu\n", num_occurrences, sum_term, sq_term);
        mean_t = (uint32_t) sum_term/(num_occurrences - 1);
        printf("Channel %d: Kperiod(%d,%d): mean: %lu:",itr_cnt, clusters[cluster_id].burst_size, 
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
        
        if ((clusters[cluster_id].burst_size >= 2) && (clusters[cluster_id].burst_size < 43))
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
            distance = ((clusters[j].burst_size - X[i][0])*(clusters[j].burst_size - X[i][0])) +
                    ((clusters[j].plevel - X[i][1])*(clusters[j].plevel - X[i][1]));
                    
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
                        printf(",%lu", (uint32_t) (burst_ts[i] - burst_ts[prev_burst]));
                    else
                        printf("%lu", (uint32_t) (burst_ts[i] - burst_ts[prev_burst]));
                }
                prev_burst = i;
                num_occurrences++;
            }
        }
        printf("\n");
        check_periodic(itr_cnt, j);
    }
        
}


void interpret_clusters(int num_clusters) {
    
    uint16_t prev_burst = 0;   
    
    //! Label every burst according to the cluster to which it belongs
    for (i = 0; i < num_bursts; i++)
    {
        sq_distance = 65535;
        for (j = 0; j < num_clusters; j++)
        {
            distance = ((clusters[j].burst_size - X[i][0])*(clusters[j].burst_size - X[i][0])) +
                    ((clusters[j].plevel - X[i][1])*(clusters[j].plevel - X[i][1]));
                    
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
        for (i = 0; i < 5; i++) intf_class[i] = 0;

        for (i = 0; i < num_bursts; i++)
        {
            if (burst_label[i] == j)
            {
                if (num_occurrences)  
                { 
                    mean_period = mean_period + (uint32_t) ((burst_ts[i] - burst_ts[prev_burst] - X[prev_burst][0]));
                    intf_class[classify(j, (uint32_t) ((burst_ts[i] - burst_ts[prev_burst] - X[prev_burst][0])))]++;
                }   
                prev_burst = i;
                num_occurrences++;
            }
        }
        
        //! Compute interburst_separation errorbars
        clusters[j].num_occurrences = num_occurrences;	
        if (num_occurrences > 1)
        clusters[j].mean_period = mean_period / (num_occurrences - 1);
        else
        clusters[j].mean_period = 0;
        
        for (i = 0; i < 5; i++) {
            if (intf_class[i] > max_cnt)
            {
                max_cnt = intf_class[i];
                intf_idx = i;
            }
        }
        
        if (max_cnt == -1)
            printf("problem!!!\n");
        
        mean_period = (mean_period*50);
        if (num_occurrences > 1) {
            switch (intf_idx) {
                case WIFI_BEACON:
                    clusters[j].intf_type = WIFI_BEACON;
                    printf("K(%d,%d): %d percent occurrence: %d percent confidence: Wifi beacon with mean interval approximately %lu us\n", 
                    clusters[j].burst_size, clusters[j].plevel, (clusters[j].num_occurrences * 100)/num_bursts, (max_cnt*100)/(num_occurrences-1), mean_period/(num_occurrences-1));
                break;
            
                case BLUETOOTH_SINGLE_SLOT:
                    clusters[j].intf_type = BLUETOOTH_SINGLE_SLOT;
                    printf("K(%d,%d): %d percent occurrence: %d percent confidence: Bluetooth single slot with mean interval approximately %lu us\n", 
                    clusters[j].burst_size, clusters[j].plevel, (clusters[j].num_occurrences * 100)/num_bursts, (max_cnt*100)/(num_occurrences-1), mean_period/(num_occurrences-1));
                break;
                
                case BLUETOOTH_MULTI_SLOT:
                    clusters[j].intf_type = BLUETOOTH_MULTI_SLOT;
                    printf("K(%d,%d): %d percent occurrence: %d percent confidence: Bluetooth multi-slot with mean interval approximately %lu us\n", 
                    clusters[j].burst_size, clusters[j].plevel, (clusters[j].num_occurrences * 100)/num_bursts, (max_cnt*100)/(num_occurrences-1), mean_period/(num_occurrences-1));
                break;
                
                case DATA:
                    clusters[j].intf_type = DATA;
                    printf("K(%d,%d): %d percent occurrence: %d percent confidence: data packets with mean interval approximately %lu us\n", 
                    clusters[j].burst_size, clusters[j].plevel, (clusters[j].num_occurrences * 100)/num_bursts, (max_cnt*100)/(num_occurrences-1), mean_period/(num_occurrences-1));
                    
                break;
                
                case UNKNOWN:
                    clusters[j].intf_type = UNKNOWN;
                    printf("K(%d,%d): %d percent occurrence: Unknown inteferer with period approximately %lu us\n", 
                    clusters[j].burst_size, clusters[j].plevel, (clusters[j].num_occurrences * 100)/num_bursts, mean_period/(num_occurrences-1));
                break;
                
                default:
                
                break;
                
            }
        }
    }
}

// void printX() {
//     for (i = 0; i < num_bursts; i++) {
//         if (i) printf(",");
//         printf("%d:%d", X[i][0], X[i][1]);
//     }
//     printf("\n");
// }

int classify(int cluster_id , uint32_t delta_t)
{
    uint32_t sample_interval = (delta_t*50)/1000;
    //! Classify WiFi Beacon based on the default 100 ms assumption
    if ((sample_interval < 110) && (sample_interval > 90))
	return WIFI_BEACON;
    else if ((clusters[cluster_id].burst_size < 12) && sample_interval > 25)
	return BLUETOOTH_SINGLE_SLOT;
    else if ((clusters[cluster_id].burst_size <= 63) && (clusters[cluster_id].burst_size >= 55) && (sample_interval > 25))
	return BLUETOOTH_MULTI_SLOT;
    else if ((clusters[cluster_id].burst_size < 20) && (clusters[cluster_id].burst_size > 4) && (sample_interval <= 30))
	return DATA;
    else if ((clusters[cluster_id].burst_size > 30))
	return DATA;
    else
	return UNKNOWN;
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
      
//       num_bursts = j;
//     printf("samples : %lu, %lu\n", free_samples, busy_samples);
    cluster_test = 0;
    while ((cluster_test < 10) && (diffcost_between_clusters > 3))
    {
	cost_runs = 65535;
	nruns = 0;
// 	printf(".");
// 	printf("cluster test %d ...\n", cluster_test+1);
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
    // 		printf(".");
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
// 	      num_clusters = cluster_test;
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
    // 		      num_clusters--;
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
            sq_distance = ((K[burst_label[i]][0] - X[i][0])*(K[burst_label[i]][0] - X[i][0])) +
                        ((K[burst_label[i]][1] - X[i][1])*(K[burst_label[i]][1] - X[i][1]));
            
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
    // 		num_clusters_final = num_clusters;
	    }
	    nruns++;
	}
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
// 	printf("cluster_test %d cost_final %lu, cost_runs %lu\n", cluster_test, cost_final, cost_runs);
    }
    
    printf("num_clusters:%d, cost:%ld, samples: %lu (%lu,%lu)\n", prev_num_clusters_final, prev_cost_final, free_samples + busy_samples, free_samples, busy_samples);
//     printX();
    for (i = 0; i < prev_num_clusters_final; i++)
    {	  
// 	  printf("K%d: (%d,%d)\n",i, prev_K_final[i][0], prev_K_final[i][1]);
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