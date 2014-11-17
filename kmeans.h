#ifndef KMEANS_H
#define KMEANS_H
#include <stdio.h>
#include "stdlib.h"
#include "lib/random.h"
#endif


struct record{
     int 	rssi_rle[RUN_LENGTH][2];
     uint8_t 	sequence_num;
};




int kmeans(struct record *, int rle_ptr);
float channel_metric_rssi_threshold(struct record *record, int rle_ptr);

void interpret_clusters(int num_clusters);
int classify(int cluster_id, uint32_t delta_t);
void channel_rate(struct record *record, int n_clusters);
void print_interarrival(uint16_t itr_cnt, int num_clusters);
void add_to_tlist(uint16_t ts_diff);
void update_tlist(uint16_t ts_diff);
uint16_t abs_diff(uint16_t a, uint16_t b);

enum intf_types {WIFI_BEACON = 0, DATA, BLUETOOTH_SINGLE_SLOT, BLUETOOTH_MULTI_SLOT, UNKNOWN};


