#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include "printing.h"

#include "neural_nets.h"
#include "ms5849.h"
#include "ms5849_funcs.h"

extern int32_t pressure_raw[8];
extern const NeuralNet fingertip_net;


class ForceSensor{
public:
    ForceSensor();
    void Sample();
    void SamplePT();
    int8_t Initialize();
    void Calibrate();
    void Evaluate();
    
    int _channel;
    int raw_data[8];
    int offsets[8];
    int offset_data[8];
    float input_data[8];
    float output_data[5];
    int temp_data[8];
    
    
private:
    int8_t config_dev(struct ms5849_dev *dev);
//    const NeuralNet *_net;
    uint8_t sensor_comp;
    
    float l1[12]; // to be evaluated on-line
    float l2[64]; //[25];
    float l3[64]; //[5];
    float l4[5];

    // TODO: convert this into a struct array?
    struct ms5849_dev s1; // sets up dev as a 'bmp3_dev structure' w/ associated variables
    struct ms5849_dev s2;
    struct ms5849_dev s3;
    struct ms5849_dev s4;
    struct ms5849_dev s5;
    struct ms5849_dev s6;
    struct ms5849_dev s7;
    struct ms5849_dev s8;
    // TODO: convert this into a struct array?
    struct ms5849_data data1; // structs to store sensor data
    struct ms5849_data data2;
    struct ms5849_data data3;
    struct ms5849_data data4;
    struct ms5849_data data5;
    struct ms5849_data data6;
    struct ms5849_data data7;
    struct ms5849_data data8;

    bool     temp_pending;            // (legacy, no longer used by Sample())
    uint32_t cached_d2[8];           // cached temperature raw values
    uint32_t loop_count;              // for round-robin temperature refresh
    bool first_sample;                // skip first read (no data yet)

    // --- pipeline state for the corrected Sample() ---------------------
    // A pressure conversion is started on EVERY loop for every channel,
    // so Phase 1 can always read a valid pressure result. Temperature is
    // refreshed one channel per loop on a separate round-robin cycle
    // (all 8 channels every 8 loops) without ever displacing a pressure
    // conversion.
    int      temp_rr_channel;         // which channel's temp to service
    bool     temp_rr_pending;         // a temp conversion is in flight
    uint32_t last_good_d1[8];         // last in-range pressure ADC code
    uint32_t last_good_d2[8];         // last in-range temperature ADC code
    uint32_t bad_read_count[8];       // diagnostics: rejected reads/channel

};

// A valid MS5849 pressure/temperature ADC result is a 24-bit code, so it
// must lie in [0, 0xFFFFFF]. A value of exactly 0 is what the ADC result
// register returns when read before the conversion has completed; treat
// 0 and the all-ones bus-error pattern as invalid.
#define MS5849_ADC_CODE_MAX   0x00FFFFFFUL
#define MS5849_ADC_CODE_MIN   0x00000001UL   // 0 == conversion-not-ready

#endif
