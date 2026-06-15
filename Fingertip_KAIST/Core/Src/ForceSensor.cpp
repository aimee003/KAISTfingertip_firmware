#include "ForceSensor.h"
#include "math_ops.h"

ForceSensor::ForceSensor(){

    // initialize other values
    sensor_comp = 1; //| uint8_t(1<<1); // sensor_comp = BMP3_PRESS | BMP3_TEMP;
    // Pipeline state init
//    loop_count = 0;
//    temp_pending = false;
//    for (int i = 0; i < 8; i++) {
//        cached_d2[i] = 0;
//    }
}
    
int8_t ForceSensor::Initialize(){
	int8_t rslt=0;
	// configure sensor devices
	// start with writing all CS pins high
	writeHigh(1);
	writeHigh(2);
	writeHigh(3);
	writeHigh(4);
	writeHigh(5);
	writeHigh(6);
	writeHigh(7);
	writeHigh(8);

	//printf("Initializing force sensor.\n\r");

	s1.dev_id = 1;  // tells which cs pin associated with device
	config_dev(&s1);
	s2.dev_id = 2;  // tells which cs pin associated with device
	config_dev(&s2);
	s3.dev_id = 3;  // tells which cs pin associated with device
	config_dev(&s3);
	s4.dev_id = 4;  // tells which cs pin associated with device
	config_dev(&s4);
	s5.dev_id = 5;  // tells which cs pin associated with device
	config_dev(&s5);
	s6.dev_id = 6;  // tells which cs pin associated with device
	config_dev(&s6);
	s7.dev_id = 7;  // tells which cs pin associated with device
	config_dev(&s7);
	s8.dev_id = 8;  // tells which cs pin associated with device
	config_dev(&s8);

    struct ms5849_dev* sensors[8] = {&s1,&s2,&s3,&s4,&s5,&s6,&s7,&s8};

    // Prime cached temperatures (one-time blocking reads — only happens at startup)
    for (int i = 0; i < 8; i++) {
    	rslt |= ms5849_start_temp_conversion(sensors[i]);
        HAL_Delay(2);  // wait for conversion at OSR=1
        rslt |= ms5849_read_temp_raw(sensors[i], &cached_d2[i]);
        // seed the validated-read fallbacks so the very first Sample()
        // never substitutes an uninitialised value
        last_good_d2[i] = cached_d2[i];
        bad_read_count[i] = 0;
    }

    // Prime the pressure pipeline: start one conversion per channel and read
    // it back once (blocking) so last_good_d1[] holds a real value before the
    // first Sample(). Sample() then keeps the pipeline running continuously.
    for (int i = 0; i < 8; i++) {
        rslt |= ms5849_start_press_conversion(sensors[i]);
    }
    HAL_Delay(2);  // wait for conversion at OSR=1
    for (int i = 0; i < 8; i++) {
        uint32_t code = 0;
        ms5849_read_press_raw(sensors[i], &code);
        last_good_d1[i] = (code >= MS5849_ADC_CODE_MIN &&
                           code <= MS5849_ADC_CODE_MAX) ? code : 0x800000UL;
    }

    // Kick off the first round of pressure conversions for Sample()'s Phase 1
    for (int i = 0; i < 8; i++) {
    	rslt |= ms5849_start_press_conversion(sensors[i]);
    }

    // Temperature round-robin starts at channel 0; no temp conversion is in
    // flight yet, so the first Sample() will start one rather than read one.
    temp_rr_channel = 0;
    temp_rr_pending = false;
    loop_count      = 0;
    temp_pending    = false;  // legacy field, unused

    first_sample = true;  // first Sample() call will read these results

    return rslt;
}
    
    
int8_t ForceSensor::config_dev(struct ms5849_dev *dev){
    
	int8_t rslt=0;//BMP3_OK; // get error with rslt = BMP3_OK;

    dev -> intf = MS5849_SPI_INTF;
    dev -> read = &ms5849_spi1_read;
    dev -> write = &ms5849_spi1_write;

    dev -> delay_ms = &ms5849_delay_ms;
    dev -> delay_us = &ms5849_delay_us;

    rslt = ms5849_init(dev);
    //printf("* initialize sensor result = 0x%x *\r\n", rslt);
    HAL_Delay(250);

    // ***** Configuring settings of sensor
    // Normal Mode - bmp3_set_op_mode
    // Temp En, Press En
    // OSR = no oversampling temp, press
    // ODR = 200Hz temp, press
    // IRR = no IRR filter
    // ^^^all 4 above =  bmp3_set_sensor_settings

    // Set sensor settings (press en, temp en, OSR, ODR, IRR)
//    dev -> settings.press_en = 0x01; // BMP3_ENABLE
//    dev -> settings.temp_en = 0x01; //BMP3_ENABLE
//    dev -> settings.odr_filter.press_os = 0x00; //BMP3_NO_OVERSAMPLING
//    dev -> settings.odr_filter.temp_os = 0x00; //BMP3_NO_OVERSAMPLING
//    dev -> settings.odr_filter.odr = 0x00; //BMP3_ODR_200_HZ
//    dev -> settings.odr_filter.iir_filter = 0x00; //BMP3_IIR_Filter_disable
//    uint16_t settings_sel;
//    //settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_IIR_FILTER_SEL | BMP3_ODR_SEL;
//    settings_sel = uint16_t(1 << 1) | uint16_t(1 << 2) | uint16_t(1 << 4) | uint16_t(1 << 5) | uint16_t(1 << 6) | uint16_t(1 << 7);
//    //settings_sel = uint16_t(1 << 1) | uint16_t(1 << 2);
//    rslt = bmp3_set_sensor_settings(settings_sel, dev);
//
//    // Set operating (power) mode
//    dev -> settings.op_mode = 0x03; /// normal mode = 0x03
//    rslt = bmp3_set_op_mode(dev);
//
//    // Check settings
//    rslt = bmp3_get_sensor_settings(dev);
    dev -> settings.ratio = MS5849_SNSR_CFG_RATIO_OFF;
    dev -> settings.filter = MS5849_SNSR_CFG_FILTER_OFF;
    dev -> settings.resolution = MS5849_SNSR_CFG_RES_24_BIT;
    dev -> settings.osr = MS5849_SNSR_CFG_OSR_1; // Minimal oversampling

    dev -> settings.sel = MS5849_SNSR_CFG_SEL_PRESS;
    rslt |= ms5849_write_config(dev);
    HAL_Delay(10);

    dev -> settings.sel = MS5849_SNSR_CFG_SEL_TEMP;
    rslt |= ms5849_write_config(dev);
    HAL_Delay(10);
    return rslt;
}
    
void ForceSensor::Sample(){


//	ms5849_get_sensor_data(sensor_comp, &data1, &s1);
//	ms5849_get_sensor_data(sensor_comp, &data2, &s2);
//	ms5849_get_sensor_data(sensor_comp, &data3, &s3);
//	ms5849_get_sensor_data(sensor_comp, &data4, &s4);
//	ms5849_get_sensor_data(sensor_comp, &data5, &s5);
//	ms5849_get_sensor_data(sensor_comp, &data6, &s6);
//	ms5849_get_sensor_data(sensor_comp, &data7, &s7);
//	ms5849_get_sensor_data(sensor_comp, &data8, &s8);
//
//	// store data
//	raw_data[0] = int(data1.pressure)-100000; // pressure is returned in Pa, could subtract actual sea level pressure here
//	raw_data[1] = int(data2.pressure)-100000;
//	raw_data[2] = int(data3.pressure)-100000;
//	raw_data[3] = int(data4.pressure)-100000;
//	raw_data[4] = int(data5.pressure)-100000;
//	raw_data[5] = int(data6.pressure)-100000;
//	raw_data[6] = int(data7.pressure)-100000;
//	raw_data[7] = int(data8.pressure)-100000;
//
//    // could combine this with previous step
//    offset_data[0] = raw_data[0]-offsets[0];
//    offset_data[1] = raw_data[1]-offsets[1];
//    offset_data[2] = raw_data[2]-offsets[2];
//    offset_data[3] = raw_data[3]-offsets[3];
//    offset_data[4] = raw_data[4]-offsets[4];
//    offset_data[5] = raw_data[5]-offsets[5];
//    offset_data[6] = raw_data[6]-offsets[6];
//    offset_data[7] = raw_data[7]-offsets[7];

	struct ms5849_dev*  sensors[8]  = {&s1,&s2,&s3,&s4,&s5,&s6,&s7,&s8};
	struct ms5849_data* datasets[8] = {&data1,&data2,&data3,&data4,&data5,&data6,&data7,&data8};
	uint32_t d1[8], d2[8];

	uint16_t conv_delay = ms5849_conversion_delay_us(&s1);

	// Phase A: start ALL 8 pressure conversions, wait ONCE, read ALL 8.
	for (int i = 0; i < 8; i++)
		ms5849_start_conversion(sensors[i], MS5849_CNV_ADC_SEL_PRESS);
	sensors[1]->delay_us(conv_delay);
	for (int i = 0; i < 8; i++)
		ms5849_read_adc(sensors[i], MS5849_CNV_ADC_SEL_PRESS, &d1[i]);

	// Phase B: start ALL 8 temperature conversions, wait ONCE, read ALL 8.
	for (int i = 0; i < 8; i++)
		ms5849_start_conversion(sensors[i], MS5849_CNV_ADC_SEL_TEMP);
	sensors[1]->delay_us(conv_delay);
	for (int i = 0; i < 8; i++)
		ms5849_read_adc(sensors[i], MS5849_CNV_ADC_SEL_TEMP, &d2[i]);

	// Phase C: compensate + store. Each conversion was waited out for its
	// correct settling time before being read, so d1/d2 are never read
	// early -> never 0 -> compensate() never produces a spike.
	for (int i = 0; i < 8; i++) {
		ms5849_compensate(sensors[i], d1[i], d2[i], datasets[i]);
		raw_data[i]    = (int)datasets[i]->pressure - 100000;
		offset_data[i] = raw_data[i] - offsets[i];
	}

//    // === Phase 1: read the pressure results started on the PREVIOUS loop ===
//    // Because Phase 2 below starts a pressure conversion on EVERY loop for
//    // EVERY channel without exception, there is always a valid conversion to
//    // read here. Each raw code is validated: a 24-bit ADC result must be in
//    // (0, 0xFFFFFF]. A value of 0 means the conversion was not ready; an
//    // out-of-range value means a corrupt SPI read. In either case we reject
//    // it and reuse the last good code, so a bad read can never reach
//    // ms5849_compensate() -- which is quadratic in d1 and would otherwise
//    // turn a not-ready 0 into a multi-million-Pa spike.
//    for (int i = 0; i < 8; i++) {
//        uint32_t code = 0;
//        ms5849_read_press_raw(sensors[i], &code);
//        if (code < MS5849_ADC_CODE_MIN || code > MS5849_ADC_CODE_MAX) {
//            d1[i] = last_good_d1[i];   // reuse last valid sample
//            bad_read_count[i]++;
//        } else {
//            d1[i] = code;
//            last_good_d1[i] = code;
//        }
//    }
//
//    // === Phase 2: ALWAYS start the next pressure conversion, every channel ==
//    // This is the core fix. Pressure conversions are never skipped, so the
//    // pressure pipeline is never broken. (The old code skipped pressure once
//    // every 200 loops to do temperature, which left Phase 1 reading an
//    // unstarted register -> a 0 -> a spike.)
//    for (int i = 0; i < 8; i++) {
//        ms5849_start_press_conversion(sensors[i]);
//    }
//
//    // === Temperature: round-robin, one channel per loop =====================
//    // Temperature changes slowly, so it does not need to be sampled every
//    // loop -- but it must be refreshed far more often than the old once-per-
//    // second. Here one channel is serviced per loop: its temperature result
//    // (started one loop earlier) is read, validated, and cached; then a new
//    // temperature conversion for the NEXT channel is started. All 8 channels
//    // are refreshed every 8 loops (~25 ms at 200 Hz). This never displaces a
//    // pressure conversion -- the MS5849 pressure and temperature ADC result
//    // registers are independent, and a temp conversion can be in flight at
//    // the same time as a pressure conversion.
//    loop_count++;
//    if (temp_rr_pending) {
//        // read the temp conversion started for temp_rr_channel last loop
//        uint32_t code = 0;
//        ms5849_read_temp_raw(sensors[temp_rr_channel], &code);
//        if (code >= MS5849_ADC_CODE_MIN && code <= MS5849_ADC_CODE_MAX) {
//            cached_d2[temp_rr_channel] = code;
//            last_good_d2[temp_rr_channel] = code;
//        } else {
//            cached_d2[temp_rr_channel] = last_good_d2[temp_rr_channel];
//        }
//        // advance to the next channel
//        temp_rr_channel = (temp_rr_channel + 1) & 0x07;
//    }
//    // start the next channel's temperature conversion (runs concurrently
//    // with all 8 pressure conversions kicked off above)
//    ms5849_start_temp_conversion(sensors[temp_rr_channel]);
//    temp_rr_pending = true;
//
//    // === Phase 3: compensate using validated d1 + freshly-cached d2 =========
//    for (int i = 0; i < 8; i++) {
//        ms5849_compensate(sensors[i], d1[i], cached_d2[i], datasets[i]);
//        raw_data[i] = (int)datasets[i]->pressure - 100000;
//        offset_data[i] = raw_data[i] - offsets[i];
//    }
}



void ForceSensor::SamplePT(){
	int multiplier = 1000;
//    // get data from every sensor
//	for(int i = 0;i<8;i++){
//		raw_data[i] = pressure_raw[i];
//	}

	// get data from every sensor
	ms5849_get_sensor_data(sensor_comp, &data1, &s1);
	ms5849_get_sensor_data(sensor_comp, &data2, &s2);
	ms5849_get_sensor_data(sensor_comp, &data3, &s3);
	ms5849_get_sensor_data(sensor_comp, &data4, &s4);
	ms5849_get_sensor_data(sensor_comp, &data5, &s5);
	ms5849_get_sensor_data(sensor_comp, &data6, &s6);
	ms5849_get_sensor_data(sensor_comp, &data7, &s7);
	ms5849_get_sensor_data(sensor_comp, &data8, &s8);

	// store data
	raw_data[0] = int(data1.pressure)-100000; // pressure is returned in Pa, could subtract actual sea level pressure here
	raw_data[1] = int(data2.pressure)-100000;
	raw_data[2] = int(data3.pressure)-100000;
	raw_data[3] = int(data4.pressure)-100000;
	raw_data[4] = int(data5.pressure)-100000;
	raw_data[5] = int(data6.pressure)-100000;
	raw_data[6] = int(data7.pressure)-100000;
	raw_data[7] = int(data8.pressure)-100000;

    // could combine this with previous step
    offset_data[0] = raw_data[0]-offsets[0];
    offset_data[1] = raw_data[1]-offsets[1];
    offset_data[2] = raw_data[2]-offsets[2];
    offset_data[3] = raw_data[3]-offsets[3];
    offset_data[4] = raw_data[4]-offsets[4];
    offset_data[5] = raw_data[5]-offsets[5];
    offset_data[6] = raw_data[6]-offsets[6];
    offset_data[7] = raw_data[7]-offsets[7];

    // sample temp data
    temp_data[0] = int(data1.temperature*multiplier);
    temp_data[1] = int(data2.temperature*multiplier);
    temp_data[2] = int(data3.temperature*multiplier);
    temp_data[3] = int(data4.temperature*multiplier);
    temp_data[4] = int(data5.temperature*multiplier);
    temp_data[5] = int(data6.temperature*multiplier);
    temp_data[6] = int(data7.temperature*multiplier);
    temp_data[7] = int(data8.temperature*multiplier);

    temp_data[0] = 5;


}
    
void ForceSensor::Evaluate(){
    // scales raw input data, evaluates neural network, scales and stores output data
        
    // scale sensor data
    for (int i=0; i<8; i++){
        input_data[i] = 0.0f;
        input_data[i] = (((float)offset_data[i]) - (fingertip_net.minims[i+5]))/(fingertip_net.maxims[i+5]-fingertip_net.minims[i+5]); // / fingertip_net.max_pressure;
        // check that inputs are between 0 and 1?
    }

    // decode sensor data here....521*4 operations (multiply,add,activation,add)
    // reset values
    for (int i = 0; i<12; i++){
        l1[i] = 0.0f;
    }
    for (int i = 0; i<64; i++){ //i<25
        l2[i] = 0.0f;
        l3[i] = 0.0f;
    }
    for (int i = 0; i<5; i++){
        l4[i] = 0.0f;
    }
        
    // layer 1
    for(int i = 0; i<12; i++){ // for each node in the next layer
        for(int j = 0; j<8; j++){ // add contribution of node in prev. layer
            l1[i] +=  (fingertip_net.w1[j][i]*input_data[j]);
        }
        l1[i] += fingertip_net.b1[i]; // add bias
        l1[i] = fmaxf(0.0f, l1[i]); // relu activation
    }
        
    // layer 2
    for(int i = 0; i<64; i++){ // for each node in the next layer
        for(int j = 0; j<12; j++){ // add contribution of node in prev. layer
            l2[i] += (fingertip_net.w2[j][i]*l1[j]);
        }
        l2[i] += fingertip_net.b2[i]; // add bias
        l2[i] = fmaxf(0.0f, l2[i]); // relu activation
    }   
    
    // layer 3 // added for larger network architecture
    for(int i = 0; i<64; i++){ // for each node in the next layer
        for(int j = 0; j<64; j++){ // add contribution of node in prev. layer
            l3[i] += (fingertip_net.w3[j][i]*l2[j]);
        }
        l3[i] += fingertip_net.b3[i]; // add bias
        l3[i] = fmaxf(0.0f, l3[i]); // relu activation
    }

    // layer 4
    for(int i = 0; i<5; i++){ // for each node in the next layer
        for(int j = 0; j<64; j++){ // add contribution of node in prev. layer
            l4[i] += fingertip_net.w4[j][i]*l3[j];
        }
        l4[i] += fingertip_net.b4[i];// add bias
        l4[i] = fmaxf(0.0f, l4[i]); // relu activation
    }

    // post-process, re-scale decoded data
    for (int i=0; i<5; i++) {
        output_data[i] = 0.0f;
        output_data[i] = (l4[i]*(fingertip_net.maxims[i]-fingertip_net.minims[i])) + fingertip_net.minims[i]; // - abs(fingertip_net.minims[i]);
    
    }      
    
}
    


void ForceSensor::Calibrate(){

    float temp_offsets[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int num_samples = 10;
    for (int i=0; i<num_samples; i++){
        Sample();
        for (int j=0; j<8; j++){
            temp_offsets[j] += ((float)raw_data[j])/((float)num_samples);
        }
        HAL_Delay(10); // wait for 10ms for next sample
    }
    for (int i=0; i<8; i++){
        offsets[i] = (int)temp_offsets[i];
    }

}
    
    




    





//void calibrateSensor(uint16_t* offsets){
//    
//    float temp_offsets[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//    int samps = 10;
//    
//    for (int i=0; i<samps; i++){
//        for (int j=0; j<8; j++){
//            temp_offsets[j] += (float)spi3.binary(j);
//        }
//        wait_ms(1);
//    }
//    
//    for (int i=0; i<8; i++){
//        temp_offsets[i] = temp_offsets[i]/((float)samps); // get overall offset
//        offsets[i] = (uint16_t)temp_offsets[i]; // convert to int
//    }
//
//    }
