#include "dshot600.h"

/*
 *  电调
 */
rmt_data_t dshotPacket[16];
rmt_obj_t* rmt_send1 = NULL;
rmt_obj_t* rmt_send2 = NULL;
//hw_timer_t * timer = NULL;
uint16_t dshotUserInputValue1 = 111;
uint16_t dshotUserInputValue2 = 111;



//电调初始化
void dshot_init(void)
{
    // 64 128 192 256 
    if ((rmt_send1 = rmtInit(2, true, RMT_MEM_64)) == NULL) {   //左边电机
        Serial.println("rmt_send1 init sender failed\n");
    }
    
    if ((rmt_send2 = rmtInit(0, true, RMT_MEM_128)) == NULL) {  //右边电机
        Serial.println("rmt_send2 init sender failed\n");
    }

    
    float realTick = rmtSetTick(rmt_send1, 12.5); // 12.5ns sample rate
    Serial.printf("rmt_send1 tick set to: %fns\n", realTick);
    realTick = rmtSetTick(rmt_send2, 12.5); // 12.5ns sample rate
    Serial.printf("rmt_send2 tick set to: %fns\n", realTick);

    while (millis() < 1111) 
    {
        dshotOutput1(0, false);//左边电机
        dshotOutput2(0, false);//右边电机
        delay(1);  
    }    
     
}


//设置左边电调值：0~2000
void dshotOutput1(uint16_t value, bool telemetry) {
    
    uint16_t packet;

    if(value<Accelerator_Min)
      value = Accelerator_Min;
    else if(value>Accelerator_Max)
      value = Accelerator_Max;
    
    // telemetry bit    
    if (telemetry) {
        packet = (value << 1) | 1;
    } else {
        packet = (value << 1) | 0;
    }

    // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
    // 计算校验和
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    packet = (packet << 4) | csum;

    // durations are for dshot600
    // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
    // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
    // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
    // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
    for (int i = 0; i < 16; i++) {
        if (packet & 0x8000)
        {
              dshotPacket[i].level0 = 1;
              dshotPacket[i].duration0 = 100;
              dshotPacket[i].level1 = 0;
              dshotPacket[i].duration1 = 34;
        } 
        else 
        {
              dshotPacket[i].level0 = 1;
              dshotPacket[i].duration0 = 50;
              dshotPacket[i].level1 = 0;
              dshotPacket[i].duration1 = 84;
        }
        packet <<= 1;
    }
    
    rmtWrite(rmt_send1, dshotPacket, 16);
    
    return;

}


//设置右边电调值：0~2000
void dshotOutput2(uint16_t value, bool telemetry) {
    
    uint16_t packet;
    
    if(value<Accelerator_Min)
      value = Accelerator_Min;
    else if(value>Accelerator_Max)
      value = Accelerator_Max;
    
        
    // telemetry bit    
    if (telemetry) {
        packet = (value << 1) | 1;
    } else {
        packet = (value << 1) | 0;
    }

    // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    packet = (packet << 4) | csum;

    // durations are for dshot600
    // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
    // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
    // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
    // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
    for (int i = 0; i < 16; i++) {
        if (packet & 0x8000)
        {
              dshotPacket[i].level0 = 1;
              dshotPacket[i].duration0 = 100;
              dshotPacket[i].level1 = 0;
              dshotPacket[i].duration1 = 34;
        } 
        else 
        {
              dshotPacket[i].level0 = 1;
              dshotPacket[i].duration0 = 50;
              dshotPacket[i].level1 = 0;
              dshotPacket[i].duration1 = 84;
        }
        packet <<= 1;
    }
    
    rmtWrite(rmt_send2, dshotPacket, 16);
    
    return;

}
