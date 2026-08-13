#ifndef __ULTRASOUND_H__
#define __ULTRASOUND_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
typedef struct UltraSound_Communicate
{
    bool head_flag;
    bool rx_flag;
    uint8_t rx_it;
    uint8_t rx_data[4];
    uint16_t dis;
    uint8_t sum;
}UltraSound_Comm;

void UltraSound_Receive(uint8_t rx_data,UltraSound_Comm *uls);

extern UltraSound_Comm UltraSound_back;
extern UltraSound_Comm UltraSound_front;
#endif /* __ULTRASOUND_H__ */
