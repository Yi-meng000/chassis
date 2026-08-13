#include "Ultrasound_new.h"
UltraSound_Comm UltraSound_back = {0};
UltraSound_Comm UltraSound_front = {0};

void UltraSound_Receive(uint8_t rx_data,UltraSound_Comm *uls)
{

    if(uls->rx_flag)
    {
        uls->rx_data[++uls->rx_it] = rx_data;
        if(uls->rx_it >= 3)
        {
            if(uls->sum == rx_data)
            {
                uls->dis = (uls->rx_data[1] << 8 | uls->rx_data[2]) * 0.1;
//                uls->dis = uls->dis * 0.1;
            }
						uls->rx_it = 0;
            uls->rx_flag = 0;
            uls->sum = 0;
        }
        else
            uls->sum += rx_data;
    }
    else
    {
        if(rx_data == 0xFF)
        {
            uls->sum = 0xFF;
            uls->rx_flag = 1;
        }
    }
}


