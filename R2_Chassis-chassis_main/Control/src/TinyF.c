#include "TinyF.h"

// 定义全局状态数组
TinyF_Context_t TinyF_CTX[TINYF_PORT_NUM] = {0};

void MINIF_Processing_Data(TinyF_Port_t port, uint8_t RXdata) {
	
    if (port >= TINYF_PORT_NUM) {
        return;
    }
   
				
    // 获取当前串口的状态指针
    TinyF_Context_t *ctx = &TinyF_CTX[port];
    
    // 防溢出：如果缓冲区已满，重置状态机
    if (ctx->index >= sizeof(ctx->recv_buf)) {
        ctx->index = 0;
        ctx->parsing = 0;
        ctx->comma_pos = 0;
        return;
    }
    
    // 存储接收到的字节
    ctx->recv_buf[ctx->index++] = RXdata;
    
    // 状态机解析
    switch (ctx->parsing) {
        case 0:  // 等待帧头0x20 (空格)
            if (RXdata == 0x20) {
                ctx->parsing = 1;   // 进入距离解析状态
                ctx->index = 1;
            } else {
                ctx->index = 0;     // 非头字符，重置
            }
            break;
            
        case 1:  // 解析距离值
            if (RXdata == 0x2C) {   // 遇到逗号
                ctx->parsing = 2;   // 进入分隔符检查状态
                ctx->comma_pos = ctx->index - 1;  // 记录逗号位置
            }
            break;
            
        case 2:  // 检查分隔符0x20 (空格)
            if (RXdata == 0x20) {
                ctx->parsing = 3;   // 进入置信度解析状态
            } else {
                // 格式错误，重置状态机
                ctx->parsing = 0;
                ctx->index = 0;
                ctx->comma_pos = 0;
            }
            break;
            
        case 3:  // 解析置信度
            if (RXdata == 0x0A) {
                // 提取距离值
                uint8_t dist_len = ctx->comma_pos - 1;
                if (dist_len > 5) dist_len = 5;
                char dist_str[6] = {0};
                memcpy(dist_str, &ctx->recv_buf[1], dist_len);
                dist_str[dist_len] = '\0';
                
                // 提取置信度
                uint8_t conf_start = ctx->comma_pos + 2;
                uint8_t conf_len = ctx->index - conf_start - 1;
                if (conf_len > 2) conf_len = 2;
                char conf_str[3] = {0};
                memcpy(conf_str, &ctx->recv_buf[conf_start], conf_len);
                conf_str[conf_len] = '\0';
                
                // 转换为数值
                ctx->distance_value = atoi(dist_str);
                ctx->confidence_value = atoi(conf_str);
                
                // 检查数据有效性
                if (ctx->distance_value < DISTANCE_MIN || 
                    ctx->distance_value > DISTANCE_MAX || 
                    ctx->confidence_value > CONFIDENCE_MAX) {
                    ctx->distance_value = 0;
                    ctx->confidence_value = 0;
                }
                
                // 设置数据就绪标志
                ctx->data_ready = 1;
                
                // 重置状态机
                ctx->index = 0;
                ctx->parsing = 0;
                ctx->comma_pos = 0;
            }
            break;
            
        default:
            // 未知状态，重置状态机
            ctx->parsing = 0;
            ctx->index = 0;
            ctx->comma_pos = 0;
            break;
    }
}
