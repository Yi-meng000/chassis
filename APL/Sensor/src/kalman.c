#include "kalman.h"

Sensor_Filter Radar_Filter;

void Kalman_Init(Kalman *kal, float _Q, float _R)
{
    kal->x_last = 0;
    kal->p_last = 1;

    kal->Q = _Q;
    kal->R = _R;

    kal->A = 1;
    kal->H = 1;
}
float Kalman_Filter(Kalman *kal, float input)
{
    kal->x_mid = kal->A * kal->x_last;
    kal->p_mid = kal->A * kal->A * kal->p_last + kal->Q;
    kal->kg = (kal->H * kal->p_mid) /
              ((kal->H * kal->H * kal->p_mid) + kal->R);
    kal->x_now = kal->x_mid +
                 (kal->kg * (input - (kal->H * kal->x_mid)));
    kal->p_now = (1 - kal->kg) * kal->p_mid;

    kal->p_last = kal->p_now;
    kal->x_last = kal->x_now;

    return kal->x_now;
}
void Filter_Init(Sensor_Filter *filter)
{
    memset(filter, 0, sizeof(Sensor_Filter));
    filter->mode = NO_FILTER;
}
float MAD_Caculate(float *data, uint8_t len)
{
    float sorted[WINDOW_SIZE];
    float dev[WINDOW_SIZE];
    float median;

    memcpy(sorted, data, len * sizeof(float));
    Insertion_Sort(sorted, len);

    median = calc_median(sorted, len);
    for (int i = 0; i < len; i++)
        dev[i] = fabsf(data[i] - median);
    Insertion_Sort(dev, len);
    return calc_median(dev, len);
}

void Filter_BuffPush(Filter_State *state, float value)
{
    state->buf[state->head] = value;
    state->head = (state->head + 1) % WINDOW_SIZE;
    if (state->count < WINDOW_SIZE)
        state->count++;
}

void Filter_BuffLinearize(Filter_State *state, float *out_data)
{
    uint8_t begin = (state->count == WINDOW_SIZE) ? state->head : 0;
    for (int i = 0; i < state->count; i++)
        out_data[i] = state->buf[(begin + i) % WINDOW_SIZE];
}

float Filter_alpha_Adjust(float change)
{
    if (change <= EMA_ALPHA_CHANGETHRESH)
        return EMA_ALPHA_MIN;
    else if (change >= EMA_ALPHA_CHANGETHRESH * 2.0f)
        return EMA_ALPHA_MAX;
    return (EMA_ALPHA_MIN + (change - EMA_ALPHA_CHANGETHRESH) / EMA_ALPHA_CHANGETHRESH * (EMA_ALPHA_MAX - EMA_ALPHA_MIN));
}
float Filter_Average(Filter_State *state, float raw_data)
{
    float ave_data[WINDOW_SIZE];

    Filter_BuffPush(state, raw_data);
    Filter_BuffLinearize(state, ave_data);
    return calc_average(ave_data, state->count);
}
float Filter_Median(Filter_State *state, float raw_data)
{
    float window_buff[WINDOW_SIZE];
    float alpha = 0;
    float median, mad;
    float score = 0;
    Filter_BuffPush(state, raw_data);
    if (state->count >= 3)
    {
        Filter_BuffLinearize(state, window_buff);
        Insertion_Sort(window_buff, state->count);
        median = calc_median(window_buff, state->count);
        mad = MAD_Caculate(window_buff, state->count);

        mad = (mad < 1e-4f) ? 1e-4f : (mad / MAD_CORRECTTHRESH);
        score = fabsf(raw_data - median) / mad;
        if (score > MAD_THRESHOLD)
            state->output_valid = false;
        else
            state->output_valid = true;
    }
    float ema_input = (state->output_valid) ? raw_data : median;
#if EMA_USE
    float ema_output;
    if (state->EMA_initialize)
    {
        alpha = Filter_alpha_Adjust(ema_input - state->EMA_pre);
        ema_output = alpha * ema_input + (1.0f - alpha) * state->EMA_pre;
    }
    else
    {
        alpha = EMA_ALPHA_MIN;
        ema_output = ema_input;
        state->EMA_initialize = true;
    }
    state->EMA_pre = ema_output;
    return ema_output;
#else
    return ema_input;
#endif
}
void Filter_Func(Sensor_Filter *filter, vector2d raw_data, vector2d *output)
{

    switch (filter->mode)
    {
    case NO_FILTER:
        output->x = raw_data.x;
        output->y = raw_data.y;
        break;
    case AVERAGE:
        output->x = Filter_Average(&filter->f_x, raw_data.x);
        output->y = Filter_Average(&filter->f_y, raw_data.y);
        break;
    case MEDIAN:
        output->x = Filter_Median(&filter->f_x, raw_data.x);
        output->y = Filter_Median(&filter->f_y, raw_data.y);
        break;
    default:
        break;
    }
}
