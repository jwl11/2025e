#ifndef __FILTER_H
#define __FILTER_H
#include "stdint.h"


/////////////PT1低通滤波器//////////////////
typedef struct {
    float tau;          // 时间常数（控制滤波强度，tau越大滤波越平缓）
    float last_output;  // 上一次滤波输出
    float dt;           // 数据采样周期（单位：秒，如飞控常用1kHz采样则dt=0.001）
} PT1Filter_t;
void PT1Filter_Init(PT1Filter_t *filter, float tau, uint16_t Hz);
void PT1Filter_InitWithFreq(PT1Filter_t *filter, float fc, uint16_t Hz);
float PT1Filter_Apply(PT1Filter_t *filter, float input); // 应用滤波器
/////////////PT1低通滤波器//////////////////


//////////////////////PT2滤波器(二阶低通滤波器)////////////////////////////
typedef struct {
    float cutoffHz;        // 截止频率
    float sampleRateHz;    // 采样频率
    float b0, b1, b2;      // 滤波器系数
    float a1, a2;          // 滤波器系数
    float x1, x2;          // 输入历史值
    float y1, y2;          // 输出历史值
    uint8_t initialized;   // 初始化标志
} pt2Filter_t;
void pt2FilterInit(pt2Filter_t *filter, float cutoffHz, float sampleRateHz, float damping);
float pt2FilterApply(pt2Filter_t *filter, float input); // 应用滤波器
//////////////////////PT2滤波器(二阶低通滤波器)////////////////////////////


/////////////////////卡尔曼滤波器/////////////////////////
typedef struct {
    float q; // 过程噪声协方差
    float r; // 测量噪声协方差
    float x; // 状态估计值
    float p; // 估计误差协方差
    float k; // 卡尔曼增益
} KalmanFilter;
void KalmanFilter_Init(KalmanFilter *kf, float q,float r,float x,float p);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif
