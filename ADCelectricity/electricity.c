#include <msp430.h>
#include <stdint.h>
#include <math.h>
/*
 * electricity.c
 *
 *  Created on: 2023年4月12日
 *      Author: lin09
 */

#define VOLTAGE_MIN 0.5f // 最小電壓值
#define VOLTAGE_MAX 3.3f // 最大電壓值
#define BATTERY_MIN 0 // 最小電量值
#define BATTERY_MAX 100 // 最大電量值

void ADC_init(void)
{
  // 配置ADC模組
  //P6SEL |= BIT0; // Configure P6.0 as analog input channel
  ADC12CTL0 = ADC12SHT0_8 | ADC12ON; // 設置ADC時鐘和採樣保持時間
  ADC12CTL1 = ADC12SHP; // 選擇ADC採樣觸發方式
  ADC12CTL2 |= ADC12RES_2; // 設置ADC的分辨率
  ADC12MCTL0 |= ADC12INCH_1; // 選擇ADC輸入通道（A0引腳）

  P6SEL |= BIT7; // 配置 P6.7 為 ADC 輸入

}

uint16_t ADC_sample(void)
{
  ADC12CTL0 |= ADC12ENC | ADC12SC; // 啟用ADC，啟動一次採樣和轉換
  while (ADC12CTL1 & ADC12BUSY); // 等待轉換完成
  return ADC12MEM0; // 返回ADC轉換結果
}

uint8_t battery_level(float voltage)
{
  float percent = (voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN); // 計算電量百分比
  percent = percent < 0 ? 0 : percent; // 確保電量百分比不小於0%
  percent = percent > 1 ? 1 : percent; // 確保電量百分比不大於100%
  return (uint8_t)(percent * (BATTERY_MAX - BATTERY_MIN) + BATTERY_MIN); // 返回電量值
}

float roundToOneDecimal(float num)
{
    return roundf(num * 100) / 100.0f;
}



