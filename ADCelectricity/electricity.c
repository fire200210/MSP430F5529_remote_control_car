#include <msp430.h>
#include <stdint.h>
#include <math.h>
/*
 * electricity.c
 *
 *  Created on: 2023�~4��12��
 *      Author: lin09
 */

#define VOLTAGE_MIN 0.5f // �̤p�q����
#define VOLTAGE_MAX 3.3f // �̤j�q����
#define BATTERY_MIN 0 // �̤p�q�q��
#define BATTERY_MAX 100 // �̤j�q�q��

void ADC_init(void)
{
  // �t�mADC�Ҳ�
  //P6SEL |= BIT0; // Configure P6.0 as analog input channel
  ADC12CTL0 = ADC12SHT0_8 | ADC12ON; // �]�mADC�����M�ļ˫O���ɶ�
  ADC12CTL1 = ADC12SHP; // ���ADC�ļ�Ĳ�o�覡
  ADC12CTL2 |= ADC12RES_2; // �]�mADC������v
  ADC12MCTL0 |= ADC12INCH_1; // ���ADC��J�q�D�]A0�޸}�^

  P6SEL |= BIT7; // �t�m P6.7 �� ADC ��J

}

uint16_t ADC_sample(void)
{
  ADC12CTL0 |= ADC12ENC | ADC12SC; // �ҥ�ADC�A�Ұʤ@���ļ˩M�ഫ
  while (ADC12CTL1 & ADC12BUSY); // �����ഫ����
  return ADC12MEM0; // ��^ADC�ഫ���G
}

uint8_t battery_level(float voltage)
{
  float percent = (voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN); // �p��q�q�ʤ���
  percent = percent < 0 ? 0 : percent; // �T�O�q�q�ʤ��񤣤p��0%
  percent = percent > 1 ? 1 : percent; // �T�O�q�q�ʤ��񤣤j��100%
  return (uint8_t)(percent * (BATTERY_MAX - BATTERY_MIN) + BATTERY_MIN); // ��^�q�q��
}

float roundToOneDecimal(float num)
{
    return roundf(num * 100) / 100.0f;
}



