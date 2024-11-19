/*
 * electricity.h
 *
 *  Created on: 2023¦~4¤ë12¤é
 *      Author: lin09
 */

#ifndef ELECTRICITY_H_
#define ELECTRICITY_H_

void ADC_init(void);

uint16_t ADC_sample(void);

uint8_t battery_level(float voltage);

float roundToOneDecimal(float num);

#endif /* ELECTRICITY_H_ */
