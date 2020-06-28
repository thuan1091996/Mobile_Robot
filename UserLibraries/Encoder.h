/*
 * Encoder.h
 *
 *  Created on: Oct 15, 2019
 *      Author: minht
 */

#ifndef USERLIBRARIES_ENCODER_H_
#define USERLIBRARIES_ENCODER_H_

#define ENCODER0_RESOLUTION      1324
#define ENCODER1_RESOLUTION      1324
#define SPEED_SAMPLE_PERIOD     0.1
#define QEI0_PRIORITY           2
#define QEI1_PRIORITY           2

void QEI0_INTHandler(void);
void QEI1_INTHandler(void);

void QEI_Init(uint8_t ui8Priority0,
							uint8_t ui8Priority1);

void Update_Position0(float *Pt_Pos0);
void Update_Position1(float *Pt_Pos1);
void Update_Velocity0(int16_t *Pt_Vel0);
void Update_Velocity1(int16_t *Pt_Vel1);
#endif /* USERLIBRARIES_ENCODER_H_ */
