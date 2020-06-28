/*
 * Profile.h
 *
 *  Created on: Nov 20, 2019
 *      Author: Minh Thuan
 */

#ifndef USERLIBRARIES_PROFILE_H_
#define USERLIBRARIES_PROFILE_H_

#define PROFILE0 HWREG(0x40025004)		//PF0
#define PROFILE_PIN0 GPIO_PIN_0

#define PROFILE1 HWREG(0x40025008)		//PF1
#define PROFILE_PIN1 GPIO_PIN_1

#define PROFILE2 HWREG(0x40025010)		//PF2
#define PROFILE_PIN2 GPIO_PIN_2

#define PROFILE3 HWREG(0x40025020)		//PF3
#define PROFILE_PIN3 GPIO_PIN_3

#define PROFILE4 HWREG(0x40025040)		//PF4
#define PROFILE_PIN4 GPIO_PIN_4

#define PROFILE5 HWREG(0x40025080)		//PF5
#define PROFILE_PIN5 GPIO_PIN_5

#define PROFILE6 HWREG(0x40025100)		//PF6
#define PROFILE_PIN6 GPIO_PIN_6

#define PROFILE7 HWREG(0x40025200)		//PF7
#define PROFILE_PIN7 GPIO_PIN_7

#define DATA_Profile HWREG(0x400253FC)

#define PROFILE_PORT GPIO_PORTF_BASE

void Profile_Init(void);
void Profile_Toggle(uint8_t ui8Pin);

#define Profile_Set0() (PROFILE0 = PROFILE_PIN0)
#define Profile_Clear0() (PROFILE0 = 0x00)
#define Profile_Toggle0() (PROFILE0 ^= PROFILE_PIN0)

#define Profile_Set1() (PROFILE1 = PROFILE_PIN1)
#define Profile_Clear1() (PROFILE1 = 0x00)
#define Profile_Toggle1() (PROFILE1 ^= PROFILE_PIN1)

#define Profile_Set2() (PROFILE2 = PROFILE_PIN2)
#define Profile_Clear2() (PROFILE2 = 0x00)
#define Profile_Toggle2() (PROFILE2 ^= PROFILE_PIN2)

#define Profile_Set3() (PROFILE3 = PROFILE_PIN3)
#define Profile_Clear3() (PROFILE3 = 0x00)
#define Profile_Toggle3() (PROFILE3 ^= PROFILE_PIN3)

#define Profile_Set4() (PROFILE4 = PROFILE_PIN4)
#define Profile_Clear4() (PROFILE4 = 0x00)
#define Profile_Toggle4() (PROFILE4 ^= PROFILE_PIN4)

#define Profile_Set5() (PROFILE5 = PROFILE_PIN5)
#define Profile_Clear5() (PROFILE5 = 0x00)
#define Profile_Toggle5() (PROFILE5 ^= PROFILE_PIN5)

#define Profile_Set6() (PROFILE6 = PROFILE_PIN6)
#define Profile_Clear6() (PROFILE6 = 0x00)
#define Profile_Toggle6() (PROFILE6 ^= PROFILE_PIN6)

#endif /* USERLIBRARIES_PROFILE_H_ */
