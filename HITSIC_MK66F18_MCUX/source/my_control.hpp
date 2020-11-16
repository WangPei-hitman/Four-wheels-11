/*
 * my_control.hpp
 *
 *  Created on: 2020年11月14日
 *      Author: WangP
 */

#ifndef MY_CONTROL_HPP_
#define MY_CONTROL_HPP_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

/** HITSIC_Module_APP */
#include "app_menu.hpp"

/** HITSIC_Module_SYS */

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"

/** HITSIC_Module_LIB */
#include "lib_pidctrl.h"

#include "sc_ftm.h"
#include "image.h"


extern float servo_ctrl;
extern int myerror1 ,myerror2;
extern float kp,kd;
extern int front;
extern int midint,thro;
extern float kt;
extern float speedL[3],speedR[3];

void CTRL_MENUSETUP(menu_list_t*);
void controlInit(void);
void motorCTRL (void);
void servoCTRL (void);
void directionCTRL(void);
void motorSetSpeed(float speedL,float speedR);


#endif /* MY_CONTROL_HPP_ */
