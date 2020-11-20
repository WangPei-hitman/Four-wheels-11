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
#include "my_elecmag.hpp"


extern float servo_ctrlOutput;
extern int front;
extern int thro;
extern float speedL[3],speedR[3];
extern pidCtrl_t dirPID_PIC, dirPID_EMA;
extern uint32_t PicSwitch[3];
extern uint32_t EmaSwitch[3];
extern uint32_t spdenable[3];

/*
 * @brief:控制参数菜单
 */
void CTRL_MENUSETUP(menu_list_t*);
/*
 * @brief:控制环初始化
 */
void controlInit(void);
/*
 * @brief:电机控制(速度环)
 */
void motorCTRL (void*);
/*
 * @brief:转向环
 */
void directionCTRL(void*);
/*
 * @brief:电机速度设置封装(-100.0f,100.0f)
 */
void motorSetSpeed(float speedL,float speedR);
/*
 * @brief:图像控制车启动菜单接口函数，可通过菜单proctype调用延时启动开跑
 */
void StartPicture(menu_keyOp_t* op);

#endif /* MY_CONTROL_HPP_ */
