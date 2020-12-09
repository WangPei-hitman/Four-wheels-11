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
#include"sc_host.h"

#include "image.h"
#include "my_elecmag.hpp"


#define CTRL_SPD_CTRL_MS    (5U)
#define CTRL_DIR_CTRL_MS    (20U)

typedef struct PID_para
{
    float kp,ki,kd;
}PID_para_t;

typedef struct error_para
{
    float errorCurr,
    errorLast,
    errorPrev;
}error_para_t;

typedef enum CTRL_state
{
    stop=0,
    start=1,
    movement,
}CTRL_state_t;
typedef enum CTRL_event
{
    manual,
    stopline,
    motionsigns,
    other,
}CTRL_event_t;


extern float transform[6];///<用于发送数据

extern int dir_front,spd_front;
extern int thro;
extern float speedL[3],speedR[3];
extern pidCtrl_t dirPID_PIC, dirPID_EMA;
extern uint32_t PicSwitch[3];
extern uint32_t EmaSwitch[3];
extern uint32_t spdenable[3];

extern float servo_ctrlOutput;
extern float ctrl_spdL , ctrl_spdR ;

extern float kinner[3],kL,kR;
extern PID_para_t spdPID;
extern error_para_t spdLerror,spdRerror;
extern float motorLSet,motorRSet;
extern float motorLOutput,motorROutput;
extern int timerCount;


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
/*
 * @brief:增量式PID误差更新，返回增量
 */
float UpdatePIDandCacul(PID_para_t PID,error_para_t* Err,float error);

void AC(menu_keyOp_t*  op);

void TimerCount (void*);
void StateMachineUpdate(void*);
void StateMachineInit(void);
CTRL_event_t eventJudge(GG gg);


#endif /* MY_CONTROL_HPP_ */
