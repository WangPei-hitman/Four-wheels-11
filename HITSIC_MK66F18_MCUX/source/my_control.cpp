/*
 * my_control.cpp
 *
 *  Created on: 2020年11月14日
 *      Author: WangP
 */

#include"my_control.hpp"

#define EPS 10

/*中断任务句柄*/
pitMgr_t* motorcontrol =nullptr;
pitMgr_t* servocontrol =nullptr;
pitMgr_t* directiontask=nullptr;

float servo_ctrl=7.5f;
int myerror1 = 0,myerror2=0;
float kp=0,kd=0;
int front = 50;
int midint,thro;
extern int protect;//protection
float speedL[3]={0.0f,0.0f,100.0f},speedR[3]={0.0f,0.0f,100.0f};
float kt=0.0f;

uint32_t error = 0;

void CTRL_MENUSETUP(menu_list_t* List)
{
    static menu_list_t *TestList = MENU_ListConstruct("para_control", 20, List);
            assert(TestList);
             MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, TestList, "para_control", 0, 0));
              {
                   MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedL[0], "speedL",10 ,menuItem_data_global));
                   MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedR[0], "speedR",11 ,menuItem_data_global));
                   MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedL[2], "speedLmax",20 ,menuItem_data_global));
                   MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedR[2], "speedRmax",21 ,menuItem_data_global));

                   MENU_ListInsert(TestList, MENU_ItemConstruct(variType,&front, "front",13 ,menuItem_data_global));
                   MENU_ListInsert(TestList, MENU_ItemConstruct(variType,&thro, "threshold",19 ,menuItem_data_global));
               }

     static menu_list_t *pidList = MENU_ListConstruct("pidList", 20,List);
                 assert(pidList);
                 MENU_ListInsert(List, MENU_ItemConstruct(menuType,pidList , "PID_control", 0, 0));
                 {
                     MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&dirPID.kp, "kp",14 ,menuItem_data_global));
                     MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&dirPID.kd, "kd",15 ,menuItem_data_global));
                     MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&dirPID.ki, "ki",22 ,menuItem_data_global));
                 }
                 MENU_ListInsert(List, MENU_ItemConstruct(varfType, &dirPID.errCurr, "error", 17,menuItem_data_ROFlag|menuItem_data_NoSave|menuItem_data_NoLoad));
                 MENU_ListInsert(List, MENU_ItemConstruct(variType, &midint, "mid", 18,menuItem_data_ROFlag|menuItem_data_NoSave|menuItem_data_NoLoad));
                 MENU_ListInsert(List, MENU_ItemConstruct(varfType,&servo_ctrlOutput, "servo",12 ,menuItem_data_ROFlag|menuItem_data_NoSave|menuItem_data_NoLoad));
}


/**控制环初始化*/
void controlInit(void)
{
    motorcontrol =  pitMgr_t::insert(6U,2U,motorCTRL,pitMgr_t::enable);
    assert(motorcontrol);

    directiontask = pitMgr_t::insert(20U,3U,directionCTRL,pitMgr_t::enable);
    assert(directiontask);

}

void motorCTRL (void)
{
        if(protect>=5)
        {
            SCFTM_PWM_Change(FTM0,kFTM_Chnl_0,20000U,0.0f);
            SCFTM_PWM_Change(FTM0,kFTM_Chnl_1,20000U,0.0f);

            SCFTM_PWM_Change(FTM0,kFTM_Chnl_2,20000U,0.0f);
            SCFTM_PWM_Change(FTM0,kFTM_Chnl_3,20000U,0.0f);
        }
        else
        {
      if(abs(myerror2)<EPS)
      {
              SCFTM_PWM_Change(FTM0,kFTM_Chnl_0,20000U,0.0f);
              SCFTM_PWM_Change(FTM0,kFTM_Chnl_1,20000U,speedR[2]);

              SCFTM_PWM_Change(FTM0,kFTM_Chnl_2,20000U,speedL[2]);
              SCFTM_PWM_Change(FTM0,kFTM_Chnl_3,20000U,0.0f);
      }
      else
      {
          if(myerror2>0)//right
          {
                  SCFTM_PWM_Change(FTM0,kFTM_Chnl_0,20000U,0.0f);
                  SCFTM_PWM_Change(FTM0,kFTM_Chnl_1,20000U,speedR[0]);

                  SCFTM_PWM_Change(FTM0,kFTM_Chnl_2,20000U,speedL[0]+kt*abs(servo_ctrl-7.5));
                  SCFTM_PWM_Change(FTM0,kFTM_Chnl_3,20000U,0.0f);
          }
          else //left
          {
              SCFTM_PWM_Change(FTM0,kFTM_Chnl_0,20000U,0.0f);
              SCFTM_PWM_Change(FTM0,kFTM_Chnl_1,20000U,speedR[0]+kt*abs(servo_ctrl-7.5));

              SCFTM_PWM_Change(FTM0,kFTM_Chnl_2,20000U,speedL[0]);
              SCFTM_PWM_Change(FTM0,kFTM_Chnl_3,20000U,0.0f);
          }
       }
    }
}


pidCtrl_t dirPID =
{ .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f, };

float servo_ctrlOutput =7.5f;

void directionCTRL(void)
{
    midint =(int)(mid_line[front]);
    servo_ctrlOutput =7.5f - PIDCTRL_UpdateAndCalcPID(&dirPID, (float)(midint-94));
    if(255==midint)
        servo_ctrlOutput=7.5f;
    else if(servo_ctrlOutput>8.5f)
        servo_ctrlOutput = 8.5f;
    else if(servo_ctrlOutput<6.6f)
        servo_ctrlOutput = 6.6f;

        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_ctrlOutput);
}


void motorSetSpeed(float speedL, float speedR)
{
    if(speedL>100.0f)
    {
        speedR-=speedL-100.0f;
        speedL =100.0f;
    }
    if(speedL<-100.f)
    {
        speedR-=speedL+100.0f;
        speedL = -100.0f;
    }

    if(speedR>100.0f)
    {
        speedL-=speedR-100.0f;
        speedR =100.0f;
    }
    if(speedR<-100.f)
    {
        speedL-=speedR+100.0f;
        speedR = -100.0f;
    }
    if (speedR > 0)
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_1, 20000U, speedR);
    }
    else
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_0, 20000U, -speedR);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_1, 20000U, 0.0f);
    }
    if (speedL > 0)
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_2, 20000U, speedL);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_3, 20000U, -speedL);
    }
}
