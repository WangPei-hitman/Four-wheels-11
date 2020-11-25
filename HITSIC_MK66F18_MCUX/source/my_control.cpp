/*
 * my_control.cpp
 *
 *  Created on: 2020年11月14日
 *      Author: WangP
 */

#include"my_control.hpp"

#define EPS 10
#define DRP(x,y)  ((x-y)/(x*y))
#define SPD_COEFF 0.03463802

/*中断任务句柄*/
pitMgr_t* motorcontrol =nullptr;
pitMgr_t* directiontask=nullptr;
pitMgr_t* EMAcolloction=nullptr;




int front = 50;//前瞻
int thro;//摄像头阈值






void CTRL_MENUSETUP(menu_list_t* List)
{
    static menu_list_t *picture = MENU_ListConstruct("picture", 20, List);
    assert(picture);
    MENU_ListInsert(List, MENU_ItemConstruct(menuType, picture, "picture", 0, 0));
    {
        static menu_list_t *Low = MENU_ListConstruct("LOW", 20, picture);
        assert(Low);
        MENU_ListInsert(picture, MENU_ItemConstruct(menuType, Low, "LOW", 0, 0));
        {
            static menu_list_t *pidcontrol_low = MENU_ListConstruct("PID", 20, picture);
            assert(pidcontrol_low);
            MENU_ListInsert(Low, MENU_ItemConstruct(menuType, pidcontrol_low, "PID", 0, 0));
            {
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &dirPID_PIC.kp, "dir_kp", 3, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &dirPID_PIC.kd, "dir_kd", 4, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &dirPID_PIC.ki, "dir_ki", 5, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &spdPID.kp, "spd_kp", 14, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &spdPID.kd, "spd_kd", 15, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &spdPID.ki, "spd_ki", 16, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(variType, &front, "front", 6, menuItem_data_region));
            }
            MENU_ListInsert(Low, MENU_ItemConstruct(variType, &thro, "threshold", 11, menuItem_data_global));
            MENU_ListInsert(Low,
                    MENU_ItemConstruct(varfType, &dirPID_PIC.errCurr, "error_pic", 0U, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(Low, MENU_ItemConstruct(procType, StartPicture, "StartPicture", 0U, 0U));
        }
    }
    static menu_list_t *basicpara = MENU_ListConstruct("basic_para", 20, List);
    assert(basicpara);
    MENU_ListInsert(List, MENU_ItemConstruct(menuType, basicpara, "basic_para", 0, 0));
    {
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &motorLSet, "speedL", 1, menuItem_data_region));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &motorRSet, "speedR", 2, menuItem_data_region));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &speedL[2], "speedMax", 17, menuItem_data_region));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &speedL[1], "speedMin", 18, menuItem_data_region));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &ctrl_spdL, "ctrl_spdL", 0U, menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &ctrl_spdR, "ctrl_spdR", 0U, menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(varfType, &servo_ctrlOutput, "Servo", 0U, menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(basicpara, MENU_ItemConstruct(procType,AC, "AC", 0U, menuItem_proc_runOnce));

    }
    static menu_list_t *Switch = MENU_ListConstruct("Switch", 20, List);
    assert(Switch);
    MENU_ListInsert(List, MENU_ItemConstruct(menuType, Switch, "Switch", 0, 0));
    {
        MENU_ListInsert(Switch,
                MENU_ItemConstruct(variType, &PicSwitch, "PicSwitch", 0U, menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(Switch,
                MENU_ItemConstruct(variType, &EmaSwitch, "EmaSwitch", 0U, menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(Switch,
                MENU_ItemConstruct(variType, &spdenable, "spdenable", 0U, menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
    }
}


/**控制环初始化*/
void controlInit(void)
{
    motorcontrol =  pitMgr_t::insert(CTRL_SPD_CTRL_MS,2U,motorCTRL,pitMgr_t::enable);//速度环5ms
    assert(motorcontrol);

    directiontask = pitMgr_t::insert(CTRL_DIR_CTRL_MS,4U,directionCTRL,pitMgr_t::enable);//转向环20ms
    assert(directiontask);

    //EMAcolloction = pitMgr_t::insert(CTRL_DIR_CTRL_MS,4U,FilterHandler,pitMgr_t::enable);
    //assert(EMAcolloction);
}


float speedL[3]={0.0f,-100.0f,100.0f},speedR[3]={0.0f,-100.0f,100.0f};

float ctrl_spdL = 0.0f, ctrl_spdR = 0.0f;

float motorLOutput=0.0f,motorROutput=0.0f;

float motorLSet=0.0f,motorRSet=0.0f;

uint32_t spdenable[3]={0,0,1};///<速度环使能

PID_para_t spdPID =
{
     .kp=0.0f,.ki=0.0f,.kd=0.0f
};
error_para_t spdLerror=
{
      .errorCurr=0.0f, .errorLast=0.0f, .errorPrev=0.0f
};
error_para_t spdRerror=
{
      .errorCurr=0.0f, .errorLast=0.0f, .errorPrev=0.0f
};

float transform[4];
void motorCTRL (void*)
{
    ctrl_spdL = ((float)SCFTM_GetSpeed(FTM1)) * SPD_COEFF;
    SCFTM_ClearSpeed(FTM1);
    ctrl_spdR = -((float)SCFTM_GetSpeed(FTM2)) * SPD_COEFF;
    SCFTM_ClearSpeed(FTM2);

    transform[0]=ctrl_spdL;
    transform[1]=ctrl_spdR;
    transform[2]=motorLSet;
    transform[3]=motorRSet;

    speedR[2]=speedL[2];///<速度最值设定
    speedR[1]=speedL[1];

    if(1 == spdenable[0])
    {
        speedL[0]+=UpdatePIDandCacul(spdPID,&spdLerror,motorLSet-ctrl_spdL);
        speedR[0]+=UpdatePIDandCacul(spdPID,&spdRerror,motorRSet-ctrl_spdR);
        speedL[0]=(speedL[0]>speedL[2])?speedL[2]:speedL[0];
        speedR[0]=(speedR[0]>speedR[2])?speedR[2]:speedR[0];
        speedL[0]=(speedL[0]<speedL[1])?speedL[1]:speedL[0];
        speedR[0]=(speedR[0]<speedR[1])?speedR[1]:speedR[0];
    }
    else
    {
        speedL[0] = 0.0f;
        speedR[0] = 0.0f;
    }
    motorSetSpeed(speedL[0],speedR[0]);
}


pidCtrl_t dirPID_PIC =
{ .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f, };
pidCtrl_t dirPID_EMA =
{ .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f, };
float servo_ctrlOutput =7.5f;

uint32_t PicSwitch[3]={0,0,1};
uint32_t EmaSwitch[3]={0,0,1};
void directionCTRL(void*)
{
    if(PicSwitch[0]==1&&EmaSwitch[0]==0)//图像开
    {
           servo_ctrlOutput =7.5f - PIDCTRL_UpdateAndCalcPID(&dirPID_PIC, (float)(mid_line[front]-94));
         if(255==mid_line[front])
            {
             servo_ctrlOutput=7.5f;
            }
           else if(servo_ctrlOutput>8.5f)
           {  servo_ctrlOutput = 8.5f;}
           else if(servo_ctrlOutput<6.6f)
           { servo_ctrlOutput = 6.6f;}
    }
    else if(PicSwitch[0]==0&&EmaSwitch[0]==1)//电磁开
    {
        servo_ctrlOutput =7.5f - PIDCTRL_UpdateAndCalcPID(&dirPID_EMA,DRP(adc[1],adc[0]));
        if(servo_ctrlOutput>8.4f)
            servo_ctrlOutput = 8.4f;
        else if(servo_ctrlOutput<6.7f)
            servo_ctrlOutput = 6.7f;
    }
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_ctrlOutput);
}


void motorSetSpeed(float speedL, float speedR)
{
//    if(speedL>100.0f)
//    {
//        speedR-=speedL-100.0f;
//        speedL =100.0f;
//    }
//    if(speedL<-100.f)
//    {
//        speedR-=speedL+100.0f;
//        speedL = -100.0f;
//    }
//
//    if(speedR>100.0f)
//    {
//        speedL-=speedR-100.0f;
//        speedR =100.0f;
//    }
//    if(speedR<-100.f)
//    {
//        speedL-=speedR+100.0f;
//        speedR = -100.0f;
//    }
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

void StartPicture(menu_keyOp_t*  op)
{
    PicSwitch[0]=1;
    SDK_DelayAtLeastUs(2000000,CLOCK_GetFreq(kCLOCK_CoreSysClk));
    spdenable[0]=1;
}

float UpdatePIDandCacul(PID_para_t PID,error_para_t* Err,float error)
{
Err->errorPrev=Err->errorLast;
Err->errorLast=Err->errorCurr;
Err->errorCurr=error;
float increase;
    increase= PID.kp*(Err->errorCurr-Err->errorLast)+
            PID.ki*Err->errorCurr+
            PID.kd*(Err->errorCurr-2*Err->errorLast+Err->errorPrev);
   return increase;
}

void AC(menu_keyOp_t*  op)
{
    SCFTM_ClearSpeed(FTM1);
    SCFTM_ClearSpeed(FTM2);
}
