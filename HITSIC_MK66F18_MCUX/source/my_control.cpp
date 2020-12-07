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
#define DELAY 3000
#define SERVOMID 7.49f
#define SERVOLEFT 8.45f
#define SERVORIGHT 6.7f
#define DIFFSPEED(x)  (0.3131*(x)*(x)*(x) + 0.2317*(x)*(x) + 0.4825*(x) - 0.0041)


/*中断任务句柄*/
pitMgr_t* motorcontrol =nullptr;
pitMgr_t* directiontask=nullptr;
pitMgr_t* EMAcolloction=nullptr;

pitMgr_t* timerCounthandler=nullptr;



int dir_front = 50;//前瞻
int thro;//摄像头阈值

float transform[6];///< Wi-Fi 数据传输数组

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
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &kinner[0], "kinner", 17, menuItem_data_region|menuItem_dataExt_HasMinMax));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &kL, "kL", 18, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(varfType, &kR, "kR", 19, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(variType, &dir_front, "dir_front", 6, menuItem_data_region));
                MENU_ListInsert(pidcontrol_low, MENU_ItemConstruct(variType, &spd_front, "spd_front",20,menuItem_data_region));
            }
            MENU_ListInsert(Low, MENU_ItemConstruct(variType, &thro, "threshold", 11, menuItem_data_global));
            MENU_ListInsert(Low,
                    MENU_ItemConstruct(varfType, &dirPID_PIC.errCurr, "error_pic", 0U, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(Low, MENU_ItemConstruct(procType, StartPicture, "StartPicture", 0U, 0U));   //插入函数行
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

pidCtrl_t* SpeedDiff=nullptr;
/**控制环初始化*/
void controlInit(void)
{
    motorcontrol =  pitMgr_t::insert(CTRL_SPD_CTRL_MS,2U,motorCTRL,pitMgr_t::enable);//速度环5ms
    assert(motorcontrol);

    directiontask = pitMgr_t::insert(CTRL_DIR_CTRL_MS,4U,directionCTRL,pitMgr_t::enable);//转向环20ms
    assert(directiontask);

    //EMAcolloction = pitMgr_t::insert(CTRL_DIR_CTRL_MS,4U,FilterHandler,pitMgr_t::enable);
    //assert(EMAcolloction);

    timerCounthandler= pitMgr_t::insert(500U,51U,TimerCount,pitMgr_t::enable);
    assert(timerCounthandler);

     SpeedDiff= PIDCTRL_Construct(dirPID_PIC.kp,dirPID_PIC.ki,dirPID_PIC.kd);
}


float speedL[3]={0.0f,-100.0f,100.0f},speedR[3]={0.0f,-100.0f,100.0f};//速度设定

float ctrl_spdL = 0.0f, ctrl_spdR = 0.0f;

float motorLOutput=0.0f,motorROutput=0.0f;//电机输出

float motorLSet=0.0f,motorRSet=0.0f;//电机速度设定（差速用）

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
float kinner[3]={1.0f,0.0f,1.0f},kL=1.0f,kR=1.0f;///<内外轮差速

/*电机控制*/
///////////////////////////////////////////////////////////////////////////////////////
void motorCTRL (void*)
{
    ctrl_spdL = ((float)SCFTM_GetSpeed(FTM1)) * SPD_COEFF;//编码器获取速度
    SCFTM_ClearSpeed(FTM1);//清空编码器
    ctrl_spdR = -((float)SCFTM_GetSpeed(FTM2))* SPD_COEFF;
    SCFTM_ClearSpeed(FTM2);

    transform[0]=ctrl_spdL;
    transform[1]=ctrl_spdR;
    transform[2]=motorLSet;
    transform[3]=motorRSet;//WiFi传值

    speedR[2]=speedL[2];///<速度最值设定
    speedR[1]=speedL[1];

    if(1 == spdenable[0])
    {
        //  float err_servo = - PIDCTRL_UpdateAndCalcPID(SpeedDiff, (float)(mid_line[spd_front]-94));
        float err_servo =servo_ctrlOutput-SERVOMID;
        float spdFix = DIFFSPEED(err_servo)*motorLSet;

        transform[4]=(motorLSet-kinner[0]*spdFix);
        transform[5]=(motorRSet+(1-kinner[0])*spdFix);

        if(err_servo>0)//舵机左打，内轮为左侧
        {
            speedL[0]+=UpdatePIDandCacul(spdPID,&spdLerror,(motorLSet-kL*kinner[0]*spdFix)-ctrl_spdL);
            speedR[0]+=UpdatePIDandCacul(spdPID,&spdRerror,(motorRSet+kR*(1.0f-kinner[0]) * spdFix) - ctrl_spdR);
            transform[4]=(motorLSet-kL*kinner[0]*spdFix);
            transform[5]=(motorRSet+kR*(1-kinner[0])*spdFix);
            speedL[0]=(speedL[0]>speedL[2])?speedL[2]:speedL[0];
            speedL[0]=(speedL[0]<speedL[1])?speedL[1]:speedL[0];
            speedR[0]=(speedR[0]>speedR[2])?speedR[2]:speedR[0];
            speedR[0]=(speedR[0]<speedR[1])?speedR[1]:speedR[0];
        }
        else
        {
            speedL[0]+=UpdatePIDandCacul(spdPID,&spdLerror,(motorLSet-kL*(1.0f-kinner[0])*spdFix)-ctrl_spdL);
            speedR[0]+=UpdatePIDandCacul(spdPID,&spdRerror,(motorRSet+kR*kinner[0]*spdFix)-ctrl_spdR);
            transform[4]=(motorLSet-kL*(1.0f-kinner[0])*spdFix);
            transform[5]=(motorRSet+kR*kinner[0]*spdFix);
            speedL[0]=(speedL[0]>speedL[2])?speedL[2]:speedL[0];
            speedL[0]=(speedL[0]<speedL[1])?speedL[1]:speedL[0];
            speedR[0]=(speedR[0]>speedR[2])?speedR[2]:speedR[0];
            speedR[0]=(speedR[0]<speedR[1])?speedR[1]:speedR[0];
        }
    }
    else
    {
        speedL[0] = 0.0f;
        speedR[0] = 0.0f;
    }
    motorSetSpeed(speedL[0],speedR[0]);//电机输出
}


pidCtrl_t dirPID_PIC =
{ .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f, };
pidCtrl_t dirPID_EMA =
{ .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f, };
float servo_ctrlOutput =SERVOMID;///<舵机输出

uint32_t PicSwitch[3]={0,0,1};
uint32_t EmaSwitch[3]={0,0,1};

int spd_front=50;

void directionCTRL(void*)
{
    PIDCTRL_Setup(SpeedDiff,dirPID_PIC.kp,dirPID_PIC.ki,dirPID_PIC.kd);

    if(PicSwitch[0]==1&&EmaSwitch[0]==0)//图像开
    {

        servo_ctrlOutput =SERVOMID - PIDCTRL_UpdateAndCalcPID(&dirPID_PIC, (float)(mid_line[dir_front]-94));
         if(255==mid_line[dir_front])
            {
             servo_ctrlOutput=SERVOMID;
            }
           else if(servo_ctrlOutput>SERVOLEFT)
           {  servo_ctrlOutput = SERVOLEFT;}
           else if(servo_ctrlOutput<SERVORIGHT)
           { servo_ctrlOutput = SERVORIGHT;}//输出舵机打角
    }
    else if(PicSwitch[0]==0&&EmaSwitch[0]==1)//电磁开
    {
        servo_ctrlOutput =7.5f - PIDCTRL_UpdateAndCalcPID(&dirPID_EMA,DRP(adc[1],adc[0]));
        if(servo_ctrlOutput>SERVOLEFT)
            servo_ctrlOutput = SERVOLEFT;
        else if(servo_ctrlOutput<SERVORIGHT)
            servo_ctrlOutput = SERVORIGHT;
    }
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_ctrlOutput);
}


void motorSetSpeed(float spdL, float spdR)
{
    if(spdL>speedL[2])
    {
        spdR-=spdL-speedL[2];
        spdL =speedL[2];
    }
    if(spdL<speedL[1])
    {
        spdR-=spdL-speedL[1];
        spdL = speedL[1];
    }

    if(spdR>speedR[2])
    {
        spdL-=spdR-speedR[2];
        spdR =speedR[2];
    }
    if(spdR<speedR[1])
    {
        spdL-=spdR-speedR[1];
        spdR = speedR[1];
    }
    if (spdR > 0)
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_1, 20000U, spdR);
    }
    else
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_0, 20000U, -spdR);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_1, 20000U, 0.0f);
    }
    if (spdL > 0)
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_2, 20000U, spdL);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_Change(FTM0, kFTM_Chnl_3, 20000U, -spdL);
    }
}

void StartPicture(menu_keyOp_t*  op)
{
    PicSwitch[0]=1;
    int recorder = timerCount;
while(timerCount<recorder+4)continue;
spdenable[0]=1;
}

float UpdatePIDandCacul(PID_para_t PID,error_para_t* Err,float error)
{
Err->errorPrev=Err->errorLast;
Err->errorLast=Err->errorCurr;
Err->errorCurr=error;           //更新error
float increase;
    increase= PID.kp*(Err->errorCurr-Err->errorLast)+
            PID.ki*Err->errorCurr+
            PID.kd*(Err->errorCurr-2*Err->errorLast+Err->errorPrev);
   return increase;
}

void AC(menu_keyOp_t*  op)  //采集时用于清零打角
{
    SCFTM_ClearSpeed(FTM1);
    SCFTM_ClearSpeed(FTM2);
}

int timerCount=0;

void TimerCount(void*)
{
    timerCount++;
}
