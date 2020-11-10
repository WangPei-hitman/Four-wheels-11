/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hitsic_common.h"

/** HITSIC_Module_DRV */
#include "drv_ftfx_flash.hpp"
#include "drv_disp_ssd1306.hpp"
#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "image.h"
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_APP */
#include "app_menu.hpp"
#include "app_svbmp.hpp"

/** FATFS */
#include "ff.h"
#include "sdmmc_config.h"
FATFS fatfs;                                   //逻辑驱动器的工作区

#include "sc_adc.h"
#include "sc_ftm.h"

/** HITSIC_Module_TEST */
#include "drv_cam_zf9v034_test.hpp"
#include "app_menu_test.hpp"
#include "drv_imu_invensense_test.hpp"
#include "sys_fatfs_test.hpp"
#include "sys_fatfs_diskioTest.hpp"

/** SCLIB_TEST */
#include "sc_test.hpp"


pitMgr_t* motorcontrol =nullptr;
pitMgr_t* servocontrol =nullptr;
pitMgr_t* directiontask=nullptr;

float speedL = 8.0f,speedR=8.0f,servo_ctrl=7.5f;
int myerror1 = 0,myerror2=0;
float kp=0,kd=0;
int front = 50;
int midint;


void motorCTRL (void);
void controlInit(void);
void servoCTRL (void);
void MENU_DataSetUp(void);
void directionCTRL(void);

cam_zf9v034_configPacket_t cameraCfg;
dmadvp_config_t dmadvpCfg;
dmadvp_handle_t dmadvpHandle;
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);

void main(void)
{
    front =50;
    myerror1 = 0;myerror2=0;kp=0.5;kd=0;

    /** 初始化阶段，关闭总中断 */
    HAL_EnterCritical();
    /** 初始化时钟 */
    RTECLK_HsRun_180MHz();
    /** 初始化引脚路由 */
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    /** 初始化外设 */
    RTEPIP_Basic();
    RTEPIP_Device();
    /** 初始化调试串口 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    /** 初始化CMBackTrace */
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");
    /** 初始化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化EasyFlash */
    //easyflash_init();
    /** 初始化PIT中断管理器 */
    pitMgr_t::init();
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    /** 初始化菜单 */
    MENU_Init();
    MENU_Data_NvmReadRegionConfig();
    MENU_Data_NvmRead(menu_currRegionNum);
    /** 菜单挂起 */
    MENU_Suspend();
    /** 初始化摄像头 */

    //TODO: 在这里初始化摄像头
    /** 初始化IMU */
    //TODO: 在这里初始化IMU（MPU6050）
    /** 菜单就绪 */
    MENU_Resume();
    /** 控制环初始化 */
    //TODO: 在这里初始化控制环
    controlInit();


    //MENU_Suspend();
    /** 初始化结束，开启总中断 */
    HAL_ExitCritical();

    float f = arm_sin_f32(0.6f);

    cam_zf9v034_configPacket_t cameraCfg;
    CAM_ZF9V034_GetDefaultConfig(&cameraCfg);                                   //设置摄像头配置
    CAM_ZF9V034_CfgWrite(&cameraCfg);                                   //写入配置
    dmadvp_config_t dmadvpCfg;
    CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);    //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
    DMADVP_Init(DMADVP0, &dmadvpCfg);
    dmadvp_handle_t dmadvpHandle;
    DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0, CAM_ZF9V034_UnitTestDmaCallback);
    uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
    uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];

    disp_ssd1306_frameBuffer_t *dispBuffer = new disp_ssd1306_frameBuffer_t;
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
    DMADVP_TransferStart(DMADVP0, &dmadvpHandle);

    while (true)
    {
        //TODO: 在这里添加车模保护代码

        while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));
        image_main();

        dispBuffer->Clear();
        const uint8_t imageTH = 160;
        for (int i = 0; i < cameraCfg.imageRow; i += 2)
        {
            int16_t imageRow = i >> 1;//除以2,为了加速;
            int16_t dispRow = (imageRow / 8) + 1, dispShift = (imageRow % 8);
            for (int j = 0; j < cameraCfg.imageCol; j += 2)
            {
                int16_t dispCol = j >> 1;
                if (fullBuffer[i * cameraCfg.imageCol + j] > imageTH && j != 94 && j!= mid_line[i])
                {
                    dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                }
            }
        }

      DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
      DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
    }
}

void MENU_DataSetUp(void)
{
    //TODO: 在这里添加子菜单和菜单项
    static menu_list_t *TestList = MENU_ListConstruct("para_control", 20, menu_menuRoot);
         assert(TestList);
          MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, TestList, "para_control", 0, 0));
           {
                MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedL, "speedL",10 ,menuItem_data_global));
                MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedR, "speedR",11 ,menuItem_data_global));
                MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(varfType,&servo_ctrl, "servo",12 ,menuItem_data_global));
                MENU_ListInsert(TestList, MENU_ItemConstruct(variType,&front, "front",13 ,menuItem_data_global));
            }

       static menu_list_t *pidList = MENU_ListConstruct("pidList", 20, menu_menuRoot);
              assert(pidList);
              MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,pidList , "PID_control", 0, 0));
              {
                  MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&kp, "kp",14 ,menuItem_data_global));
                  MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&kd, "kd",15 ,menuItem_data_global));
              }
              MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(variType, &myerror2, "error", 17,menuItem_data_ROFlag));
              MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(variType, &midint, "mid", 18,menuItem_data_ROFlag));

}

void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    //TODO: 补完本回调函数

    //TODO: 添加图像处理（转向控制也可以写在这里）
    dmadvp_handle_t *dmadvpHandle = (dmadvp_handle_t*) userData;

    DMADVP_EdmaCallbackService(dmadvpHandle, transferDone);
    //PRINTF("new full buffer: 0x%-8.8x = 0x%-8.8x\n", handle->fullBuffer.front(), handle->xferCfg.destAddr);
    if (kStatus_Success != DMADVP_TransferStart(dmadvpHandle->base, dmadvpHandle))
    {
        DMADVP_TransferStop(dmadvpHandle->base, dmadvpHandle);
        //PRINTF("transfer stop! insufficent buffer\n");
    }
}



void controlInit(void)
{
    motorcontrol =  pitMgr_t::insert(6U,2U,motorCTRL,pitMgr_t::enable);
    assert(motorcontrol);
    //servocontrol = pitMgr_t::insert(20U,2U,servoCTRL,pitMgr_t::enable);
    //assert(servocontrol);
    directiontask = pitMgr_t::insert(20U,3U,directionCTRL,pitMgr_t::enable);
    assert(directiontask);
}

void motorCTRL (void)
{
    SCFTM_PWM_Change(FTM0,kFTM_Chnl_0,20000U,0.0f);
    SCFTM_PWM_Change(FTM0,kFTM_Chnl_1,20000U,speedR);

    SCFTM_PWM_Change(FTM0,kFTM_Chnl_2,20000U,speedL);
    SCFTM_PWM_Change(FTM0,kFTM_Chnl_3,20000U,0.0f);
}

void servoCTRL (void)
{
     SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_ctrl);
}


void directionCTRL(void)
{
    midint =(int)(mid_line[front]);
    myerror2 = midint-94;
    servo_ctrl=7.5-0.01*(kp*myerror2*1.0+kd*(myerror2*1.0-myerror1*1.0));
    myerror1 =myerror2;
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,servo_ctrl);
}


