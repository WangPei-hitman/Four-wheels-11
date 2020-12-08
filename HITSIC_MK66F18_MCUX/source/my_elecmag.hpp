/*
 * my_elecmag.hpp
 *
 *  Created on: 2020年11月18日
 *      Author: WangP
 */

#ifndef MY_ELECMAG_HPP_
#define MY_ELECMAG_HPP_

#include "fsl_common.h"

#include "inc_stdlib.hpp"

#include "app_menu.hpp"

#include "sc_adc.h"
#include "my_control.hpp"

extern float adc[2];
/*
 * @brief:电磁控制菜单建立
 */
void electronMenuSetup(menu_list_t* root);
/*
 * @brief:通道读取中断服务函数（已废弃）
 */
void GetEMASignalHandLer(void*);
/*
 * @brief:冒泡排序（低效）
 */
void bubbleSort(uint32_t *arr, uint32_t n);
/*
 * @brief:交换
 */
void swap(uint32_t* a,uint32_t* b);
/*
 * @brief:滤波器中断服务函数
 */
void FilterHandler(void*);
/*
 * @brief:图像控制车启动菜单接口函数，可通过菜单proctype调用延时启动开跑
 */
void StartEma(menu_keyOp_t*  op);


#endif /* MY_ELECMAG_HPP_ */
