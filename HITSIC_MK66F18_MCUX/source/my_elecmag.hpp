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
void electronMenuSetup(menu_list_t* root);

void GetEMASignalHandLer(void*);
void bubbleSort(uint32_t *arr, uint32_t n);
void swap(uint32_t* a,uint32_t* b);
void FilterHandler(void*);
void StartEma(menu_keyOp_t*  op);


#endif /* MY_ELECMAG_HPP_ */
