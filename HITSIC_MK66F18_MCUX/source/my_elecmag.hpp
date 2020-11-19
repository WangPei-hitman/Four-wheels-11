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


void electronMenuSetup(menu_list_t* root);

void GetEMASignalHandLer(void*);

#endif /* MY_ELECMAG_HPP_ */
