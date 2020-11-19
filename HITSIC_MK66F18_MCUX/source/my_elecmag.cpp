/*
 * my_elecmag.cpp
 *
 *  Created on: 2020年11月18日
 *      Author: WangP
 */

#include "my_elecmag.hpp"



uint32_t adc[8] = {0,0,0,0,0,0,0,0};

void electronMenuSetup(menu_list_t* root)
{
    static menu_list_t *List1 = MENU_ListConstruct("EMA_parameter", 20, root);
                assert(List1);
                 MENU_ListInsert(root, MENU_ItemConstruct(menuType, List1, "EMA_parameter", 0, 0));
                  {
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[0], "adc0",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[1], "adc1",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[2], "adc2",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[3], "adc3",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[4], "adc4",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[5], "adc5",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[6], "adc6",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                       MENU_ListInsert(List1, MENU_ItemConstruct(variType,&adc[7], "adc7",0U, menuItem_data_NoSave| menuItem_data_NoLoad));
                   }
}

void GetEMASignalHandLer(void*)
{
    adc[0] = SCADC_Sample(ADC0, 0, 10);
    adc[1] = SCADC_Sample(ADC0, 0, 11);
    adc[2] = SCADC_Sample(ADC0, 0, 12);
    adc[3] = SCADC_Sample(ADC0, 0, 13);
    adc[4] = SCADC_Sample(ADC0, 0, 16);
    adc[5] = SCADC_Sample(ADC0, 0, 17);
    adc[6] = SCADC_Sample(ADC0, 0, 18);
    adc[7] = SCADC_Sample(ADC0, 0, 23);
}


