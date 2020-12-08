/*
 * my_elecmag.cpp
 *
 *  Created on: 2020年11月18日
 *      Author: WangP
 */

#include "my_elecmag.hpp"

#define MinLVGot 1

float adc[2] = {0,0};

void electronMenuSetup(menu_list_t* root)
{
    static menu_list_t *EMAList = MENU_ListConstruct("EMA", 20, root);
    assert(EMAList);
    MENU_ListInsert(root, MENU_ItemConstruct(menuType, EMAList, "EMA", 0, 0));
    {
        static menu_list_t *pidList = MENU_ListConstruct("pidList", 20, EMAList);
        assert(pidList);
        MENU_ListInsert(EMAList, MENU_ItemConstruct(menuType, pidList, "PID_control", 0, 0));
        {
            MENU_ListInsert(pidList, MENU_ItemConstruct(varfType, &dirPID_EMA.kp, "kp", 10, menuItem_data_region));
            MENU_ListInsert(pidList, MENU_ItemConstruct(varfType, &dirPID_EMA.kd, "kd", 11, menuItem_data_region));
            MENU_ListInsert(pidList, MENU_ItemConstruct(varfType, &dirPID_EMA.ki, "ki", 12, menuItem_data_region));
        }

        static menu_list_t *adcList = MENU_ListConstruct("adcList", 20, EMAList);
        assert(adcList);
        MENU_ListInsert(EMAList, MENU_ItemConstruct(menuType, adcList, "adc", 0, 0));
        {
            MENU_ListInsert(adcList, MENU_ItemConstruct(varfType, &adc[0], "adc16", 0U, menuItem_data_NoSave | menuItem_data_NoLoad));
            MENU_ListInsert(adcList, MENU_ItemConstruct(varfType, &adc[1], "adc12", 0U, menuItem_data_NoSave | menuItem_data_NoLoad));
        }
        MENU_ListInsert(EMAList,MENU_ItemConstruct(varfType, &dirPID_EMA.errCurr, "error_ema", 0U, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(EMAList, MENU_ItemConstruct(procType, StartEma, "StartEma", 0U, 0U));
    }
}

void GetEMASignalHandLer(void*)
{
    //adc[0] = SCADC_Sample(ADC0, 0, 10);
    //adc[1] = SCADC_Sample(ADC0, 0, 11);
    adc[1] = SCADC_Sample(ADC0, 0, 12);//right
    //adc[3] = SCADC_Sample(ADC0, 0, 13);
    adc[2] = SCADC_Sample(ADC0, 0, 16);//left
    //adc[5] = SCADC_Sample(ADC0, 0, 17);
    //adc[6] = SCADC_Sample(ADC0, 0, 18);
    //adc[7] = SCADC_Sample(ADC0, 0, 23);
}

void FilterHandler(void*)
{

    const uint32_t sampleTimes = 25;
    uint32_t LV_Temp[2][sampleTimes];
    for (uint8_t i = 0; i < sampleTimes; i++)
    {
        LV_Temp[0][i] = SCADC_Sample(ADC0, 0, 16);
        LV_Temp[1][i] = SCADC_Sample(ADC0, 0, 12);
        LV_Temp[0][i] = (LV_Temp[0][i] > 255) ? 255 : LV_Temp[0][i];
        LV_Temp[1][i] = (LV_Temp[1][i] > 255) ? 255 : LV_Temp[1][i];
        LV_Temp[0][i] = (LV_Temp[0][i] < 1) ? 1 : LV_Temp[0][i];
        LV_Temp[1][i] = (LV_Temp[1][i] < 1) ? 1 : LV_Temp[1][i];
    }

    //sort
    for (uint8_t k = 0; k < 2; k++)
    {
        bubbleSort(*(LV_Temp+k),sampleTimes);
    }
    float LV[2]={0,0};
    for(uint8_t k=0;k<2;k++)
    {

        for(uint8_t i=3;i<sampleTimes - 3;i++)
        {
            LV[k]+=(float)LV_Temp[k][i];
        }
        LV[k]=LV[k]/(sampleTimes-6);
        if (LV[k] < MinLVGot)
         {
                  LV[k] = MinLVGot;
         }
    }
    adc[0]=LV[0];
    adc[1]=LV[1];
}


void swap(uint32_t* a, uint32_t* b)
{
    uint32_t temp;
    temp = *a;
    *a = *b;
    *b = temp;
    return;
}
void bubbleSort(uint32_t *arr, uint32_t n)
{
    uint32_t i, j;

    for (i = 0; i < n - 1; i++)
    {
        for (j = 1; j < n; j++)
        {
            if (arr[j] < arr[j - 1])
            {
                swap(&arr[j], &arr[j - 1]);
            }
        }
    }
}

void StartEma(menu_keyOp_t*  op)
{
    EmaSwitch[0]=1;
    SDK_DelayAtLeastUs(1500000,CLOCK_GetFreq(kCLOCK_CoreSysClk));
    spdenable[0]=1;
}
