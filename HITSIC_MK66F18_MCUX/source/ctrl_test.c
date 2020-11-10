/*
 * ctrl_test.c
 *
 *  Created on: 2020年11月10日
 *      Author: WangP
 */
#include"ctrl_test.h"


void ctrlMenuSetup(menu_list_t *menu)
{
    static menu_list_t *TestList = MENU_ListConstruct("para_control", 20, menu);
    assert(TestList);
    MENU_ListInsert(menu, MENU_ItemConstruct(menuType, scTestList, "para_control", 0, 0));
    {
        MENU_ListInsert(menu, MENU_ItemConstruct(varfType, scTestList, "para_control", 0, 0));
    }
}



