/*
 * image.cpp
 *
 *  Created on: 2020年11月9日
 *      Author: Skywalker
 */

#include "image.h"

uint8_t* fullBuffer=NULL;//究极指针，图像核心
int f[10 * CAMERA_H];//考察连通域联通性

//position of point，坐标xy
typedef struct position
{
    uint8_t x;
    uint8_t y;
}POS;

//十字四点，包含位置与存在判据
typedef struct corner
{
    //uint8_t situation;//4:四点具在；1:近点；2:远点；0:无
    POS pos;
    uint8_t exit;//1在0不在
}COR;

//每个白条子属性，左右边际及联通标记
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   connect_num;//连通标记
}BAR;

//每行的所有白条子，一行里白条数量及特征
typedef struct {
    uint8_t   num;//每行白条数量
    BAR   character[white_num_MAX];//该行各白条特征
}line_bar;

//属于赛道的每个白条子属性，一行里确定的赛道
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}true_line_bar;

//每行属于赛道的每个白条子，white_num：行数编号，connected:特征
typedef struct {
    uint8_t   white_num;//0--120
    true_line_bar   connected[white_num_MAX];
}ROAD;

line_bar all_white[CAMERA_H];//所有白条子
ROAD final_road[CAMERA_H];//赛道

uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组

uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_bar = 0;//所有白条子数
uint8_t road_top;//赛道最高处所在行数
//uint8_t threshold = 150;//阈值
//int front = 45;//图像专属伪前瞻
uint8_t length = 15;
uint8_t farlength = 20;
uint8_t head = 77;//车头前点
uint8_t head_left = 52;//车头左点
uint8_t head_right = 140;//车头右点
int threshold = 0;


//uint8_t Start_line = Max(front - length, 1);
//uint8_t End_line = Min(front + length, head);

COR cor[4] = {};//四点标识

GG gg;

//void IMG_MENUSETUP(menu_list_t* List)
//{
//    static menu_list_t *TestList = MENU_ListConstruct("para_control", 20, List);
//        assert(TestList);
//         MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, TestList, "para_control", 0, 0));
//          {
//               MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedL[0], "speedL",10 ,menuItem_data_global));
//               MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedR[0], "speedR",11 ,menuItem_data_global));
//               MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedL[2], "speedLmax",20 ,menuItem_data_global));
//               MENU_ListInsert(TestList, MENU_ItemConstruct(varfType,&speedR[2], "speedRmax",21 ,menuItem_data_global));
//
//               MENU_ListInsert(TestList, MENU_ItemConstruct(variType,&front, "front",13 ,menuItem_data_global));
//               MENU_ListInsert(TestList, MENU_ItemConstruct(variType,&thro, "threshold",19 ,menuItem_data_global));
//           }
//
//     static menu_list_t *pidList = MENU_ListConstruct("pidList", 20,List);
//             assert(pidList);
//             MENU_ListInsert(List, MENU_ItemConstruct(menuType,pidList , "PID_control", 0, 0));
//             {
//                 MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&kp, "kp",14 ,menuItem_data_global));
//                 MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&kd, "kd",15 ,menuItem_data_global));
//                 MENU_ListInsert(pidList, MENU_ItemConstruct(varfType,&kt, "kt",22 ,menuItem_data_global));
//             }
//             MENU_ListInsert(List, MENU_ItemConstruct(variType, &myerror2, "error", 17,menuItem_data_ROFlag));
//             MENU_ListInsert(List, MENU_ItemConstruct(variType, &midint, "mid", 18,menuItem_data_ROFlag));
//             MENU_ListInsert(List, MENU_ItemConstruct(varfType,&servo_ctrl, "servo",12 ,menuItem_data_global));
//}


////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{

    uint8_t t;
    uint8_t* map;
    uint8_t* my_map;
    
    map = fullBuffer;
    //threshold = 160;//myOtsu(map);

    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if (j >= head_left && j <= head_right)
            {
                t = threshold + 20;
            }
            else if (i < length)
            {
                t = threshold + 20;
            }
            else
            {
                t = threshold;
            }

            if ((*map) > t)
                (*my_map) = 1;
            else (*my_map) = 0;

            map++;
            my_map++;
        }
    }
}

////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    for (int i = 119; i >= head; i--)
    {
        my_map = &IMG[i][0];
        for (int j = head_left; j <= head_right; j++)
        {
            *(my_map+j) = white;
        }
    }
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_first(int node)
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_first(f[node]);//向上寻找自己的父节点
    return f[node];//递归
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void find_bar(void)
{
    uint8_t i, j;
    int t_num = 0;//当前行白条数
    all_bar = 0;//白条编号初始化
    uint8_t* map = NULL;
    for (i = NEAR_LINE; i >= FAR_LINE; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        t_num = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                t_num++;
                if (t_num >= white_num_MAX)
                    break;
                BAR* now_white = &all_white[i].character[t_num];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                now_white->connect_num = ++all_bar;//白条数加一，给这个白条编号
            }
        }
        all_white[i].num = t_num;
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_bar; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    line_bar* u_bar = NULL;
    line_bar* d_bar = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = all_white[i - 1].num;
        d_num = all_white[i].num;
        u_bar = &all_white[i - 1];
        d_bar = &all_white[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_bar->character[i_u].left;
            u_right = u_bar->character[i_u].right;
            d_left = d_bar->character[i_d].left;
            d_right = d_bar->character[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_first(u_bar->character[i_u].connect_num)] = find_first(d_bar->character[i_d].connect_num);//父节点连起来

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)
                i_u++;
            if (d_right < u_right)
                i_d++;
            if (d_right == u_right) 
            { 
                i_u++; 
                i_d++; 
            }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    road_top = NEAR_LINE;//赛道最近处所在行数，先初始化话为最低处
    int road_first = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    line_bar* twhite_range = NULL;
    ROAD* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= all_white[istart].num; i++)
        if (all_white[istart].character[i].left <= CAMERA_W / 2
            && all_white[istart].character[i].right >= CAMERA_W / 2 && (all_white[istart].character[i].right - all_white[istart].character[i].left) >= 90)
            road_first = find_first(all_white[istart].character[i].connect_num);

    if (road_first == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= all_white[istart].num; i++)
            if (all_white[istart].character[i].right - all_white[istart].character[i].left > widthmax)
            {
                widthmax = all_white[istart].character[i].right - all_white[istart].character[i].left;
                jselect = i;
            }
        road_first = find_first(all_white[istart].character[jselect].connect_num);
    }

    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &all_white[i];
        tmy_road = &final_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_first(twhite_range->character[j].connect_num) == road_first)
            {
                road_top = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->character[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->character[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->character[j].right - twhite_range->character[j].left;

            }
        }
    }
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  起始j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分最多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;//选定白条编号
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//如果没找到，输出255
    if (j_start > final_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= final_road[i_start - 1].white_num; j++)
    {
        dleft = final_road[i_start].connected[j_start].left;
        dright = final_road[i_start].connected[j_start].right;
        uleft = final_road[i_start - 1].connected[j].left;
        uright = final_road[i_start - 1].connected[j].right;
        if (//相连
            dleft < uright
            &&
            dright > uleft
            )
        {
            //计算重叠大小
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max 
                && uleft < 114 
                && uright > 74)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////

void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //寻找起始行最宽的白条子
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= final_road[i_start].white_num; j++)
    {
        if (final_road[i_start].connected[j].width > width_max)
        {
            width_max = final_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //记录连贯区域编号
    for (i = i_start; i > i_end; i--)
    {
        //如果相连编号大于该行白条数，非正常，从此之后都MISS
        if (j_continue[i] > final_road[i].white_num)
        {
            j_continue[i - 1] = MISS;
        }

        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }

    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);

    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] <= final_road[i].white_num)
        {
            left_line[i] = final_road[i].connected[j_continue[i]].left;
            right_line[i] = final_road[i].connected[j_continue[i]].right;
        }

        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

////////////////////////////////////////////
//功能：边界修理
//输入：
//输出：
//备注：zxecf
////////////////////////////////////////////
void boarder_fixer(void)
{
    switch (gg)
    {
    
    case CROSS_OUT:
        cross_out();
        break;
    case CROSS_PROCESSING:
        //cross_process();
        cross_in();
        break;
    case CROSS_IN:
        cross_in();
        break;
    default:
        break;
    }
    
}
//void boarder_fixer(void)
//{
//    int i;
//    uint8_t jud_wid = 2;//判断域
//    
//    float kl_1,kr_1, kl_2, kr_2, bl_1, br_1, bl_2, br_2;
//    COR cor[4];
//
//    for ( i = 0; i < 4; i++)
//    {
//        cor[i].pos.x = 0;
//        cor[i].pos.y = 0;
//        cor[i].exit = 0;
//    }
//    for (i = End_line; i > Start_line; i--)
//    {
//        if (left_line[i+1] > left_line[i] + jud_wid)
//        {
//            cor[0].pos.x = i + 1;
//            cor[0].pos.y = left_line[i + 1];
//            cor[0].exit = 1;
//            break;
//        }
//    }
//    for (i = End_line; i > Start_line; i--)
//    {
//        if (right_line[i] > right_line[i + 1] + jud_wid)
//        {
//            cor[1].pos.x = i + 1;
//            cor[1].pos.y = right_line[i + 1];
//            cor[1].exit = 1;
//            break;
//        }
//    }
//    for (i = Start_line; i < End_line; i++)
//    {
//        if (left_line[i] > left_line[i + 1] + jud_wid)
//        {
//            cor[2].pos.x = i;
//            cor[2].pos.y = left_line[i];
//            cor[2].exit = 1;
//            break;
//        }
//    }
//    for (i = Start_line; i < End_line; i++)
//    {
//        if (right_line[i + 1] > right_line[i] + jud_wid)
//        {
//            cor[3].pos.x = i;
//            cor[3].pos.y = right_line[i];
//            cor[3].exit = 1;
//            break;
//        }
//    }
//    if (cor[0].exit && cor[1].exit && cor[2].exit && cor[3].exit)
//    {
//        fxyk(left_line, cor[0].pos.x,Min(cor[0].pos.x+length,End_line), &kl_1, &bl_1);
//        fxyk(right_line, cor[1].pos.x, Min(cor[1].pos.x + length, End_line), &kr_1, &br_1);
//        fxyk(left_line, Max(cor[2].pos.x - length, Start_line), cor[2].pos.x, &kl_2, &bl_2);
//        fxyk(right_line, Max(cor[3].pos.x - length, Start_line), cor[3].pos.x, &kr_2, &br_2);
//        for (i = Min(cor[0].pos.x + length, End_line); i > Max(cor[2].pos.x - length, Start_line); i--)
//        {
//            left_line[i] = (int)(0.5 * (kl_1 * i + kl_2 * i + bl_1 + bl_2));
//        }
//        for (i = Min(cor[1].pos.x + length, End_line); i > Max(cor[3].pos.x - length, Start_line); i--)
//        {
//            right_line[i] = (int)(0.5 * (kr_1 * i + kr_2 * i + br_1 + br_2));
//        }
//    }
//    else if (cor[2].exit && cor[3].exit)
//    {
//        fxyk(left_line, Max(cor[2].pos.x - length, Start_line), cor[2].pos.x, &kl_2, &bl_2);
//        fxyk(right_line, Max(cor[3].pos.x - length, Start_line), cor[3].pos.x, &kr_2, &br_2);
//        for (i = Min(cor[2].pos.x, cor[3].pos.x); i < End_line; i++)
//        {
//            left_line[i] = (int)(kl_2 * i + bl_2);
//            right_line[i] = (int)(kr_2 * i + br_2);
//        }
//    }
//    else if (cor[0].exit && cor[1].exit)
//    {
//        fxyk(left_line, cor[0].pos.x, Min(cor[0].pos.x + length, End_line), &kl_1, &bl_1);
//        fxyk(right_line, cor[1].pos.x, Min(cor[1].pos.x + length, End_line), &kr_1, &br_1);
//        for (i = Max(cor[0].pos.x, cor[1].pos.x); i > Start_line; i--)
//        {
//            left_line[i] = (int)(kl_1 * i + bl_1);
//            right_line[i] = (int)(kr_1 * i + br_1);
//        }
//    }
//}



////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
    {
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }
        else
        {
            mid_line[i] = MISS;
        }
        /*IMG[i][right_line[i]] = purple;
        IMG[i][left_line[i]] = purple;*/
    }
        
}

////////////////////////////////////////////
//功能：十字处理进入
//输入：元素判断，连通域,进入点
//输出：修正边界
//备注：
///////////////////////////////////////////
void cross_in(void)
{
    uint8_t i, m;
    float kl, kr, bl, br;
    
    fxyk(left_line, cor[0].pos.x, Min(cor[0].pos.x + length, head), &kl, &bl);
    fxyk(right_line, cor[1].pos.x, Min(cor[1].pos.x + length, head), &kr, &br);
    for (i = 1; i <= Min(cor[0].pos.x + length, head); i++)
    {
        left_line[i] = (uint8_t)(kl * i + bl);
    }
    for (i = 1; i <= Min(cor[1].pos.x + length, head); i++)
    {
        right_line[i] = (uint8_t)(kr * i + br);
    }
}

////////////////////////////////////////////
//功能：十字处理离开
//输入：元素判断，连通域,进入点
//输出：修正边界
//备注：
///////////////////////////////////////////

void cross_out(void)
{
    uint8_t i;
    float kl, kr, bl, br;

    if (Max((int)cor[2].pos.x - (int)length, 1) + 2 < cor[2].pos.x)
    {
        fxyk(left_line, Max((int)cor[2].pos.x - (int)length, 1), cor[2].pos.x, &kl, &bl);
        fxyk(right_line, Max((int)cor[3].pos.x - (int)length, 1), cor[3].pos.x, &kr, &br);
       // printf("[%f]", kl);
        for (i = cor[2].pos.x; i < NEAR_LINE; i++)
        {
            left_line[i] = (uint8_t)(kl * (float)i + bl);
        }
        for (i = cor[3].pos.x; i < NEAR_LINE; i++)
        {
            right_line[i] = (uint8_t)(kr * (float)i + br);
        }
    }
}

////////////////////////////////////////////
//功能：十字处理经过
//输入：元素判断，连通域,进入点
//输出：修正边界
//备注：
///////////////////////////////////////////
void cross_process(void)
{
    uint8_t i;
    float kl = 0;
    float kr = 0;
    
    kl = (float)((int)cor[0].pos.y - (int)cor[2].pos.y) / (float)((int)cor[0].pos.x - (int)cor[2].pos.x);
    kr = (float)((int)cor[1].pos.y - (int)cor[3].pos.y) / (float)((int)cor[1].pos.x - (int)cor[3].pos.x);
    for (i = Min((int)cor[0].pos.x + length, head); i > Max((int)cor[2].pos.x - length, 1); i--)
    {
        left_line[i] = (int)(kl * (i - (int)cor[0].pos.x) + cor[0].pos.y);
    }
    for (i = Min(cor[1].pos.x + length, head); i > Max((int)cor[3].pos.x - length, 1); i--)
    {
        right_line[i] = (int)(kr * (i - (int)cor[1].pos.x) + cor[1].pos.y);
    }
}

////////////////////////////////////////////
//功能：判断元素种类
//输入：左右边界
//输出：标志量
//备注：特别鸣谢周禹轩
////////////////////////////////////////////
GG General_Judge(void)
{
    POS jud_points[3];//123求边沿
    for (uint8_t i = 0; i < 3; i++)
    {
        jud_points[i].x = 0;
        jud_points[i].y = 0;
    }
    
    for (uint8_t i = 0; i < 3; i++)
    {
        cor[i].pos.x = 0;
        cor[i].pos.y = 0;
        cor[i].exit = 0;
    }
    //标志位s
    uint8_t zebra_count = 0;
    uint8_t protect = 0;
    uint8_t d_i = 2;
    int left = 0;
    int right = 0;

    //两个判断斜率
    float k_up = 0;
    float k_down = 0;

    //斜率夹角
    float w = 0;

    //斜率差判断标志位
    float k_flage = 4;

    for (uint8_t i = head - 5; i >= FAR_LINE + farlength; i--)
    {
        jud_points[0].x = i;
        jud_points[0].y = left_line[i];
        jud_points[1].x = i + d_i;
        jud_points[1].y = left_line[i + d_i];
        jud_points[2].x = i + 2 * d_i;
        jud_points[2].y = left_line[i + 2 * d_i];
        

        k_up = ((float)jud_points[0].y - (float)jud_points[1].y) 
            / ((float)jud_points[0].x - (float)jud_points[1].x);
        k_down = ((float)jud_points[1].y - (float)jud_points[2].y)
            / ((float)jud_points[1].x - (float)jud_points[2].x);

        if (all_white[i].num > 6 && all_white[i].num < 11)
        {
            zebra_count++;
        }

        if (i > head - length && i <= head)
        {
            if (left_line[i] == MISS && right_line[i] == MISS)
            {
                protect++;
            }
        }

        if (protect > 5)
        {
            return OUT;
        }

        if (zebra_count > 2)
        {
            return  ZEBRA;
        }

        if (k_up - k_down > k_flage)
        {
            cor[0].pos.x = jud_points[1].x;
            cor[0].pos.y = jud_points[1].y;
            cor[0].exit = 1;
            left = -1;
            break;
        }

        if ( k_down - k_up > k_flage)
        {
            left = 1;
            break;
        }
    }

    for (uint8_t i = head - 5; i >= FAR_LINE + farlength; i--)
    {
        jud_points[0].x = i;
        jud_points[0].y = right_line[i];
        jud_points[1].x = i + d_i;
        jud_points[1].y = right_line[i + d_i];
        jud_points[2].x = i + 2 * d_i;
        jud_points[2].y = right_line[i + 2 * d_i];

        k_up = ((float)jud_points[0].y - (float)jud_points[1].y)
            / ((float)jud_points[0].x - (float)jud_points[1].x);
        k_down = ((float)jud_points[1].y - (float)jud_points[2].y)
            / ((float)jud_points[1].x - (float)jud_points[2].x);

        if (k_down - k_up > k_flage)
        {
            cor[1].pos.x = jud_points[1].x;
            cor[1].pos.y = jud_points[1].y;
            cor[1].exit = 1;
            right = 1;
            break;
        }

        if (k_up - k_down > k_flage)
        {
            right = -1;
            break;
        }
    }

    for (uint8_t i = FAR_LINE + farlength; i < head - 5; i++)
    {
        jud_points[0].x = i;
        jud_points[0].y = left_line[i];
        jud_points[1].x = i + d_i;
        jud_points[1].y = left_line[i + d_i];
        jud_points[2].x = i + 2 * d_i;
        jud_points[2].y = left_line[i + 2 * d_i];

        k_up = ((float)jud_points[0].y - (float)jud_points[1].y)
            / ((float)jud_points[0].x - (float)jud_points[1].x);
        k_down = ((float)jud_points[1].y - (float)jud_points[2].y)
            / ((float)jud_points[1].x - (float)jud_points[2].x);

        if (k_up - k_down > k_flage)
        {
            cor[2].pos.x = jud_points[1].x;
            cor[2].pos.y = jud_points[1].y;
            cor[2].exit = 1;
            break;
        }
    }

    for (uint8_t i = FAR_LINE + farlength; i < head - 5; i++)
    {
        jud_points[0].x = i;
        jud_points[0].y = right_line[i];
        jud_points[1].x = i + d_i;
        jud_points[1].y = right_line[i + d_i];
        jud_points[2].x = i + 2 * d_i;
        jud_points[2].y = right_line[i + 2 * d_i];

        k_up = ((float)jud_points[0].y - (float)jud_points[1].y)
            / ((float)jud_points[0].x - (float)jud_points[1].x);
        k_down = ((float)jud_points[1].y - (float)jud_points[2].y)
            / ((float)jud_points[1].x - (float)jud_points[2].x);

        if (k_down - k_up > k_flage)
        {
            cor[3].pos.x = jud_points[1].x;
            cor[3].pos.y = jud_points[1].y;
            cor[3].exit = 1;
            break;
        }
    }

    if (cor[3].exit && cor[2].exit && left == 1 && right == -1 && (!cor[0].exit || !cor[1].exit))//&& !cor[0].exit && !cor[1].exit);
    {
        return CROSS_OUT;
    }

    if ( right == 1 && left == -1 && cor[3].exit && cor[2].exit)
    {
        
        return CROSS_IN;
    }

    if (cor[0].exit && cor[1].exit)// && cor[2].exit && cor[3].exit) //&& right == 1 && left == -1)
    {
        return CROSS_PROCESSING;
    }
    
    if (right == 1 && left == 1)
    {
        return RIGHT_TURN;
    }
    if (right == -1 && left == -1)
    {
        return LEFT_TURN;
    }
    return STRAIGHT;
}


////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
GG image_main(void)
{
    THRE();
    head_clear();
    find_bar();
    find_all_connect();
    find_road();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
    ordinary_two_line();
    gg = General_Judge();
    //printf("<%d>", gg);
    boarder_fixer();
    get_mid_line();

//    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
//    {
//        if (mid_line[i] != MISS)
//        {
//            IMG[i][mid_line[i]] = green;
//            IMG[i][left_line[i]] = blue;
//            IMG[i][right_line[i]] = blue;
//        }
//    }
//    IMG[front][mid_line[front]] = purple;
//    IMG[front][mid_line[front] + 1] = purple;
//    IMG[front][mid_line[front] - 1] = purple;
//    for (int i = 0; i < 4; i++)
//    {
//        if(cor[i].exit)
//            IMG[cor[i].pos.x][cor[i].pos.y] = red;
//    }
    return gg;
}

