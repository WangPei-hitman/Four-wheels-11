/*
 * image.cpp
 *
 *  Created on: 2020年11月9日
 *      Author: Skywalker
 */

#include "image.h"

uint8_t* fullBuffer = NULL;//究极指针，图像核心
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
    uint8_t   white_num;//0--10
    true_line_bar   connected[white_num_MAX];
}ROAD;

line_bar all_white[CAMERA_H];//所有白条子
ROAD final_road[CAMERA_H];//赛道

uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组

uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_bar = 0;//所有白条子数
uint8_t road_top;//赛道最高处所在行数
uint8_t j_continue[CAMERA_H];//第一条连通路径
//uint8_t threshold = 150;//阈值
int front = 30;//图像专属伪前瞻
uint8_t length = 10;
uint8_t farlength = 10;
uint8_t head = 67;//车头前点
uint8_t head_left = 35;//车头左点
uint8_t head_right = 145;//车头右点
POS jud_points[2][3];//123求边沿

uint8_t Start_line ;
uint8_t End_line ;

int threshold = 160;


COR cor[4] = {};//四点标识
GG gg;//标志位

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

        //分段阈值
        for (int j = 0; j < 188; j++)
        {
            if (i < 50 || (j >= head_left && j <= head_right))//远端、赛道压亮纹
            {
                t = threshold + 20;
            }

            if (i > head || j<LEFT_SIDE + 30 || j>RIGHT_SIDE - 30)//近端加亮，两侧加亮
            {
                t = threshold - 20;
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
    int aaa = 0;
    int bbb = 0;
    uint8_t* my_map;
    for (int i = 119; i >= head; i--)
    {
        my_map = &IMG[i][0];
        aaa = static_cast<int>(0.45 * (120 - i));//c++强转，感谢学嘉
        bbb = static_cast<int>(0.45 * (120 - i));
        //梯形清车头
        for (int j = head_left + aaa; j <= head_right - bbb; j++)
        {
            *(my_map + j) = white;
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
    uint8_t width_max = 1;
    uint8_t width_new = 1;
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

            left = (uint8_t)Max((int)left, LEFT_SIDE + 40);
            right = (uint8_t)Min((int)right, RIGHT_SIDE - 40);

            width_new = (uint8_t)Max(((int)right - (int)left + 1), 1);//垃圾赛道修正，可用角度30左右

            ////小心翼翼的
            //if (road_top <= 1)
            //{
            //    if (width_new > width_max
            //        && uleft < 104
            //        && uright>84)
            //    {
            //        width_max = width_new;
            //        j_return = j;
            //    }
            //}

            /*else
            {*/
                if (width_new > width_max)
                    /*&& uleft < 94
                    && uright>94)*/
                {
                    width_max = width_new;
                    j_return = j;
                }
            //}          
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
            *(left_line + i) = final_road[i].connected[j_continue[i]].left;
            *(right_line + i) = final_road[i].connected[j_continue[i]].right;
        }

        else
        {
            *(left_line + i) = MISS;
            *(right_line + i) = MISS;
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
    case LEFT_TURN:
//        for (uint8_t i = 0; i < 120; i++)
//        {
//            *(right_line + i) = (*(right_line + i) + *(left_line + i)) / 2;
//        }
        break;
    case RIGHT_TURN:
//        for (uint8_t i = 0; i < 120; i++)
//        {
//            *(left_line + i) = (*(right_line + i) + *(left_line + i)) / 2;
//        }
        break;
    case CROSS_OUT:
        cross_out();
        break;
    case CROSS_IN:
        cross_in();
        break;
    default:
        break;
    }
    
}

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
        if (*(left_line+i) != MISS)
        {
            *(mid_line + i) = (*(left_line + i) + *(right_line + i)) / 2;
        }
        else
        {
            *(mid_line + i) = MISS;
        }
        /*IMG[i][right_line[i]] = purple;
        IMG[i][left_line[i]] = purple;*/
    }
        
}

////////////////////////////////////////////
//功能：十字处理进入
//输入：元素判断，连通域,进入点
//输出：修正边界
//备注：xxzxec
///////////////////////////////////////////
void cross_in(void)
{
    uint8_t i;
    float kl, kr, bl, br;
    
    fxyk(left_line, cor[0].pos.x, Min(cor[0].pos.x + length, NEAR_LINE), &kl, &bl);
    fxyk(right_line, cor[1].pos.x, Min(cor[1].pos.x + length, NEAR_LINE), &kr, &br);
    if (kl > 1)kl = 1;
    if (kr > 1)kr = 1;
    if (kl < -1)kl = -1;
    if (kr < -1)kr = -1;
    for (i = 1; i <= Min(cor[0].pos.x, NEAR_LINE); i++)
    {
        *(left_line + i) = (uint8_t)(kl * i + bl);
    }
    for (i = 1; i <= Min(cor[1].pos.x, NEAR_LINE); i++)
    {
        *(right_line + i) = (uint8_t)(kr * i + br);
    }
}

////////////////////////////////////////////
//功能：十字处理离开
//输入：元素判断，连通域,进入点
//输出：修正边界
//备注：sxzzec
///////////////////////////////////////////

void cross_out(void)
{
    uint8_t i;
    float kl, kr, bl, br;

    if (Max((int)cor[2].pos.x - (int)length, 1) + 2 < cor[2].pos.x)
    {
        fxyk(left_line, Max((int)cor[2].pos.x - (int)length, Start_line), cor[2].pos.x, &kl, &bl);
        fxyk(right_line, Max((int)cor[3].pos.x - (int)length, Start_line), cor[3].pos.x, &kr, &br);
        if (kl > 1)kl = 1;
        if (kr > 1)kr = 1;
        if (kl < -1)kl = -1;
        if (kr < -1)kr = -1;
       // printf("[%f]", kl);
        for (i = cor[2].pos.x; i < NEAR_LINE; i++)
        {
            *(left_line + i) = (uint8_t)(kl * (float)i + bl);
        }
        for (i = cor[3].pos.x; i < NEAR_LINE; i++)
        {
            *(right_line + i) = (uint8_t)(kr * (float)i + br);
        }
    }
}

////////////////////////////////////////////
//功能：十字处理经过
//输入：元素判断，连通域,进入点
//输出：修正边界
//备注：
///////////////////////////////////////////
//void cross_process(void)
//{
//    uint8_t i;
//    float kl = 0;
//    float kr = 0;
//    
//    kl = (float)((int)cor[0].pos.y - (int)cor[2].pos.y) / (float)((int)cor[0].pos.x - (int)cor[2].pos.x);
//    kr = (float)((int)cor[1].pos.y - (int)cor[3].pos.y) / (float)((int)cor[1].pos.x - (int)cor[3].pos.x);
//    for (i = Min((int)cor[0].pos.x + length, head); i > Max((int)cor[2].pos.x - length, 1); i--)
//    {
//        left_line[i] = (int)(kl * (i - (int)cor[0].pos.x) + cor[0].pos.y);
//    }
//    for (i = Min(cor[1].pos.x + length, head); i > Max((int)cor[3].pos.x - length, 1); i--)
//    {
//        right_line[i] = (int)(kr * (i - (int)cor[1].pos.x) + cor[1].pos.y);
//    }
//}

////////////////////////////////////////////
//功能：判断元素种类
//输入：左右边界
//输出：标志量
//备注：特别鸣谢周禹轩
////////////////////////////////////////////
GG General_Judge(void)
{
    //标志位s
    uint8_t zebra_count = 0;
    uint8_t protect = 0;
    int cross_flager = 0;
    int left = 0;
    int right = 0;
    uint8_t d_i = 2;

    //四个判断斜率
    float kl_up = 0;
    float kl_down = 0;
    float kr_up = 0;
    float kr_down = 0;

    //斜率夹角
    //float w = 0;

    //斜率差判断标志位
    float k_flage = 5;
    
    //判断点初始化，较稳
    for (uint8_t i = 0; i < 3; i++)
    {
        jud_points[0][i].x = 0;
        jud_points[0][i].y = 0;
        jud_points[1][i].x = 0;
        jud_points[1][i].y = 0;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        cor[i].pos.x = 0;
        cor[i].pos.y = 0;
        cor[i].exit = 0;
    }

    //全域判断循环
    for (uint8_t i = head; i > farlength; i--)
    {
        if (final_road[i].white_num > 6 && i<End_line && i>Start_line)
        {
            *(left_line + i) = final_road[i].connected[1].left;
            *(right_line + i) = final_road[i].connected[final_road[i].white_num].right;
            zebra_count++;
        }

        if (zebra_count > 3)
        {
            for(int j = -length; j < length; j++)
            {
                *(left_line + i + j) = final_road[i + j].connected[1].left;
                *(right_line + i + j) = final_road[i + j].connected[final_road[i + j].white_num].right;
            }
            return  ZEBRA;
        }

        if (i > NEAR_LINE - length && i <= NEAR_LINE)
        {
            if (*(left_line + i) == MISS && *(right_line + i) == MISS)
            {
                protect++;
            }
        }

        if (protect > 5)
        {
            return OUT;
        }

        if (road_top > farlength && i > Start_line && i < End_line && i>road_top)
        {
            if (final_road[i].connected[j_continue[i]].left < LEFT_SIDE + 4
                && final_road[i].connected[j_continue[i]].right < head_right)
            {
                return LEFT_TURN;
            }

            if (final_road[i].connected[j_continue[i]].left > head_left
                && final_road[i].connected[j_continue[i]].right > RIGHT_SIDE - 4)
            {
                return RIGHT_TURN;
            }

        }
        
        if (road_top < Start_line
            && final_road[i].connected[j_continue[i]].left < LEFT_SIDE + 20
            && final_road[i].connected[j_continue[i]].right > RIGHT_SIDE - 20
            && i>Start_line && i < End_line)
        {
            cross_flager = i;
            break;
        }    
    }
    
    if (cross_flager != 0)
    {
        for (uint8_t i = End_line; i >= cross_flager; i--)
        {
            //左判断点
            jud_points[0][0].x = i;
            jud_points[0][0].y = left_line[i];
            jud_points[0][1].x = i + d_i;
            jud_points[0][1].y = left_line[i + d_i];
            jud_points[0][2].x = i + 2 * d_i;
            jud_points[0][2].y = left_line[i + 2 * d_i];

            //右判断点
            jud_points[1][0].x = i;
            jud_points[1][0].y = right_line[i];
            jud_points[1][1].x = i + d_i;
            jud_points[1][1].y = right_line[i + d_i];
            jud_points[1][2].x = i + 2 * d_i;
            jud_points[1][2].y = right_line[i + 2 * d_i];

            //左斜率
            kl_up = ((float)jud_points[0][0].y - (float)jud_points[0][1].y)
                / ((float)jud_points[0][0].x - (float)jud_points[0][1].x);
            kl_down = ((float)jud_points[0][1].y - (float)jud_points[0][2].y)
                / ((float)jud_points[0][1].x - (float)jud_points[0][2].x);

            //右斜率
            kr_up = ((float)jud_points[1][0].y - (float)jud_points[1][1].y)
                / ((float)jud_points[1][0].x - (float)jud_points[1][1].x);
            kr_down = ((float)jud_points[1][1].y - (float)jud_points[1][2].y)
                / ((float)jud_points[1][1].x - (float)jud_points[1][2].x);

            if (kl_up - kl_down > k_flage && kl_up > 1 && left != -1 && kl_down > -2 && kl_down < 2)
            {
                cor[0].pos.x = jud_points[0][1].x;
                cor[0].pos.y = jud_points[0][1].y;
                cor[0].exit = 1;
                left = -1;
            }

            if (kr_down - kr_up > k_flage && kr_up < -1 && right != 1 && kr_down > -2 && kr_down < 2)
            {
                cor[1].pos.x = jud_points[1][1].x;
                cor[1].pos.y = jud_points[1][1].y;
                cor[1].exit = 1;
                right = 1;
            }

            if (left == -1 && right == 1)
            {
                return CROSS_IN;
            }
        }
        
        for (uint8_t i = Start_line; i <= cross_flager; i++)
        {
            //左判断点
            jud_points[0][0].x = i;
            jud_points[0][0].y = left_line[i];
            jud_points[0][1].x = i + d_i;
            jud_points[0][1].y = left_line[i + d_i];
            jud_points[0][2].x = i + 2 * d_i;
            jud_points[0][2].y = left_line[i + 2 * d_i];

            //右判断点
            jud_points[1][0].x = i;
            jud_points[1][0].y = right_line[i];
            jud_points[1][1].x = i + d_i;
            jud_points[1][1].y = right_line[i + d_i];
            jud_points[1][2].x = i + 2 * d_i;
            jud_points[1][2].y = right_line[i + 2 * d_i];

            //左斜率
            kl_up = ((float)jud_points[0][0].y - (float)jud_points[0][1].y)
                / ((float)jud_points[0][0].x - (float)jud_points[0][1].x);
            kl_down = ((float)jud_points[0][1].y - (float)jud_points[0][2].y)
                / ((float)jud_points[0][1].x - (float)jud_points[0][2].x);

            //右斜率
            kr_up = ((float)jud_points[1][0].y - (float)jud_points[1][1].y)
                / ((float)jud_points[1][0].x - (float)jud_points[1][1].x);
            kr_down = ((float)jud_points[1][1].y - (float)jud_points[1][2].y)
                / ((float)jud_points[1][1].x - (float)jud_points[1][2].x);

            if (kl_up - kl_down > k_flage && left != -1 && kl_down < -1 && kl_up < 2 && kl_up>-2)
            {
                cor[2].pos.x = jud_points[0][1].x;
                cor[2].pos.y = jud_points[0][1].y;
                cor[2].exit = 1;
                left = -1;
            }

            if (kr_down - kr_up > k_flage && right != 1 && kr_down > 1 && kr_up > -2 && kr_up < 2)
            {
                cor[3].pos.x = jud_points[1][1].x;
                cor[3].pos.y = jud_points[1][1].y;
                cor[3].exit = 1;
                right = 1;
            }

            if (left == -1 && right == 1)
            {
                return CROSS_OUT;
            }
        }
    }

    return STRAIGHT;
}

////////////////////////////////////////////
//功能：中线修正
//输入：中线
//输出：中线
//备注：
////////////////////////////////////////////
void midline_fixer(void)
{
    uint8_t i;
    uint8_t pic[120] = {};
    uint8_t* p_pic;
    int ml;
    p_pic = pic;
    float k, b;
    switch (gg)
    {
    
    case RIGHT_TURN:
        for (i = End_line; i > Start_line; i--)
        {
            if (*(mid_line + i) > * (mid_line + i + 1) + 1 || *(mid_line + i) + 1 < *(mid_line + i + 1))
            {
                break;
            }
        }
        fxyk(mid_line, i, i + 10, &k, &b);
        if (k > 0)k = 0;
        if (k > 0)k = 0;
        if (k < -1)k = -1;
        if (k < -1)k = -1;
        for (; i > Start_line; i--)
        {
            ml = (int)(k * i + b);
            if (ml > 187) ml = 187;
            if (ml < 0) ml = 0;
            *(mid_line + i) = (uint8_t)ml;
        }
        break;
    case LEFT_TURN:
        for (i = End_line; i > Start_line; i--)
        {
            if (*(mid_line + i) > * (mid_line + i + 1) + 1 || *(mid_line + i) + 1 < *(mid_line + i + 1))
            {
                break;
            }
        }
        fxyk(mid_line, i, i + 10, &k, &b);
        if (k > 1)k = 1;
        if (k > 1)k = 1;
        if (k < 0)k = 0;
        if (k < 0)k = 0;
        for (; i > Start_line; i--)
        {
            ml = (int)(k * i + b);
            if (ml > 187) ml = 187;
            if (ml < 0) ml = 0;
            *(mid_line + i) = (uint8_t)ml;
        }
        break;
    case CROSS_IN:
    case CROSS_OUT:
    case ZEBRA:
    case STRAIGHT:
        
        break;   
    default:
        break;
    }
    for (i = NEAR_LINE; i > farlength; i--)
    {
        *(p_pic + i) =
            (*(mid_line + i - 2) + *(mid_line + i - 1) + *(mid_line + i) + *(mid_line + i + 1) + *(mid_line + i + 2)) / 5;
    }
    for (i = NEAR_LINE; i > farlength; i--)
    {
        *(mid_line + i) = *(p_pic + i);
    }
}


////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
GG image_main(void)
{
    Start_line = Max(front - 25, farlength);
    End_line = Min(front + 25, NEAR_LINE);
    front = dir_front;
    THRE();
    head_clear();
    find_bar();
    find_all_connect();
    find_road();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
    ordinary_two_line();
    gg = General_Judge();
    boarder_fixer();
    get_mid_line();
    midline_fixer();

//    printf("<%d>", gg);
//    for (int i = 119; i >= road_top; i--)
//    {
//        for (int j = final_road[i].connected[j_continue[i]].left;
//            j < final_road[i].connected[j_continue[i]].right; j++)
//        {
//            IMG[i][j] =blue;
//        }
//    }
//
//    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
//    {
//        if (mid_line[i] != MISS)
//        {
//            IMG[i][mid_line[i]] = gray;
//            IMG[i][left_line[i]] = purple;
//            IMG[i][right_line[i]] = purple;
//        }
//
//
//    }
//    for (int j = 0; j < 188; j++)
//    {
//        IMG[farlength][j] = red;
//    }
//    IMG[front][mid_line[front]] = red;
//    IMG[front][mid_line[front] + 1] = red;
//    IMG[front][mid_line[front] - 1] = red;
//    for (int i = 0; i < 2; i++)
//    {
//        if(cor[i].exit)
//            IMG[cor[i].pos.x][cor[i].pos.y] = green;
//    }
//    for (int i = 2; i < 4; i++)
//    {
//        if (cor[i].exit)
//            IMG[cor[i].pos.x][cor[i].pos.y] = green;
//    }
    return gg;
}

