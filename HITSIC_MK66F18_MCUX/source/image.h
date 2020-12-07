//#Param warning(disable:4996)
#ifndef _IMAGE_H
#define _IMAGE_H
//#define _CRT_SECURE_NO_WARNINGS
//#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1


#include "hz_tools.hpp"

#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

//通用判断，重中之重
typedef enum  general_judgement
{
    OUT,
    STRAIGHT,
    LEFT_TURN,
    RIGHT_TURN,
    CROSS_IN,
    CROSS_OUT,
    CROSS_PROCESSING,
    ZEBRA,
}GG;

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];
extern int threshold;


void head_clear(void);
void THRE(void);
int find_first(int a);
void find_bar();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
GG image_main(void);
void get_mid_line(void);
void boarder_fixer(void);
void midline_fixer(void);
void cross_in(void);
void cross_out(void);
void cross_process(void);
GG General_Judge(void);
#endif //
