#include "hz_tools.hpp"

//////////////////////////////////////////////
////功能：最小二乘
////输入：x,y_start
////输出：k
////备注：在start和end之间算k
/////////////////////////////////////////////
void fxyk(uint8_t* y, uint8_t x_Start, uint8_t x_End, float* k, float* b)
{
    int x;
    int xy_sum = 0;
    int x_sum = 0;
    int y_sum = 0;
    int x2_sum = 0;
    int n = x_End - x_Start;
    for (x = x_Start; x < x_End; x++)
    {
        xy_sum += (*(y+x)) * x;
        x_sum += x;
        y_sum += *(y+x);
        x2_sum += (x * x);
    }
    *k = (float)(n * xy_sum - x_sum * y_sum) / (float)(n * x2_sum - x_sum * x_sum);
    *b = (y_sum - *k * x_sum) / n;
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}

////////////////////////////////////////////
//功能：abs族
//输入：
//输出：
//备注：
///////////////////////////////////////////
float abs_float(float n)
{
    if (n < 0) return -1 * n;
    return n;
}

////////////////////////////////////////////
//功能：一看就懂的工具人函数
//输入：
//输出：
//备注：
///////////////////////////////////////////
int Max(int x, int y)
{
    if (x > y)return x;
    return y;
}

int Min(int x, int y)
{
    if (x < y)return x;
    return y;
}

float f_Max(float x, float y)
{
    if (x > y)return x;
    return y;
}

float f_Min(float x, float y)
{
    if (x < y)return x;
    return y;
}

////////////////////////////////////////////
//功能：Otsu
//输入：求阈值
//输出：
//备注：
///////////////////////////////////////////
uint8_t myOtsu(uint8_t* image)
{

    uint16_t width = 188;
    uint16_t height = 120;
    int pixelCount[GrayScale];
    int* p_pixelCount = pixelCount;
    float pixelPro[GrayScale];
    float* p_pixelPro = pixelPro;
    int i, j, pixelSum = width * height / 4;
    uint8_t thres = 0;
    uint8_t* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        *(p_pixelCount + i) = 0;
        *(p_pixelPro + i) = 0;
    }

    uint32_t gray_sum = 0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 2)
    {
        for (j = 0; j < width; j += 2)
        {
            (*(p_pixelCount + (int)(*(data + i * width + j))))++; //将当前的点的像素值作为计数数组的下标
            gray_sum += (int)(*(data + i * width + j));      //灰度值总和
        }
    }

    //计算每个像素值的点在整幅图像中的比例

    for (i = 0; i < GrayScale; i++)
    {
        *(p_pixelPro + i) = (float)(*(p_pixelCount + i)) / pixelSum;

    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;


    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {

        w0 += *(p_pixelPro + j);  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * (*(p_pixelPro + j));  //背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            thres = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }

    }
    if (thres > 200)
        thres = 200;
    else if (thres < 120)
        thres = 120;

    return thres;
}
//int myOtsu(const uint8_t* frame) //大津法求阈值
//{
//    int width = 188;
//    int height = 120;
//    static int pixelCount[GrayScale] = { };
//    static float pixelPro[GrayScale] = { };
//    int i, j,k, threshol = 0;
//    int  pixelSum = width * height;
//
//    //uchar* data = (uchar*)frame->imageData;
//
//    //统计每个灰度级中像素的个数
//    for (i = 0; i < height; i++)
//    {
//        for (j = 0; j < width; j++)
//        {
//            pixelCount[(int)*(frame + i * width + j)]++;
//            //pixelCount[(int)data[i * width + j]]++;
//        }
//    }
//
//    //计算每个灰度级的像素数目占整幅图像的比例
//    for (i = 0; i < GrayScale; i++)
//    {
//        pixelPro[i] = (float)pixelCount[i] / pixelSum;
//    }
//
//    //遍历灰度级[0,255],寻找合适的threshold
//    static float w0, w1, u0tmp, u1tmp, u0, u1, deltaTmp, deltaMax = 0;
//    for (i = 0; i < GrayScale; i++)
//    {
//        w0 = w1 = u0tmp = u1tmp = u0 = u1 = deltaTmp = 0;
//        for (j = 0; j < GrayScale; j++)
//        {
//            if (j <= i)   //背景部分
//            {
//                w0 += pixelPro[j];
//                u0tmp += j * pixelPro[j];
//            }
//            else   //前景部分
//            {
//                w1 += pixelPro[j];
//                u1tmp += j * pixelPro[j];
//            }
//        }
//        u0 = u0tmp / w0;
//        u1 = u1tmp / w1;
//        deltaTmp = (float)(w0 * w1 * (u0 - u1) * (u0 - u1));
//        if (deltaTmp > deltaMax)
//        {
//            deltaMax = deltaTmp;
//            threshol = i;
//        }
//    }
//    return threshol;
//}

