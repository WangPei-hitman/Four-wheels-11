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
        xy_sum += y[x] * x;
        x_sum += x;
        y_sum += y[x];
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

////////////////////////////////////////////
//功能：Otsu
//输入：求阈值
//输出：
//备注：
///////////////////////////////////////////
int myOtsu(const uint8_t* frame) //大津法求阈值
{
    int width = 188;
    int height = 120;
    int pixelCount[GrayScale] = { };
    float pixelPro[GrayScale] = { };
    int i, j,k, threshol = 0;
    int  pixelSum = width * height;

    //uchar* data = (uchar*)frame->imageData;

    //统计每个灰度级中像素的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)*(frame + i * width + j)]++;
            //pixelCount[(int)data[i * width + j]]++;
        }
    }

    //计算每个灰度级的像素数目占整幅图像的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    //遍历灰度级[0,255],寻找合适的threshold
    float w0, w1, u0tmp, u1tmp, u0, u1, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++)
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        deltaTmp = (float)(w0 * w1 * pow((u0 - u1), 2));
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshol = i;
        }
    }
    return threshol;
}

