#ifndef GLOBALDEFINE_H
#define GLOBALDEFINE_H
//  一些标准库和必需库
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fstream> 


#define M_PIi 3.141592653
#define gSpeed (M_PIi/180.0)


#define vmax  (160*gSpeed)
#define amaxl  (80*gSpeed)
#define jmax  (40 *gSpeed)



// define ns and us per seond
#define NSEC_PER_SEC 1000000000
#define USEC_PER_SEC 1000000

#define DOFS 7
#define CYCLETIME 0.001


// 任何模式都要每个周期更新m_dynamics.q 关节位置 ！！！
// m_dynamics.setJointPositions(q); 
enum MOTION_TYPE
{
    JOINT_RML = 0x01,
    CARTESIAN_RML_6D = 0x03,
    CARTESIAN_RML_TRANSLATION = 0x04,
    CARTESIAN_RML_ROTATION = 0x05,
    TRAJ_OFFLINE = 0x06,
    CIRCLE = 0x07,
    PENDULUM = 0x11,
    TEST_LINE = 0x08,
    TEST_CIRCLE = 0x09
};

#define Gravity 9.8


#define ER15 3

// 笛卡尔控制增益与限幅（便于集中修改）
#ifndef CART_KP_POS
  #define CART_KP_POS   50.0    // 位置误差比例增益（m/s 每 m）
#endif
#ifndef CART_KP_ORI
  #define CART_KP_ORI   1.0    // 姿态误差比例增益（rad/s 每 rad）
#endif
#ifndef CART_V_LIN_MAX
  #define CART_V_LIN_MAX 1.0   // 线速度上限 m/s
#endif
#ifndef CART_V_ANG_MAX
  #define CART_V_ANG_MAX 2.0   // 角速度上限 rad/s
#endif



/**
 * MICRODEFINE-BASED LOG  
 * IERROR()
 * IWARNING()
 * IPRINT()
 * 仅用于非实时线程中！打印显著影响实时性抖动！
*/

// 定义程序中的输出类型
#define __ERROR__
#define __WARNING__
#define __PRINT__
#define __RESULT__

// 打印运行错误数据
#ifdef __ERROR__
    #define IERROR(format, ...)  printf("\033[7m\033[1m\033[31m[ERROR]\033[0m FILE: [ %s %d ] \n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
    #define IERROR(...)
#endif

// 打印关键程序警告信息
#ifdef __WARNING__
    #define IWARNING(format, ...)  printf("\033[7m\033[1m\033[33m[WARNING]\033[0m FILE: [ %s %d ] \n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
    #define IWARNING(...)
#endif
// \033[0m
// 打印程序运行过程输出
#ifdef __PRINT__
    #define IPRINT(format, ...)  printf("\033[7m\033[1m\033[37m[PRINT]\033[0m  FILE: [ %s %d ]\n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
    #define IPRINT(...)
#endif
// 打印一些程序结果
#ifdef __RESULT__
    #define IRESULT(format, ...)  printf("\033[7m\033[1m\033[32m[RESULT]\033[0m FILE: [ %s %d ]\n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
    #define IRESULT(...)
#endif

/*
字颜色
字颜色范围:30----37
30:黑色 
31:深红 
32:绿色
33:黄色 
34:蓝色 
35:紫色 
36:深绿 
37:白色 

字背景颜色
字背景颜色范围:40----47
40:黑色 
41:深红 
42:绿色 
43:黄色 
44:蓝色 
45:紫色 
46:深绿 
47:白色 

显示设置
显示设置：

\33[0m :关闭所有属性 
\33[1m :设置高亮度 
\33[4m :下划线 
\33[5m :闪烁 
\33[7m :反显 （字体和背景对换了颜色）
\33[8m :消隐 
\33[30m -- \33[37m :设置前景色 （字体颜色）
\33[40m -- \33[47m :设置背景色 （背景颜色）
\33[nA :光标上移n行 
\33[nB :光标下移n行 
\33[nC :光标右移n行 
\33[nD :光标左移n行 
\33[y;xH :设置光标位置 
\33[2J :清屏 
\33[K :清除从光标到行尾的内容 
\33[s :保存光标位置 
\33[u :恢复光标位置 
\33[?25l :隐藏光标 
\33[?25h :显示光标
*/

#endif