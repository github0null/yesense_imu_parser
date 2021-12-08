/**
 * Yesense 驱动相关定义
 * 
 * @author github0null
 * @version 1.0
 * @see https://github.com/github0null/
*/

#ifndef _H_YS_DEF
#define _H_YS_DEF

//////////////////////////////////////////////////////
//                    相关配置
//////////////////////////////////////////////////////

/* 若要覆盖相关宏，请在 'ys_conf.h' 中进行定义 */
#if defined __has_include
#if __has_include("ys_conf.h")
#include "ys_conf.h"
#endif
#else
#include "ys_conf.h"
#endif

/* 内存分配器，默认使用 malloc */
#if !defined(ys_malloc) || !defined(ys_free)
#include <stdlib.h>
#define ys_malloc malloc
#define ys_free free
#endif

/* 断言，默认为空 */
#ifndef ys_assert
#define ys_assert(expr)
#endif

/* 静态内联关键字，默认为 static */
#ifndef ys_static_inline
#define ys_static_inline static
#endif

/* 报文中 message 字段最小长度，小于此值的报文将被忽略 */
#ifndef YS_PARSER_MIN_MSG_LEN
#define YS_PARSER_MIN_MSG_LEN 4
#endif

/* 解析第二颗 IMU 输出 */
#ifdef YS_DUAL_IMU_EN
#define YS_HAS_DUAL_IMU
#endif

#ifndef ys_critical_enter
#define ys_critical_enter()
#endif

#ifndef ys_critical_exit
#define ys_critical_exit()
#endif

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////
//                    数组索引
//////////////////////////////////////////////////////

typedef enum
{
    X = 0,
    Y,
    Z
} IMU_XYZ_AXIS;

/* 站心坐标系轴 */
typedef enum
{
    E = 0,
    N,
    U
} IMU_ENU_AXIS;

typedef enum
{
    PITCH = 0, /* 俯仰角 */
    ROLL,      /* 横滚角 */
    YAW        /* 航向角 */
} IMU_EULAR_ANGLE;

typedef enum
{
    LAT = 0, /* 纬度 */
    LON,     /* 经度 */
    ALT      /* 海拔 */
} IMU_LOCATION;

//////////////////////////////////////////////////////
//                 IMU 输出数据定义
//////////////////////////////////////////////////////

typedef struct
{
    /* 采样时间戳，单位：us */
    uint32_t sample_timestamp;

    /* 数据就绪时间戳，单位：us */
    uint32_t data_ready_timestamp;

    /* IMU 温度，单位：°C */
    float imu_temp;

    /* 加速度，单位：m/s^2 */
    float accel[3];

    /* 角速度，单位：deg/s */
    float angle[3];
    
    /* 磁场归一化值 */
    float mag[3];

    /* 磁场强度：单位：mGauss */
    float raw_mag[3];
    
    /**
     * 欧拉角，单位：° (deg)
     * 数组索引值见: IMU_EULAR_ANGLE 
     */
    float euler_angle[3];

    /* 四元数 */
    float quaternion[4];
    
    /* 方位增量 */
    float quaternion_inc[4];

    /**
     * 位置
     * 
     * 纬度，单位：deg
     * 经度，单位：deg
     * 海拔，单位：m
     * 
     * 数组索引值见: @ref IMU_LOCATION 
     */
    double location[3];

    /* 速度，单位：m/s，使用站心坐标系(E,N,U)，索引值见: IMU_ENU_AXIS */
    float velocity[3];
    
    /* 速度增量，单位：m/s */
    float speed_inc[3];

#ifdef YS_HAS_DUAL_IMU

    /* 第二颗 IMU 温度，单位：°C */
    float second_imu_temp;

    /* 第二颗 IMU 加速度，单位：m/s^2 */
    float second_accel[3];

    /* 第二颗 IMU 角速度，单位：deg/s */
    float second_angle[3];

#endif // YS_HAS_DUAL_IMU

} ys_sensor_data_t;

#ifdef __cplusplus
}
#endif

#endif
