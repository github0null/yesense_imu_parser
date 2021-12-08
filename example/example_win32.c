/**
 * Yesense 数据解析 win32 例程
 * 
 *  本处使用串口：COM3，波特率：460800
 * 
 * @author github0null
 * @version 1.0
 * @see https://github.com/github0null/
*/

#include "ys_parser.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "ctype.h"
#include "time.h"
#include "windows.h"

//
// 串口配置
//
#define SERIAL_PORT_NAME "COM3"
#define SERIAL_BAUD_RATE 460800

//
// 传感器数据回调，在此处对接收到的传感器数据进行打印
//
static void ys_data_handler(ys_result_callback_params_t *params)
{
    printf("---\r\n");
    printf("tid: %d\r\n", params->tid);
    printf("\ttemp: %.6f\r\n", params->result->imu_temp);
    printf("\tacce: %.6f, %.6f, %.6f\r\n", params->result->accel[X], params->result->accel[Y], params->result->accel[Z]);
    printf("\tgyro: %.6f, %.6f, %.6f\r\n", params->result->angle[X], params->result->angle[Y], params->result->angle[Z]);
    printf("\teulr: %.6f, %.6f, %.6f\r\n", params->result->euler_angle[PITCH], params->result->euler_angle[ROLL], params->result->euler_angle[YAW]);
    printf("\tq0_4: %.6f, %.6f, %.6f, %.6f\r\n", params->result->quaternion[0], params->result->quaternion[1], params->result->quaternion[2], params->result->quaternion[3]);
}

//
// functions
//
#define error(fmt, ...) fprintf(stderr, fmt "\n", ##__VA_ARGS__)
bool start_parser(char *port, DWORD buad);

// 定义报文解析器
static ys_parser_t ys_parser;

//
// 主程序
//
int main(int argc, char *argv[])
{
    // 初始化 YS 报文解析器
    ys_parser_create_static(&ys_parser, ys_data_handler);

    // 开始从串口解析数据
    start_parser(SERIAL_PORT_NAME, SERIAL_BAUD_RATE);

    error("program exited !");
}

bool start_parser(char *port, DWORD baud)
{
    char port_name[PATH_MAX];
    sprintf(port_name, "\\\\.\\%s", port);

    HANDLE h_com;
    h_com = CreateFile(port_name,    //COM口
                       GENERIC_READ, //允许读和写
                       0,            //独占方式
                       NULL,
                       OPEN_EXISTING, //打开而不是创建
                       0,             //同步方式
                       NULL);

    if (h_com == INVALID_HANDLE_VALUE)
    {
        error("open port: \"%s\" failed !, last_error: 0x%08x", port, GetLastError());
        return FALSE;
    }

    SetupComm(h_com, 4096, 4096);

    COMMTIMEOUTS timeout;
    GetCommTimeouts(h_com, &timeout);
    //设定读超时
    timeout.ReadIntervalTimeout = 1000;
    timeout.ReadTotalTimeoutMultiplier = 500;
    timeout.ReadTotalTimeoutConstant = 5000;
    //设定写超时
    timeout.WriteTotalTimeoutMultiplier = 500;
    timeout.WriteTotalTimeoutConstant = 2000;
    SetCommTimeouts(h_com, &timeout);

    DCB port_inf;
    GetCommState(h_com, &port_inf);
    port_inf.BaudRate = baud;
    port_inf.ByteSize = 8;          //每个字节有8位
    port_inf.Parity = NOPARITY;     //无奇偶校验位
    port_inf.StopBits = ONESTOPBIT; //两个停止位
    SetCommState(h_com, &port_inf);

    PurgeComm(h_com, PURGE_TXCLEAR | PURGE_RXCLEAR);

    // read start
    uint8_t byte;
    size_t size;

    while (ReadFile(h_com, (LPVOID)&byte, 1, (LPDWORD)&size, NULL))
    {
        // 向 YS 报文解析器不断输入字节数据
        ys_parser_input(&ys_parser, byte);
    }

    return TRUE;
}
