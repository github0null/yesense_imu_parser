/**
 * Yesense 产品报文解析器
 * 
 * @author github0null
 * @version 1.0
 * @see https://github.com/github0null/
*/

#ifndef H_YS_PARSER
#define H_YS_PARSER

#include "ys_def.h"

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////
//                 Internal Type
//////////////////////////////////////////////////////

#define YS_BUFFER_SIZE 264

typedef struct
{
    uint8_t buffer[YS_BUFFER_SIZE];
    uint16_t count;
} ys_buffer;

typedef enum
{
	ON_PARSE_HEADDER_1 = 0,
	ON_PARSE_HEADDER_2,
	ON_PARSE_TID_L,
	ON_PARSE_TID_H,
	ON_PARSE_LENGTH,
	ON_PARSE_MESSAGE,
	ON_PARSE_CK1,
	ON_PARSE_CK2
} ys_parser_action;

typedef struct
{
	uint16_t tid;
	int16_t len;
	uint8_t *msg;
	uint8_t crc[2];
} ys_frame;

//////////////////////////////////////////////////////
//                  Type Define
//////////////////////////////////////////////////////

typedef struct 
{
    uint16_t tid;
    ys_sensor_data_t *result;
    void *user_data;
    uint8_t field_li[64]; /* sensor data id list */
    uint8_t field_cnt;
} ys_result_callback_params_t;

typedef void(*ys_result_callback_t)(ys_result_callback_params_t *params);

typedef enum
{
    YS_STATUS_CHK_ERR = -1,
    YS_STATUS_UNKNOWN_ERR = -2,
    YS_STATUS_MSG_LEN_ERR = -3,
    YS_STATUS_RUNNING = 0,
    YS_STATUS_DONE = 1,
} ys_parser_status_t;

/* trace info */
typedef struct
{
    int16_t status;          /* current parser status */
    uint16_t err_frame_cnt;  /* crc error cnt */
    uint16_t done_frame_cnt; /* valid frame cnt */
} ys_trace_info_t;

typedef struct
{
    ys_buffer data_buf;
    ys_parser_action cur_action;
    ys_frame cur_frame;
    int16_t msg_remain_len;
    ys_result_callback_t callbk;  /* data ready callbk */
    ys_sensor_data_t sensor_data; /* sensor data */
    ys_trace_info_t trace_inf;    /* trace info */
    void *user_data;              /* user data */
} ys_parser_t;

//////////////////////////////////////////////////////
//                  YS Parser API
//////////////////////////////////////////////////////

/**
 * 创建一个 YS 协议解析器。
 * 
 * @param result_ready_callbk 回调函数，当成功解析完一帧数据包时，将调用该回调函数。
 * 
 * @return YS 解析器对象
*/
ys_parser_t *ys_parser_create(ys_result_callback_t result_ready_callbk);

/**
 * 初始化一个使用静态内存分配的 YS 协议解析器对象。
 * 
 * @param parser YS 解析器对象
 * 
 * @param result_ready_callbk 回调函数，当成功解析完一帧数据包时，将调用该回调函数。
 * 
 * @return YS 解析器对象
*/
ys_parser_t *ys_parser_create_static(ys_parser_t *parser, ys_result_callback_t result_ready_callbk);

/**
 * 释放一个 YS 协议解析器，回收其内存。
 * 
 * @param parser YS 解析器对象
*/
void ys_parser_free(ys_parser_t *parser);

/**
 * @brief set user data for parser
 * @param user_data a pointer for your data
 */
void ys_parser_set_user_data(ys_parser_t *parser, void *user_data);

/**
 * @brief get user data for parser
 * @return a pointer for your data
 */
void *ys_parser_get_user_data(ys_parser_t *parser);

/**
 * 向解析器输入一个字节的报文数据。
 * 
 * 通过不断调用该函数，向解析器输入数据触发解析，当成功解析完一帧数据包时，将调用之前设置的回调函数。
 * 
 * 注意：在提供的回调函数中不应该进行耗时操作，否则会阻塞该函数。
 * 
 * @param parser YS 解析器对象
 * 
 * @param byte 一个字节数据
 * 
 * @param return 解析器状态
*/
ys_parser_status_t ys_parser_input(ys_parser_t *parser, uint8_t byte);

/**
 * 使用解析器解析一个缓冲区内的所有内容。
 * 
 * 该函数会调用 @ref ys_parser_input 在整个缓冲区中搜索并解析报文 
 * 
 * @param parser YS 解析器对象
 * 
 * @param buffer 缓冲区
 * 
 * @param len 缓冲区大小
*/
void ys_parse_buf(ys_parser_t *parser, uint8_t *buffer, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif