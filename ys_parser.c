#include <stdint.h>
#include <stddef.h>
#include <ys_parser.h>
#include <string.h>
#include <stdbool.h>

#define YS_HEADER_1    0x59
#define YS_HEADER_2    0x53

#define INTEGER_LEN    4
#define INTEGER_64_LEN 8

/* ys data id */
#define IMU_TEMP_ID             (uint8_t)0x01
#define SECOND_IMU_TEMP_ID      (uint8_t)0x02
#define FREE_ACCEL_ID           (uint8_t)0x11
#define SPEED_INCREMENT_ID      (uint8_t)0x12
#define SECOND_ACCEL_ID         (uint8_t)0x18
#define SECOND_ANGLE_ID         (uint8_t)0x28
#define QUATERNION_INCREMENT_ID (uint8_t)0x42

#define ACCEL_ID                (uint8_t)0x10
#define ANGLE_ID                (uint8_t)0x20
#define MAGNETIC_ID             (uint8_t)0x30
#define RAW_MAGNETIC_ID         (uint8_t)0x31
#define EULER_ID                (uint8_t)0x40
#define QUATERNION_ID           (uint8_t)0x41
#define UTC_ID                  (uint8_t)0x50
#define SAMPLE_TIMESTAMP_ID     (uint8_t)0x51
#define DATA_READY_TIMESTAMP_ID (uint8_t)0x52
#define LOCATION_ID             (uint8_t)0x60
#define HIGH_PRECI_LOCATION_ID  (uint8_t)0x68
#define SPEED_ID                (uint8_t)0x70

/* ys data len */
#define IMU_TEMP_DATA_LEN             (uint8_t)2
#define SECOND_IMU_TEMP_DATA_LEN      (uint8_t)2
#define FREE_ACCEL_DATA_LEN           (uint8_t)12
#define SPEED_INCREMENT_DATA_LEN      (uint8_t)12
#define SECOND_ACCEL_DATA_LEN         (uint8_t)12
#define SECOND_ANGLE_DATA_LEN         (uint8_t)12
#define QUATERNION_INCREMENT_DATA_LEN (uint8_t)16

#define ACCEL_DATA_LEN                (uint8_t)12
#define ANGLE_DATA_LEN                (uint8_t)12
#define MAGNETIC_DATA_LEN             (uint8_t)12
#define MAGNETIC_RAW_DATA_LEN         (uint8_t)12
#define EULER_DATA_LEN                (uint8_t)12
#define QUATERNION_DATA_LEN           (uint8_t)16
#define UTC_DATA_LEN                  (uint8_t)11
#define SAMPLE_TIMESTAMP_DATA_LEN     (uint8_t)4
#define DATA_READY_TIMESTAMP_DATA_LEN (uint8_t)4
#define LOCATION_DATA_LEN             (uint8_t)12
#define HIGH_PRECI_LOCATION_DATA_LEN  (uint8_t)20
#define SPEED_DATA_LEN                (uint8_t)12

/* factor for sensor data */
#define NOT_MAG_DATA_FACTOR 0.000001f
#define MAG_RAW_DATA_FACTOR 0.001f
#define IMU_TEMP_FACTOR     0.01f

/* factor for gnss data */
#define LONG_LAT_DATA_FACTOR            0.0000001
#define HIGH_PRECI_LONG_LAT_DATA_FACTOR 0.0000000001
#define ALT_DATA_FACTOR                 0.001f
#define SPEED_DATA_FACTOR               0.001f

//-------------------------- type define  -----------------------------------

#pragma pack(1)
typedef struct
{
    uint8_t id;
    uint8_t len;
} ys_packet_info_t;
#pragma pack()

//-------------------------- internal func ----------------------------------

static int32_t get_signed_int(uint8_t *buf, uint16_t offset);

static int64_t get_signed_int64(uint8_t *buf, uint16_t offset);

static void int_to_float_arr(float *dst, uint16_t dst_size, uint8_t *src, float ratio);

static void ys_buffer_push(ys_parser_t *parser, uint8_t dat);

static void ys_calcu_checksum(uint8_t *crc_buf, uint8_t byte);

static void ys_buffer_reset(ys_parser_t *parser);

static int8_t ys_parse_frame(ys_parser_t *parser, ys_frame *frame);

static int32_t find_ys_header(uint8_t *buf, uint32_t len);

static bool parse_data_by_id(ys_sensor_data_t *sensor_data, uint8_t id, uint8_t len, uint8_t *data);

#define ys_action_go_next(_parser)    _parser->cur_action++

#define ys_action_reset(_parser)      _parser->cur_action = ON_PARSE_HEADDER_1

#define ys_record_error(_parser, err) _parser->trace_inf.err_frame_cnt++, _parser->trace_inf.status = err

#define ys_check_msg_len(len)         (((len) >= YS_PARSER_MIN_MSG_LEN) && ((len) < 200))

//---------------------------------------------------------------------------

ys_parser_t *ys_parser_create(ys_result_callback_t data_ready_callbk)
{
    ys_parser_t *parser = ys_malloc(sizeof(ys_parser_t));
    memset(parser, 0, sizeof(ys_parser_t));
    parser->callbk = data_ready_callbk;
    return parser;
}

ys_parser_t *ys_parser_create_static(ys_parser_t *parser, ys_result_callback_t data_ready_callbk)
{
    memset(parser, 0, sizeof(ys_parser_t));
    parser->callbk = data_ready_callbk;
    return parser;
}

void ys_parser_free(ys_parser_t *parser)
{
    ys_free(parser);
}

void ys_parser_set_user_data(ys_parser_t *parser, void *user_data)
{
    parser->user_data = user_data;
}

void *ys_parser_get_user_data(ys_parser_t *parser)
{
    return parser->user_data;
}

ys_parser_status_t ys_parser_input(ys_parser_t *parser, uint8_t byte)
{
    /* reset parser status */
    parser->trace_inf.status = YS_STATUS_RUNNING;

    // YS HEADER_1
    if (parser->cur_action == ON_PARSE_HEADDER_1)
    {
        if (byte != YS_HEADER_1) return (ys_parser_status_t)parser->trace_inf.status;
        parser->msg_remain_len   = 0;
        parser->cur_frame.crc[0] = 0;
        parser->cur_frame.crc[1] = 0;
        parser->cur_frame.tid    = 0;
        parser->cur_frame.len    = 0;
        ys_buffer_reset(parser);
        ys_buffer_push(parser, byte);
        ys_action_go_next(parser);
    }

    // YS HEADER_2
    else if (parser->cur_action == ON_PARSE_HEADDER_2)
    {
        if (byte != YS_HEADER_2)
        {
            ys_action_reset(parser);
            return (ys_parser_status_t)parser->trace_inf.status;
        }

        ys_buffer_push(parser, byte);
        ys_action_go_next(parser);
    }

    // TID LOW BYTE
    else if (parser->cur_action == ON_PARSE_TID_L)
    {
        parser->cur_frame.tid = byte;
        ys_buffer_push(parser, byte);
        ys_calcu_checksum(parser->cur_frame.crc, byte); // calcu crc
        ys_action_go_next(parser);
    }

    // TID HIGH BYTE
    else if (parser->cur_action == ON_PARSE_TID_H)
    {
        parser->cur_frame.tid |= ((uint16_t)byte << 8);
        ys_buffer_push(parser, byte);
        ys_calcu_checksum(parser->cur_frame.crc, byte); // calcu crc
        ys_action_go_next(parser);
    }

    // MESSAGE LENGTH
    else if (parser->cur_action == ON_PARSE_LENGTH)
    {
        if (ys_check_msg_len(byte)) // limit the min size we want parse
        {
            parser->msg_remain_len = byte;
            parser->cur_frame.len  = byte;
            parser->cur_frame.msg  = &parser->data_buf.buffer[parser->data_buf.count + 1];
            ys_buffer_push(parser, byte);
            ys_calcu_checksum(parser->cur_frame.crc, byte); // calcu crc
            ys_action_go_next(parser);
        }
        else // else, reset parser !
        {
            ys_action_reset(parser);
            ys_record_error(parser, YS_STATUS_MSG_LEN_ERR);
        }
    }

    // MESSAGE
    else if (parser->cur_action == ON_PARSE_MESSAGE)
    {
        parser->msg_remain_len--;
        ys_buffer_push(parser, byte);
        ys_calcu_checksum(parser->cur_frame.crc, byte); // calcu crc

        // msg end, go next action
        if (parser->msg_remain_len <= 0)
            ys_action_go_next(parser);
    }

    // CRC_1
    else if (parser->cur_action == ON_PARSE_CK1)
    {
        if (parser->cur_frame.crc[0] == byte) // check ck1
        {
            ys_action_go_next(parser);
        }
        else // ck1 error
        {
            ys_action_reset(parser);
            ys_record_error(parser, YS_STATUS_CHK_ERR);
        }
    }

    // CRC_2
    else if (parser->cur_action == ON_PARSE_CK2)
    {
        // if check done !, parse full frame
        if (parser->cur_frame.crc[1] == byte)
        {
            ys_parse_frame(parser, &parser->cur_frame);
            parser->trace_inf.done_frame_cnt++;        /* increment done frame cnt */
            parser->trace_inf.status = YS_STATUS_DONE; /* set done flags */
        }
        else
        {
            ys_record_error(parser, YS_STATUS_CHK_ERR);
        }

        // parse end, reset
        ys_action_reset(parser);
    }

    // error action type, reset
    else
    {
        ys_action_reset(parser);
        ys_record_error(parser, YS_STATUS_UNKNOWN_ERR);
    }

    return (ys_parser_status_t)parser->trace_inf.status;
}

void ys_parse_buf(ys_parser_t *parser, uint8_t *buffer, uint32_t buffer_size)
{
    uint16_t prev_index = 0;

    for (uint16_t index = 0; index < buffer_size;)
    {
        int32_t ys_pos = find_ys_header(&buffer[index], buffer_size - index);

        if (ys_pos == -1)
            break; /* not found ys header, exit */

        /* locate start pos */
        index += ys_pos;
        prev_index = index;

        int16_t status = YS_STATUS_RUNNING;

        for (; index < buffer_size && status == YS_STATUS_RUNNING; index++)
        {
            status = ys_parser_input(parser, buffer[index]);
        }

        if (status < YS_STATUS_RUNNING) /* if parse failed, rollback index ! */
        {
            index = prev_index + 2; /* '+2': skip ys header */
        }
    }
}

int8_t ys_parse_frame(ys_parser_t *parser, ys_frame *frame)
{
    int16_t msg_len;
    uint8_t *packet_ptr;

    ys_result_callback_params_t cb_params = {
        .tid       = frame->tid,
        .result    = &parser->sensor_data,
        .user_data = parser->user_data,
        .field_cnt = 0,
    };

    memset(&parser->sensor_data, 0, sizeof(ys_sensor_data_t));

    for (packet_ptr = frame->msg, msg_len = frame->len; msg_len > 0;)
    {
        // get packet info
        ys_packet_info_t *packet_info = (ys_packet_info_t *)packet_ptr;

        if (parse_data_by_id(
                &parser->sensor_data,
                packet_info->id,
                packet_info->len,
                packet_ptr + sizeof(ys_packet_info_t)))
        {
            cb_params.field_li[cb_params.field_cnt++] = packet_info->id; /* set available field id */
            msg_len -= (sizeof(ys_packet_info_t) + packet_info->len);
            packet_ptr += (sizeof(ys_packet_info_t) + packet_info->len);
        }
        else
        {
            msg_len--;
            packet_ptr++;
        }
    }

    /* invoke result callbk */
    ys_assert(parser->callbk != NULL);
    parser->callbk(&cb_params);

    return true;
}

bool parse_data_by_id(ys_sensor_data_t *sensor_data, uint8_t id, uint8_t len, uint8_t *data)
{
    switch (id)
    {
        case IMU_TEMP_ID:
        {
            if (IMU_TEMP_DATA_LEN == len)
            {
                volatile int16_t temp = *((int16_t *)data);
                sensor_data->imu_temp = (float)temp * IMU_TEMP_FACTOR;
                return true;
            }
        }
        break;

        case SPEED_INCREMENT_ID:
        {
            if (SPEED_INCREMENT_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->speed_inc, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

#ifdef YS_HAS_DUAL_IMU

        case SECOND_IMU_TEMP_ID:
        {
            if (SECOND_IMU_TEMP_DATA_LEN == len)
            {
                volatile int16_t temp        = *((int16_t *)data);
                sensor_data->second_imu_temp = ((float)temp) * IMU_TEMP_FACTOR;
                return true;
            }
        }
        break;

        case SECOND_ACCEL_ID:
        {
            if (SECOND_ACCEL_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->second_accel, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case SECOND_ANGLE_ID:
        {
            if (SECOND_ANGLE_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->second_angle, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

#endif

        case QUATERNION_INCREMENT_ID:
        {
            if (QUATERNION_INCREMENT_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->quaternion_inc, 4, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case ACCEL_ID:
        {
            if (ACCEL_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->accel, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case ANGLE_ID:
        {
            if (ANGLE_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->angle, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case MAGNETIC_ID:
        {
            if (MAGNETIC_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->mag, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case RAW_MAGNETIC_ID:
        {
            if (MAGNETIC_RAW_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->raw_mag, 3, data, MAG_RAW_DATA_FACTOR);
                return true;
            }
        }
        break;

        case EULER_ID:
        {
            if (EULER_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->euler_angle, 3, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case QUATERNION_ID:
        {
            if (QUATERNION_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->quaternion, 4, data, NOT_MAG_DATA_FACTOR);
                return true;
            }
        }
        break;

        case LOCATION_ID:
        {
            if (LOCATION_DATA_LEN == len)
            {
                sensor_data->location[LAT] = get_signed_int(data, 0) * LONG_LAT_DATA_FACTOR;
                sensor_data->location[LON] = get_signed_int(data, INTEGER_LEN) * LONG_LAT_DATA_FACTOR;
                sensor_data->location[ALT] = get_signed_int(data, INTEGER_LEN * 2) * ALT_DATA_FACTOR;
                return true;
            }
        }
        break;

        case HIGH_PRECI_LOCATION_ID:
        {
            if (HIGH_PRECI_LOCATION_DATA_LEN == len)
            {
                sensor_data->location[LAT] = get_signed_int64(data, 0) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
                sensor_data->location[LON] = get_signed_int64(data, INTEGER_64_LEN * 1) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
                sensor_data->location[ALT] = get_signed_int64(data, INTEGER_64_LEN * 2) * ALT_DATA_FACTOR;
                return true;
            }
        }
        break;

        case SPEED_ID:
        {
            if (SPEED_DATA_LEN == len)
            {
                int_to_float_arr(sensor_data->velocity, 3, data, SPEED_DATA_FACTOR);
                return true;
            }
        }
        break;

        case SAMPLE_TIMESTAMP_ID:
        {
            if (SAMPLE_TIMESTAMP_DATA_LEN == len)
            {
                sensor_data->sample_timestamp = *((uint32_t *)data);
                return true;
            }
        }
        break;

        case DATA_READY_TIMESTAMP_ID:
        {
            if (DATA_READY_TIMESTAMP_DATA_LEN == len)
            {
                sensor_data->data_ready_timestamp = *((uint32_t *)data);
                return true;
            }
        }
        break;

        default: // no such id
            return false;
    }

    return false;
}

//-------------------------- internal func ----------------------------------

static int32_t get_signed_int(uint8_t *buf, uint16_t offset)
{
    int32_t temp = 0;

    for (int8_t i = 3; i >= 0; i--)
    {
        temp <<= 8;
        temp |= buf[offset + i];
    }

    return temp;
}

static int64_t get_signed_int64(uint8_t *buf, uint16_t offset)
{
    int64_t temp = 0;

    for (int8_t i = 7; i >= 0; i--)
    {
        temp <<= 8;
        temp |= buf[offset + i];
    }

    return temp;
}

static void int_to_float_arr(float *dst, uint16_t dst_size, uint8_t *src, float ratio)
{
    for (uint16_t i = 0; i < dst_size; i++)
    {
        dst[i] = get_signed_int(src, i * INTEGER_LEN) * ratio;
    }
}

static void ys_buffer_push(ys_parser_t *parser, uint8_t dat)
{
    parser->data_buf.buffer[parser->data_buf.count++] = dat;
    ys_assert(parser->data_buf.count < YS_BUFFER_SIZE);
}

static void ys_calcu_checksum(uint8_t *crc, uint8_t byte)
{
    crc[0] += byte;
    crc[1] += crc[0];
}

static void ys_buffer_reset(ys_parser_t *parser)
{
    parser->data_buf.count = 0;
}

static int32_t find_ys_header(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        if (buf[i + 0] == 'Y' &&
            buf[i + 1] == 'S' &&
            ys_check_msg_len(buf[i + 4]))
        {
            return i;
        }
    }

    return -1;
}
