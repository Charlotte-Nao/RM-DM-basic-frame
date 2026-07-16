/**
 * @file YB_SD15M.c
 * @brief YB-SD15M serial bus servo device implementation.
 */

#include "YB_SD15M.h"

#include <stddef.h>
#include <string.h>

#include "../../bsp/uart/uart.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/* Protocol configuration                                                     */
/* -------------------------------------------------------------------------- */

#define YB_SD15M_UART_NAME                    "uart7_dma"

#define YB_SD15M_FRAME_HEADER                 0xFFU

#define YB_SD15M_INST_READ                    0x02U
#define YB_SD15M_INST_WRITE                   0x03U
#define YB_SD15M_REG_TARGET_POSITION          0x2AU
#define YB_SD15M_REG_CURRENT_POSITION         0x38U
#define YB_SD15M_MOVE_PACKET_LENGTH           0x07U
#define YB_SD15M_READ_PACKET_LENGTH           0x04U
#define YB_SD15M_POSITION_REPLY_LENGTH        0x04U
#define YB_SD15M_MOVE_FRAME_SIZE              11U
#define YB_SD15M_READ_FRAME_SIZE              8U
#define YB_SD15M_MIN_FRAME_SIZE               6U
#define YB_SD15M_RX_FRAME_MAX_SIZE            32U
#define YB_SD15M_POSITION_READ_SIZE           0x02U
#define YB_SD15M_FEEDBACK_PERIOD_MS           100U
#define YB_SD15M_REPLY_TIMEOUT_MS             30U
#define YB_SD15M_OFFLINE_TIMEOUT_MS           500U

typedef struct
{
    volatile uint16_t target_position;
    volatile uint16_t move_time_ms;
    volatile uint32_t target_revision;
    uint32_t sent_revision;
    volatile uint16_t current_position;
    volatile uint8_t error;
    volatile uint8_t position_valid;

    uint8_t waiting_position_reply;
    uint8_t initialised;
    uint32_t position_request_tick;
    uint32_t last_feedback_query_tick;

    volatile uint32_t tx_frame_count;
    volatile uint32_t rx_frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t reply_timeout_count;
} yb_sd15m_data_t;

typedef struct
{
    uint8_t frame[YB_SD15M_RX_FRAME_MAX_SIZE];

    uint16_t count;
    uint16_t expected_size;
} yb_sd15m_parser_t;

/* -------------------------------------------------------------------------- */
/* Shared UART state                                                          */
/* -------------------------------------------------------------------------- */

static struct uart_device *yb_sd15m_uart7;
static yb_sd15m_parser_t yb_sd15m_parser;

/* -------------------------------------------------------------------------- */
/* YB-SD15M type-function declarations                                        */
/* -------------------------------------------------------------------------- */

static void yb_sd15m_init(
    struct yb_sd15m_device *servo,
    uint8_t servo_id,
    struct uart_device *uart
);

static void yb_sd15m_feedback_calculate(
    const struct yb_sd15m_device *servo,
    const uint8_t *frame,
    uint16_t frame_length
);

static int yb_sd15m_send_ctrl_cmd(
    struct yb_sd15m_device *servo
);

static int yb_sd15m_request_position(
    struct yb_sd15m_device *servo
);

static void yb_sd15m_update(
    struct yb_sd15m_device *servo
);

static int yb_sd15m_device_set_target(
    struct yb_sd15m_device *servo,
    uint16_t target_position,
    uint16_t move_time_ms
);

static int yb_sd15m_device_get_status(
    const struct yb_sd15m_device *servo,
    const char *which_status,
    void *status_data
);

static void yb_sd15m_uart_receive_callback(
    struct uart_device *uart,
    const uint8_t *data,
    uint16_t length
);

/* -------------------------------------------------------------------------- */
/* Static physical servo instances                                            */
/* -------------------------------------------------------------------------- */
static yb_sd15m_data_t yb_sd15m_1_data;
static struct yb_sd15m_device yb_sd15m_1 =
{
    .servo_name = "YB_SD15M_1",
    .servo_id = YB_SD15M_DEFAULT_ID,
    .servo_uart = NULL,
    .servo_data = &yb_sd15m_1_data,
    .last_rx_tick = 0U,
    .init = yb_sd15m_init,
    .feedback_calculate = yb_sd15m_feedback_calculate,
    .send_ctrl_cmd = yb_sd15m_send_ctrl_cmd,
    .request_position = yb_sd15m_request_position,
    .update = yb_sd15m_update,
    .set_target = yb_sd15m_device_set_target,
    .get_status = yb_sd15m_device_get_status,
};

static struct yb_sd15m_device *const yb_sd15m_list[] =
{
    &yb_sd15m_1,
};


/* -------------------------------------------------------------------------- */
/* Utility functions                                                          */
/* -------------------------------------------------------------------------- */
static uint8_t yb_sd15m_checksum(
    const uint8_t *frame,
    uint16_t frame_length
)
{
    uint8_t sum;
    uint16_t index;
    if (frame == NULL ||
        frame_length < YB_SD15M_MIN_FRAME_SIZE) {
        return 0U;
    }
    sum = 0U;
    for (index = 2U;
         index < (uint16_t)(frame_length - 1U);
         ++index) {
        sum = (uint8_t)(sum + frame[index]);
    }
    return (uint8_t)(~sum);
}

static struct yb_sd15m_device *yb_sd15m_find_by_id(
    uint8_t servo_id
)
{
    uint32_t index;

    for (index = 0U;
         index < YB_SD15M_Get_Count();
         ++index) {
        if (yb_sd15m_list[index]->servo_id == servo_id) {
            return yb_sd15m_list[index];
        }
    }

    return NULL;
}

static int yb_sd15m_send_frame(
    struct yb_sd15m_device *servo,
    const uint8_t *frame,
    uint16_t frame_length
)
{
    yb_sd15m_data_t *servo_data;
    int result;

    if (servo == NULL ||
        servo->servo_data == NULL ||
        servo->servo_uart == NULL ||
        servo->servo_uart->uart_send_bytes == NULL ||
        frame == NULL ||
        frame_length == 0U) {
        return YB_SD15M_ERR_NOT_READY;
    }

    servo_data = servo->servo_data;

    result = servo->servo_uart->uart_send_bytes(
        servo->servo_uart,
        frame,
        frame_length
    );

    if (result != 0) {
        return YB_SD15M_ERR_UART;
    }

    ++servo_data->tx_frame_count;

    return YB_SD15M_OK;
}
static void yb_sd15m_init(struct yb_sd15m_device *servo,uint8_t servo_id,struct uart_device *uart)
{
    yb_sd15m_data_t *servo_data;
    if (servo == NULL ||
        servo->servo_data == NULL ||
        uart == NULL) {
        return;
    }
    if (servo_id < 1U || servo_id > 250U) {
        return;
    }

    servo_data = servo->servo_data;

    memset(
        servo_data,
        0,
        sizeof(*servo_data)
    );
    servo->servo_id = servo_id;
    servo->servo_uart = uart;
    servo->last_rx_tick = 0U;
    servo_data->target_position =
        YB_SD15M_POSITION_CENTER;

    servo_data->move_time_ms = 500U;

    servo_data->last_feedback_query_tick =
        HAL_GetTick();

    servo_data->initialised = 1U;
}

static void yb_sd15m_feedback_calculate(const struct yb_sd15m_device *servo,const uint8_t *frame,uint16_t frame_length)
{
    yb_sd15m_data_t *servo_data;
    uint8_t packet_length;

    if (servo == NULL ||
        servo->servo_data == NULL ||
        frame == NULL ||
        frame_length < YB_SD15M_MIN_FRAME_SIZE) {
        return;
    }

    servo_data = servo->servo_data;

    packet_length = frame[3];

    /*
     * In a status response, byte 4 is the error/status byte.
     */
    servo_data->error = frame[4];

    ++servo_data->rx_frame_count;

    /*
     * Current-position response:
     *
     * FF FF ID 04 ERROR POS_H POS_L CHECKSUM
     */
    if (packet_length == YB_SD15M_POSITION_REPLY_LENGTH &&
        frame_length == 8U) {
        servo_data->waiting_position_reply = 0U;

        if (servo_data->error == 0U) {
            servo_data->current_position =
                (uint16_t)(
                    ((uint16_t)frame[5] << 8) |
                    (uint16_t)frame[6]
                );

            servo_data->position_valid = 1U;
        }
    } else if (
        servo_data->waiting_position_reply != 0U &&
        servo_data->error != 0U
    ) {
        /*
         * The servo returned an error status instead of position data.
         */
        servo_data->waiting_position_reply = 0U;
    }
}

/**
 * Send the currently stored position target.
 *
 * Frame:
 *
 * FF FF ID 07 03 2A POS_H POS_L TIME_H TIME_L CHECKSUM
 */
static int yb_sd15m_send_ctrl_cmd(
    struct yb_sd15m_device *servo
)
{
    yb_sd15m_data_t *servo_data;

    uint8_t frame[YB_SD15M_MOVE_FRAME_SIZE];

    uint16_t target_position;
    uint16_t move_time_ms;

    uint32_t target_revision;

    int result;

    if (servo == NULL ||
        servo->servo_data == NULL) {
        return YB_SD15M_ERR_PARAM;
    }

    servo_data = servo->servo_data;

    if (servo_data->initialised == 0U) {
        return YB_SD15M_ERR_NOT_READY;
    }
    target_revision =
        servo_data->target_revision;

    __DMB();

    target_position =
        servo_data->target_position;

    move_time_ms =
        servo_data->move_time_ms;

    frame[0] = YB_SD15M_FRAME_HEADER;
    frame[1] = YB_SD15M_FRAME_HEADER;

    frame[2] = servo->servo_id;

    frame[3] = YB_SD15M_MOVE_PACKET_LENGTH;

    frame[4] = YB_SD15M_INST_WRITE;

    frame[5] = YB_SD15M_REG_TARGET_POSITION;
    frame[6] =
        (uint8_t)(target_position >> 8);

    frame[7] =
        (uint8_t)(target_position & 0xFFU);

    frame[8] =
        (uint8_t)(move_time_ms >> 8);

    frame[9] =
        (uint8_t)(move_time_ms & 0xFFU);

    frame[10] = yb_sd15m_checksum(
        frame,
        YB_SD15M_MOVE_FRAME_SIZE
    );

    result = yb_sd15m_send_frame(
        servo,
        frame,
        YB_SD15M_MOVE_FRAME_SIZE
    );

    if (result == YB_SD15M_OK) {
        servo_data->sent_revision =
            target_revision;
    }

    return result;
}

static int yb_sd15m_request_position(struct yb_sd15m_device *servo)
{
    yb_sd15m_data_t *servo_data;

    uint8_t frame[YB_SD15M_READ_FRAME_SIZE];

    int result;

    if (servo == NULL ||
        servo->servo_data == NULL) {
        return YB_SD15M_ERR_PARAM;
    }

    servo_data = servo->servo_data;

    if (servo_data->initialised == 0U) {
        return YB_SD15M_ERR_NOT_READY;
    }

    frame[0] = YB_SD15M_FRAME_HEADER;
    frame[1] = YB_SD15M_FRAME_HEADER;

    frame[2] = servo->servo_id;

    frame[3] = YB_SD15M_READ_PACKET_LENGTH;

    frame[4] = YB_SD15M_INST_READ;

    frame[5] = YB_SD15M_REG_CURRENT_POSITION;

    frame[6] = YB_SD15M_POSITION_READ_SIZE;

    frame[7] = yb_sd15m_checksum(
        frame,
        YB_SD15M_READ_FRAME_SIZE
    );

    result = yb_sd15m_send_frame(
        servo,
        frame,
        YB_SD15M_READ_FRAME_SIZE
    );

    if (result == YB_SD15M_OK) {
        servo_data->waiting_position_reply = 1U;

        servo_data->position_request_tick =
            HAL_GetTick();

        servo_data->last_feedback_query_tick =
            servo_data->position_request_tick;
    }

    return result;
}

static void yb_sd15m_update(struct yb_sd15m_device *servo)
{
    yb_sd15m_data_t *servo_data;
    uint32_t now;

    if (servo == NULL ||
        servo->servo_data == NULL) {
        return;
    }

    servo_data = servo->servo_data;

    if (servo_data->initialised == 0U) {
        return;
    }

    now = HAL_GetTick();
    if (servo_data->waiting_position_reply != 0U) {
        if ((uint32_t)(
                now -
                servo_data->position_request_tick
            ) < YB_SD15M_REPLY_TIMEOUT_MS) {
            return;
        }
        servo_data->waiting_position_reply = 0U;

        ++servo_data->reply_timeout_count;
    }

    if (servo_data->sent_revision !=
        servo_data->target_revision) {
        (void)servo->send_ctrl_cmd(servo);

        return;
    }
    if ((uint32_t)(now - servo_data->last_feedback_query_tick) >= YB_SD15M_FEEDBACK_PERIOD_MS) {
        (void)servo->request_position(servo);
    }
}

static int yb_sd15m_device_set_target(struct yb_sd15m_device *servo,uint16_t target_position,uint16_t move_time_ms)
{
    yb_sd15m_data_t *servo_data;
    if (servo == NULL ||servo->servo_data == NULL) {
        return YB_SD15M_ERR_PARAM;
    }

    if (target_position < YB_SD15M_POSITION_MIN ||target_position > YB_SD15M_POSITION_MAX) {
        return YB_SD15M_ERR_RANGE;
    }

    servo_data = servo->servo_data;

    if (servo_data->initialised == 0U) {
        return YB_SD15M_ERR_NOT_READY;
    }

    servo_data->target_position =target_position;
    servo_data->move_time_ms =move_time_ms;

    __DMB();

    ++servo_data->target_revision;

    return YB_SD15M_OK;
}

/**
 * Read one status value.
 */
static int yb_sd15m_device_get_status(const struct yb_sd15m_device *servo,const char *which_status,void *status_data)
{
    yb_sd15m_data_t *servo_data;

    if (servo == NULL ||servo->servo_data == NULL ||which_status == NULL ||status_data == NULL) {
        return YB_SD15M_ERR_PARAM;
    }

    servo_data = servo->servo_data;

    if (strcmp(which_status, "POS") == 0) {*(uint16_t *)status_data = servo_data->current_position;}
    else if (strcmp(which_status, "TARGET") == 0) {*(uint16_t *)status_data = servo_data->target_position;}
    else if (strcmp(which_status, "TIME") == 0) {*(uint16_t *)status_data = servo_data->move_time_ms;}
    else if (strcmp(which_status, "ERR") == 0) {*(uint8_t *)status_data = servo_data->error;}
    else if (strcmp(which_status, "POS_VALID") == 0) {*(bool *)status_data = (servo_data->position_valid != 0U);}
    else if (strcmp(which_status, "ONLINE") == 0) {*(bool *)status_data = yb_sd15m_is_online(servo);}
    else if (strcmp(which_status, "TX_COUNT") == 0) {*(uint32_t *)status_data = servo_data->tx_frame_count;}
    else if (strcmp(which_status, "RX_COUNT") == 0) {*(uint32_t *)status_data = servo_data->rx_frame_count;}
    else if (strcmp(which_status, "CHECKSUM_ERR") == 0) {*(uint32_t *)status_data = servo_data->checksum_error_count;}
    else if (strcmp(which_status, "TIMEOUT_COUNT") == 0) {*(uint32_t *)status_data = servo_data->reply_timeout_count;}
    else {return YB_SD15M_ERR_PARAM;}

    return YB_SD15M_OK;
}

static void yb_sd15m_parser_reset(void)
{
    yb_sd15m_parser.count = 0U;
    yb_sd15m_parser.expected_size = 0U;
}
static void yb_sd15m_dispatch_frame(
    const uint8_t *frame,
    uint16_t frame_length
)
{
    struct yb_sd15m_device *servo;

    if (frame == NULL ||frame_length < YB_SD15M_MIN_FRAME_SIZE) {
        return;
    }
    servo = yb_sd15m_find_by_id(frame[2]);
    if (servo == NULL ||servo->feedback_calculate == NULL) {
        return;
    }
    servo->feedback_calculate(servo,frame,frame_length);

    servo->last_rx_tick = HAL_GetTick();
}


static void yb_sd15m_record_checksum_error(
    uint8_t servo_id
)
{
    struct yb_sd15m_device *servo;
    yb_sd15m_data_t *servo_data;

    servo = yb_sd15m_find_by_id(servo_id);

    if (servo == NULL ||servo->servo_data == NULL) {
        return;
    }

    servo_data = servo->servo_data;

    ++servo_data->checksum_error_count;
}

/**
 * Feed one byte into the YB-SD15M stream parser.
 */
static void yb_sd15m_parser_push_byte(
    uint8_t byte
)
{
    uint8_t packet_length;
    uint8_t received_checksum;
    uint8_t calculated_checksum;

    if (yb_sd15m_parser.count == 0U) {
        if (byte == YB_SD15M_FRAME_HEADER) {
            yb_sd15m_parser.frame[0] = byte;

            yb_sd15m_parser.count = 1U;
        }

        return;
    }
    if (yb_sd15m_parser.count == 1U) {
        if (byte == YB_SD15M_FRAME_HEADER) {
            yb_sd15m_parser.frame[1] = byte;

            yb_sd15m_parser.count = 2U;
        } else {
            yb_sd15m_parser_reset();
        }

        return;
    }

    if (yb_sd15m_parser.count >=
        YB_SD15M_RX_FRAME_MAX_SIZE) {
        yb_sd15m_parser_reset();

        if (byte == YB_SD15M_FRAME_HEADER) {
            yb_sd15m_parser.frame[0] = byte;

            yb_sd15m_parser.count = 1U;
        }

        return;
    }

    yb_sd15m_parser.frame[yb_sd15m_parser.count] = byte;

    ++yb_sd15m_parser.count;

    if (yb_sd15m_parser.count == 4U) {
        packet_length =yb_sd15m_parser.frame[3];

        yb_sd15m_parser.expected_size =
            (uint16_t)packet_length + 4U;

        if (packet_length < 2U ||yb_sd15m_parser.expected_size >YB_SD15M_RX_FRAME_MAX_SIZE) {
            yb_sd15m_parser_reset();
            return;
        }
    }

    if (yb_sd15m_parser.expected_size != 0U && yb_sd15m_parser.count == yb_sd15m_parser.expected_size)
        {received_checksum = yb_sd15m_parser.frame[yb_sd15m_parser.expected_size - 1U];

        calculated_checksum =yb_sd15m_checksum(yb_sd15m_parser.frame,yb_sd15m_parser.expected_size);

        if (received_checksum == calculated_checksum) {
            yb_sd15m_dispatch_frame(yb_sd15m_parser.frame,yb_sd15m_parser.expected_size);
        } else {
            yb_sd15m_record_checksum_error(yb_sd15m_parser.frame[2]);
        }

        yb_sd15m_parser_reset();
    }
}

static void yb_sd15m_uart_receive_callback(struct uart_device *uart,const uint8_t *data,uint16_t length)
{
    uint16_t index;

    if (uart == NULL ||uart != yb_sd15m_uart7 ||data == NULL ||length == 0U) {
        return;
    }

    for (index = 0U; index < length; ++index) {
        yb_sd15m_parser_push_byte(data[index]);}
}

/* -------------------------------------------------------------------------- */
/* Public device-management API                                               */
/* -------------------------------------------------------------------------- */


struct yb_sd15m_device *yb_sd15m_get_device(const char *name)
{
    uint32_t index;

    if (name == NULL) {
        return NULL;
    }

    for (index = 0U;
         index < YB_SD15M_Get_Count();
         ++index) {
        if (strcmp(
                name,
                yb_sd15m_list[index]->servo_name
            ) == 0) {
            return yb_sd15m_list[index];
        }
    }

    return NULL;
}


uint32_t YB_SD15M_Get_Count(void)
{
    return (uint32_t)(
        sizeof(yb_sd15m_list) /
        sizeof(yb_sd15m_list[0])
    );
}


int YB_SD15M_System_PowerOn_Init(void)
{
    struct uart_device *uart;
    uint32_t index;

    uart = uart_get_device(
        YB_SD15M_UART_NAME
    );

    if (uart == NULL ||
        uart->uart_send_bytes == NULL) {
        return YB_SD15M_ERR_NOT_READY;
    }

    yb_sd15m_uart7 = uart;
    yb_sd15m_parser_reset();

    uart->uart_recv_callback =
        yb_sd15m_uart_receive_callback;

    for (index = 0U;index < YB_SD15M_Get_Count();++index) {
        yb_sd15m_list[index]->init(
            yb_sd15m_list[index],
            yb_sd15m_list[index]->servo_id,
            uart
        );
    }

    return YB_SD15M_OK;
}

void YB_SD15M_All_Update(void)
{
    uint32_t index;

    for (index = 0U;
         index < YB_SD15M_Get_Count();
         ++index) {
        if (yb_sd15m_list[index] != NULL &&
            yb_sd15m_list[index]->update != NULL) {
            yb_sd15m_list[index]->update(
                yb_sd15m_list[index]
            );
        }
    }
}

int yb_sd15m_set_target(struct yb_sd15m_device *servo,uint16_t target_position,uint16_t move_time_ms
)
{
    if (servo == NULL ||
        servo->set_target == NULL) {
        return YB_SD15M_ERR_PARAM;
    }

    return servo->set_target(
        servo,
        target_position,
        move_time_ms
    );
}

int yb_sd15m_get_status(const struct yb_sd15m_device *servo,const char *which_status,void *status_data)
{
    if (servo == NULL ||
        servo->get_status == NULL) {
        return YB_SD15M_ERR_PARAM;
    }

    return servo->get_status(
        servo,
        which_status,
        status_data
    );
}

bool yb_sd15m_is_online(const struct yb_sd15m_device *servo)
{
    const yb_sd15m_data_t *servo_data;

    if (servo == NULL ||
        servo->servo_data == NULL) {
        return false;
    }

    servo_data = servo->servo_data;

    if (servo_data->rx_frame_count == 0U) {
        return false;
    }

    return (uint32_t)(
        HAL_GetTick() -
        servo->last_rx_tick
    ) <= YB_SD15M_OFFLINE_TIMEOUT_MS;
}