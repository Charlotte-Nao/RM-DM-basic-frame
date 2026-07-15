/**
 * @file YB_SD15M.h
 * @brief Static, object-like YB-SD15M serial bus servo interface.
 */

#ifndef YB_SD15M_H
#define YB_SD15M_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uart_device;

/* YB-SD15M protocol range. */
#define YB_SD15M_DEFAULT_ID          1U

#define YB_SD15M_POSITION_MIN        96U
#define YB_SD15M_POSITION_MAX        4000U
#define YB_SD15M_POSITION_CENTER     2048U

/* Device-layer return values. */
#define YB_SD15M_OK                  0
#define YB_SD15M_ERR_PARAM          (-1)
#define YB_SD15M_ERR_NOT_READY      (-2)
#define YB_SD15M_ERR_RANGE          (-3)
#define YB_SD15M_ERR_UART           (-4)

/**
 * YB-SD15M device object.
 *
 * One physical servo corresponds to:
 *
 * 1. One private yb_sd15m_data_t object.
 * 2. One yb_sd15m_device object.
 */
struct yb_sd15m_device
{
    const char *servo_name;

    uint8_t servo_id;

    struct uart_device *servo_uart;

    void *servo_data;

    volatile uint32_t last_rx_tick;

    /**
     * Initialise one physical servo instance.
     */
    void (*init)(
        struct yb_sd15m_device *servo,
        uint8_t servo_id,
        struct uart_device *uart
    );

    /**
     * Parse one complete and validated feedback frame.
     */
    void (*feedback_calculate)(
        const struct yb_sd15m_device *servo,
        const uint8_t *frame,
        uint16_t frame_length
    );

    /**
     * Send the currently stored target-position command.
     */
    int (*send_ctrl_cmd)(
        struct yb_sd15m_device *servo
    );

    /**
     * Request current position feedback.
     */
    int (*request_position)(
        struct yb_sd15m_device *servo
    );

    /**
     * Periodic update function.
     */
    void (*update)(
        struct yb_sd15m_device *servo
    );

    /**
     * Change target data without sending immediately.
     */
    int (*set_target)(
        struct yb_sd15m_device *servo,
        uint16_t target_position,
        uint16_t move_time_ms
    );

    /**
     * Read one status value.
     */
    int (*get_status)(
        const struct yb_sd15m_device *servo,
        const char *which_status,
        void *status_data
    );
};

/**
 * Find a registered servo instance by name.
 *
 * Current instance:
 *
 * YB_SD15M_1
 */
struct yb_sd15m_device *yb_sd15m_get_device(
    const char *name
);

/**
 * Return the number of registered servo instances.
 */
uint32_t YB_SD15M_Get_Count(void);

/**
 * Initialise UART7 and every registered YB-SD15M instance.
 */
int YB_SD15M_System_PowerOn_Init(void);

/**
 * Periodically update every registered YB-SD15M instance.
 *
 * Call this from servo_task.
 */
void YB_SD15M_All_Update(void);

/**
 * Update the desired position and movement time.
 *
 * This function does not directly send UART data.
 * YB_SD15M_All_Update() sends the command later.
 */
int yb_sd15m_set_target(
    struct yb_sd15m_device *servo,
    uint16_t target_position,
    uint16_t move_time_ms
);

/**
 * Read one status value.
 *
 * Supported status strings:
 *
 * "POS"            uint16_t
 * "TARGET"         uint16_t
 * "TIME"           uint16_t
 * "ERR"            uint8_t
 * "POS_VALID"      bool
 * "ONLINE"         bool
 * "TX_COUNT"       uint32_t
 * "RX_COUNT"       uint32_t
 * "CHECKSUM_ERR"   uint32_t
 * "TIMEOUT_COUNT"  uint32_t
 */
int yb_sd15m_get_status(
    const struct yb_sd15m_device *servo,
    const char *which_status,
    void *status_data
);

/**
 * Return true when valid feedback has recently been received.
 */
bool yb_sd15m_is_online(
    const struct yb_sd15m_device *servo
);

#ifdef __cplusplus
}
#endif

#endif /* YB_SD15M_H */