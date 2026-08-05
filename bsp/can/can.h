/**
 * @file can.h
 * @brief Classic CAN configuration for the DM-MC-Board02.
 */

#ifndef DM_CAN_H
#define DM_CAN_H

/* Motor standard identifiers. */
#define CAN_GM6020_PITCH_ID          0x205U
#define CAN_GM6020_YAW_ID          0x206U
#define CAN_M3508_2_ID              0x202U

#ifdef __cplusplus
extern "C" {
#endif

/** Configure filters, start FDCAN1/2/3 in Classic CAN mode, and enable FIFO0 RX notification. */
void can_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_CAN_H */
