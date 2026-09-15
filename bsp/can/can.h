/**
 * @file can.h
 * @brief Classic CAN configuration for the DM-MC-Board02.
 */

#ifndef DM_CAN_H
#define DM_CAN_H

/* Motor standard identifiers. */
#define CAN_GM6020_PITCH_ID          0x205U
#define CAN_GM6020_YAW_ID          0x206U
#define CAN_M3508_1_ID              0x201U
#define CAN_MG4005E_ID              0x141U
#define CAN_MG4005E_1_ID            CAN_MG4005E_ID

#ifdef __cplusplus
extern "C" {
#endif

void can_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_CAN_H */
