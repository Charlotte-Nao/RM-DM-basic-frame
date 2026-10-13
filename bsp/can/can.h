/**
 * @file can.h
 * @brief Classic CAN configuration for the DM-MC-Board02.
 */

#ifndef DM_CAN_H
#define DM_CAN_H

/* Motor standard identifiers. */
#define CAN_M3508_L_ID                        0x201U
#define CAN_M3508_R_ID                        0x202U

#define CAN_DM8009P_LF_COMMAND_ID             0x001U
#define CAN_DM8009P_LF_FEEDBACK_ID            0x011U
#define CAN_DM_8009P_LB_COMMAND_ID            0x002U
#define CAN_DM_8009P_LB_FEEDBACK_ID           0x012U
#define CAN_DM_8009P_RF_COMMAND_ID            0x003U
#define CAN_DM_8009P_RF_FEEDBACK_ID           0x013U
#define CAN_DM_8009P_RB_COMMAND_ID            0x004U
#define CAN_DM_8009P_RB_FEEDBACK_ID           0x014U

#define CAN_GM6020_YAW_ID                     0x206U
#define CAN_MG4005E_STIR_ID                   0x141U

#ifdef __cplusplus
extern "C" {
#endif

void can_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_CAN_H */
