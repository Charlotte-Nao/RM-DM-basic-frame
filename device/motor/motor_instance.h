#ifndef DM_MOTOR_INSTANCE_H
#define DM_MOTOR_INSTANCE_H

#include <stdint.h>

struct motor_device;

uint32_t motor_instance_count(void);
struct motor_device *motor_instance_get(uint32_t index);

#endif /* DM_MOTOR_INSTANCE_H */
