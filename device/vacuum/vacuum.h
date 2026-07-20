//
// Created by charlotte on 7/20/26.
//

//
// Created by charlotte on 7/20/26.
//

#ifndef DM_VACUUM_H
#define DM_VACUUM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct vacuum_device {
    const char *device_name;
    void *device_data;
    void (*init)(struct vacuum_device *device);
    void (*enable)(struct vacuum_device *device);
    void (*disable)(struct vacuum_device *device);
    void (*update)(struct vacuum_device *device);
    void (*get_status)(const struct vacuum_device *device, const char *which_status, void *status_data);
};

struct vacuum_device *vacuum_get_device(const char *name);
uint32_t Vacuum_Get_Count(void);
void Vacuum_System_PowerOn_Init(void);
void Vacuum_System_PowerOff(void);
void Vacuum_All_Update(void);

#ifdef __cplusplus
}
#endif

#endif
