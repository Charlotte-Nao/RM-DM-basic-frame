//
// Created by ASUS on 2026/11/7.
//

#ifndef DM_TRACE_TRACKING_H
#define DM_TRACE_TRACKING_H

#include "../../application/global_data.h"

// 状态更新后调用，替代零参考的 lqr_calculate，不与其同时调用。
void trace_tracking_calculate(void);

#endif //DM_TRACE_TRACKING_H