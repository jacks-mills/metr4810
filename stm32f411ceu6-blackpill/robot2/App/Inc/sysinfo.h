/**
 * @file    sysinfo.h
 * @brief   Heap and stack figures for the debug panel.
 */
#ifndef SYSINFO_H
#define SYSINFO_H

#include <stdint.h>

#define SYSINFO_MAX_TASKS 10

struct TaskInfo {
    char name[12];
    uint32_t stackFree;         /* words never used since the task started (CubeMX's unit) */
};

struct SysInfo {
    uint32_t heapFree;          /* FreeRTOS heap, bytes */
    uint32_t heapMinFree;       /* lowest it has been */
    uint32_t logDropped;        /* log messages lost because the queue was full */
    uint8_t taskCount;
    struct TaskInfo tasks[SYSINFO_MAX_TASKS];
};

void sysinfo_read(struct SysInfo *info);

#endif /* SYSINFO_H */
