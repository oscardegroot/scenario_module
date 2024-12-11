#ifndef __LOGGING_H__
#define __LOGGING_H__

#include <scenario_module/config.h>

#include <ros_tools/logging.h>

/** Logging Pragmas */
#define SCENARIO_INFO(msg)               \
    if (SCENARIO_CONFIG.debug_output_)   \
    {                                    \
        LOG_INFO("[SCENARIO]: " << msg); \
    }

#define SCENARIO_WARN(msg)               \
    if (SCENARIO_CONFIG.debug_output_)   \
    {                                    \
        LOG_WARN("[SCENARIO]: " << msg); \
    }

#define SCENARIO_ERROR(msg) ROS_ERROR_STREAM("[SCENARIO]: " << msg)

#define SCENARIO_INFO_STREAM(msg)        \
    if (SCENARIO_CONFIG.debug_output_)   \
    {                                    \
        LOG_INFO("[SCENARIO]: " << msg); \
    }

#define SCENARIO_WARN_STREAM(msg)        \
    if (SCENARIO_CONFIG.debug_output_)   \
    {                                    \
        LOG_WARN("[SCENARIO]: " << msg); \
    }

#define SCENARIO_SUCCESS(msg)                                 \
    if (SCENARIO_CONFIG.debug_output_)                        \
    {                                                         \
        LOG_INFO("\033[32m[SCENARIO]: " << msg << "\033[0m"); \
    }

#define SCENARIO_ERROR_STREAM(msg) LOG_ERROR("[SCENARIO]: " << msg)

#define SCENARIO_INFO_ALWAYS(msg) LOG_INFO("[SCENARIO]: " << msg)
#define SCENARIO_WARN_ALWAYS(msg) LOG_WARN("[SCENARIO]: " << msg)
#define SCENARIO_SUCCESS_ALWAYS(msg) LOG_INFO("\033[32m[SCENARIO]: " << msg << "\033[0m");

#define SCENARIO_INFO_FUNCTION                    \
    if (SCENARIO_CONFIG.debug_output_)            \
    {                                             \
        LOG_INFO("[SCENARIO]: " << __FUNCTION__); \
    }
#define SCENARIO_WARN_FUNCTION                    \
    if (SCENARIO_CONFIG.debug_output_)            \
    {                                             \
        LOG_WARN("[SCENARIO]: " << __FUNCTION__); \
    }

#endif // __LOGGING_H__