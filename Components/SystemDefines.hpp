/**
  ******************************************************************************
  * @file           : SystemDefines.hpp
  * @brief          : Macros and wrappers
  ******************************************************************************
  *
  * Contains system wide macros, defines, and wrappers
  *
  ******************************************************************************
  */
#ifndef CUBE_MAIN_SYSTEM_DEFINES_H
#define CUBE_MAIN_SYSTEM_DEFINES_H

/* Environment Defines ------------------------------------------------------------------*/
//#define COMPUTER_ENVIRONMENT        // Define this if we're in Windows, Linux or Mac (not when flashing on DMB)

#ifdef COMPUTER_ENVIRONMENT
#define __CC_ARM
#endif

/* System Wide Includes ------------------------------------------------------------------*/
#include "main_system.hpp" // C++ Main File Header
#include "UARTDriver.hpp"

/* Cube++ Required Configuration ------------------------------------------------------------------*/
#include "CubeDefines.hpp"
constexpr UARTDriver* const DEFAULT_DEBUG_UART_DRIVER = UART::Debug;    // UART Handle that ASSERT messages are sent over

//Barometer constants
constexpr int TEMP_LOW = 2000;
constexpr int TEMP_VERY_LOW = -1500;
constexpr int CMD_SIZE = 1;
constexpr int CMD_TIMEOUT = 150;

enum GLOBAL_COMMANDS : uint8_t
{
    COMMAND_NONE = 0,        // No command, packet can probably be ignored
    TASK_SPECIFIC_COMMAND,    // Runs a task specific command when given this object
    DATA_COMMAND,// Data command, used to send data to a task. Target is stored in taskCommand
	DATA_BROKER_COMMAND,
	GPS_COMMAND,
};

/* Cube++ Optional Code Configuration ------------------------------------------------------------------*/


/* Task Parameter Definitions ------------------------------------------------------------------*/
/* - Lower priority number means lower priority task ---------------------------------*/

// UART TASK
constexpr uint8_t UART_TASK_RTOS_PRIORITY = 2;            // Priority of the uart task
constexpr uint8_t UART_TASK_QUEUE_DEPTH_OBJS = 10;        // Size of the uart task queue
constexpr uint16_t UART_TASK_STACK_DEPTH_WORDS = 512;    // Size of the uart task stack

// DEBUG TASK
constexpr uint8_t TASK_DEBUG_PRIORITY = 2;            // Priority of the debug task
constexpr uint8_t TASK_DEBUG_QUEUE_DEPTH_OBJS = 10;        // Size of the debug task queue
constexpr uint16_t TASK_DEBUG_STACK_DEPTH_WORDS = 512;        // Size of the debug task stack

// PROFILING TASK
constexpr uint8_t TASK_PROFILER_PRIORITY = 2;            // Priority of the profiling task
constexpr uint8_t TASK_PROFILER_QUEUE_DEPTH_OBJS = 10;        // Size of the profiling task queue
constexpr uint16_t TASK_PROFILER_STACK_DEPTH_WORDS = 512;        // Size of the profiling task stack

constexpr uint8_t TASK_LOGGING_PRIORITY = 3;
constexpr uint8_t TASK_LOGGING_QUEUE_DEPTH_OBJS = 32;	// Was 10: ~6 messages per poll filled it while logging (re)started
constexpr uint16_t TASK_LOGGING_QUEUE_DEPTH_WORDS = 512;

//SENSORFRAME TASK
constexpr uint8_t TASK_SENSORFRAME_PRIORITY = 3;         // Priority of the sensorframe task (APF). Flight critical, must not wait behind logging
constexpr uint8_t TASK_SENSORFRAME_DEPTH_OBJS = 16;      // Size of the sensorframe task queue
constexpr uint16_t TASK_SENSORFRAME_DEPTH_WORDS = 1024; // Size of the sensorframe task stack

// FLASH TASK
constexpr uint8_t TASK_FLASH_TASK_PRIORITY = 3;         // Priority of the flash task
constexpr uint8_t TASK_FLASH_QUEUE_DEPTH_OBJS = 8;      // Size of the flash task queue
constexpr uint16_t TASK_FLASH_STACK_DEPTH_WORDS = 1024; // Size of the flash task stack

// FLIGHT TASK
constexpr uint8_t TASK_FLIGHT_PRIORITY = 3;            // Priority of the flight task
constexpr uint8_t TASK_FLIGHT_QUEUE_DEPTH_OBJS = 32;    // Size of the flight task queue (all IMU + baro messages per poll)
constexpr uint16_t TASK_FLIGHT_STACK_DEPTH_WORDS = 768; // Size of the flight task stack

constexpr uint8_t TASK_GPS_PRIORITY = 2;           // Priority of the barometer task
constexpr uint8_t TASK_GPS_QUEUE_DEPTH_OBJS = 10;        // Size of the barometer task queue
constexpr uint16_t TASK_GPS_STACK_DEPTH_WORDS = 896;        // Size of the barometer task stack

#endif // CUBE_MAIN_SYSTEM_DEFINES_H

