/**
 ******************************************************************************
 * File Name          : main_system.cpp
 * Description        : This file acts as an interface supporting CubeIDE Codegen
    while having a clean interface for development.
 ******************************************************************************
*/
/* Includes -----------------------------------------------------------------*/
#include "SystemDefines.hpp"
#include "UARTDriver.hpp"

// Tasks
#include "UARTTask.hpp"
#include "CubeTask.hpp"
#include "DebugTask.hpp"
#include "LoggingTask.hpp"
#include "FlashTask.hpp"


/* Drivers ------------------------------------------------------------------*/
namespace Driver {
    UARTDriver usart8(UART8);
    UARTDriver uart7(UART7);
}

/* Interface Functions ------------------------------------------------------------*/
/**
 * @brief Main function interface, called inside main.cpp before os initialization takes place.
*/
void run_main() {
    // Init Tasks
	UARTTask::Inst().InitTask();
	CubeTask::Inst().InitTask();
	DebugTask::Inst().InitTask();
    LoggingTask::Inst().InitTask();


    // start profiling task if run_time_stats enabled
    #if (configGENERATE_RUN_TIME_STATS == 1)
    ProfilerTask::Inst().InitTask();
    #endif

    // Print System Boot Info : Warning, don't queue more than 10 prints before scheduler starts
    SOAR_PRINT("\n-- CUBE SYSTEM --\n");
    SOAR_PRINT("System Reset Reason: [TODO]\n"); //TODO: System reset reason can be implemented via. Flash storage
    SOAR_PRINT("Current System Free Heap: %d Bytes\n", xPortGetFreeHeapSize());
    SOAR_PRINT("Lowest Ever Free Heap: %d Bytes\n\n", xPortGetMinimumEverFreeHeapSize());
    // Start the Scheduler
    // Guidelines:
    // - Be CAREFUL with race conditions after osKernelStart
    // - All uses of new and delete should be closely monitored after this point


    osKernelStart();

    // Should never reach here
    SOAR_ASSERT(false, "osKernelStart() failed");

    while (1)
    {
        osDelay(100);
        HAL_NVIC_SystemReset();
    }
}

