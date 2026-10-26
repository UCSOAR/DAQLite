/**
 ******************************************************************************
 * File Name          : FlashTask.hpp
 * Description        : Flash task for MX66L1G45GMI test operations
 ******************************************************************************
 */
#ifndef CUBE_SYSTEM_FLASH_TASK_HPP_
#define CUBE_SYSTEM_FLASH_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"
#include <stdint.h>

/* Enums ------------------------------------------------------------------*/
enum FLASH_TASK_COMMANDS
{
    FLASH_TASK_COMMAND_NONE = 0,
    EVENT_FLASH_INIT,
    EVENT_FLASH_TEST,
	FLASH_DUMP,
	STOP_DUMP,
	FLASH_CLEAR,

};

/* Class ------------------------------------------------------------------*/
class FlashTask : public Task
{
public:
    static FlashTask &Inst()
    {
        static FlashTask inst;
        return inst;
    }

    void InitTask();

    // Public interface for other tasks to trigger operations
    void TriggerTest();

    void dumoflash();
    bool IsFlashInitialized() const { return flashInitialized; }

protected:
    static void RunTask(void *pvParams)
    {
        FlashTask::Inst().Run(pvParams);
    } // Static Task Interface, passes control to the instance Run();

    void Run(void *pvParams); // Main run code
    void HandleCommand(Command &cm);

private:
    FlashTask();                             // Private constructor
    FlashTask(const FlashTask &);            // Prevent copy-construction
    FlashTask &operator=(const FlashTask &); // Prevent assignment

    // Task operation functions
    void InitializeFlash();
    void RunFlashTests();

    // Member variables
    volatile bool flashInitialized;
    uint32_t testCounter;
};

#endif // CUBE_SYSTEM_FLASH_TASK_HPP_
