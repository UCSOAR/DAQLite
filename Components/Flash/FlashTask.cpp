/**
 ******************************************************************************
 * File Name          : FlashTask.cpp
 * Description        : Flash task implementation for MX66L1G45GMI test operations
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "FlashTask.hpp"
#include "SystemDefines.hpp"
#include "mx66xx_qspi.hpp"
#include "SPIFlash.hpp"
#include <cstring>
#include "stm32h7xx_hal.h"
#include "LoggingService.hpp"

/* Constants ------------------------------------------------------------------*/
constexpr uint16_t FLASH_TEST_SECTOR = 32;          // TODO: Move to a reserved sector if needed
constexpr uint32_t FLASH_TEST_SIZE_BYTES = 512;    // Two pages
constexpr uint32_t FLASH_ERASE_VERIFY_BYTES = 256; // One page

/* Helpers -------------------------------------------------------------------*/
static void FillPattern(uint8_t *buf, uint32_t size, uint8_t seed)
{
    for (uint32_t i = 0; i < size; i++)
    {
        buf[i] = static_cast<uint8_t>(seed + (i * 7) + (i >> 3));
    }
}

static int32_t FindMismatch(const uint8_t *a, const uint8_t *b, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        if (a[i] != b[i])
        {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

static bool VerifyErased(uint16_t sector)
{
    uint8_t readBuf[FLASH_ERASE_VERIFY_BYTES];
    memset(readBuf, 0, sizeof(readBuf));

    uint32_t addr = ((uint32_t)sector * FS_SECTOR_SIZE);
    MX66xxQSPI_4ReadBytes(readBuf, addr, sizeof(readBuf));
    for (uint32_t i = 0; i < sizeof(readBuf); i++)
    {
        if (readBuf[i] != 0xFF)
        {
            SOAR_PRINT("VerifyErased() - sector=%u offset=%lu value=0x%02X (expected 0xFF)\n",
                       (unsigned int)sector,
                       (uint32_t)i,
                       readBuf[i]);
            return false;
        }
    }
    return true;
}

/**
 * @brief Constructor, sets up task
 */
FlashTask::FlashTask()
    : Task(TASK_FLASH_QUEUE_DEPTH_OBJS),
      flashInitialized(false),
      testCounter(0)
{
}

/**
 * @brief Initialize the FlashTask
 */
void FlashTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Flash task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FlashTask::RunTask,
                    (const char *)"FlashTask",
                    (uint16_t)TASK_FLASH_STACK_DEPTH_WORDS,
                    (void *)this,
                    (UBaseType_t)TASK_FLASH_TASK_PRIORITY,
                    (TaskHandle_t *)&rtTaskHandle);

    // Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "FlashTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Flash Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlashTask::Run(void *pvParams)
{

    SOAR_PRINT("FlashTask::Run() - Starting task\n");

    InitializeFlash();

    while (1)
    {
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if (res)
        {
            HandleCommand(cm);
        }
    }
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void FlashTask::HandleCommand(Command &cm)
{
    if (cm.GetCommand() == TASK_SPECIFIC_COMMAND)
    {
        switch (cm.GetTaskCommand())
        {
        case EVENT_FLASH_INIT:
            InitializeFlash();
            break;
        case EVENT_FLASH_TEST:
            RunFlashTests();
            break;
        case FLASH_DUMP:
        	dumoflash();
        	break;
        case FLASH_CLEAR:
            LoggingService::FlashClear();
            break;


        default:
            SOAR_PRINT("FlashTask - Received Unsupported Task Command {%d}\n", cm.GetTaskCommand());
            break;
        }
    }
    else
    {
        SOAR_PRINT("FlashTask - Received Unsupported Global Command {%d}\n", cm.GetCommand());
    }

    cm.Reset();
}

/**
 * @brief Initialize flash and basic connectivity checks
 */
void FlashTask::InitializeFlash()
{
    SOAR_PRINT("FlashTask::InitializeFlash() - Initializing MX66 flash\n");

    MX66xxQSPI_ReleaseFromDeepPowerDown();
    MX66xxQSPI_RSTEN();
    MX66xxQSPI_RST();
    MX66xxQSPI_EQIO_1LINE();
    MX66xxQSPI_EN4B();


    if (!MX66xxQSPI_Init())
    {
        SOAR_PRINT("FlashTask::InitializeFlash() - MX66 init failed\n");
        return;
    }

    uint8_t status = MX66xxQSPI_ReadStatusRegister();
    SOAR_PRINT("FlashTask::InitializeFlash() - Status1: 0x%02x\n", status);

    uint32_t id = MX66xxQSPI_QPIReadID();
    SOAR_PRINT("FlashTask::InitializeFlash() - MX66 ReadID: 0x%06lx\n", id);

    flashInitialized = true;
}

/**
 * @brief Run flash tests: erase, write, readback, verify
 */
void FlashTask::RunFlashTests()
{
    if (!flashInitialized)
    {
        InitializeFlash();
    }

    SOAR_PRINT("FlashTask::RunFlashTests() - Starting tests\n");

    if (MX66xxQSPI_IsWriteProtected())
    {
        SOAR_PRINT("FlashTask::RunFlashTests() - Flash is write protected, aborting\n");
        return;
    }

    uint8_t sfdp[256]{0};
    //    memset(sfdp, 0, sizeof(sfdp));
    //MX66xxQSPI_ReadSFDP(sfdp);

    SOAR_PRINT("FlashTask::RunFlashTests() - SFDP[0..15]: ");
    for (uint32_t i = 0; i < 16; i++)
    {
        SOAR_PRINT("%02X ", sfdp[i]);
    }
    SOAR_PRINT("\n");

    // Test sector erase
    SOAR_PRINT("FlashTask::RunFlashTests() - Erasing sector %u\n", FLASH_TEST_SECTOR);
    //mx66xx.Addr4Byte = false;
    MX66xxQSPI_EraseSector(FLASH_TEST_SECTOR);

    if (!VerifyErased(FLASH_TEST_SECTOR))
    {
        SOAR_PRINT("FlashTask::RunFlashTests() - Erase verify FAILED\n");
        return;
    }
    SOAR_PRINT("FlashTask::RunFlashTests() - Erase verify OK\n");

    // Write block test (multi-page)
    uint8_t txBuf[FLASH_TEST_SIZE_BYTES];
    uint8_t rxBuf[FLASH_TEST_SIZE_BYTES];
    FillPattern(txBuf, sizeof(txBuf), 0xA5);
    memset(rxBuf, 0, sizeof(rxBuf));

    SOAR_PRINT("FlashTask::RunFlashTests() - Writing %lu bytes using MX66xxQSPI_WriteSector\n", (uint32_t)sizeof(txBuf));
    MX66xxQSPI_WriteSector(txBuf, FLASH_TEST_SECTOR, 0, sizeof(txBuf));

    MX66xxQSPI_ReadSector(rxBuf, FLASH_TEST_SECTOR, 0, sizeof(rxBuf));
    int32_t mismatch = FindMismatch(txBuf, rxBuf, sizeof(txBuf));
    if (mismatch >= 0)
    {
        SOAR_PRINT("FlashTask::RunFlashTests() - Write_Block verify FAILED at %ld (0x%02X != 0x%02X)\n",
                   mismatch, txBuf[mismatch], rxBuf[mismatch]);
        return;
    }
    SOAR_PRINT("FlashTask::RunFlashTests() - Write_Block verify OK\n");

    // Re-erase and do page write test
    MX66xxQSPI_EraseSector(FLASH_TEST_SECTOR);

    const uint32_t testPage = (FLASH_TEST_SECTOR * (FS_SECTOR_SIZE / FS_PAGE_SIZE));
    FillPattern(txBuf, FS_PAGE_SIZE, 0x5A);
    memset(rxBuf, 0, FS_PAGE_SIZE);

    SOAR_PRINT("FlashTask::RunFlashTests() - Writing page %lu using MX66xxQSPI_WritePage\n", testPage);
    MX66xxQSPI_WritePage(txBuf, testPage, 0, FS_PAGE_SIZE);

    MX66xxQSPI_4ReadBytes(rxBuf, FLASH_TEST_SECTOR * FS_SECTOR_SIZE, FS_PAGE_SIZE);
    mismatch = FindMismatch(txBuf, rxBuf, FS_PAGE_SIZE);
    if (mismatch >= 0)
    {
        SOAR_PRINT("FlashTask::RunFlashTests() - Write_Page verify FAILED at %ld (0x%02X != 0x%02X)\n",
                   mismatch, txBuf[mismatch], rxBuf[mismatch]);
        return;
    }

    testCounter++;
    SOAR_PRINT("FlashTask::RunFlashTests() - Tests completed (run #%lu)\n", testCounter);
}

/**
 * @brief Trigger flash tests from external task
 */
void FlashTask::TriggerTest()
{
    Command cm(TASK_SPECIFIC_COMMAND, EVENT_FLASH_TEST);
    qEvtQueue->Send(cm);
}

void FlashTask::dumoflash() {
	LoggingService::ProcessFlashDump();
}
