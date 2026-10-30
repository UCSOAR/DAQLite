/**
 ******************************************************************************
 * File Name          : DebugTask.cpp
 * Description        : Task for controlling debug input
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <SOARDebug/DebugTask.hpp>
#include "Command.hpp"
#include "CubeUtils.hpp"
#include <cstring>
#include "LoggingService.hpp"
#include "stm32h7xx_hal.h"
#include "FlashTask.hpp"
#include "PollingTask.hpp"
#include "GpsTask.hpp"
#include "FlightTask.hpp"
#include "SensorFrameTask.hpp"
#include "cmsis_os.h"


// External Tasks (to send debug commands to)

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;
extern I2C_HandleTypeDef hi2c2;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Constructor, sets all member variables
 */
DebugTask::DebugTask()
    : Task(TASK_DEBUG_QUEUE_DEPTH_OBJS), kUart_(DEFAULT_DEBUG_UART_DRIVER)
{
  memset(debugBuffer, 0, sizeof(debugBuffer));
  debugMsgIdx = 0;
  isDebugMsgReady = false;
}

/**
 * @brief Init task for RTOS
 */
void DebugTask::InitTask()
{
  // Make sure the task is not already initialized
  SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Debug task twice");

  // Start the task
  BaseType_t rtValue = xTaskCreate(
      (TaskFunction_t)DebugTask::RunTask, (const char *)"DebugTask",
      (uint16_t)TASK_DEBUG_STACK_DEPTH_WORDS, (void *)this,
      (UBaseType_t)TASK_DEBUG_PRIORITY, (TaskHandle_t *)&rtTaskHandle);

  // Ensure creation succeded
  SOAR_ASSERT(rtValue == pdPASS, "DebugTask::InitTask - xTaskCreate() failed");
}

// TODO: Only run thread when appropriate GPIO pin pulled HIGH (or by define)
/**
 *    @brief Runcode for the DebugTask
 */
void DebugTask::Run(void *pvParams)
{
  // Arm the interrupt
  ReceiveData();

  while (1)
  {
    Command cm;

    // Wait forever for a command
    qEvtQueue->ReceiveWait(cm);

    // Process the command
    if (cm.GetCommand() == DATA_COMMAND &&
        cm.GetTaskCommand() == EVENT_DEBUG_RX_COMPLETE)
    {
      HandleDebugMessage((const char *)debugBuffer);
    }

    cm.Reset();
  }
}

namespace
{
	// flight_reset arms, confirm_reset within 10 s does it
	constexpr uint32_t CONFIRM_WINDOW_MS = 10000;
	uint32_t flightResetArmedMs = 0;
	bool flightResetArmed = false;
}

/**
 * @brief Handles debug messages, assumes msg is null terminated
 * @param msg Message to read, must be null termianted
 */
void DebugTask::HandleDebugMessage(const char *msg)
{
  //-- SYSTEM / CHAR COMMANDS -- (Must be last)
  if (strcmp(msg, "sysreset") == 0)
  {
    // Reset the system
    SOAR_ASSERT(false, "System reset requested");
  }
  else if (strcmp(msg, "sysinfo") == 0)
  {
    // Print message
    SOAR_PRINT("\n\n-- CUBE SYSTEM --\n");
    SOAR_PRINT("Current System Free Heap: %d Bytes\n", xPortGetFreeHeapSize());
    SOAR_PRINT("Lowest Ever Free Heap: %d Bytes\n",
               xPortGetMinimumEverFreeHeapSize());
    SOAR_PRINT("Debug Task Runtime  \t: %d ms\n\n",
               TICKS_TO_MS(xTaskGetTickCount()));
  } 
  else if (strcmp(msg, "flash_test") == 0)
  {
    SOAR_PRINT("Debug: Triggering flash tests\n");
    FlashTask::Inst().TriggerTest();
  }
  else if (strcmp(msg, "flash_dump") == 0)
  {
    Command cmd(TASK_SPECIFIC_COMMAND, FLASH_DUMP);
    FlashTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "flash_clear") == 0)
  {
    Command cmd(TASK_SPECIFIC_COMMAND, FLASH_CLEAR);
    FlashTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "flight_reset") == 0)
  {
    // The only way out of the filter's saved FLIGHT / LANDED state: back to the pad and reboot
    flightResetArmed = true;
    flightResetArmedMs = TICKS_TO_MS(xTaskGetTickCount());
    SOAR_PRINT("flight_reset: filter back to PAD (re-tare) and reboot. Send confirm_reset within 10 s\n");
  }
  else if (strcmp(msg, "confirm_reset") == 0)
  {
    const bool armed = flightResetArmed && (TICKS_TO_MS(xTaskGetTickCount()) - flightResetArmedMs) <= CONFIRM_WINDOW_MS;
    flightResetArmed = false;
    if (!armed) {
      SOAR_PRINT("confirm_reset: send flight_reset first (10 s window)\n");
    } else {
      FilterLog::PersistedState state{};
      LoggingService::LoadFlightState(state);		// keeps the boot count
      const uint32_t boots = state.bootCount;
      state = FilterLog::PersistedState{};
      state.bootCount = boots;
      state.phase = FilterLog::PHASE_PAD;
      if (LoggingService::SaveFlightStateNow(state)) {
        SOAR_PRINT("flight_reset: saved PAD, rebooting\n");
        osDelay(50);
        HAL_NVIC_SystemReset();
      } else {
        SOAR_PRINT("flight_reset FAILED: flash not ready or write failed, state unchanged\n");
      }
    }
  }
  else if (strcmp(msg, "stop_dump") == 0)
  {
    LoggingService::StopDump();
  }
  else if (strcmp(msg, "imu32a") == 0){
	  Command cmd(DATA_COMMAND, PollingTask::IMU32_A_TEST);
	  PollingTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "imu32b") == 0){
  	  Command cmd(DATA_COMMAND, PollingTask::IMU32_B_TEST);
  	  PollingTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "imu16") == 0){
    	  Command cmd(DATA_COMMAND, PollingTask::IMU16_TEST);
    	  PollingTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "baro07") == 0){
  	  Command cmd(DATA_COMMAND, PollingTask::BARO07_TEST);
  	  PollingTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "baro11") == 0){
  	  Command cmd(DATA_COMMAND, PollingTask::BARO11_TEST);
  	  PollingTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "mag") == 0){
  	  Command cmd(DATA_COMMAND, PollingTask::MAG_TEST);
  	  PollingTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "gps") == 0){
  	  Command cmd(TASK_SPECIFIC_COMMAND, GpsTask::GPS_TEST);
  	  GpsTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "gps_init") == 0){
  	  Command cmd(TASK_SPECIFIC_COMMAND, GpsTask::GPS_INIT);
  	  GpsTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "flight_state") == 0){
  	  Command cmd(TASK_SPECIFIC_COMMAND, FlightTask::PRINT_STATE);
  	  FlightTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "flight_test") == 0){
  	  Command cmd(TASK_SPECIFIC_COMMAND, FlightTask::TOGGLE_TEST_MODE);
  	  FlightTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strcmp(msg, "force_flight") == 0){
  	  Command cmd(TASK_SPECIFIC_COMMAND, FlightTask::TOGGLE_FORCE_FLIGHT);
  	  FlightTask::Inst().GetEventQueue()->Send(cmd);
  }
  else if (strncmp(msg, "APF_", 4) == 0)
  {
    // Altitude prediction filter debug commands, handled by the SensorFrameTask
    if (!SensorFrameTask::HandleApfCommand(msg)) {
      SOAR_PRINT("Debug, unknown APF command: %s (try APF_Help)\n", msg);
    }
  }

#if (configGENERATE_RUN_TIME_STATS == 1)  // enable profiling commands if profiling enabled
  else if (strcmp(msg, "profile") == 0) {
    profileSystem = true;
  } else if (strcmp(msg, "stop profiling") == 0) {
    profileSystem = false;
  }
#endif
    else {
    // Single character command, or unknown command
    switch (msg[0])
    {
    default:
      SOAR_PRINT("Debug, unknown command: %s\n", msg);
      break;
    }
  }

  // We've read the data, clear the buffer
  debugMsgIdx = 0;
  isDebugMsgReady = false;
}

/**
 * @brief Receive data, currently receives by arming interrupt
 */
bool DebugTask::ReceiveData() { return kUart_->ReceiveIT(&debugRxChar, this); }

/**
 * @brief Receive data to the buffer
 * @return Whether the debugBuffer is ready or not
 */
void DebugTask::InterruptRxData(uint8_t errors)
{
  // If we already have an unprocessed debug message, ignore this byte
  if (!isDebugMsgReady)
  {
    // Check byte for end of message - note if using termite you must turn on
    // append CR
    if (debugRxChar == '\r' || debugMsgIdx == DEBUG_RX_BUFFER_SZ_BYTES)
    {
      // Null terminate and process
      debugBuffer[debugMsgIdx++] = '\0';
      isDebugMsgReady = true;

      // Notify the debug task
      Command cm(DATA_COMMAND, EVENT_DEBUG_RX_COMPLETE);
      bool res = qEvtQueue->SendFromISR(cm);

      // If we failed to send the event, we should reset the buffer, that way
      // DebugTask doesn't stall
      if (res == false)
      {
        debugMsgIdx = 0;
        isDebugMsgReady = false;
      }
    }
    else
    {
      debugBuffer[debugMsgIdx++] = debugRxChar;
    }
  }

  // Re-arm the interrupt
  ReceiveData();
}

/* Helper Functions
 * --------------------------------------------------------------*/
/**
 * @brief Extracts an integer parameter from a string
 * @brief msg Message to extract from, MUST be at least identifierLen long, and
 * properly null terminated
 * @brief identifierLen Length of the identifier eg. 'rsc ' (Including the
 * space) is 4
 * @return ERRVAL on failure, otherwise the extracted value
 */
int32_t DebugTask::ExtractIntParameter(const char *msg,
                                       uint16_t identifierLen)
{
  // Handle a command with an int parameter at the end
  if (static_cast<uint16_t>(strlen(msg)) < identifierLen + 1)
  {
    SOAR_PRINT("Int parameter command insufficient length\r\n");
    return ERRVAL;
  }

  // Extract the value and attempt conversion to integer
  const int32_t val = Utils::StringToLong(&msg[identifierLen]);
  if (val == ERRVAL)
  {
    SOAR_PRINT("Int parameter command invalid value\r\n");
  }

  return val;
}
