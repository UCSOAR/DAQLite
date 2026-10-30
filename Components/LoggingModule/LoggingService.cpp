/*
 * LoggingService.cpp
 *
 *  Created on: Jan 21, 2026
 *      Author: jaddina
 */
#include "LoggingService.hpp"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "FlashTask.hpp"
#include "Mutex.hpp"

extern QSPI_HandleTypeDef hqspi;

const char* SensorTypeName(LoggingData type);	//defined below, used in error messages

uint8_t  LoggingService::ramLog[RAM_LOG_SIZE] = {0}; 	//buffer that when full logs all to flash (500 bytes)
uint32_t LoggingService::ramHead = 0;					//the head of the buffer across all instances
uint16_t LoggingService::sectorAddress = 0;				//the address of the sector cursor is at
uint8_t LoggingService::bufferPerSector = 0;			//the buffer offset within each sector across all states
bool LoggingService::flashStateLoaded = false;			//flag indicating whether the flash state is loaded or not
volatile uint8_t LoggingService::done = true;			//if logging is finished or flash is full
volatile uint8_t LoggingService::doneDump =0;			//flag indicating if dump is done

static uint8_t sectorBuf[RAM_LOG_SIZE];					//buffer used when reading back chunks from the flash
static uint32_t dumpSector = 0;							//dump sector address (keeps trak of current dump sector)
static uint16_t dumpOffset = 0;							//dump sector offset
static uint8_t txBuf[RAM_LOG_SIZE];						//write buffer when logging

namespace
{
	constexpr uint16_t FLASH_STATE_SECTOR = NUM_SECTORS - 1;		//Reserves the last sector of flash chip for saving logger state
	constexpr uint16_t FLASH_TEST_SECTOR = NUM_SECTORS - 2;			//Reserves the second-last sector for testing
	constexpr uint16_t FLIGHT_STATE_SECTOR = NUM_SECTORS - 3;		//Reserves the third-last sector for the filter's flight state (power loss)
	constexpr uint16_t FLASH_DATA_LIMIT_SECTOR = FLIGHT_STATE_SECTOR;	//The sector limit at which data can be written to in flash

	constexpr uint32_t FLASH_STATE_MAGIC = 0x4C4F4731;				//Recognizable constant to check if saved flash state is valid

	//16 byte flash state record for saving flash state between program runs
	struct FlashStateRecord
	{
		uint32_t magic;					// validates that this is actually a logger state record
		uint32_t sector;				// next sector to write to
		uint32_t bufferPerSector;		// next chunk inside that sector
		uint32_t checksum;				// simple corruption check
	};

	/* The state sector is used as an append-only log of records: each save writes the next empty
	 * 16 byte slot and the newest valid record is the cursor. The sector is only erased once all
	 * 256 slots are used, so a reset mid-save can at worst lose the newest record, not the cursor.
	 */
	constexpr uint32_t STATE_RECORDS_PER_SECTOR = FS_SECTOR_SIZE / sizeof(FlashStateRecord);	//256 records
	constexpr uint32_t STATE_RECORDS_PER_READ = FS_PAGE_SIZE / sizeof(FlashStateRecord);		//16 records per page read
	static uint32_t stateSlot = 0;			//next empty record slot in the state sector
	static bool stateSlotKnown = false;		//stateSlot is only valid once the state sector has been read

	static Mutex flashLock;					//LoggingTask writes while FlashTask can clear, keep them from overlapping

	//computes a checksum which is XOR between the following
	static uint32_t FlashStateChecksum(const FlashStateRecord &record)
	{
		return record.magic ^ record.sector ^ record.bufferPerSector ^ 0xA5A55A5A;
	}

	static bool IsValidRecord(const FlashStateRecord &record)
	{
		return (record.magic == FLASH_STATE_MAGIC) && (record.checksum == FlashStateChecksum(record));
	}

	//finds the newest valid record in the state sector, also sets stateSlot to the first empty slot
	static bool ReadFlashState(FlashStateRecord &record)
	{
		FlashStateRecord page[STATE_RECORDS_PER_READ];
		bool found = false;
		uint32_t corrupt = 0;	//written slots that fail the magic / checksum check

		stateSlot = STATE_RECORDS_PER_SECTOR;	//full unless an empty slot is found
		stateSlotKnown = true;

		for (uint32_t base = 0; base < STATE_RECORDS_PER_SECTOR; base += STATE_RECORDS_PER_READ)
		{
			MX66xxQSPI_ReadSector(reinterpret_cast<uint8_t*>(page), FLASH_STATE_SECTOR, base * sizeof(FlashStateRecord), sizeof(page));

			for (uint32_t i = 0; i < STATE_RECORDS_PER_READ; ++i)
			{
				//records are appended in order, so the first erased slot is the end of the log
				if (page[i].magic == 0xFFFFFFFFU)
				{
					stateSlot = base + i;
					if (corrupt != 0)
					{
						SOAR_PRINT("LoggingService: skipped %lu corrupt cursor records in state sector (reset mid-save or bad write)\n",
								   (unsigned long)corrupt);
					}
					return found;
				}
				//a slot that is written but invalid (reset mid-save) is skipped
				if (IsValidRecord(page[i]))
				{
					record = page[i];
					found = true;
				}
				else
				{
					corrupt++;
				}
			}
		}

		if (corrupt != 0)
		{
			SOAR_PRINT("LoggingService: skipped %lu corrupt cursor records in state sector (reset mid-save or bad write)\n",
					   (unsigned long)corrupt);
		}
		return found;
	}

	//checks if a log chunk is empty (500 bytes) at a specific sector and chunk
	static bool IsLogChunkEmpty(uint16_t sector, uint8_t chunk)
	{
		//return false if not a writable sector of chunk
		if ((sector >= FLASH_DATA_LIMIT_SECTOR) || (chunk >= 8U))
		{
			return false;
		}

		//set the sector buffer to all 0xFF since 0xFF is erased memory for NOR flash
		memset(sectorBuf, 0xFF, sizeof(sectorBuf));
		MX66xxQSPI_ReadSector(sectorBuf, sector, static_cast<uint32_t>(chunk) * RAM_LOG_SIZE, RAM_LOG_SIZE);

		//check return false if any byte does not equal 0xFF (means chunk is not empty
		for (uint32_t i = 0; i < RAM_LOG_SIZE; ++i)
		{
			if (sectorBuf[i] != 0xFF)
			{
				return false;
			}
		}
		return true;
	}

	//CRC32 (reflected, 0xEDB88320), used for the flight state records
	static uint32_t Crc32(const uint8_t* data, uint32_t n)
	{
		uint32_t crc = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < n; ++i)
		{
			crc ^= data[i];
			for (uint8_t b = 0; b < 8; ++b)
			{
				crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
			}
		}
		return ~crc;
	}

	/* Flight state sector (altitude prediction filter): same append-only scheme as the logger cursor,
	 * 256 byte records. The newest valid record is the state, a reset mid-save loses at most that record
	 */
	constexpr uint32_t FLIGHT_STATE_MAGIC = 0x46535431;			//"FST1"
	constexpr uint32_t FLIGHT_STATE_SLOT_BYTES = 256;
	constexpr uint32_t FLIGHT_STATE_SLOTS = FS_SECTOR_SIZE / FLIGHT_STATE_SLOT_BYTES;
	struct FlightStateSlot
	{
		uint32_t magic;
		uint32_t sequence;
		FilterLog::PersistedState state;
		uint32_t crc;					//CRC32 of everything before it
	};
	static_assert(sizeof(FlightStateSlot) <= FLIGHT_STATE_SLOT_BYTES, "flight state record too big for its slot");

	static uint8_t flightSlotBuf[FLIGHT_STATE_SLOT_BYTES];
	static uint32_t flightSlot = 0;				//next empty slot
	static bool flightSlotKnown = false;
	static uint32_t flightSequence = 0;
	static FilterLog::PersistedState pendingFlightState{};
	static volatile bool flightSavePending = false;

	static bool IsValidFlightSlot(const FlightStateSlot& s)
	{
		return (s.magic == FLIGHT_STATE_MAGIC)
			&& (s.crc == Crc32(reinterpret_cast<const uint8_t*>(&s), offsetof(FlightStateSlot, crc)));
	}

	//finds the newest valid flight state, sets flightSlot to the first empty slot
	static bool ReadFlightState(FilterLog::PersistedState& state)
	{
		bool found = false;
		flightSlot = FLIGHT_STATE_SLOTS;
		flightSlotKnown = true;
		for (uint32_t i = 0; i < FLIGHT_STATE_SLOTS; ++i)
		{
			MX66xxQSPI_ReadSector(flightSlotBuf, FLIGHT_STATE_SECTOR, i * FLIGHT_STATE_SLOT_BYTES, FLIGHT_STATE_SLOT_BYTES);
			FlightStateSlot s;
			memcpy(&s, flightSlotBuf, sizeof(s));
			if (s.magic == 0xFFFFFFFFU)
			{
				flightSlot = i;
				break;
			}
			if (IsValidFlightSlot(s) && (!found || s.sequence >= flightSequence))
			{
				state = s.state;
				flightSequence = s.sequence;
				found = true;
			}
		}
		return found;
	}

	//appends one record, erases the sector first when it is full. Call with flashLock held
	static bool WriteFlightState(const FilterLog::PersistedState& state)
	{
		if (!flightSlotKnown)
		{
			FilterLog::PersistedState unused{};
			ReadFlightState(unused);
		}
		if (flightSlot >= FLIGHT_STATE_SLOTS)
		{
			MX66xxQSPI_EraseSector(FLIGHT_STATE_SECTOR);
			flightSlot = 0;
		}
		FlightStateSlot s{};
		s.magic = FLIGHT_STATE_MAGIC;
		s.sequence = ++flightSequence;
		s.state = state;
		s.crc = Crc32(reinterpret_cast<const uint8_t*>(&s), offsetof(FlightStateSlot, crc));
		memset(flightSlotBuf, 0, sizeof(flightSlotBuf));
		memcpy(flightSlotBuf, &s, sizeof(s));
		const uint32_t offset = flightSlot * FLIGHT_STATE_SLOT_BYTES;
		MX66xxQSPI_WriteSector(flightSlotBuf, FLIGHT_STATE_SECTOR, offset, FLIGHT_STATE_SLOT_BYTES);
		flightSlot++;
		//read back so a failed save is reported now, not after the next power loss
		FlightStateSlot check;
		MX66xxQSPI_ReadSector(flightSlotBuf, FLIGHT_STATE_SECTOR, offset, FLIGHT_STATE_SLOT_BYTES);
		memcpy(&check, flightSlotBuf, sizeof(check));
		if (!IsValidFlightSlot(check) || check.sequence != s.sequence)
		{
			SOAR_PRINT("LoggingService: flight state save FAILED (slot %lu)\n", (unsigned long)(flightSlot - 1));
			return false;
		}
		return true;
	}
}

bool LoggingService::LoadFlightState(FilterLog::PersistedState& state)
{
	if (!FlashTask::Inst().IsFlashInitialized())
	{
		return false;
	}
	flashLock.Lock();
	const bool found = ReadFlightState(state);
	flashLock.Unlock();
	return found;
}

void LoggingService::RequestFlightStateSave(const FilterLog::PersistedState& state)
{
	taskENTER_CRITICAL();
	pendingFlightState = state;
	flightSavePending = true;
	taskEXIT_CRITICAL();
}

void LoggingService::ServiceFlightState()
{
	if (!flightSavePending || !FlashTask::Inst().IsFlashInitialized())
	{
		return;
	}
	FilterLog::PersistedState state;
	taskENTER_CRITICAL();
	state = pendingFlightState;
	flightSavePending = false;
	taskEXIT_CRITICAL();
	flashLock.Lock();
	WriteFlightState(state);
	flashLock.Unlock();
}

bool LoggingService::SaveFlightStateNow(const FilterLog::PersistedState& state)
{
	if (!FlashTask::Inst().IsFlashInitialized())
	{
		return false;
	}
	flashLock.Lock();
	const bool ok = WriteFlightState(state);
	flashLock.Unlock();
	return ok;
}

//Constructor for logging service
LoggingService::LoggingService(LoggingDest dest, LoggingData dataType, uint8_t* ldata, uint32_t dataSize, LoggingPriority priority)
{
	loggingData.dest = dest;
	loggingData.dataType = dataType;
	loggingData.data = ldata;
	loggingData.dataSize = dataSize;
	loggingData.priority = priority;


}

//Saves the flash state by appending a record to the flash state sector
void LoggingService::SaveFlashState()
{
	//the next empty record slot is only known after the state sector has been read once
	if (!stateSlotKnown)
	{
		FlashStateRecord unused{};
		ReadFlashState(unused);
	}

	//creates a FlashStateRecord with the current flash state/cursor
	FlashStateRecord record{};
	record.magic = FLASH_STATE_MAGIC;
	record.sector = sectorAddress;
	record.bufferPerSector = bufferPerSector;
	record.checksum = FlashStateChecksum(record);

	//all record slots used, start the log over
	if (stateSlot >= STATE_RECORDS_PER_SECTOR)
	{
		MX66xxQSPI_EraseSector(FLASH_STATE_SECTOR);
		stateSlot = 0;
	}

	const uint32_t offset = stateSlot * sizeof(FlashStateRecord);
	MX66xxQSPI_WriteSector(reinterpret_cast<uint8_t*>(&record), FLASH_STATE_SECTOR, offset, sizeof(record));
	stateSlot++;

	//read the record back so a failed save shows up now instead of on the next boot
	FlashStateRecord check{};
	MX66xxQSPI_ReadSector(reinterpret_cast<uint8_t*>(&check), FLASH_STATE_SECTOR, offset, sizeof(check));
	if (!IsValidRecord(check) || check.sector != record.sector || check.bufferPerSector != record.bufferPerSector)
	{
		const char* reason = (check.magic == 0xFFFFFFFFU) ? "record slot still blank, write did not happen"
						   : (check.magic != FLASH_STATE_MAGIC) ? "bad magic, record slot was not blank or data corrupted"
						   : (check.checksum != FlashStateChecksum(check)) ? "checksum mismatch, data corrupted"
						   : "cursor values differ from what was written";
		SOAR_PRINT("LoggingService: flash state save FAILED at record %lu: %s\n",
				   (unsigned long)(stateSlot - 1), reason);
		SOAR_PRINT("  read back magic=0x%08lX sector=%lu slot=%lu (wrote sector=%lu slot=%lu), cursor may be lost on reset\n",
				   (unsigned long)check.magic,
				   (unsigned long)check.sector,
				   (unsigned long)check.bufferPerSector,
				   (unsigned long)record.sector,
				   (unsigned long)record.bufferPerSector);
		const uint8_t* wrote = reinterpret_cast<const uint8_t*>(&record);
		const uint8_t* read = reinterpret_cast<const uint8_t*>(&check);
		uint32_t bad = 0, firstBad = 0;
		for (uint32_t i = 0; i < sizeof(record); i++)
		{
			if (wrote[i] != read[i])
			{
				if (bad == 0) { firstBad = i; }
				bad++;
			}
		}
		PrintFlashFailDetails(bad, sizeof(record), firstBad, wrote[firstBad], read[firstBad]);
	}
}

//moves the write cursor to the next chunk, saving the cursor every time a sector is finished
void LoggingService::AdvanceSlot()
{
	/* Each buffer is 500 bytes, 8 can fit in 4000 bytes, when a sector is full
	 * (bufferPerSector == 8) go to the next sector
	 */
	bufferPerSector++;
	if (bufferPerSector == 8)
	{
		sectorAddress++;
		bufferPerSector = 0;

		//finish logging if the sectorAddress reaches address that does not exist
		if (sectorAddress >= FLASH_DATA_LIMIT_SECTOR)
		{
			sectorAddress = FLASH_STATE_SECTOR;
			done = true;
		}
		SaveFlashState(); //save flash state after every sector write
	}
}

/* The cursor is saved once per finished sector, so after a reset the chunks after the saved
 * slot in that sector may already be written. Step past them, at most 7 chunk reads and never
 * outside the current sector (chunk 0 of the next sector is erased before it is written).
 */
void LoggingService::SkipUsedSlots()
{
	while (!done && (bufferPerSector != 0) && !IsLogChunkEmpty(sectorAddress, bufferPerSector))
	{
		SOAR_PRINT("LoggingService: sector=%u slot=%u already written, skipping\n",
				   (unsigned int)sectorAddress,
				   (unsigned int)bufferPerSector);
		AdvanceSlot();
	}
}

void LoggingService::LoadFlashStateFromStorage()
{
	//if the flash state has been loaded since program start up, do nothing
	if (flashStateLoaded)
	{
		return;
	}
	flashStateLoaded = true;

	FlashStateRecord record{};
	if (ReadFlashState(record))
	{
		if (record.sector >= FLASH_DATA_LIMIT_SECTOR)
		{
			//the saved cursor marks the flash as full, stay stopped until a flash clear
			sectorAddress = FLASH_STATE_SECTOR;
			bufferPerSector = 0;
			done = true;
			SOAR_PRINT("LoggingService: saved flash cursor says flash is full, run flash_clear to log again\n");
			return;
		}

		if (record.bufferPerSector < 8U)
		{
			//sets the saved state sectorAddress and bufferPerSector, this preserves the write cursor in between program runs
			sectorAddress = static_cast<uint16_t>(record.sector);
			bufferPerSector = static_cast<uint8_t>(record.bufferPerSector);
			done = false;
			SOAR_PRINT("LoggingService: resuming from saved flash cursor sector=%u slot=%u\n",
					   (unsigned int)sectorAddress,
					   (unsigned int)bufferPerSector);
			SkipUsedSlots();
			return;
		}

		SOAR_PRINT("LoggingService: saved flash cursor is invalid (sector=%lu slot=%lu, slot must be < 8), starting over\n",
				   (unsigned long)record.sector, (unsigned long)record.bufferPerSector);
	}

	//no usable cursor saved (first boot or after the state sector was wiped), start logging from the beginning
	SOAR_PRINT("LoggingService: no saved flash cursor, starting at sector 0\n");
	sectorAddress = 0;
	bufferPerSector = 0;
	done = false;
	SaveFlashState();
}

LoggingStatus LoggingService::LogData(){

	LoggingStatus err;

	switch(loggingData.dest){
	case LoggingDest::RAM:
		//internal flash api
		err = LogToInternalMemory();

		break;

	case LoggingDest::FLASH_EXTERN:
		err = LogToMX66();
		break;

	case LoggingDest::FILE_SYSTEM:
		//TODO FS write api
		SOAR_PRINT("LoggingService::LogData FAIL: %s packet sent to FILE_SYSTEM, which is not implemented\n",
				   SensorTypeName(loggingData.dataType));
		err = LoggingStatus::LOGGING_ERR;
		break;

	case LoggingDest::DMA:

		//TODO DMA write api
		SOAR_PRINT("LoggingService::LogData FAIL: %s packet sent to DMA, which is not implemented\n",
				   SensorTypeName(loggingData.dataType));
		err = LoggingStatus::LOGGING_ERR;
		break;

	default:
		SOAR_PRINT("LoggingService::LogData FAIL: %s packet has unknown destination %d\n",
				   SensorTypeName(loggingData.dataType), (int)loggingData.dest);
		err = LoggingStatus::LOGGING_ERR;
		break;
	}

	return err;
}


LoggingStatus LoggingService::LogToMX66(){

	//polling starts at boot now, so data can arrive before FlashTask has reset the chip and set QPI / 4-byte mode.
	//touching the flash before that runs commands in the wrong mode, so drop data until the flash is ready
	static uint32_t droppedBeforeInit = 0;	//packets dropped while waiting for FlashTask to set up the chip
	if (!FlashTask::Inst().IsFlashInitialized()){
		if(droppedBeforeInit == 0){
			SOAR_PRINT("FLASH LOG: flash not initialized yet, dropping data until FlashTask finishes init\n");
		}
		droppedBeforeInit++;
		return LoggingStatus::LOG_FLASH_NOT_READY;
	}
	if(droppedBeforeInit != 0){
		SOAR_PRINT("FLASH LOG: flash ready, %lu packets were dropped before init\n", (unsigned long)droppedBeforeInit);
		droppedBeforeInit = 0;
	}

	//load flash state from storage (this will only actually do something the first time this function is run)
	//we do not want state to be loaded more than once in the lifetime of this code
	LoadFlashStateFromStorage();

	//appends data to buffer and returns flag determining if buffer is full (full => write data)
	LoggingStatus status = MemAppend(&loggingData);

	//MemAppend already printed why
	if(status == LoggingStatus::LOGGING_ERR){
		return LoggingStatus::LOGGING_ERR;
	}

	//stops logging, triggered by flash dump, or flashchip is full
	static bool stopReported = false;	//print the reason once instead of on every packet
	if(done){
		if(!stopReported){
			SOAR_PRINT("FLASH LOG: logging stopped (%s)\n",
					   (sectorAddress == FLASH_STATE_SECTOR) ? "flash full" : "flash dump / StopLogging");
			stopReported = true;
		}
		return LoggingStatus::FLASH_FULL;
	}
	stopReported = false;

	//checks if 500 byte chunck is full (writes when it is full)
	if(status != LoggingStatus::LOG_FLASH_READY){
		return LoggingStatus::LOG_FLASH_NOT_READY;
	}

	flashLock.Lock();

	//chunk 0 of a sector erases the sector, other chunks should still be empty unless the cursor is behind
	if(bufferPerSector != 0){
		SkipUsedSlots();
	}
	if(done){
		flashLock.Unlock();
		return LoggingStatus::FLASH_FULL;
	}
	if(bufferPerSector == 0){
		//if a new sector is reached erase before writing
		MX66xxQSPI_EraseSector(sectorAddress);
	}

	const uint32_t slotOffset = bufferPerSector * RAM_LOG_SIZE;

	//the slot must be blank (all 0xFF) before programming, flash can only clear bits
	MX66xxQSPI_ReadSector(sectorBuf, sectorAddress, slotOffset, RAM_LOG_SIZE);
	uint32_t notBlank = 0;
	uint32_t firstNotBlank = 0;
	for(uint32_t i = 0; i < RAM_LOG_SIZE; i++){
		if(sectorBuf[i] != 0xFF){
			if(notBlank == 0){ firstNotBlank = i; }
			notBlank++;
		}
	}
	if(notBlank != 0){
		SOAR_PRINT("FLASH LOG FAIL sector=%u slot=%u: slot not blank before write (%s)\n",
				   (unsigned int)sectorAddress, (unsigned int)bufferPerSector,
				   (bufferPerSector == 0) ? "sector erase failed" : "slot already written");
		PrintFlashFailDetails(notBlank, RAM_LOG_SIZE, firstNotBlank, 0xFF, sectorBuf[firstNotBlank]);
		AdvanceSlot();
		flashLock.Unlock();
		return LoggingStatus::LOGGING_ERR;
	}

	//Write data to bufferPerSector * 500 this calculates the offset at which the buffer will be written
	memcpy(txBuf, ramLog, RAM_LOG_SIZE);

	MX66xxQSPI_WriteSector(txBuf, sectorAddress, slotOffset,  RAM_LOG_SIZE);

	MX66xxQSPI_ReadSector(sectorBuf, sectorAddress, slotOffset,  RAM_LOG_SIZE);

	//check if the bytes written equal the bytes read
	bool written = BytesEqual(sectorBuf, txBuf, RAM_LOG_SIZE);

	if(!written){
		//work out how the read back differs from what was written
		uint32_t bad = 0, firstBad = 0;
		uint32_t needed1Got0 = 0;	//bytes with bits read 0 that should be 1 (slot was not blank / corrupted)
		uint32_t needed0Got1 = 0;	//bytes with bits read 1 that should be 0 (program did not finish)
		bool allFF = true;
		for(uint32_t i = 0; i < RAM_LOG_SIZE; i++){
			if(sectorBuf[i] != 0xFF){ allFF = false; }
			if(sectorBuf[i] != txBuf[i]){
				if(bad == 0){ firstBad = i; }
				bad++;
				if(txBuf[i] & ~sectorBuf[i]){ needed1Got0++; }
				if(sectorBuf[i] & ~txBuf[i]){ needed0Got1++; }
			}
		}
		const uint8_t badRead = sectorBuf[firstBad];

		//read a second time, if it matches now the data on flash is fine and the first read was bad
		MX66xxQSPI_ReadSector(sectorBuf, sectorAddress, slotOffset, RAM_LOG_SIZE);
		const bool rereadOk = BytesEqual(sectorBuf, txBuf, RAM_LOG_SIZE);

		const char* reason;
		if(rereadOk){
			reason = "read back glitch, data OK on 2nd read (QSPI read timing/noise)";
		} else if(allFF){
			reason = "nothing was programmed (write did not happen)";
		} else if(needed0Got1 != 0 && needed1Got0 == 0){
			reason = "program incomplete (some bits still 1)";
		} else if(needed1Got0 != 0 && needed0Got1 == 0){
			reason = "bits read 0 that should be 1 (slot not blank / corrupted)";
		} else {
			reason = "data corrupted (mixed bit errors)";
		}

		SOAR_PRINT("FLASH LOG %s sector=%u slot=%u: %s\n",
				   rereadOk ? "WARN" : "FAIL",
				   (unsigned int)sectorAddress, (unsigned int)bufferPerSector, reason);
		PrintFlashFailDetails(bad, RAM_LOG_SIZE, firstBad, txBuf[firstBad], badRead);

		//the chunk is on flash correctly, only the verify read was wrong
		written = rereadOk;
	}

	//move on either way, a chunk that failed read back is left behind instead of being retried
	AdvanceSlot();

	flashLock.Unlock();

	return written ? LoggingStatus::LOGGING_SUCCESS : LoggingStatus::LOGGING_ERR;
}

//second line of a flash log failure: which byte failed and the chip / QSPI status at that moment
void LoggingService::PrintFlashFailDetails(uint32_t badBytes, uint32_t totalBytes, uint32_t firstBad, uint8_t expected, uint8_t got){
	const uint8_t sr = MX66xxQSPI_ReadStatusRegister();
	const uint8_t scur = MX66xxQSPI_ReadSecurityRegister();

	//totalBytes == 0 means there is no byte comparison, only the chip status
	if(totalBytes != 0){
		SOAR_PRINT("  bad=%u/%u first@%u expected=0x%02X read=0x%02X\n",
				   (unsigned int)badBytes, (unsigned int)totalBytes, (unsigned int)firstBad,
				   (unsigned int)expected, (unsigned int)got);
	}
	SOAR_PRINT("  SR=0x%02X%s%s SCUR=0x%02X%s%s qspiErr=0x%lX\n",
			   (unsigned int)sr,
			   (sr & 0x01) ? " BUSY" : "",				//WIP, chip still busy with the last program / erase
			   (sr & 0x3C) ? " WRITE_PROTECTED" : "",	//block protect bits set
			   (unsigned int)scur,
			   (scur & 0x20) ? " P_FAIL" : "",			//chip reported the last program failed
			   (scur & 0x40) ? " E_FAIL" : "",			//chip reported the last erase failed
			   (unsigned long)hqspi.ErrorCode);
}

//Sensor type enum to string conversion function (used for flash dump printing)
const char* SensorTypeName(LoggingData type)
{
    switch(type)
    {
        case LoggingData::IMU32G: return "IMU32G";
        case LoggingData::IMU16G: return "IMU16G";
        case LoggingData::MAG:    return "MAG";
        case LoggingData::BARO07: return "BARO07";
        case LoggingData::BARO11: return "BARO11";
        case LoggingData::GPS:    return "GPS";
        case LoggingData::FILTER: return "FILTER";
        case LoggingData::FILTER2: return "FILTER2";
        case LoggingData::EVENT:  return "EVENT";

        default: return "UNKNOWN";
    }
}

void LoggingService::ProcessFlashDump()
{
	LoadFlashStateFromStorage();	//ensures updated flash state is loaded from the storage

	const uint8_t wasDone = done;  //saves done flag since it gets set to true regardless because logging must stop during flash dump
	done = true;
	doneDump = false;
	dumpSector = 0;		//start dumpSector at 0
	dumpOffset = 0;		//start dumpOffset (chunk) at 0

	constexpr uint32_t RECORD_SIZE = 20;	//size of each data record
	constexpr uint32_t CHUNK_SIZE  = 500;	//size of each chunk

	//variables for gps read back
	char gpsSentence[256] = {0};

	//records that could not be printed, reported at the end
	uint32_t blankRecords = 0;		//type byte 0xFF, part of a chunk that was never written (failed write left behind)
	uint32_t unknownRecords = 0;	//type byte is not a LoggingData value (corrupted data)
	uint16_t gpsSentenceLen = 0;
	uint32_t gpsTimestamp = 0;

	// Snapshot the logged extent so we don't walk the entire flash device.
	uint32_t maxSector = sectorAddress;			//sectorAddress cursor is at (from writes)
	uint16_t maxOffset = static_cast<uint16_t>(bufferPerSector * RAM_LOG_SIZE);		//offset the cursor is at

	//sets the max offset and max sector to the max if the flash is full
	if (sectorAddress == FLASH_STATE_SECTOR)
	{
		maxSector = FLASH_DATA_LIMIT_SECTOR - 1;
		maxOffset = SECTOR_READ;
	}

	//if the offset is 0 then the read goes back one sector and sets the maxOffset to the end of the sector
	if (maxOffset == 0)
	{
		if (maxSector == 0)
		{
			//if the max sector is 0 there is nothing to dump, return
			SOAR_PRINT("------FLASH DUMP EMPTY------\n");
			done = wasDone;
			return;
		}
		maxSector -= 1;
		maxOffset = SECTOR_READ;
	}

	//legend for the minimal dump format: type[data] timestamp
	SOAR_PRINT("0=%s 1=%s 2=%s 3=%s 4=%s 5=%s\n",
			   SensorTypeName(LoggingData::IMU32G), SensorTypeName(LoggingData::IMU16G),
			   SensorTypeName(LoggingData::MAG), SensorTypeName(LoggingData::BARO07),
			   SensorTypeName(LoggingData::BARO11), SensorTypeName(LoggingData::GPS));

	/*
	 * this loops through the flash chip while the dumpSector is less than the max sector or if the max sector is
	 * reached it reads back up to the max offset of the sector. Done dump flag as well must be false to continue
	 * flash dump, if it is true it will stop dump.
	 */
	while ((dumpSector < maxSector || (dumpSector == maxSector && dumpOffset < maxOffset)) && !doneDump) {

		memset(sectorBuf, 0, sizeof(sectorBuf));
		MX66xxQSPI_ReadSector(sectorBuf, dumpSector, dumpOffset, CHUNK_SIZE);

		for (uint32_t i = 0; i + RECORD_SIZE <= CHUNK_SIZE; i += RECORD_SIZE) //each record is 20 bytes, Chunck size is 500 to prevent hardfault
		{
			LoggingData type = static_cast<LoggingData>(sectorBuf[i]); //get type byte first
			uint32_t timestamp;
			memcpy(&timestamp, sectorBuf + i + 1, sizeof(timestamp));

			//parse the data back into structs

			//parse imu and print
			if(type == LoggingData::IMU16G || type == LoggingData::IMU32G)
			{
				int16_t accel[3], gyro[3], temp;

				memcpy(accel, sectorBuf + i + 5, sizeof(accel));
				memcpy(gyro, sectorBuf + i + 11, sizeof(gyro));
				memcpy(&temp, sectorBuf + i + 17, sizeof(temp));

				//minimal format: type[accel xyz,gyro xyz,temp] timestamp
				SOAR_PRINT("%u[%d,%d,%d,%d,%d,%d,%d] %lu\n",
						   (unsigned int)type,
						   accel[0], accel[1], accel[2],
						   gyro[0], gyro[1], gyro[2],
						   temp, (unsigned long)timestamp);
			}
			//parse baro and print
			else if(type == LoggingData::BARO07 || type == LoggingData::BARO11)
			{
				int32_t pressure;
				int16_t temperature;
				memcpy(&pressure, sectorBuf + i + 5, sizeof(pressure));
				memcpy(&temperature, sectorBuf + i + 9, sizeof(temperature));

				//minimal format: type[pressure Pa,temp in 0.01 C] timestamp
				SOAR_PRINT("%u[%ld,%d] %lu\n",
						   (unsigned int)type, (long)pressure, temperature, (unsigned long)timestamp);

			}
			//parse mag and print
			else if (type == LoggingData::MAG){

				int32_t magX, magY, magZ;

				memcpy(&magX, sectorBuf + i + 5,  sizeof(int32_t));
				memcpy(&magY, sectorBuf + i + 9,  sizeof(int32_t));
				memcpy(&magZ, sectorBuf + i + 13, sizeof(int32_t));

				//minimal format: type[x,y,z] timestamp
				SOAR_PRINT("%u[%ld,%ld,%ld] %lu\n",
						   (unsigned int)type, (long)magX, (long)magY, (long)magZ, (unsigned long)timestamp);

			}
			//parse gps and print
			else if (type == LoggingData::GPS)
			{
				//get the chunk index, count, and payload length
				uint8_t chunkIdx = sectorBuf[i + 5];
				uint8_t chunkCount = sectorBuf[i + 6];
				uint8_t payloadLen = sectorBuf[i + 7];

				//clamp payload length at 12
				if (payloadLen > 12)
				{
					payloadLen = 12;
				}

				//if the chunk index is 0 set the sentence buffer to 0,
				//set the length to 0, set the chunks and time stamp tp read back values
				if (chunkIdx == 0)
				{
					memset(gpsSentence, 0, sizeof(gpsSentence));
					gpsSentenceLen = 0;
					gpsTimestamp = timestamp;
				}

				//if payload length is greater than 0, and the sentence length plus the payload length is less than the
				//size of the sentence then the appropriate bytes from the sector buffer get added into the sentence
				if ((payloadLen > 0U) && (gpsSentenceLen + payloadLen < sizeof(gpsSentence)))
				{
					memcpy(gpsSentence + gpsSentenceLen, sectorBuf + i + 8, payloadLen);
					gpsSentenceLen = static_cast<uint16_t>(gpsSentenceLen + payloadLen);
					gpsSentence[gpsSentenceLen] = '\0';
				}

				//If the last chunk is detected the sentence can br printed to the serial terminal
				if ((chunkCount > 0U) && (chunkIdx + 1U >= chunkCount))
				{
					//minimal format: type[sentence] timestamp
					SOAR_PRINT("%u[%s] %lu\n",
							   (unsigned int)type, gpsSentence, (unsigned long)gpsTimestamp);
				}
			}
			//parse altitude prediction filter output and print (layout: AltitudePredictionFilter/Output/Inc/FilterLog.hpp)
			else if (type == LoggingData::FILTER)
			{
				const FilterData d = FilterLog::Unpack(sectorBuf + i + 5, timestamp);
				SOAR_PRINT("FILTER Timestamp=%lu AltCm=%ld VelCms=%ld AccCms2=%ld SdCm=%ld ApogeeM=%ld EventDs=%ld State=%u Flags=%u\n",
						   (unsigned long)timestamp, (long)FilterLog::Round(d.altitude * 100.0f),
						   (long)FilterLog::Round(d.velocity * 100.0f), (long)FilterLog::Round(d.acceleration * 100.0f),
						   (long)FilterLog::Round(d.altitudeStd * 100.0f), (long)FilterLog::Round(d.predictedApogee),
						   (long)FilterLog::Round(d.timeToEvent * 10.0f), (unsigned int)d.apfState, (unsigned int)d.flags);
			}
			//filter diagnostics (layout: AltitudePredictionFilter/Output/Inc/FilterLog.hpp)
			else if (type == LoggingData::FILTER2)
			{
				const uint8_t* p = sectorBuf + i + 5;
				uint16_t velSd, sigma;
				int16_t inn;
				memcpy(&velSd, p + 0, 2);
				memcpy(&inn, p + 5, 2);
				memcpy(&sigma, p + 12, 2);
				SOAR_PRINT("FILTER2 Timestamp=%lu VelSdCms=%u CaseA=%d CaseB=%d WeightA=%u InnovCm=%d KAlt=%u KVel=%u KAcc=%u Baro0W=%u Baro1W=%u SigmaCm=%u\n",
						   (unsigned long)timestamp, (unsigned int)velSd, (int)(int8_t)p[2], (int)(int8_t)p[3],
						   (unsigned int)p[4], (int)inn, (unsigned int)p[7], (unsigned int)p[8], (unsigned int)p[9],
						   (unsigned int)p[10], (unsigned int)p[11], (unsigned int)sigma);
			}
			//filter flight event
			else if (type == LoggingData::EVENT)
			{
				int32_t value;
				memcpy(&value, sectorBuf + i + 6, sizeof(value));
				SOAR_PRINT("EVENT Timestamp=%lu Code=%u Value=%ld State=%u\n",
						   (unsigned long)timestamp, (unsigned int)sectorBuf[i + 5], (long)value, (unsigned int)sectorBuf[i + 10]);
			}
			else if (sectorBuf[i] == 0xFF)
			{
				blankRecords++;
			}
			else
			{
				unknownRecords++;
			}

		}
		/*the dump offset at this point should be 3500, this means sector is full
		 * (writtern up to 4000 bytes, if this is true set the offset back to 0 and
		 * move to the next sector, otherwise increment dumpoffset to the next chunk
		 */
		if (dumpOffset == SECTOR_READ - CHUNK_SIZE) {
			dumpOffset = 0;
			dumpSector++;
		} else {
			dumpOffset += 500;
		}

		// Let lower-priority UART/debug tasks run while the dump is in progress.
		osDelay(1);
	}

	if (blankRecords != 0 || unknownRecords != 0)
	{
		SOAR_PRINT("FLASH DUMP: skipped %lu blank records (unwritten chunks) and %lu records with unknown type (corrupted)\n",
				   (unsigned long)blankRecords, (unsigned long)unknownRecords);
	}

	if (doneDump)
	{
		SOAR_PRINT("------FLASH DUMP STOPPED------\n");
	}
	else
	{
		SOAR_PRINT("------FLASH DUMP COMPLETE------\n");
	}

	done = wasDone;
}

//sets done dump flag (used to stop from debug task)
//lets other users of the flash chip (flash_test) keep log writes off the QSPI bus
void LoggingService::LockFlash(){
	flashLock.Lock();
}
void LoggingService::UnlockFlash(){
	flashLock.Unlock();
}

void LoggingService::StopDump(){
	doneDump = true;
}
//starts logging (this is done to ensure logging continues with state changes)
void LoggingService::StartLogging(){
	done = false;
}
//ensures logging is stopped (this is done to ensure logging stops with appropriate state changes)
void LoggingService::StopLogging(){
	flashLock.Lock();
	done = true;
	SaveFlashState(); //saves the flash state at which the logging stopped
	flashLock.Unlock();
}


LoggingStatus LoggingService::LogToInternalMemory(){

	//MemAppend prints the reason if the packet has no data
	return MemAppend(&loggingData);

}

//Verifies if log read-back is successful by checking if the bytes written equal to the bytes read
bool LoggingService::BytesEqual(const uint8_t* a, const uint8_t* b, uint32_t n){

	for(uint32_t i = 0; i < n; i++){

		if(a[i] != b[i]){
			return false;
		}

	}
	return true;
}

/*
 * Appends memory to the head of the buffer that gets written to flash.
 * This returns a status that determines whether the buffer is full and
 * is ready to be written to the flash chip
 */
LoggingStatus LoggingService::MemAppend(const LoggingPacket *data){

	if(!data){
		SOAR_PRINT("LoggingService::MemAppend FAIL: packet pointer is null\n");
		return LoggingStatus::LOGGING_ERR;
	}

	if(data->data == nullptr || data->dataSize == 0){
		SOAR_PRINT("LoggingService::MemAppend FAIL: %s packet has no data (data=%s size=%u)\n",
				   SensorTypeName(data->dataType),
				   (data->data == nullptr) ? "null" : "ok",
				   (unsigned int)data->dataSize);
		return LoggingStatus::LOGGING_ERR;
	}

	uint32_t size = data->dataSize;

	if (size > MAX_LOG_SIZE){
		SOAR_PRINT("LoggingService::MemAppend WARN: %s packet is %u bytes, truncated to %u\n",
				   SensorTypeName(data->dataType), (unsigned int)size, (unsigned int)MAX_LOG_SIZE);
		size = MAX_LOG_SIZE;
	}

	for(uint32_t i = 0; i < size; i++){

		ramLog[ramHead++] = data->data[i];
	}

	for(uint32_t i = size; i < MAX_LOG_SIZE; i++){
		ramLog[ramHead++] = 0;
	}

	//store the packet first, then report full when the next one will not fit
	//(used to return full before storing, which dropped one packet per chunk)
	if(ramHead + MAX_LOG_SIZE > RAM_LOG_SIZE){
		ramHead = 0;

		return LoggingStatus::LOG_FLASH_READY;
	}

	return LoggingStatus::LOG_FLASH_NOT_READY;

}

void LoggingService::FlashClear(){
	flashLock.Lock();

	//clear flash chip bounds, sector 0 for the new data and the state sector for the cursor log
	MX66xxQSPI_EraseSector(0);
	if (!MX66xxQSPI_IsEmptySector(0, 0, FS_SECTOR_SIZE))
	{
		SOAR_PRINT("Flash clear FAIL: sector 0 not blank after erase, first log chunk will fail its blank check\n");
		PrintFlashFailDetails(0, 0, 0, 0xFF, 0);
	}
	MX66xxQSPI_EraseSector(FLASH_STATE_SECTOR);
	if (!MX66xxQSPI_IsEmptySector(FLASH_STATE_SECTOR, 0, FS_SECTOR_SIZE))
	{
		SOAR_PRINT("Flash clear FAIL: state sector %u not blank after erase, old cursor records may remain\n",
				   (unsigned int)FLASH_STATE_SECTOR);
		PrintFlashFailDetails(0, 0, 0, 0xFF, 0);
	}
	stateSlot = 0;
	stateSlotKnown = true;

	//reset flash cursor and all necessary flash state variables, no need to load the old cursor
	ramHead = 0;
	sectorAddress = 0;
	bufferPerSector = 0;
	flashStateLoaded = true;
	done = 0;
	doneDump = 0;

	//save the flash state
	SaveFlashState();
	flashLock.Unlock();

	SOAR_PRINT("Flash clear: logging cursor reset\n");
}
