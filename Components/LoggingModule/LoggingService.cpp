/*
 * LoggingService.cpp
 *
 *  Created on: Jan 21, 2026
 *      Author: jaddina
 */
#include "LoggingService.hpp"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

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
	constexpr uint16_t FLASH_DATA_LIMIT_SECTOR = FLASH_TEST_SECTOR;	//The sector limit at which data can be written to in flash
	constexpr uint32_t FLASH_STATE_MAGIC = 0x4C4F4731;				//Recognizable constant to check if saved flash state is valid

	//16 byte flash state record for saving flash state between program runs
	struct FlashStateRecord
	{
		uint32_t magic;					// validates that this is actually a logger state record
		uint32_t sector;				// next sector to write to
		uint32_t bufferPerSector;		// next chunk inside that sector
		uint32_t checksum;				// simple corruption check
	};

	//computes a checksum which is XOR between the following
	static uint32_t FlashStateChecksum(const FlashStateRecord &record)
	{
		return record.magic ^ record.sector ^ record.bufferPerSector ^ 0xA5A55A5A;
	}

	//takes a reference to a FlashStateRecotd and fills it with the record from the flash
	static bool ReadFlashState(FlashStateRecord &record)
	{
		//creates raw read buffer and reads into it the data from the FLASH_STATE_SECTOR
		uint8_t raw[sizeof(FlashStateRecord)] = {0};
		MX66xxQSPI_ReadSector(raw, FLASH_STATE_SECTOR, 0, sizeof(raw));
		memcpy(&record, raw, sizeof(record)); 				//parses the data into the record struct

		//sends the magic and checksum result to indicate successful state readback
		return (record.magic == FLASH_STATE_MAGIC) && (record.checksum == FlashStateChecksum(record));
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

	//given a start sector and a start chunk this function finds the next available slot to write a buffer of data to
	static bool FindNextWritableSlot(uint16_t startSector, uint8_t startChunk, uint16_t &sector, uint8_t &chunk)
	{
		/*iterates through each chunk in a sector, then repeats for every sector sequentially in
		 * the falsh chip. The first empty chunk is the next writable slot for the flash chip
		 * sector and chunk are passed by reference, so they are set to the next writable slot
		 */
		for (uint16_t s = startSector; s < FLASH_DATA_LIMIT_SECTOR; ++s)
		{
			const uint8_t firstChunk = (s == startSector) ? startChunk : 0U;
			for (uint8_t c = firstChunk; c < 8U; ++c)
			{
				if (IsLogChunkEmpty(s, c))
				{
					sector = s;
					chunk = c;
					return true;
				}
			}
		}

		//returns false if the flash chip is full up to the data limit sector
		return false;
	}
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

//Saves the flash state to the flash state sector
void LoggingService::SaveFlashState()
{
	//creates a FlashStateRecord with the current flash state/cursor
	FlashStateRecord record{};
	record.magic = FLASH_STATE_MAGIC;
	record.sector = sectorAddress;
	record.bufferPerSector = bufferPerSector;
	record.checksum = FlashStateChecksum(record);

	//fills the buffer with record bytes
	uint8_t raw[sizeof(record)] = {0};
	memcpy(raw, &record, sizeof(record));

	//writes the buffer to the flash state sector (the last sector on the chip always)
	MX66xxQSPI_EraseSector(FLASH_STATE_SECTOR);
	MX66xxQSPI_WriteSector(raw, FLASH_STATE_SECTOR, 0, sizeof(raw));	//writes at sector offset 0

	/*
	 * NOTE: the flash state sector always contains the last known write cursor
	 */
}

void LoggingService::LoadFlashStateFromStorage()
{
	//if the flash state has been loaded since program start up, do nothing
	if (flashStateLoaded)
	{
		return;
	}

	//tries to read the flash state if false, the read state was either corrupted, or failed to read
	FlashStateRecord record{};
	if (ReadFlashState(record) && record.sector < FLASH_DATA_LIMIT_SECTOR && record.bufferPerSector < 8U)
	{
		//sets the saved state sectorAddress and bufferPerSector, this preserves the write cursor in between program runs
		sectorAddress = static_cast<uint16_t>(record.sector);
		bufferPerSector = static_cast<uint8_t>(record.bufferPerSector);

		//checks if the log chunk at which the state was saved is empty
		if (!IsLogChunkEmpty(sectorAddress, bufferPerSector))
		{
			SOAR_PRINT("LoggingService: saved flash cursor sector=%u slot=%u is occupied, scanning forward\n",
					   (unsigned int)sectorAddress,
					   (unsigned int)bufferPerSector);
			/*if the flash chip is full set the cursor to the end of the chip, set done = 1 indidcating
			 * that the flash chip is full, then as well save the state since the cursor moved from the last
			 * saved state
			 */
			if (!FindNextWritableSlot(sectorAddress, bufferPerSector, sectorAddress, bufferPerSector))
			{
				sectorAddress = FLASH_STATE_SECTOR;
				bufferPerSector = 0;
				done = 1;
				flashStateLoaded = true;
				SaveFlashState();
				return;
			}
			//if flash chip is not full code will make it here and saves the next writable slot to the flash state sector
			SaveFlashState();
		}
		//set done to 0 and cursor is set for logging continuation
		done = 0;
		flashStateLoaded = true;
		return;
	}

	/*
	 * code should typically not reach here unless state sector is erased, state record checksum is wrong, magic number is missing,
	 * saved sector is out of range, or saved chunk is out of range. This manually scans for the first empty chunk
	 * if the state saved was corrupted in some way
	 *
	 */

	//scans each sector and chunk manually to see where the writing stopped and where the write cursor should be set
	for (uint16_t sector = 0; sector < FLASH_DATA_LIMIT_SECTOR; ++sector) 	//starts at sector 0 -> flash data write limit
	{
		for (uint8_t chunk = 0; chunk < 8U; ++chunk)	//iterates through each chunk in the sector
		{
			//sets the sectorBuf to all 0xFF and reads from the flash chip
			memset(sectorBuf, 0xFF, sizeof(sectorBuf));
			MX66xxQSPI_ReadSector(sectorBuf, sector, static_cast<uint32_t>(chunk) * RAM_LOG_SIZE, RAM_LOG_SIZE);

			//checks each byte in the chunk to see if chunk is empty
			bool empty = true;
			for (uint32_t i = 0; i < RAM_LOG_SIZE; ++i)
			{
				if (sectorBuf[i] != 0xFF)
				{
					//if not empty goes to next chunk
					empty = false;
					break;
				}
			}

			//if an empty chunk is found set the write cursor and save the flash state
			if (empty)
			{
				sectorAddress = sector;
				bufferPerSector = chunk;
				done = 0;
				flashStateLoaded = true;
				SaveFlashState();
				return;
			}
		}
	}

	/*if the flash chip is full set the cursor to the end of the chip, set done = 1 indicating
	 * that the flash chip is full, then as well save the state since the cursor moved from the last
	 * saved state
	 */
	sectorAddress = FLASH_STATE_SECTOR;
	bufferPerSector = 0;
	done = 1;
	flashStateLoaded = true;
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

		err = LoggingStatus::LOGGING_ERR;
		break;

	case LoggingDest::DMA:

		//TODO DMA write api
		err = LoggingStatus::LOGGING_ERR;
		break;
	}

	return err;
}


LoggingStatus LoggingService::LogToMX66(){

	//load flash state from storage (this will only actually do something the first time this function is run)
	//we do not want state to be loaded more than once in the lifetime of this code
	LoadFlashStateFromStorage();

	//appends data to buffer and returns flag determining if buffer is full (full => write data)
	LoggingStatus status = MemAppend(&loggingData);
	//stops logging, triggered by flash dump, or flashchip is full
	if(!done){
		//checks if 500 byte chunck is full (writes when it is full)
		if(status == LoggingStatus::LOG_FLASH_READY){
			if(sectorAddress >= FLASH_DATA_LIMIT_SECTOR){
				done = true;
				return LoggingStatus::FLASH_FULL;
			}

			if(bufferPerSector == 0){
				//if a new sector is reached erase before writing
				MX66xxQSPI_EraseSector(sectorAddress);
			}

			//checks if log chunk cursor is currently at is full
			else if (!IsLogChunkEmpty(sectorAddress, bufferPerSector))
			{
				SOAR_PRINT("LoggingService: sector=%u slot=%u is occupied, moving to next writable slot\n",
						   (unsigned int)sectorAddress,
						   (unsigned int)bufferPerSector);

				//find the next writeable slot, if false then the flash chip is full
				if (!FindNextWritableSlot(sectorAddress, bufferPerSector, sectorAddress, bufferPerSector))
				{
					//set cursor to last sector and set done to true so logging stops, save the state as well
					sectorAddress = FLASH_STATE_SECTOR;
					bufferPerSector = 0;
					done = true;
					SaveFlashState();
					return LoggingStatus::FLASH_FULL;
				}

				//if next writeable slot ends up being chunk 0 erase the sector before writing
				if(bufferPerSector == 0){
					MX66xxQSPI_EraseSector(sectorAddress);
				}
			}

			//Write data to bufferPerSector * 500 this calculates the offset at which the buffer will be written
			memcpy(txBuf, ramLog, RAM_LOG_SIZE);

			MX66xxQSPI_WriteSector(txBuf, sectorAddress, (bufferPerSector * 500),  RAM_LOG_SIZE);

			MX66xxQSPI_ReadSector(sectorBuf, sectorAddress,(bufferPerSector * 500),  RAM_LOG_SIZE);

			//check if the bytes written equal the bytes read
			if(BytesEqual(sectorBuf, txBuf, RAM_LOG_SIZE)){

				/* Each buffer is 500 bytes, 8 can fit in 4000 bytes
				 * bufferPerSector increases each successful buffer write which
				 * is used to keep track of when a sector is full, if it is full
				 * (bufferPerSector == 8) then go to the next sector.
				 */
				bufferPerSector++;
				if(bufferPerSector == 8){
					sectorAddress++;
					bufferPerSector = 0;
					SaveFlashState(); //save flash state after every sector write
					//finish logging if the sectorAddress reaches address that does not exist
					if(sectorAddress >= FLASH_DATA_LIMIT_SECTOR){
						sectorAddress = FLASH_STATE_SECTOR;
						done = true;
					}
				}

				return LoggingStatus::LOGGING_SUCCESS;
			}


			return LoggingStatus::LOGGING_ERR;

		}
		return LoggingStatus::LOG_FLASH_NOT_READY;
	}
	
	return LoggingStatus::FLASH_FULL;


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
	uint16_t gpsSentenceLen = 0;
	uint8_t gpsExpectedChunks = 0;
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
			uint8_t id = sectorBuf[i + 19];

			//parse the data back into structs

			//parse imu and print
			if(type == LoggingData::IMU16G || type == LoggingData::IMU32G)
			{
				int16_t accel[3], gyro[3], temp;

				memcpy(accel, sectorBuf + i + 5, sizeof(accel));
				memcpy(gyro, sectorBuf + i + 11, sizeof(gyro));
				memcpy(&temp, sectorBuf + i + 17, sizeof(temp));

				SOAR_PRINT("%s(ID=%u) Timestamp=%lu Accel=[%d,%d,%d] Gyro=[%d,%d,%d] Temp=%d\n",
									SensorTypeName(type), id, timestamp,
									accel[0], accel[1], accel[2],
									gyro[0], gyro[1], gyro[2],
									temp);
			}
			//parse baro and print
			else if(type == LoggingData::BARO07 || type == LoggingData::BARO11)
			{
				int32_t pressure;
				int16_t temperature;
				memcpy(&pressure, sectorBuf + i + 5, sizeof(pressure));
				memcpy(&temperature, sectorBuf + i + 9, sizeof(temperature));

				int16_t temp_c = temperature / 100;          // integer part
				int16_t temp_frac = temperature % 100;       // fractional part
				if (temp_frac < 0) temp_frac = -temp_frac;   // handle negative temperatures

				SOAR_PRINT("%s(ID=%u) Timestamp=%lu Pressure=%ld Temp=%d.%02d\n",
							SensorTypeName(type), id, timestamp, pressure, temp_c, temp_frac);

			}
			//parse mag and print
			else if (type == LoggingData::MAG){

				int32_t magX, magY, magZ;

				memcpy(&magX, sectorBuf + i + 5,  sizeof(int32_t));
				memcpy(&magY, sectorBuf + i + 9,  sizeof(int32_t));
				memcpy(&magZ, sectorBuf + i + 13, sizeof(int32_t));

				SOAR_PRINT("%s Timestamp=%lu Mag=[%ld,%ld,%ld]\n",
					SensorTypeName(type), timestamp,
					(long)magX, (long)magY, (long)magZ);

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
					gpsExpectedChunks = chunkCount;
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
					SOAR_PRINT("GPS Timestamp=%lu Chunks=%u Sentence=%s\n",
							   (unsigned long)gpsTimestamp,
							   (unsigned int)gpsExpectedChunks,
							   gpsSentence);
				}
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
void LoggingService::StopDump(){
	doneDump = true;
}
//starts logging (this is done to ensure logging continues with state changes)
void LoggingService::StartLogging(){
	done = false;
}
//ensures logging is stopped (this is done to ensure logging stops with appropriate state changes)
void LoggingService::StopLogging(){
	done = true;
	SaveFlashState(); //saves the flash state at which the logging stopped
}


LoggingStatus LoggingService::LogToInternalMemory(){

	if(loggingData.data == nullptr || loggingData.dataSize == 0){
		return LoggingStatus::LOGGING_ERR;
	}

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

	if(!data){return LoggingStatus::LOGGING_ERR;}

	uint32_t size = data->dataSize;

	if (size > MAX_LOG_SIZE){
		size = MAX_LOG_SIZE;
	}

	if(ramHead + MAX_LOG_SIZE > RAM_LOG_SIZE){ //check if word will fit in buffer
		ramHead = 0;

		return LoggingStatus::LOG_FLASH_READY;
	}


	for(uint32_t i = 0; i < size; i++){

		ramLog[ramHead++] = data->data[i];
	}

	for(uint32_t i = size; i < MAX_LOG_SIZE; i++){
		ramLog[ramHead++] = 0;
	}


	return LoggingStatus::LOG_FLASH_NOT_READY;

}

void LoggingService::FlashClear(){
	//Ensures the current flash state is loaded
	LoadFlashStateFromStorage();

	//clear flash chip bounds
	MX66xxQSPI_EraseSector(0);
	MX66xxQSPI_EraseSector(FLASH_STATE_SECTOR);

	//reset flash cursor and all necessary flash state variables
	ramHead = 0;
	sectorAddress = 0;
	bufferPerSector = 0;
	flashStateLoaded = true;
	done = 0;
	doneDump = 0;

	//save the flash state
	SaveFlashState();
	SOAR_PRINT("Flash clear: logging cursor reset\n");

}
