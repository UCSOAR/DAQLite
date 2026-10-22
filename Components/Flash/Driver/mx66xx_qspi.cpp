#include "mx66xx_qspi.hpp"

#include "mx66xxConf.hpp"
#include "SystemDefines.hpp"
#include "main.h"
#include "stm32h7xx_hal.h"

#if (_MX66XX_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define MX66xxQSPI_Delay(delay) osDelay(delay)
#else
#define MX66xxQSPI_Delay(delay) HAL_Delay(delay)
#endif

#define MX66XX_QSPI_QREAD_DUMMY_CYCLES 8
#define MX66XX_QSPI_4READ_DUMMY_CYCLES 6
#define MX66XX_QSPI_UNIQID_DUMMY_CYCLES 32
#define MX66XX_QSPI_SFDP_DUMMY_CYCLES 8
#define MX66XX_QSPI_SFDP_READ_SIZE 256
#define MX66XX_QSPI_SR1_WIP_BIT 0x01
#define MX66XX_QSPI_SR1_WEL_BIT 0x02
#define MX66XX_QSPI_SR1_BP_MASK 0x3C
#define MX66XX_QSPI_WRITE_POLL_TIMEOUT_MS 10000

#define MX66XX_CMD_QPIID 0xAF
#define MX66XX_CMD_RUID 0x4B
#define MX66XX_CMD_EQIO 0x35
#define MX66XX_CMD_EN4B 0xB7
#define MX66XX_CMD_EX4B 0xE9
#define MX66XX_CMD_RSTQIO 0xF5
#define MX66XX_CMD_RSTEN 0x66
#define MX66XX_CMD_RST 0x99
#define MX66XX_CMD_SFDP 0x5A
#define MX66XX_CMD_RDPD 0xAB
#define MX66XX_CMD_WREN 0x06
#define MX66XX_CMD_WRDI 0x04
#define MX66XX_CMD_RDSR 0x05
#define MX66XX_CMD_WRSR 0x01
#define MX66XX_CMD_CE 0xC7
#define MX66XX_CMD_SE 0x20
#define MX66XX_CMD_SE4B 0x21
#define MX66XX_CMD_BE 0xD8
#define MX66XX_CMD_READ 0x03
#define MX66XX_CMD_QREAD 0x6B
#define MX66XX_CMD_4READ 0xEB
#define MX66XX_CMD_4READ4B 0xEC
#define MX66XX_CMD_PP 0x02
#define MX66XX_CMD_PP4B 0x12
#define MX66XX_CMD_PP_QUAD 0x38

mx66xx_t mx66xx_qspi;
extern QSPI_HandleTypeDef _MX66XX_SPI;

static void MX66xxQSPI_BuildCommand(QSPI_CommandTypeDef *cmd,
                                    uint8_t instruction,
                                    uint32_t instructionMode,
                                    uint32_t address,
                                    uint32_t addressMode,
                                    uint32_t dataMode,
                                    uint32_t nbData,
                                    uint32_t dummyCycles)
{
    *cmd = {0};
    cmd->Instruction = instruction;
    cmd->InstructionMode = instructionMode;
    cmd->AddressMode = addressMode;

    if (addressMode == QSPI_ADDRESS_NONE)
    {
        cmd->AddressSize = QSPI_ADDRESS_24_BITS;
    }
    else
    {
        cmd->AddressSize = (mx66xx_qspi.Addr4Byte == 1)
                               ? QSPI_ADDRESS_32_BITS
                               : QSPI_ADDRESS_24_BITS;
    }

    cmd->Address = address;
    cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd->DataMode = dataMode;
    cmd->NbData = nbData;
    cmd->DummyCycles = dummyCycles;
    cmd->DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
}

static bool MX66xxQSPI_CommandOnly(uint8_t instruction)
{
    QSPI_CommandTypeDef cmd = {};
    MX66xxQSPI_BuildCommand(&cmd, instruction, QSPI_INSTRUCTION_4_LINES, 0, QSPI_ADDRESS_NONE, QSPI_DATA_NONE, 0, 0);
    return (HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000) == HAL_OK);
}

static bool MX66xxQSPI_CommandOnlyInstructionType(uint8_t instruction, uint32_t instLines)
{
    QSPI_CommandTypeDef cmd = {};
    MX66xxQSPI_BuildCommand(&cmd, instruction, instLines, 0, QSPI_ADDRESS_NONE, QSPI_DATA_NONE, 0, 0);
    return (HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000) == HAL_OK);
}

static bool MX66xxQSPI_CommandAddressOnly(uint8_t instruction,
                                          uint32_t address,
                                          uint32_t addressMode)
{
    if (_MX66XX_SPI.State == HAL_QSPI_STATE_ERROR)
    {
        if (__HAL_QSPI_GET_FLAG(&_MX66XX_SPI, QSPI_FLAG_BUSY) != RESET)
        {
            _MX66XX_SPI.State = HAL_QSPI_STATE_BUSY;
            (void)HAL_QSPI_Abort(&_MX66XX_SPI);
        }
        __HAL_QSPI_CLEAR_FLAG(&_MX66XX_SPI, QSPI_FLAG_TC | QSPI_FLAG_TE | QSPI_FLAG_SM | QSPI_FLAG_TO);
        CLEAR_BIT(_MX66XX_SPI.Instance->CCR, QUADSPI_CCR_FMODE);
        _MX66XX_SPI.ErrorCode = HAL_QSPI_ERROR_NONE;
        _MX66XX_SPI.State = HAL_QSPI_STATE_READY;
    }

    QSPI_CommandTypeDef cmd = {};
    MX66xxQSPI_BuildCommand(&cmd, instruction, QSPI_INSTRUCTION_4_LINES, address, addressMode, QSPI_DATA_NONE, 0, 0);
    const HAL_StatusTypeDef status = HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000);
    if (status != HAL_OK)
    {
        SOAR_PRINT("MX66xxQSPI_CommandAddressOnly() failed: st=%d qspiState=%d qspiErr=0x%08lX sr=0x%08lX ccr=0x%08lX inst=0x%02X addr=0x%08lX addrMode=%lu addr4=%u\n",
                   (int)status,
                   (int)_MX66XX_SPI.State,
                   (unsigned long)_MX66XX_SPI.ErrorCode,
                   (unsigned long)_MX66XX_SPI.Instance->SR,
                   (unsigned long)_MX66XX_SPI.Instance->CCR,
                   (unsigned int)instruction,
                   (unsigned long)address,
                   (unsigned long)addressMode,
                   (unsigned int)mx66xx_qspi.Addr4Byte);

        if ((_MX66XX_SPI.State != HAL_QSPI_STATE_READY) || (status == HAL_BUSY))
        {
            (void)HAL_QSPI_Abort(&_MX66XX_SPI);
        }
        return false;
    }

    return true;
}

static bool MX66xxQSPI_CommandReceive(uint8_t instruction,
                                      uint32_t address,
                                      uint32_t addressMode,
                                      uint32_t dataMode,
                                      uint8_t *rxBuf,
                                      uint32_t length,
                                      uint32_t dummyCycles)
{
    if ((rxBuf == nullptr) || (length == 0))
        return false;

    QSPI_CommandTypeDef cmd = {};
    MX66xxQSPI_BuildCommand(&cmd, instruction, QSPI_INSTRUCTION_4_LINES, address, addressMode, dataMode, length, dummyCycles);

    if (HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000) != HAL_OK)
    {
        SOAR_PRINT("MX66xxQSPI_CommandReceive() command failed: qspiState=%d qspiErr=0x%08lX inst=0x%02X addr=0x%08lX len=%lu\n",
                   (int)_MX66XX_SPI.State,
                   (unsigned long)_MX66XX_SPI.ErrorCode,
                   (unsigned int)instruction,
                   (unsigned long)address,
                   (unsigned long)length);
        if (_MX66XX_SPI.State != HAL_QSPI_STATE_READY)
        {
            (void)HAL_QSPI_Abort(&_MX66XX_SPI);
        }
        return false;
    }

    if (HAL_QSPI_Receive(&_MX66XX_SPI, rxBuf, 1000) != HAL_OK)
    {
        SOAR_PRINT("MX66xxQSPI_CommandReceive() data failed: qspiState=%d qspiErr=0x%08lX inst=0x%02X addr=0x%08lX len=%lu\n",
                   (int)_MX66XX_SPI.State,
                   (unsigned long)_MX66XX_SPI.ErrorCode,
                   (unsigned int)instruction,
                   (unsigned long)address,
                   (unsigned long)length);
        if (_MX66XX_SPI.State != HAL_QSPI_STATE_READY)
        {
            (void)HAL_QSPI_Abort(&_MX66XX_SPI);
        }
        return false;
    }

    return true;
}

static bool MX66xxQSPI_CommandTransmit(uint8_t instruction,
                                       uint32_t address,
                                       uint32_t addressMode,
                                       uint32_t dataMode,
                                       uint8_t *txBuf,
                                       uint32_t length,
                                       uint32_t dummyCycles)
{
    if ((txBuf == nullptr) || (length == 0))
        return false;

    QSPI_CommandTypeDef cmd = {};
    MX66xxQSPI_BuildCommand(&cmd, instruction, QSPI_INSTRUCTION_4_LINES, address, addressMode, dataMode, length, dummyCycles);

    if (HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000) != HAL_OK)
    {
        SOAR_PRINT("MX66xxQSPI_CommandTransmit() command failed: qspiState=%d qspiErr=0x%08lX inst=0x%02X addr=0x%08lX len=%lu\n",
                   (int)_MX66XX_SPI.State,
                   (unsigned long)_MX66XX_SPI.ErrorCode,
                   (unsigned int)instruction,
                   (unsigned long)address,
                   (unsigned long)length);
        if (_MX66XX_SPI.State != HAL_QSPI_STATE_READY)
        {
            (void)HAL_QSPI_Abort(&_MX66XX_SPI);
        }
        return false;
    }

    if (HAL_QSPI_Transmit(&_MX66XX_SPI, txBuf, 1000) != HAL_OK)
    {
        SOAR_PRINT("MX66xxQSPI_CommandTransmit() data failed: qspiState=%d qspiErr=0x%08lX inst=0x%02X addr=0x%08lX len=%lu\n",
                   (int)_MX66XX_SPI.State,
                   (unsigned long)_MX66XX_SPI.ErrorCode,
                   (unsigned int)instruction,
                   (unsigned long)address,
                   (unsigned long)length);
        if (_MX66XX_SPI.State != HAL_QSPI_STATE_READY)
        {
            (void)HAL_QSPI_Abort(&_MX66XX_SPI);
        }
        return false;
    }

    return true;
}

static bool MX66xxQSPI_IsWriteEnabled(void)
{
    return (MX66xxQSPI_ReadStatusRegister() & MX66XX_QSPI_SR1_WEL_BIT) != 0;
}

static bool MX66xxQSPI_VerifyProgramSample(uint32_t address, const uint8_t *data, uint32_t length)
{
    if ((data == nullptr) || (length == 0U))
        return false;

    uint8_t first = 0xFF;
    MX66xxQSPI_4ReadBytes(&first, address, 1);
    if (first != data[0])
        return false;

    if (length > 1U)
    {
        uint8_t last = 0xFF;
        MX66xxQSPI_4ReadBytes(&last, address + length - 1U, 1);
        if (last != data[length - 1U])
            return false;
    }

    return true;
}

static bool MX66xxQSPI_PrepareWrite(void)
{
    MX66xxQSPI_WaitForWriteEnd();
    uint8_t s = MX66xxQSPI_ReadStatusRegister();
    MX66xxQSPI_WriteEnable();
    s = MX66xxQSPI_ReadStatusRegister();
    HAL_Delay(5);
    return MX66xxQSPI_IsWriteEnabled();
}

uint32_t MX66xxQSPI_QPIReadID(void)
{
    uint8_t rData[3] = {0};
    QSPI_CommandTypeDef cmd = {};

    MX66xxQSPI_BuildCommand(&cmd, MX66XX_CMD_QPIID, QSPI_INSTRUCTION_4_LINES, 0, QSPI_ADDRESS_NONE, QSPI_DATA_4_LINES, sizeof(rData), 0);

    if (HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000) != HAL_OK)
        return 0;

    if (HAL_QSPI_Receive(&_MX66XX_SPI, rData, 1000) != HAL_OK)
        return 0;

    return (((uint32_t)rData[0] << 16) | ((uint32_t)rData[1] << 8) | (uint32_t)rData[2]);
}

void MX66xxQSPI_ReadUniqID(void)
{
    (void)MX66xxQSPI_CommandReceive(MX66XX_CMD_RUID,
                                    0,
                                    QSPI_ADDRESS_NONE,
                                    QSPI_DATA_4_LINES,
                                    mx66xx_qspi.UniqID,
                                    sizeof(mx66xx_qspi.UniqID),
                                    MX66XX_QSPI_UNIQID_DUMMY_CYCLES);
}

void MX66xxQSPI_EQIO(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_EQIO);
    MX66xxQSPI_Delay(1);
}


void MX66xxQSPI_EQIO_1LINE(void)
{
    (void)MX66xxQSPI_CommandOnlyInstructionType(MX66XX_CMD_EQIO,QSPI_INSTRUCTION_1_LINE);
    MX66xxQSPI_Delay(1);
}
void MX66xxQSPI_EN4B(void)
{
    if (MX66xxQSPI_CommandOnly(MX66XX_CMD_EN4B) ||
        MX66xxQSPI_CommandOnlyInstructionType(MX66XX_CMD_EN4B, QSPI_INSTRUCTION_1_LINE))
    {
        MX66xxQSPI_Delay(1);
        mx66xx_qspi.Addr4Byte = 1;
    }
}

void MX66xxQSPI_EX4B(void)
{
    if (MX66xxQSPI_CommandOnly(MX66XX_CMD_EX4B) ||
        MX66xxQSPI_CommandOnlyInstructionType(MX66XX_CMD_EX4B, QSPI_INSTRUCTION_1_LINE))
    {
        MX66xxQSPI_Delay(1);
        mx66xx_qspi.Addr4Byte = 0;
    }
}

void MX66xxQSPI_RSTQIO(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_RSTQIO);
    MX66xxQSPI_Delay(1);
}

void MX66xxQSPI_RSTEN(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_RSTEN);
    MX66xxQSPI_Delay(1);
}

void MX66xxQSPI_RST(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_RST);
    MX66xxQSPI_Delay(1);
}

void MX66xxQSPI_WriteEnable(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_WREN);
    MX66xxQSPI_Delay(1);
}

void MX66xxQSPI_WriteDisable(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_WRDI);
    MX66xxQSPI_Delay(1);
}

uint8_t MX66xxQSPI_ReadStatusRegister(void)
{
    uint8_t status = mx66xx_qspi.StatusRegister1;
    if (!MX66xxQSPI_CommandReceive(MX66XX_CMD_RDSR,
                                   0,
                                   QSPI_ADDRESS_NONE,
                                   QSPI_DATA_4_LINES,
                                   &status,
                                   1,
                                   0))
    {
        SOAR_PRINT("MX66xxQSPI_ReadStatusRegister() failed: qspiState=%d qspiErr=0x%08lX\n",
                   (int)_MX66XX_SPI.State,
                   (unsigned long)_MX66XX_SPI.ErrorCode);
    }

    mx66xx_qspi.StatusRegister1 = status;

    return status;
}

void MX66xxQSPI_WriteStatusRegister(uint8_t Data)
{
    if (!MX66xxQSPI_PrepareWrite())
        return;

    (void)MX66xxQSPI_CommandTransmit(MX66XX_CMD_WRSR,
                                     0,
                                     QSPI_ADDRESS_NONE,
                                     QSPI_DATA_4_LINES,
                                     &Data,
                                     1,
                                     0);

    MX66xxQSPI_WaitForWriteEnd();
    MX66xxQSPI_WriteDisable();

    mx66xx_qspi.StatusRegister1 = Data;
}

void MX66xxQSPI_WaitForWriteEnd(void)
{
    const uint32_t start = HAL_GetTick();
    uint8_t status = MX66xxQSPI_ReadStatusRegister();

    while ((status & MX66XX_QSPI_SR1_WIP_BIT) != 0U)
    {
        if ((HAL_GetTick() - start) > MX66XX_QSPI_WRITE_POLL_TIMEOUT_MS)
        {
            SOAR_PRINT("MX66xxQSPI_WaitForWriteEnd() timeout: qspiState=%d qspiErr=0x%08lX sr1=0x%02X\n",
                       (int)_MX66XX_SPI.State,
                       (unsigned long)_MX66XX_SPI.ErrorCode,
                       (unsigned int)status);
            if (__HAL_QSPI_GET_FLAG(&_MX66XX_SPI, QSPI_FLAG_BUSY) != RESET)
            {
                _MX66XX_SPI.State = HAL_QSPI_STATE_BUSY;
                (void)HAL_QSPI_Abort(&_MX66XX_SPI);
            }
            break;
        }
        MX66xxQSPI_Delay(1);
        status = MX66xxQSPI_ReadStatusRegister();
    }
}

bool MX66xxQSPI_Init(void)
{
    mx66xx_qspi.Lock = 1;

    while (HAL_GetTick() < 15)
        HAL_Delay(1);

    HAL_Delay(15);

    mx66xx_qspi.JedecId = MX66xxQSPI_QPIReadID();
    if ((mx66xx_qspi.JedecId == 0) || (mx66xx_qspi.JedecId == 0xFFFFFF))
    {
        mx66xx_qspi.Lock = 0;
        return false;
    }

    mx66xx_qspi.PageSize = FS_PAGE_SIZE;
    mx66xx_qspi.SectorSize = FS_SECTOR_SIZE;
    mx66xx_qspi.BlockSize = FS_BLOCK_SIZE;
    mx66xx_qspi.CapacityInKiloByte = (FS_TOTAL_SIZE / 1024);
    mx66xx_qspi.BlockCount = FS_TOTAL_SIZE / FS_BLOCK_SIZE;
    mx66xx_qspi.SectorCount = FS_TOTAL_SIZE / FS_SECTOR_SIZE;
    mx66xx_qspi.PageCount = FS_TOTAL_SIZE / FS_PAGE_SIZE;

    // MX25L51245G has no RUID (4Bh) command, so the unique ID is not read here
    MX66xxQSPI_ReadStatusRegister();

    mx66xx_qspi.Lock = 0;
    return mx66xx_qspi.Addr4Byte == 1;
}

void MX66xxQSPI_EraseChip(void)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if (!MX66xxQSPI_PrepareWrite())
    {
        mx66xx_qspi.Lock = 0;
        return;
    }

    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_CE);
    MX66xxQSPI_WaitForWriteEnd();
    MX66xxQSPI_WriteDisable();

    mx66xx_qspi.Lock = 0;
}

void MX66xxQSPI_EraseSector(uint32_t SectorIndex)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if (SectorIndex >= mx66xx_qspi.SectorCount)
    {
        mx66xx_qspi.Lock = 0;
        return;
    }

    if (!MX66xxQSPI_PrepareWrite())
    {
        mx66xx_qspi.Lock = 0;
        return;
    }

    uint32_t sectorAddr = SectorIndex * mx66xx_qspi.SectorSize;
    const uint8_t sectorEraseCmd = (mx66xx_qspi.Addr4Byte == 1U) ? MX66XX_CMD_SE4B : MX66XX_CMD_SE;
    if (!MX66xxQSPI_CommandAddressOnly(sectorEraseCmd,
                                       sectorAddr,
                                       QSPI_ADDRESS_4_LINES))
    {
        MX66xxQSPI_WriteDisable();
        mx66xx_qspi.Lock = 0;
        return;
    }
    MX66xxQSPI_WaitForWriteEnd();
    MX66xxQSPI_WriteDisable();

    mx66xx_qspi.Lock = 0;
}

void MX66xxQSPI_EraseBlock(uint32_t BlockIndex)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if (BlockIndex >= mx66xx_qspi.BlockCount)
    {
        mx66xx_qspi.Lock = 0;
        return;
    }

    if (!MX66xxQSPI_PrepareWrite())
    {
        mx66xx_qspi.Lock = 0;
        return;
    }

    uint32_t blockAddr = BlockIndex * mx66xx_qspi.BlockSize;
    (void)MX66xxQSPI_CommandAddressOnly(MX66XX_CMD_BE,
                                        blockAddr,
                                        QSPI_ADDRESS_4_LINES);
    MX66xxQSPI_WaitForWriteEnd();
    MX66xxQSPI_WriteDisable();

    mx66xx_qspi.Lock = 0;
}

uint32_t MX66xxQSPI_PageToSector(uint32_t PageAddress)
{
    return ((PageAddress * mx66xx_qspi.PageSize) / mx66xx_qspi.SectorSize);
}

uint32_t MX66xxQSPI_PageToBlock(uint32_t PageAddress)
{
    return ((PageAddress * mx66xx_qspi.PageSize) / mx66xx_qspi.BlockSize);
}

uint32_t MX66xxQSPI_SectorToBlock(uint32_t SectorAddress)
{
    return ((SectorAddress * mx66xx_qspi.SectorSize) / mx66xx_qspi.BlockSize);
}

uint32_t MX66xxQSPI_SectorToPage(uint32_t SectorAddress)
{
    return (SectorAddress * mx66xx_qspi.SectorSize) / mx66xx_qspi.PageSize;
}

uint32_t MX66xxQSPI_BlockToPage(uint32_t BlockAddress)
{
    return (BlockAddress * mx66xx_qspi.BlockSize) / mx66xx_qspi.PageSize;
}

bool MX66xxQSPI_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if (((NumByteToCheck_up_to_PageSize + OffsetInByte) > mx66xx_qspi.PageSize) || (NumByteToCheck_up_to_PageSize == 0))
        NumByteToCheck_up_to_PageSize = mx66xx_qspi.PageSize - OffsetInByte;

    uint8_t pBuffer[32];
    uint32_t bytesChecked = 0;
    uint32_t baseAddr = (Page_Address * mx66xx_qspi.PageSize) + OffsetInByte;

    while (bytesChecked < NumByteToCheck_up_to_PageSize)
    {
        uint32_t chunk = NumByteToCheck_up_to_PageSize - bytesChecked;
        if (chunk > sizeof(pBuffer))
            chunk = sizeof(pBuffer);

        MX66xxQSPI_4ReadBytes(pBuffer, baseAddr + bytesChecked, chunk);
        for (uint32_t i = 0; i < chunk; ++i)
        {
            if (pBuffer[i] != 0xFF)
            {
                mx66xx_qspi.Lock = 0;
                return false;
            }
        }

        bytesChecked += chunk;
    }

    mx66xx_qspi.Lock = 0;
    return true;
}

bool MX66xxQSPI_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if (((NumByteToCheck_up_to_SectorSize + OffsetInByte) > mx66xx_qspi.SectorSize) || (NumByteToCheck_up_to_SectorSize == 0))
        NumByteToCheck_up_to_SectorSize = mx66xx_qspi.SectorSize - OffsetInByte;

    uint8_t pBuffer[32];
    uint32_t bytesChecked = 0;
    uint32_t baseAddr = (Sector_Address * mx66xx_qspi.SectorSize) + OffsetInByte;

    while (bytesChecked < NumByteToCheck_up_to_SectorSize)
    {
        uint32_t chunk = NumByteToCheck_up_to_SectorSize - bytesChecked;
        if (chunk > sizeof(pBuffer))
            chunk = sizeof(pBuffer);

        MX66xxQSPI_4ReadBytes(pBuffer, baseAddr + bytesChecked, chunk);
        for (uint32_t i = 0; i < chunk; ++i)
        {
            if (pBuffer[i] != 0xFF)
            {
                mx66xx_qspi.Lock = 0;
                return false;
            }
        }

        bytesChecked += chunk;
    }

    mx66xx_qspi.Lock = 0;
    return true;
}

bool MX66xxQSPI_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if (((NumByteToCheck_up_to_BlockSize + OffsetInByte) > mx66xx_qspi.BlockSize) || (NumByteToCheck_up_to_BlockSize == 0))
        NumByteToCheck_up_to_BlockSize = mx66xx_qspi.BlockSize - OffsetInByte;

    uint8_t pBuffer[32];
    uint32_t bytesChecked = 0;
    uint32_t baseAddr = (Block_Address * mx66xx_qspi.BlockSize) + OffsetInByte;

    while (bytesChecked < NumByteToCheck_up_to_BlockSize)
    {
        uint32_t chunk = NumByteToCheck_up_to_BlockSize - bytesChecked;
        if (chunk > sizeof(pBuffer))
            chunk = sizeof(pBuffer);

        MX66xxQSPI_4ReadBytes(pBuffer, baseAddr + bytesChecked, chunk);
        for (uint32_t i = 0; i < chunk; ++i)
        {
            if (pBuffer[i] != 0xFF)
            {
                mx66xx_qspi.Lock = 0;
                return false;
            }
        }

        bytesChecked += chunk;
    }

    mx66xx_qspi.Lock = 0;
    return true;
}

void MX66xxQSPI_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address)
{
    MX66xxQSPI_WritePage(&pBuffer, Bytes_Address / mx66xx_qspi.PageSize, Bytes_Address % mx66xx_qspi.PageSize, 1);
}

void MX66xxQSPI_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
    while (mx66xx_qspi.Lock == 1)
        MX66xxQSPI_Delay(1);
    mx66xx_qspi.Lock = 1;

    if ((pBuffer == nullptr) || (mx66xx_qspi.PageSize == 0))
    {
        mx66xx_qspi.Lock = 0;
        return;
    }

    if (((NumByteToWrite_up_to_PageSize + OffsetInByte) > mx66xx_qspi.PageSize) || (NumByteToWrite_up_to_PageSize == 0))
        NumByteToWrite_up_to_PageSize = mx66xx_qspi.PageSize - OffsetInByte;

    uint32_t addr = (Page_Address * mx66xx_qspi.PageSize) + OffsetInByte;

    if (MX66xxQSPI_PrepareWrite())
    {
        const uint8_t programCmdList[] = {MX66XX_CMD_PP, MX66XX_CMD_PP4B, MX66XX_CMD_PP_QUAD};
        bool programmed = false;

        for (uint32_t cmdIdx = 0U; cmdIdx < (uint32_t)(sizeof(programCmdList) / sizeof(programCmdList[0])); ++cmdIdx)
        {
            const uint8_t programCmd = programCmdList[cmdIdx];

            if ((cmdIdx > 0U) && !MX66xxQSPI_PrepareWrite())
                break;

            if (!MX66xxQSPI_CommandTransmit(programCmd,
                                            addr,
                                            QSPI_ADDRESS_4_LINES,
                                            QSPI_DATA_4_LINES,
                                            pBuffer,
                                            NumByteToWrite_up_to_PageSize,
                                            0))
            {
                MX66xxQSPI_WriteDisable();
                continue;
            }

            MX66xxQSPI_WaitForWriteEnd();
            MX66xxQSPI_WriteDisable();

            if (MX66xxQSPI_VerifyProgramSample(addr, pBuffer, NumByteToWrite_up_to_PageSize))
            {
                programmed = true;
                break;
            }
        }

        if (!programmed)
        {
            SOAR_PRINT("MX66xxQSPI_WritePage() failed: page=%lu addr=0x%08lX len=%lu sr1=0x%02X\n",
                       (unsigned long)Page_Address,
                       (unsigned long)addr,
                       (unsigned long)NumByteToWrite_up_to_PageSize,
                       (unsigned int)MX66xxQSPI_ReadStatusRegister());
        }
    }

    mx66xx_qspi.Lock = 0;
}

void MX66xxQSPI_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
    if (((NumByteToWrite_up_to_SectorSize + OffsetInByte) > mx66xx_qspi.SectorSize) || (NumByteToWrite_up_to_SectorSize == 0))
        NumByteToWrite_up_to_SectorSize = mx66xx_qspi.SectorSize - OffsetInByte;

    uint32_t StartPage = MX66xxQSPI_SectorToPage(Sector_Address) + (OffsetInByte / mx66xx_qspi.PageSize);
    uint32_t BytesLeft = NumByteToWrite_up_to_SectorSize;
    uint32_t OffsetInPage = OffsetInByte % mx66xx_qspi.PageSize;
    uint32_t BufferIndex = 0;

    if (OffsetInPage)
    {
        uint32_t BytesToWrite = mx66xx_qspi.PageSize - OffsetInPage;
        if (BytesToWrite > BytesLeft)
            BytesToWrite = BytesLeft;

        MX66xxQSPI_WritePage(pBuffer, StartPage, OffsetInPage, BytesToWrite);
        BytesLeft -= BytesToWrite;
        BufferIndex += BytesToWrite;
        StartPage++;
    }

    while (BytesLeft > 0)
    {
        uint32_t BytesToWrite = (BytesLeft > mx66xx_qspi.PageSize) ? mx66xx_qspi.PageSize : BytesLeft;
        MX66xxQSPI_WritePage(&pBuffer[BufferIndex], StartPage, 0, BytesToWrite);
        BytesLeft -= BytesToWrite;
        BufferIndex += BytesToWrite;
        StartPage++;
    }
}

void MX66xxQSPI_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize)
{
    if (((NumByteToWrite_up_to_BlockSize + OffsetInByte) > mx66xx_qspi.BlockSize) || (NumByteToWrite_up_to_BlockSize == 0))
        NumByteToWrite_up_to_BlockSize = mx66xx_qspi.BlockSize - OffsetInByte;

    uint32_t StartPage = MX66xxQSPI_BlockToPage(Block_Address) + (OffsetInByte / mx66xx_qspi.PageSize);
    uint32_t BytesLeft = NumByteToWrite_up_to_BlockSize;
    uint32_t OffsetInPage = OffsetInByte % mx66xx_qspi.PageSize;
    uint32_t BufferIndex = 0;

    if (OffsetInPage)
    {
        uint32_t BytesToWrite = mx66xx_qspi.PageSize - OffsetInPage;
        if (BytesToWrite > BytesLeft)
            BytesToWrite = BytesLeft;

        MX66xxQSPI_WritePage(pBuffer, StartPage, OffsetInPage, BytesToWrite);
        BytesLeft -= BytesToWrite;
        BufferIndex += BytesToWrite;
        StartPage++;
    }

    while (BytesLeft > 0)
    {
        uint32_t BytesToWrite = (BytesLeft > mx66xx_qspi.PageSize) ? mx66xx_qspi.PageSize : BytesLeft;
        MX66xxQSPI_WritePage(&pBuffer[BufferIndex], StartPage, 0, BytesToWrite);
        BytesLeft -= BytesToWrite;
        BufferIndex += BytesToWrite;
        StartPage++;
    }
}

void MX66xxQSPI_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address)
{
    MX66xxQSPI_ReadBytes(pBuffer, Bytes_Address, 1);
}

void MX66xxQSPI_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    // Keep generic reads on the known-good 4READ path for QPI mode.
    MX66xxQSPI_4ReadBytes(pBuffer, ReadAddr, NumByteToRead);
}

void MX66xxQSPI_QReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    (void)MX66xxQSPI_CommandReceive(MX66XX_CMD_QREAD,
                                    ReadAddr,
                                    QSPI_ADDRESS_4_LINES,
                                    QSPI_DATA_4_LINES,
                                    pBuffer,
                                    NumByteToRead,
                                    MX66XX_QSPI_QREAD_DUMMY_CYCLES);
}

void MX66xxQSPI_4ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    if ((pBuffer == nullptr) || (NumByteToRead == 0))
        return;

    QSPI_CommandTypeDef cmd = {};
    cmd.Instruction = (mx66xx_qspi.Addr4Byte == 1U) ? MX66XX_CMD_4READ4B : MX66XX_CMD_4READ;
    cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    cmd.AddressMode = QSPI_ADDRESS_4_LINES;
    cmd.AddressSize = (mx66xx_qspi.Addr4Byte == 1U) ? QSPI_ADDRESS_32_BITS : QSPI_ADDRESS_24_BITS;
    cmd.Address = ReadAddr;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_4_LINES;
    cmd.NbData = NumByteToRead;
    cmd.DummyCycles = MX66XX_QSPI_4READ_DUMMY_CYCLES;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&_MX66XX_SPI, &cmd, 1000) != HAL_OK)
        return;

    HAL_QSPI_Receive(&_MX66XX_SPI, pBuffer, 1000);
}

void MX66xxQSPI_FastReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    MX66xxQSPI_4ReadBytes(pBuffer, ReadAddr, NumByteToRead);
}

void MX66xxQSPI_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
    if (((NumByteToRead_up_to_PageSize + OffsetInByte) > mx66xx_qspi.PageSize) || (NumByteToRead_up_to_PageSize == 0))
        NumByteToRead_up_to_PageSize = mx66xx_qspi.PageSize - OffsetInByte;

    uint32_t addr = (Page_Address * mx66xx_qspi.PageSize) + OffsetInByte;
    MX66xxQSPI_ReadBytes(pBuffer, addr, NumByteToRead_up_to_PageSize);
}

void MX66xxQSPI_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
    if (((NumByteToRead_up_to_SectorSize + OffsetInByte) > mx66xx_qspi.SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
        NumByteToRead_up_to_SectorSize = mx66xx_qspi.SectorSize - OffsetInByte;

    uint32_t addr = (Sector_Address * mx66xx_qspi.SectorSize) + OffsetInByte;
    MX66xxQSPI_ReadBytes(pBuffer, addr, NumByteToRead_up_to_SectorSize);
}

void MX66xxQSPI_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize)
{
    if (((NumByteToRead_up_to_BlockSize + OffsetInByte) > mx66xx_qspi.BlockSize) || (NumByteToRead_up_to_BlockSize == 0))
        NumByteToRead_up_to_BlockSize = mx66xx_qspi.BlockSize - OffsetInByte;

    uint32_t addr = (Block_Address * mx66xx_qspi.BlockSize) + OffsetInByte;
    MX66xxQSPI_ReadBytes(pBuffer, addr, NumByteToRead_up_to_BlockSize);
}

void MX66xxQSPI_ReadSFDP(uint8_t *rData)
{
    if (rData == nullptr)
        return;

    (void)MX66xxQSPI_CommandReceive(MX66XX_CMD_SFDP,
                                    0,
                                    QSPI_ADDRESS_4_LINES,
                                    QSPI_DATA_4_LINES,
                                    rData,
                                    MX66XX_QSPI_SFDP_READ_SIZE,
                                    MX66XX_QSPI_SFDP_DUMMY_CYCLES);
}

void MX66xxQSPI_ReleaseFromDeepPowerDown(void)
{
    (void)MX66xxQSPI_CommandOnly(MX66XX_CMD_RDPD);
    MX66xxQSPI_Delay(1);
}

bool MX66xxQSPI_IsWriteProtected(void)
{
    return (MX66xxQSPI_ReadStatusRegister() & MX66XX_QSPI_SR1_BP_MASK) != 0;
}
