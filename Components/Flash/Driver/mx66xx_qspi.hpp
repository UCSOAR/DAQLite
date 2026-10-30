#ifndef _MX66XX_QSPI_H
#define _MX66XX_QSPI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mx66xx.hpp"

    extern mx66xx_t mx66xx_qspi;

    bool MX66xxQSPI_Init(void);
    uint32_t MX66xxQSPI_QPIReadID(void);
    void MX66xxQSPI_ReadUniqID(void);
    void MX66xxQSPI_EQIO(void);
    void MX66xxQSPI_EQIO_1LINE(void);
    void MX66xxQSPI_EN4B(void);
    void MX66xxQSPI_EX4B(void);
    void MX66xxQSPI_RSTQIO(void);
    void MX66xxQSPI_RSTEN(void);
    void MX66xxQSPI_RST(void);
    void MX66xxQSPI_ReadSFDP(uint8_t *rData);
    void MX66xxQSPI_ReleaseFromDeepPowerDown(void);
    uint8_t MX66xxQSPI_ReadStatusRegister(void);
    uint8_t MX66xxQSPI_ReadSecurityRegister(void);
    void MX66xxQSPI_WriteStatusRegister(uint8_t Data);
    void MX66xxQSPI_WriteEnable(void);
    void MX66xxQSPI_WriteDisable(void);
    void MX66xxQSPI_WaitForWriteEnd(void);
    bool MX66xxQSPI_IsWriteProtected(void);

    void MX66xxQSPI_EraseChip(void);
    void MX66xxQSPI_EraseSector(uint32_t SectorIndex);
    void MX66xxQSPI_EraseBlock(uint32_t BlockIndex);

    uint32_t MX66xxQSPI_PageToSector(uint32_t PageAddress);
    uint32_t MX66xxQSPI_PageToBlock(uint32_t PageAddress);
    uint32_t MX66xxQSPI_SectorToBlock(uint32_t SectorAddress);
    uint32_t MX66xxQSPI_SectorToPage(uint32_t SectorAddress);
    uint32_t MX66xxQSPI_BlockToPage(uint32_t BlockAddress);

    bool MX66xxQSPI_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
    bool MX66xxQSPI_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
    bool MX66xxQSPI_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

    void MX66xxQSPI_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address);
    void MX66xxQSPI_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
    void MX66xxQSPI_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
    void MX66xxQSPI_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);

    void MX66xxQSPI_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address);
    void MX66xxQSPI_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    void MX66xxQSPI_QReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    void MX66xxQSPI_4ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    void MX66xxQSPI_FastReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    void MX66xxQSPI_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
    void MX66xxQSPI_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
    void MX66xxQSPI_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize);

#ifdef __cplusplus
}
#endif

#endif
