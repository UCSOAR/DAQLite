#ifndef _MX66XX_H
#define _MX66XX_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#define FS_PAGE_SIZE 256
#define FS_SECTOR_SIZE 4096
#define FS_BLOCK_SIZE 65536
#define FS_TOTAL_SIZE 0x04000000 // MX25L51245G: 512Mb = 64MB

    typedef struct
    {
        uint32_t JedecId;
        uint8_t UniqID[8];
        uint16_t PageSize;
        uint32_t PageCount;
        uint32_t SectorSize;
        uint32_t SectorCount;
        uint32_t BlockSize;
        uint32_t BlockCount;
        uint32_t CapacityInKiloByte;
        uint8_t StatusRegister1;
        uint8_t StatusRegister2;
        uint8_t StatusRegister3;
        uint8_t Lock;
        uint8_t Addr4Byte;
    } mx66xx_t;

    extern mx66xx_t mx66xx;

    bool MX66xx_Init(void);
    uint32_t MX66xx_ReadID(void);
    void MX66xx_ReadUniqID(void);
    void MX66xx_EN4B(void);
    void MX66xx_EX4B(void);
    void MX66xx_RSTQIO(void);
    void MX66xx_RSTEN(void);
    void MX66xx_RST(void);
    void MX66xx_ReadSFDP(uint8_t *rData);
    void MX66xx_ReleaseFromDeepPowerDown(void);
    uint8_t MX66xx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3);
    void MX66xx_WriteStatusRegister(uint8_t SelectStatusRegister_1_2_3, uint8_t Data);
    void MX66xx_WriteEnable(void);
    void MX66xx_WriteDisable(void);
    void MX66xx_WaitForWriteEnd(void);
    bool MX66xx_IsWriteProtected(void);

    void MX66xx_EraseChip(void);
    void MX66xx_EraseSector(uint32_t SectorIndex);
    void MX66xx_EraseBlock(uint32_t BlockIndex);

    uint32_t MX66xx_PageToSector(uint32_t PageAddress);
    uint32_t MX66xx_PageToBlock(uint32_t PageAddress);
    uint32_t MX66xx_SectorToBlock(uint32_t SectorAddress);
    uint32_t MX66xx_SectorToPage(uint32_t SectorAddress);
    uint32_t MX66xx_BlockToPage(uint32_t BlockAddress);

    bool MX66xx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
    bool MX66xx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
    bool MX66xx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

    void MX66xx_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address);
    void MX66xx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
    void MX66xx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
    void MX66xx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);

    void MX66xx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address);
    void MX66xx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    void MX66xx_FastReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    void MX66xx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
    void MX66xx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
    void MX66xx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize);

#ifdef __cplusplus
}
#endif

#endif
