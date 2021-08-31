#ifndef _FSL_FLASH_H_
#define _FSL_FLASH_H_

#include "fsl_flexspi.h"

int flexspi_nor_init(void);
void flexspi_nor_reset(void); // just a wrapper of the inline flexspi reset to guarentee it is not in-lined into a flash address
int flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
int flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t address, const uint32_t *src);
void flexspi_nor_reset(void);

//uint32_t flash_get_sector_info(uint32_t addr, uint32_t *start_addr, uint32_t *size);
//void flash_erase(uint32_t flash_dest, const uint32_t *src, uint32_t num_word32);
//void flash_write(uint32_t flash_dest, const uint32_t *src, uint32_t num_word32);
#endif