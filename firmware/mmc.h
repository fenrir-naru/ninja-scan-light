#ifndef __MMC_H__
#define __MMC_H__

typedef enum {
  MMC_NORMAL_CODE, MMC_ERROR_CODE
} mmc_res_t;

// Physical size in bytes of one MMC FLASH sector
#define MMC_PHYSICAL_BLOCK_SIZE 512

extern __bit mmc_initialized;
extern __xdata unsigned long mmc_physical_size;
extern __xdata unsigned long mmc_physical_sectors;

void mmc_init();
mmc_res_t mmc_flush();

mmc_res_t mmc_read(unsigned long address, unsigned char *pchar);
mmc_res_t mmc_write(unsigned long address, unsigned char *wdata);
mmc_res_t mmc_get_status();

#endif /* __MMC_H__ */


