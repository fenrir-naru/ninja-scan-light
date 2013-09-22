#include <string.h>

#include "mmc.h"
#include "type.h"
#include "diskio.h"

DSTATUS disk_initialize (BYTE drive){
  if(drive != 0){return STA_NODISK;}
  mmc_init();
  return mmc_initialized ? RES_OK : RES_NOTRDY;
}

DSTATUS disk_status (BYTE drive){
  return (drive == 0) ? RES_OK : RES_NOTRDY;
}

DRESULT disk_read (BYTE drive, BYTE *buf, DWORD start_sector, BYTE sectors){
  if(drive != 0){return RES_NOTRDY;}
  while(sectors--){
    if(mmc_read(start_sector, buf) != MMC_NORMAL_CODE){
      mmc_get_status();
      return RES_ERROR;
    }
  }
  return RES_OK;
}

#if _USE_IOCTL

DRESULT disk_ioctl (BYTE drive, BYTE ctrl, void *buff){
  if(drive != 0){return RES_NOTRDY;}
  switch(ctrl){
    case CTRL_SYNC :
      if(mmc_flush() != MMC_NORMAL_CODE){return RES_ERROR;}
      break;
    case GET_SECTOR_COUNT :
      *(DWORD *)buff = mmc_physical_sectors;
      break;
    case GET_SECTOR_SIZE :
      *(u16 *)buff = MMC_PHYSICAL_BLOCK_SIZE;
      break;
#if 0
    case GET_SIZE :
      *(DWORD *)buff = mmc_physical_size;
      break;
#endif
    default:
      return RES_PARERR;
  }
  return RES_OK;
}

#endif

#if _USE_WRITE

DRESULT disk_write (BYTE drive, const BYTE *buf, DWORD start_sector, BYTE sectors){
  if(drive != 0){return STA_NODISK;}
  while(sectors--){
    if(mmc_write(start_sector, buf) != MMC_NORMAL_CODE){
      mmc_get_status();
      return RES_ERROR;
    }
  }
  return RES_OK;
}

#endif
