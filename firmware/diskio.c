/*
 * Copyright (c) 2013, M.Naruoka (fenrir)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

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
