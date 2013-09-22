#ifndef _SCSI_H_
#define _SCSI_H_

#include "type.h"

#define SCSI_PASSED 		    0
#define SCSI_FAILED 		    1
#define SCSI_PHASE_ERROR 	2
#define SCSI_PENDING       0xFF

void scsi_setup();
void scsi_ex();

extern u8 __xdata scsi_Status;
extern u32 __xdata scsi_Residue;

extern u8 __xdata scsi_lun;
extern u16 __xdata scsi_block_size;

#endif
