/* nRF54L15 RRAM with the bootloader (architecture §13). The node's memory.x has the same slots,
   and navgrid_proto::layout the same addresses. */
MEMORY
{
  FLASH             : ORIGIN = 0x00000000, LENGTH = 32K
  BOOTLOADER_STATE  : ORIGIN = 0x00008000, LENGTH = 12K
  ACTIVE            : ORIGIN = 0x0000B000, LENGTH = 720K
  DFU               : ORIGIN = 0x000BF000, LENGTH = 724K
  RAM               : ORIGIN = 0x20000000, LENGTH = 256K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);
__bootloader_active_start = ORIGIN(ACTIVE);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE);
__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
