/* nRF54L15 application core, secure, under the bootloader (architecture §13; ../bootloader has
   the same slots). The image links into the active slot; the bootloader swaps updates in from DFU.
     0x000000  bootloader (32 KiB)
     0x008000  bootloader state (12 KiB: four 16-byte progress lines per page)
     0x00B000  active slot (720 KiB): this image
     0x0BF000  update slot (724 KiB)
     0x17B000  provisioning page, written by `navgrid provision` (navgrid_proto::PROVISIONING_ADDRESS)
     0x17C000  boot epoch log (src/store.rs)
   Flashing firmware leaves the provisioning and epoch pages alone. */
MEMORY
{
  BOOTLOADER        : ORIGIN = 0x00000000, LENGTH = 32K
  BOOTLOADER_STATE  : ORIGIN = 0x00008000, LENGTH = 12K
  FLASH             : ORIGIN = 0x0000B000, LENGTH = 720K
  DFU               : ORIGIN = 0x000BF000, LENGTH = 724K
  RAM               : ORIGIN = 0x20000000, LENGTH = 256K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);
__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
