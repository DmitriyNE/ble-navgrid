/* nRF54L15 application core, secure. Same layout as TrouBLE / nrf-sdc nRF54L15 examples, except
   that the top 8 KiB of RRAM stay out of the image, so flashing firmware leaves them alone:
     0x17B000  provisioning page, written by `navgrid provision` (navgrid_proto::PROVISIONING_ADDRESS)
     0x17C000  boot epoch log (src/store.rs) */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1516K
  RAM : ORIGIN = 0x20000000, LENGTH = 256K
}
