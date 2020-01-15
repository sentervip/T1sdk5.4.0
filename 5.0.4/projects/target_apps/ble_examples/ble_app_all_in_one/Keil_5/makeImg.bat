mkimage.exe single PTM100_V1.bin sw_version.h PTM100_V1.img
mkimage.exe multi spi secondary_bootloader.bin PTM100_V1.img 0x8000 PTM100_V1.img 0x13000 0x1f000 PTM100_V1_ALL.bin
