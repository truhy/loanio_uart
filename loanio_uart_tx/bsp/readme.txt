The u-boot-spl was built using my ocram_ubootspl-build-make-scripts with the hps handoff files from this quartus project.
The folder hps_isw_handoff was copied to the source folder of the scripts project.
They are required so that the u-boot-spl will configure the loan I/O appropriately.

Also these changes made to socfpga_de10_nano_defconfig:

Embed the U-Boot SPL devicetree .dtb to the end of the elf.  Note, U-Boot has it's own separate devicetree
This enables to load only a single elf file (u-boot-spl), without needing to load a separate .dtb file when debugging in OpenOCD or GDB
CONFIG_OF_EMBED=y
