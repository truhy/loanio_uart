# Loan I/O examples

## Overview

Hardware design showing the FPGA side directly using peripherals on the HPS side.  FPGA controls the HPS LED and UART-USB on the DE10-Nano board, which are both connected to the HPS side.

In Platform Designer these HPS pins are loaned out to the FPGA:
	- LoanIO 49 is connected to FTDI FT232R TX pin (UART-USB)
	- LoanIO 50 is connected to FTDI FT232R RX pin (UART-USB)
	- LoanIO 53 is connected to HPS_LED

By default the loan I/Os are held in reset, meaning initialisation is required to configure the HPS pin mux and loan I/Os, which can be done by U-Boot or U-Boot-SPL.

Note, since the HPS uart0 controller pins are loaned out to the FPGA the HPS can no longer use the UART-USB.

## Running from USB Blaster II JTAG cable with a script

Requires OpenOCD and Quartus Prime Lite or Quartus Prime Programmer and Tools to be installed and in search paths.  The search paths are set in the scripts inside scripts-env folder.

1. Eject SD card if there is one in the slot
2. Connect USB Blaster II cable
3. Connect a USB cable between the USB-UART connector on the DE10-Nano and your computer
4. Start a serial terminal program such as PuTTY and set it to use the correct serial port, with settings 115200 baud, 8 data bits, 1 stop bit, no parity, no control flow
5. On Windows run the script rundemo-win.bat, or on linux run rundemo-linux.sh
6. Wait for U-Boot to boot up and quartus to program the FPGA.  You should an LED starts blinking
