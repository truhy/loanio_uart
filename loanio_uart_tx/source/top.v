/*
	MIT License

	Copyright (c) 2024 Truong Hy
	
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

	Project: Loan I/O UART transmit example
	HDL    : Verilog
	Target : For the DE10-Nano development kit board (SoC FPGA Cyclone V)
	Version: 20241227
	
	Hardware design showing the FPGA side directly using peripherals on the HPS
	side.
	
	When the FPGA input key0 is pressed the HPS LED is turned on and the message
	is sent.  FPGA controls the HPS LED and UART-USB on the DE10-Nano board, which
	are both connected to the HPS side.
	
	The FPGA is able to control the dedicated HPS pins using the HPS pin mux loan
	I/Os.
	
	In Platform Designer these HPS pins are loaned out to the FPGA:
	- LoanIO 49 is connected to FTDI FT232R TX pin (UART-USB)
	- LoanIO 50 is connected to FTDI FT232R RX pin (UART-USB)
	- LoanIO 53 is connected to HPS_LED
	
	By default the loan I/Os are held in reset, meaning initialisation is
	required to configure the HPS pin mux and loan I/Os, which can be done by
	U-Boot or U-Boot-SPL.
	
	Note, since the HPS uart0 controller pins are loaned out to the FPGA the HPS
	can no longer use the UART-USB.
*/

module top(
	// FPGA clock
	input FPGA_CLK1_50,
	input FPGA_CLK2_50,
	input FPGA_CLK3_50,
	
	// FPGA ADC. FPGA pins wired to LTC2308 (SPI)
	output ADC_CONVST,
	output ADC_SCK,
	output ADC_SDI,
	input  ADC_SDO,
	
	// FPGA Arduino IO. FPGA pins wired to female headers
	inout [15:0] ARDUINO_IO,
	inout        ARDUINO_RESET_N,
	
	// FPGA GPIO. FPGA pins wired to 2x male headers
	inout [35:0] GPIO_0,
	inout [35:0] GPIO_1,
	
	// FPGA HDMI. FPGA pins to HDMI transmitter chip (Analog Devices ADV7513BSWZ)
	inout          HDMI_I2C_SCL,
	inout          HDMI_I2C_SDA,
	inout          HDMI_I2S,
	inout          HDMI_LRCLK,
	inout          HDMI_MCLK,
	inout          HDMI_SCLK,
	output         HDMI_TX_CLK,
	output [23: 0] HDMI_TX_D,
	output         HDMI_TX_DE,
	output         HDMI_TX_HS,
	input          HDMI_TX_INT,
	output         HDMI_TX_VS,

	// FPGA push buttons, LEDs and slide switches
	input  [1:0] KEY,  // FPGA pins to tactile switches
	output [7:0] LED,  // FPGA pins to LEDs
	input  [3:0] SW,   // FPGA pins to slide switches
	
	// HPS user key and LED
	inout HPS_KEY,
	inout HPS_LED,
	
	// HPS DDR-3 SDRAM. HPS pins to DDR3 1GB = 2x512MB chips
	output [14:0] HPS_DDR3_ADDR,
	output [2:0]  HPS_DDR3_BA,
	output        HPS_DDR3_CAS_N,
	output        HPS_DDR3_CKE,
	output        HPS_DDR3_CK_N,
	output        HPS_DDR3_CK_P,
	output        HPS_DDR3_CS_N,
	output [3:0]  HPS_DDR3_DM,
	inout  [31:0] HPS_DDR3_DQ,
	inout  [3:0]  HPS_DDR3_DQS_N,
	inout  [3:0]  HPS_DDR3_DQS_P,
	output        HPS_DDR3_ODT,
	output        HPS_DDR3_RAS_N,
	output        HPS_DDR3_RESET_N,
	input         HPS_DDR3_RZQ,
	output        HPS_DDR3_WE_N,
	
	// HPS SD-CARD. HPS pins wired to micro SD card slot
	output      HPS_SD_CLK,
	inout       HPS_SD_CMD,
	inout [3:0] HPS_SD_DATA,

	// HPS UART (UART-USB). HPS pins wired to chip FTDI FT232R
	inout HPS_UART_RX,
	inout HPS_UART_TX,
	inout HPS_CONV_USB_N,
	
	// HPS USB OTG. HPS pins wired to the USB PHY chip (Microchip USB3300)
	input       HPS_USB_CLKOUT,
	inout [7:0] HPS_USB_DATA,
	input       HPS_USB_DIR,
	input       HPS_USB_NXT,
	output      HPS_USB_STP,
	
	// HPS EMAC. HPS pins wired to the gigabit ethernet PHY chip (Microchip KSZ9031RNX)
	output       HPS_ENET_GTX_CLK,
	output       HPS_ENET_MDC,
	inout        HPS_ENET_MDIO,
	input        HPS_ENET_RX_CLK,
	input [3:0]  HPS_ENET_RX_DATA,
	input        HPS_ENET_RX_DV,
	output [3:0] HPS_ENET_TX_DATA,
	output       HPS_ENET_TX_EN,
	inout        HPS_ENET_INT_N,

	// HPS SPI. HPS pins wired to the LTC 2x7 connector
	output HPS_SPIM_CLK,
	input  HPS_SPIM_MISO,
	output HPS_SPIM_MOSI,
	inout  HPS_SPIM_SS,

	// HPS GPIO. HPS pin wired to the LTC 2x7 connector (note, this cannot be used because the 0ohm resistor is not populated)
	//inout HPS_LTC_GPIO,
	
	// HPS I2C 1. HPS pins wired to the LTC 2x7 connector
	inout HPS_I2C1_SCLK,
	inout HPS_I2C1_SDAT,
	
	// HPS I2C 0. HPS pins wired to the ADXL345 accelerometer
	inout HPS_I2C0_SCLK,
	inout HPS_I2C0_SDAT,
	inout HPS_GSENSOR_INT  // ADXL345 interrupt 1 output (pin 9)
);
	// ================
	// HPS (SoC) module
	// ================

	// Wires for the PLL clock and reset
	wire hps_reset_n;
	wire master_reset_n = hps_reset_n;
	
	// Loan I/O bitmaps.  Each bit belongs to a specific loan I/O pin:
	// bit0 = loan I/O pin 0, bit1 = loan I/O pin 1, bit 2 = loan I/O pin 2, etc
	wire [66:0] loanio_oe;   // Pin direction, where each bit is: 0 = input mode, 1 = output mode
	wire [66:0] loanio_in;   // Read from pin, where each bit is: 1 = high, 0 = low
	wire [66:0] loanio_out;  // Write to pin, where each bit is: 1 = high, 0 = low

	// HPS (SoC) instance
	soc_system u0(
		// Clock
		.clk_clk(FPGA_CLK1_50),

		// HPS DDR-3 SDRAM pin connections
		.memory_mem_a(HPS_DDR3_ADDR),
		.memory_mem_ba(HPS_DDR3_BA),
		.memory_mem_ck(HPS_DDR3_CK_P),
		.memory_mem_ck_n(HPS_DDR3_CK_N),
		.memory_mem_cke(HPS_DDR3_CKE),
		.memory_mem_cs_n(HPS_DDR3_CS_N),
		.memory_mem_ras_n(HPS_DDR3_RAS_N),
		.memory_mem_cas_n(HPS_DDR3_CAS_N),
		.memory_mem_we_n(HPS_DDR3_WE_N),
		.memory_mem_reset_n(HPS_DDR3_RESET_N),
		.memory_mem_dq(HPS_DDR3_DQ),
		.memory_mem_dqs(HPS_DDR3_DQS_P),
		.memory_mem_dqs_n(HPS_DDR3_DQS_N),
		.memory_mem_odt(HPS_DDR3_ODT),
		.memory_mem_dm(HPS_DDR3_DM),
		.memory_oct_rzqin(HPS_DDR3_RZQ),
		
		// HPS SD-card pin connections
		.hps_io_hps_io_sdio_inst_CMD(HPS_SD_CMD),
		.hps_io_hps_io_sdio_inst_D0(HPS_SD_DATA[0]),
		.hps_io_hps_io_sdio_inst_D1(HPS_SD_DATA[1]),
		.hps_io_hps_io_sdio_inst_CLK(HPS_SD_CLK),
		.hps_io_hps_io_sdio_inst_D2(HPS_SD_DATA[2]),
		.hps_io_hps_io_sdio_inst_D3(HPS_SD_DATA[3]),
		
		// HPS EMAC1 (Ethernet) pin connections
		.hps_io_hps_io_emac1_inst_TX_CLK(HPS_ENET_GTX_CLK),
		.hps_io_hps_io_emac1_inst_TXD0(HPS_ENET_TX_DATA[0]),
		.hps_io_hps_io_emac1_inst_TXD1(HPS_ENET_TX_DATA[1]),
		.hps_io_hps_io_emac1_inst_TXD2(HPS_ENET_TX_DATA[2]),
		.hps_io_hps_io_emac1_inst_TXD3(HPS_ENET_TX_DATA[3]),
		.hps_io_hps_io_emac1_inst_RXD0(HPS_ENET_RX_DATA[0]),
		.hps_io_hps_io_emac1_inst_MDIO(HPS_ENET_MDIO),
		.hps_io_hps_io_emac1_inst_MDC(HPS_ENET_MDC),
		.hps_io_hps_io_emac1_inst_RX_CTL(HPS_ENET_RX_DV),
		.hps_io_hps_io_emac1_inst_TX_CTL(HPS_ENET_TX_EN),
		.hps_io_hps_io_emac1_inst_RX_CLK(HPS_ENET_RX_CLK),
		.hps_io_hps_io_emac1_inst_RXD1(HPS_ENET_RX_DATA[1]),
		.hps_io_hps_io_emac1_inst_RXD2(HPS_ENET_RX_DATA[2]),
		.hps_io_hps_io_emac1_inst_RXD3(HPS_ENET_RX_DATA[3]),

		// HPS USB1 2.0 OTG pin connections
		.hps_io_hps_io_usb1_inst_D0(HPS_USB_DATA[0]),
		.hps_io_hps_io_usb1_inst_D1(HPS_USB_DATA[1]),
		.hps_io_hps_io_usb1_inst_D2(HPS_USB_DATA[2]),
		.hps_io_hps_io_usb1_inst_D3(HPS_USB_DATA[3]),
		.hps_io_hps_io_usb1_inst_D4(HPS_USB_DATA[4]),
		.hps_io_hps_io_usb1_inst_D5(HPS_USB_DATA[5]),
		.hps_io_hps_io_usb1_inst_D6(HPS_USB_DATA[6]),
		.hps_io_hps_io_usb1_inst_D7(HPS_USB_DATA[7]),
		.hps_io_hps_io_usb1_inst_CLK(HPS_USB_CLKOUT),
		.hps_io_hps_io_usb1_inst_STP(HPS_USB_STP),
		.hps_io_hps_io_usb1_inst_DIR(HPS_USB_DIR),
		.hps_io_hps_io_usb1_inst_NXT(HPS_USB_NXT),
		
		// HPS SPI1 pin connections
		.hps_io_hps_io_spim1_inst_CLK(HPS_SPIM_CLK),
		.hps_io_hps_io_spim1_inst_MOSI(HPS_SPIM_MOSI),
		.hps_io_hps_io_spim1_inst_MISO(HPS_SPIM_MISO),
		.hps_io_hps_io_spim1_inst_SS0(HPS_SPIM_SS),
		
		// HPS I2C0 pin connections
		.hps_io_hps_io_i2c0_inst_SDA(HPS_I2C0_SDAT),
		.hps_io_hps_io_i2c0_inst_SCL(HPS_I2C0_SCLK),
		
		// HPS I2C1 pin connections
		.hps_io_hps_io_i2c1_inst_SDA(HPS_I2C1_SDAT),
		.hps_io_hps_io_i2c1_inst_SCL(HPS_I2C1_SCLK),
		
		// HPS GPIO pin connections
		.hps_io_hps_io_gpio_inst_GPIO09(HPS_CONV_USB_N),
		.hps_io_hps_io_gpio_inst_GPIO35(HPS_ENET_INT_N),
		//.hps_io_hps_io_gpio_inst_GPIO40(HPS_LTC_GPIO),
		.hps_io_hps_io_gpio_inst_GPIO54(HPS_KEY),
		.hps_io_hps_io_gpio_inst_GPIO61(HPS_GSENSOR_INT),
		
		// HPS loan I/O IP ports
		.hps_h2f_loan_io_in(loanio_in),
		.hps_h2f_loan_io_out(loanio_out),
		.hps_h2f_loan_io_oe(loanio_oe),

		// HPS loan I/O pin connections
		.hps_io_hps_io_gpio_inst_LOANIO49(HPS_UART_RX),
		.hps_io_hps_io_gpio_inst_LOANIO50(HPS_UART_TX),
		.hps_io_hps_io_gpio_inst_LOANIO53(HPS_LED),

		// Reset
		.hps_0_h2f_reset_reset_n(hps_reset_n),
		.reset_reset_n(hps_reset_n)
	);
	
	// ============
	// Push buttons
	// ============
	
	wire debounced_key0;
	wire debounced_key1;
	debounce #(
		.CLK_CNT_WIDTH(21),
		.SW_WIDTH(2)
	)
	debounce_0(
		.rst_n(master_reset_n),
		.clk(FPGA_CLK1_50),
		.div(1250000),
		.sw_in({ ~KEY[1], ~KEY[0] }),
		.sw_out({ debounced_key1, debounced_key0 })
	);
	
	// ====
	// LEDs
	// ====
	
	assign LED[0] = debounced_key0;
	assign LED[1] = debounced_key1;
	assign LED[7] = heart_beat[25];
	
	// ===========
	// LED blinker
	// ===========
	
	reg [25:0] heart_beat;
	always @ (posedge FPGA_CLK1_50 or negedge master_reset_n) begin
		if(!master_reset_n) begin
			heart_beat <= 0;
		end
		else begin
			heart_beat <= heart_beat + 1;
		end
	end
	
	// ===========================================================================================
	// UART transmitter (by Dan Gisselquist, see https://zipcpu.com/formal/2019/02/21/txuart.html)
	// ===========================================================================================
	
	reg       uart_tx_wr;
	reg [7:0] uart_tx_data;
	wire      uart_tx;
	wire      uart_tx_busy;
	txuartlite #(
		.CLOCKS_PER_BAUD(434)  // = clk / baud = 50000000 / 115200
	)
	txuartlite_0(
		.i_clk(FPGA_CLK1_50),
		.i_reset(!master_reset_n),
		.i_wr(uart_tx_wr),
		.i_data(uart_tx_data),
		.o_uart_tx(uart_tx),
		.o_busy(uart_tx_busy)
	);
	
	// ===================
	// Configure loan I/Os
	// ===================
	
	// Set pin directions
	assign loanio_oe[48:0]  = 0;  // Unused pins as inputs
	assign loanio_oe[49]    = 0;  // UART_RX pin as input (LoanIO 49) *WARNING: do not set this to 1
	assign loanio_oe[50]    = 1;  // UART_TX pin as output (LoanIO 50)
	assign loanio_oe[52:51] = 0;  // Unused pins as inputs
	assign loanio_oe[53]    = 1;  // HPS_LED pin as output (LoanIO 53)
	assign loanio_oe[66:54] = 0;  // Unused pins as inputs
	
	// Set I/O pin values
	assign uart_rx = loanio_out[49];         // Read from pin
	assign loanio_out[50] = uart_tx;         // Write to pin
	assign loanio_out[53] = debounced_key0;  // Write to pin
	
	// =======================================================================
	// Main state machine that waits for key press and then sends UART message
	// =======================================================================
	
	// UART messages, etc..
	localparam UART_HELLO_MSG_LEN = 26;
	localparam [8*UART_HELLO_MSG_LEN-1:0] uart_hello_msg = "Hello from the FPGA side\r\n";
	integer uart_msg_counter;
	reg [7:0] state;
	always @ (posedge FPGA_CLK1_50 or negedge master_reset_n) begin
		// STATE: Reset?
		if(!master_reset_n) begin
			uart_tx_wr <= 0;
			state <= 0;
		end
		else begin

			case(state)
				
				// Wait for key press
				0: begin
					if(debounced_key0) begin
						state <= state + 1;
					end
				end
				
				// Wait for key depress
				1: begin
					if(debounced_key0 == 0) begin
						uart_msg_counter <= UART_HELLO_MSG_LEN - 1;
						state <= state + 1;
					end
				end
				
				// Wait until UART is not busy, then transmit message (loop for each character until done)
				2: begin
					if(!uart_tx_busy) begin
						if(uart_msg_counter != -1) begin
							uart_tx_data <= uart_hello_msg[8*uart_msg_counter +: 8];
							uart_tx_wr <= 1;
							uart_msg_counter <= uart_msg_counter - 1;
							state <= state + 1;
						end
						else begin
							state <= 0;
						end
					end
				end
				
				// Wait until UART is busy, i.e. until transmit starts
				3: begin
					if(uart_tx_busy) begin
						uart_tx_wr <= 0;
						state <= state - 1;
					end
				end

			endcase
		end
	end
endmodule
