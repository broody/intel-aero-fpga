// ----------------------------------------------------------------------------
// This reference design and source code is being provided on an "as-is" basis 
// and as an accommodation, and therefore all warranties, representations or 
// guarantees of any kind (whether express, implied or statutory) including, 
// without limitation, warranties of merchantability, non-infringement, or  
// fitness for a particular purpose, are specifically disclaimed.
//
// ----------------------------------------------------------------------------

`timescale 1ns/1ps

//--------------------------
//    Declaration and ports
//-----------
module Top (
        // global
        in_CLK,

        // Shared I2C(ADC and external)
        FC1_I2C_CLK,
        FC1_I2C_SDA,

        // I2C external compass
        FC1_COMPASS_CLK,
        FC1_COMPASS_SDA,
        IO_COMPASS_CLK,
        IO_COMPASS_SDA,

        // UART motors
        IO_MOTORS_Tx,
        IO_MOTORS_Rx,
        FC1_MOTORS_SCL_Tx,
        FC1_MOTORS_SDA_Rx,

        // UART GPS
        IO_GPS_Tx,
        IO_GPS_Rx,
        FC1_GPS_Tx,
        FC1_GPS_Rx,

        // UART RC receiver
        IO_REC_Rx,
        IO_REC_Tx,
        FC1_IO3_REC_Rx,
        FC1_XBEE_CTS_REC_Tx,

        // UART CHT<>FC
        FC1_XBEE_Rx,
        FC1_XBEE_Tx,
        CHT_DBG_UART_Tx,
        CHT_DBG_UART_Rx,

        // UART telemetry
        IO_TELEM_Tx,
        IO_TELEM_Rx,
        FC1_TELEM_Tx,
        FC1_TELEM_Rx,

        // SPI CHT
        SPI_SCLK,
        SPI_MISO,
        SPI_MOSI,
        SPI_SS,

        // Telemetry I2C
        IO_TELEM_I2C_CLK,
        IO_TELEM_I2C_SDA,

        BOOTLOADER_FORCE_PIN,

        // FPGA I2C MASTER
        FPGA_I2C_SDA,
        FPGA_I2C_SCL,

        // FPGA GPIOs
        FPGA_GPIO_INPUT,
    	FPGA_GPIO_OUTPUT,

        // FPGA PWM
        FPGA_PWM_OUT,

        // FPGA UART,
        FPGA_UART_TX,
        FPGA_UART_RX,
);

parameter fpga_ver = 8'hC2;

// global
input wire in_CLK;

// Shared I2C
input wire FC1_I2C_CLK;
inout wire FC1_I2C_SDA;

// External compass I2C
output wire IO_COMPASS_CLK;
inout wire IO_COMPASS_SDA;
input wire FC1_COMPASS_CLK;
inout wire FC1_COMPASS_SDA;

output wire IO_MOTORS_Tx;
input  wire IO_MOTORS_Rx;
input  wire FC1_MOTORS_SCL_Tx;
output wire FC1_MOTORS_SDA_Rx;

output wire IO_GPS_Tx;
input  wire IO_GPS_Rx;
input  wire FC1_GPS_Tx;
output wire FC1_GPS_Rx;

input  wire IO_REC_Rx;
output wire IO_REC_Tx;
output wire FC1_IO3_REC_Rx;
input  wire FC1_XBEE_CTS_REC_Tx;

input  wire FC1_XBEE_Tx;
output wire FC1_XBEE_Rx;
input  wire CHT_DBG_UART_Tx;
output wire CHT_DBG_UART_Rx;

output wire IO_TELEM_Tx;
input  wire IO_TELEM_Rx;
input  wire FC1_TELEM_Tx;
output wire FC1_TELEM_Rx;

input wire SPI_SCLK;
output wire SPI_MISO;
input wire SPI_MOSI;
input wire SPI_SS;

output wire IO_TELEM_I2C_CLK;
inout wire IO_TELEM_I2C_SDA;

input wire [1:0] FPGA_GPIO_INPUT;

output wire [3:0] FPGA_GPIO_OUTPUT;
output wire [1:0] FPGA_PWM_OUT;
output wire FPGA_UART_TX;
input wire FPGA_UART_RX;
output wire FPGA_I2C_SCL;
inout wire FPGA_I2C_SDA;


output reg BOOTLOADER_FORCE_PIN = 0;

assign FC1_MOTORS_SDA_Rx = IO_MOTORS_Rx;
assign IO_MOTORS_Tx = FC1_MOTORS_SCL_Tx;
assign IO_GPS_Tx = FC1_GPS_Tx;
assign FC1_GPS_Rx = IO_GPS_Rx;
assign IO_REC_Tx = FC1_XBEE_CTS_REC_Tx;
assign FC1_IO3_REC_Rx = IO_REC_Rx;
assign CHT_DBG_UART_Rx = FC1_XBEE_Tx;
assign FC1_XBEE_Rx = CHT_DBG_UART_Tx;
assign FC1_TELEM_Rx = IO_TELEM_Rx;
assign IO_TELEM_Tx = FC1_TELEM_Tx;
assign FPGA_GPIO_OUTPUT = OUTPUT_GPIO_REG;



//--------------------------
// Regs and Wires
//-----------
reg reset_n0;
reg reset_n;
wire clk_core;
wire clk_adc;
wire clk_pwm;
wire locked;

//--------------------------
//    PLL
//-----------
pll pll_inst (
    .inclk0(in_CLK),
    .c0( clk_adc ),  // 10 MHz clock dedicated to ADC
    .c1( clk_core ), // 25 MHz sytem clock
    .c2( clk_pwm ),  // 1 Mhz
    .locked(locked)
);

//--------------------------
// Reset - Synchronous deassert after PLL locked
//-----------
// Synchronize Reset
always @(posedge in_CLK) begin
    if (!locked) begin
        reset_n0 <= 1'b0;
        reset_n <= 1'b0;
    end else begin
        reset_n0 <= 1'b1;
        reset_n <= reset_n0;
    end
end

// I2C in telemetry connector
i2c_bridge_new i2c_telemetry_connector_bridge_inst(
    .clk(clk_core),
    .master_sda(FC1_I2C_SDA),
    .master_clk(FC1_I2C_CLK),
    .slave_sda(IO_TELEM_I2C_SDA),
    .slave_clk(IO_TELEM_I2C_CLK)
);

// I2C bridge external compass
i2c_bridge_new i2c_external_compass_bridge_inst(
    .clk(clk_core),
    .master_sda(FC1_COMPASS_SDA),
    .master_clk(FC1_COMPASS_CLK),
    .slave_sda(IO_COMPASS_SDA),
    .slave_clk(IO_COMPASS_CLK)
);

// ADC
adc_state_machine adc_inst(
    .clk_core(clk_core),
    .clk_adc(clk_adc),
    .i2c_clk(FC1_I2C_CLK),
    .i2c_sda(FC1_I2C_SDA),
    .adc_chan0(ADC_CHAN0),
    .adc_chan1(ADC_CHAN1),
    .adc_chan2(ADC_CHAN2),
    .adc_chan3(ADC_CHAN3),
    .adc_chan4(ADC_CHAN4),
    .reset_n(reset_n),
    .locked(locked)
);

wire [15:0] ADC_CHAN0;
wire [15:0] ADC_CHAN1;
wire [15:0] ADC_CHAN2;
wire [15:0] ADC_CHAN3;
wire [15:0] ADC_CHAN4;


// FPGA I2C Master
reg [4:0] I2C_STS_REG = 0;
reg [7:0] I2C0_BAUD_L = 0;
reg [7:0] I2C0_BAUD_H = 0;
reg [7:0] I2C0_TX_REG = 0;

wire [7:0] i2c0_rx_reg;
reg i2c_tx_on;
reg i2c_busy_r;

wire i2c_start;
wire i2c_stop;
wire i2c_rx_on;
wire i2c_ack;
wire i2c_busy;

assign i2c_start = I2C_STS_REG[0];
assign i2c_stop = I2C_STS_REG[1];
assign i2c_rx_on = I2C_STS_REG[2];

i2c_master i2c0_inst (
    .clk(clk_core),
    .resetn(reset_n),
    .i2c_baud_clk_l(I2C0_BAUD_L),
    .i2c_baud_clk_h(I2C0_BAUD_H),
    .i2c_sda(FPGA_I2C_SDA),
    .i2c_scl(FPGA_I2C_SCL),
    .i2c_tx_on(i2c_tx_on),
    .i2c_start(i2c_start),
    .i2c_stop(i2c_stop),    
    .i2c_rx_on(i2c_rx_on),
    .i2c_busy(i2c_busy),
    .i2c_ack(i2c_ack),
    .i2c_tx_reg(I2C0_TX_REG),
    .i2c_rx_reg(i2c0_rx_reg)
);

// FPGA LED
reg [3:0] OUTPUT_GPIO_REG = 0;

// FPGA UART
reg [7:0] UART0_BAUD_L = 0;
reg [7:0] UART0_BAUD_H = 0;
reg [7:0] UART0_TX_REG = 0;
wire [7:0] uart0_rx_reg;

reg uart0_tx_on;
reg uart0_rx_on;
wire uart0_tx_busy;
wire uart0_rx_dat_rdy;

uart  uart0_inst (
    .clk(clk_core),
    .resetn(reset_n),  
    .uart_baud_regl(UART0_BAUD_L),
    .uart_baud_regh(UART0_BAUD_H),
    .uart_tx_reg(UART0_TX_REG),
    .uart_rx_reg(uart0_rx_reg),
    .uart_txd(FPGA_UART_TX),
    .uart_rxd(FPGA_UART_RX),
    .uart_tx_on(uart0_tx_on),
    .uart_rx_dat_rdy(uart0_rx_dat_rdy),
    .uart_tx_busy(uart0_tx_busy)
);

// FPGA PWM
reg [7:0] PWM_STS_REG = 0;
wire PWM0_ON = PWM_STS_REG[0];
wire PWM1_ON = PWM_STS_REG[1];

reg [7:0] PWM0_DUTY_L = 0;
reg [7:0] PWM0_DUTY_H = 0;
reg [7:0] PWM0_FREQ_L = 0;
reg [7:0] PWM0_FREQ_H = 0;
reg [7:0] PWM1_DUTY_L = 0;
reg [7:0] PWM1_DUTY_H = 0;
reg [7:0] PWM1_FREQ_L = 0;
reg [7:0] PWM1_FREQ_H = 0;


pwm pwm0_inst (
    .clk(clk_pwm),
    .resetn(reset_n),
    .pwm(FPGA_PWM_OUT[0]),
    .pwm_on(PWM0_ON),
    .pwmDutyl(PWM0_DUTY_L),
    .pwmDutyh(PWM0_DUTY_H),
    .pwmFreql(PWM0_FREQ_L),
    .pwmFreqh(PWM0_FREQ_H)
);

pwm pwm1_inst (
    .clk(clk_pwm),
    .resetn(reset_n),
    .pwm(FPGA_PWM_OUT[1]),
    .pwm_on(PWM1_ON),
    .pwmDutyl(PWM1_DUTY_L),
    .pwmDutyh(PWM1_DUTY_H),
    .pwmFreql(PWM1_FREQ_L),
    .pwmFreqh(PWM1_FREQ_H)
);


// SPI

wire spi_rx_byte_available;
wire [7 : 0] spi_rx_byte;
wire spi_tx_ready_to_write;
reg [7 : 0] spi_tx_byte;

// SPI state machine
reg waiting_reg = 0;
reg [7 : 0] reg_received = 0;
reg [1 :0] spi_rx_byte_available_reg;
wire spi_rx_byte_available_rissing_edge;
wire spi_transaction_begin;

spi_slave spi0_inst(
    .clk(clk_core),
    .sclk(SPI_SCLK),
    .miso(SPI_MISO),
    .mosi(SPI_MOSI),
    .ss(SPI_SS),
    .rx_byte_available(spi_rx_byte_available),
    .rx_byte(spi_rx_byte),
    .tx_byte_ready_to_write(spi_tx_ready_to_write),
    .tx_byte(spi_tx_byte),
    .transaction_begin(spi_transaction_begin)
);

assign spi_rx_byte_available_rissing_edge = (spi_rx_byte_available_reg == 2'b01);

// helper to detect edges
always @ (posedge clk_core) begin
    spi_rx_byte_available_reg[0] <= spi_rx_byte_available;
    spi_rx_byte_available_reg[1] <= spi_rx_byte_available_reg[0];
end

// to keep it compatible with read version operation the 8th bit of first byte
// will be used this way:
// - set to 0 for read
// - set to 1 for write
//
// This has to be ORed with the register in the table below according to the
// operation. Bellow is the compatibleurrent register table (using 7bit register addressing).
// All other registers are reserved for future use.
//
// -----------------------------------------------------------------------------------------------|
// | Reg  | Name             | Mode | Description                                                 |
// |----------------------------------------------------------------------------------------------|
// | 0x00 | FPGA_FW_VERSION  |  RO  | FPGA firmware version                                       |
// | 0x01 | AEROFC_FORCE_BT  |  RW  | Pin state to force aerofc to stay on bootloader             |
// -----------------------------------------------------------------------------------------------|
// |                                 IO EXPANSION BOARD SPECIFIC                                  |
// -----------------------------------------------------------------------------------------------|
// | 0x02 | OUTPUT_GPIO_REG  |  RW  | Controls GPIOs and LED on J9 and J11.4                      |
// | 0x03 | INPUT_GPIO_REG   |  RO  | Schmitt Trigger input GPIO                                  |
// | 0x04 | PWM_STS_REG      |  RW  | Bit0: PWM0 on/off, Bit 1: PWM1 on/off                       |
// | 0x05 | UART_STS_REG     |  RO  | Bit0: TX is busy, Bit1: RX has data                         |
// | 0x06 | I2C_STS_REG      |  RO  | Bit0: start, Bit1: stop, Bit2: rx_on, Bit3: ack, Bit4: busy |
// | 0x10 | PWM0_FREQ_L      |  RW  |                                                             |
// | 0x11 | PWM0_FREQ_H      |  RW  |                                                             |
// | 0x12 | PWM0_DUTY_L      |  RW  |                                                             |
// | 0x13 | PWM0_DUTY_H      |  RW  |                                                             |
// | 0x14 | PWM1_FREQ_L      |  RW  |                                                             |
// | 0x15 | PWM1_FREQ_H      |  RW  |                                                             |
// | 0x16 | PWM1_DUTY_L      |  RW  |                                                             |
// | 0x17 | PWM1_DUTY_H      |  RW  |                                                             |
// | 0x20 | UART0_BAUD_L     |  RW  |                                                             |
// | 0x21 | UART0_BAUD_H     |  RW  |                                                             |
// | 0x22 | UART0_TX_REG     |  RW  |                                                             |
// | 0x23 | UART0_RX_REG     |  RO  |                                                             |
// | 0x30 | I2C0_BAUD_L      |  RW  |                                                             |
// | 0x31 | I2C0_BAUD_H      |  RW  |                                                             |
// | 0x32 | I2C0_TX_REG      |  RW  |                                                             |
// | 0x33 | I2C0_RX_REG      |  RW  |                                                             |
// | 0x40 | ADC_CHAN0_L      |  RW  |                                                             |
// | 0x41 | ADC_CHAN0_H      |  RW  |                                                             |
// | 0x42 | ADC_CHAN1_L      |  RW  |                                                             |
// | 0x43 | ADC_CHAN1_H      |  RW  |                                                             |
// | 0x44 | ADC_CHAN2_L      |  RW  |                                                             |
// | 0x45 | ADC_CHAN2_H      |  RW  |                                                             |
// | 0x46 | ADC_CHAN3_L      |  RW  |                                                             |
// | 0x47 | ADC_CHAN3_H      |  RW  |                                                             |
// -----------------------------------------------------------------------------------------------|

parameter fpga_ver_read_reg = 7'd0;
parameter fpga_bootloader_pin_reg = 7'd1;
parameter fpga_output_gpio_reg = 4'd2;
parameter fpga_input_gpio_reg = 4'd3;
parameter fpga_pwm_sts_reg = 7'd4;
parameter fpga_uart_sts_reg = 7'd5;
parameter fpga_i2c_sts_reg = 7'd6;
parameter fpga_pwm0_freq_l = 7'h10;
parameter fpga_pwm0_freq_h = 7'h11;
parameter fpga_pwm0_duty_l = 7'h12;
parameter fpga_pwm0_duty_h = 7'h13;
parameter fpga_pwm1_freq_l = 7'h14;
parameter fpga_pwm1_freq_h = 7'h15;
parameter fpga_pwm1_duty_l = 7'h16;
parameter fpga_pwm1_duty_h = 7'h17;
parameter fpga_uart0_baud_l = 7'h20;
parameter fpga_uart0_baud_h = 7'h21;
parameter fpga_uart0_tx_reg = 7'h22;
parameter fpga_uart0_rx_reg = 7'h23;
parameter fpga_i2c0_baud_l = 7'h30;
parameter fpga_i2c0_baud_h = 7'h31;
parameter fpga_i2c0_tx_reg = 7'h32;
parameter fpga_i2c0_rx_reg = 7'h33;
parameter fpga_adc_chan0_l = 7'h40;
parameter fpga_adc_chan0_h = 7'h41;
parameter fpga_adc_chan1_l = 7'h42;
parameter fpga_adc_chan1_h = 7'h43;
parameter fpga_adc_chan2_l = 7'h44;
parameter fpga_adc_chan2_h = 7'h45;
parameter fpga_adc_chan3_l = 7'h46;
parameter fpga_adc_chan3_h = 7'h47;
parameter fpga_adc_chan4_l = 7'h48;
parameter fpga_adc_chan4_h = 7'h49;

// SPI state machine
always @ (posedge clk_core) begin
    if(uart0_tx_busy) 
        uart0_tx_on <= 1'b0;

    if(uart0_rx_dat_rdy)
        uart0_rx_on <= 1'b1;

    // #1 No idea why this needs to be done
    if(i2c_busy_r && ~i2c_busy) begin
        I2C_STS_REG[2:0] <= 3'b000;
        i2c_tx_on <= 1'b0;
    end

    if (spi_transaction_begin) begin
        waiting_reg <= 1;
        spi_tx_byte <= 0;
    end else if (spi_rx_byte_available_rissing_edge) begin
        if (waiting_reg) begin
            waiting_reg <= 0;
            reg_received <= spi_rx_byte;

            // is a read operation?
            if (spi_rx_byte[7] == 0) begin
                case (spi_rx_byte[6:0])
                    fpga_ver_read_reg:
                        spi_tx_byte <= fpga_ver;
                    fpga_bootloader_pin_reg:
                        spi_tx_byte[0] <= BOOTLOADER_FORCE_PIN;
                    fpga_output_gpio_reg: 
                        spi_tx_byte <= OUTPUT_GPIO_REG;
                    fpga_input_gpio_reg:
                        spi_tx_byte[1:0] <= FPGA_GPIO_INPUT;
                    fpga_pwm_sts_reg:
                        spi_tx_byte <= PWM_STS_REG;
                    fpga_pwm0_duty_l:
                        spi_tx_byte <= PWM0_DUTY_L;
                    fpga_pwm0_duty_h:
                        spi_tx_byte <= PWM0_DUTY_H;
                    fpga_pwm0_freq_l:
                        spi_tx_byte <= PWM0_FREQ_L;
                    fpga_pwm0_freq_h:
                        spi_tx_byte <= PWM0_FREQ_H;
                    fpga_pwm1_duty_l:
                        spi_tx_byte <= PWM1_DUTY_L;
                    fpga_pwm1_duty_h:
                        spi_tx_byte <= PWM1_DUTY_H;
                    fpga_pwm1_freq_l:
                        spi_tx_byte <= PWM1_FREQ_L;
                    fpga_pwm1_freq_h:
                        spi_tx_byte <= PWM1_FREQ_H;
                    fpga_i2c_sts_reg:
                        spi_tx_byte [4:0] <= {i2c_busy, i2c_ack, i2c_rx_on, 1'b0, 1'b0};
                    fpga_i2c0_baud_l:
                        spi_tx_byte <= I2C0_BAUD_L;
                    fpga_i2c0_baud_h:
                        spi_tx_byte <= I2C0_BAUD_H;
                    fpga_i2c0_tx_reg:
                        spi_tx_byte <= I2C0_TX_REG;
                    fpga_i2c0_rx_reg:
                        spi_tx_byte <= i2c0_rx_reg;
                    fpga_uart_sts_reg:
                        spi_tx_byte[1:0] <= {uart0_tx_busy,uart0_rx_on};
                    fpga_uart0_baud_l:
                        spi_tx_byte <= UART0_BAUD_L;
                    fpga_uart0_baud_h:
                        spi_tx_byte <= UART0_BAUD_H;
                    fpga_uart0_tx_reg:
                        spi_tx_byte <= UART0_TX_REG;
                    fpga_uart0_rx_reg: begin
                        uart0_rx_on <= 1'b0;
                        spi_tx_byte <= uart0_rx_reg;
                        end
                    fpga_adc_chan0_l: 
                        spi_tx_byte <= ADC_CHAN0[7:0];
                    fpga_adc_chan0_h:
                        spi_tx_byte <= ADC_CHAN0[15:8];
				    fpga_adc_chan1_l: 
                        spi_tx_byte <= ADC_CHAN1[7:0];
                    fpga_adc_chan1_h:
                        spi_tx_byte <= ADC_CHAN1[15:8];
				    fpga_adc_chan2_l: 
                        spi_tx_byte <= ADC_CHAN2[7:0];
                    fpga_adc_chan2_h:
                        spi_tx_byte <= ADC_CHAN2[15:8];
				    fpga_adc_chan3_l: 
                        spi_tx_byte <= ADC_CHAN3[7:0];
                    fpga_adc_chan3_h:
                        spi_tx_byte <= ADC_CHAN3[15:8];
                    default: 
                        spi_tx_byte <= 7'hFF;
                endcase
            end
        end else begin
            // is a write operation?
            if (reg_received[7] == 1) begin
                case (reg_received[6:0])
                    fpga_bootloader_pin_reg:
                        BOOTLOADER_FORCE_PIN <= spi_rx_byte[0];
                    fpga_output_gpio_reg:
                        OUTPUT_GPIO_REG <= spi_rx_byte;
                    fpga_pwm_sts_reg:
                        PWM_STS_REG <= spi_rx_byte;
                    fpga_pwm0_duty_l:
                        PWM0_DUTY_L <= spi_rx_byte;
                    fpga_pwm0_duty_h:
                        PWM0_DUTY_H <= spi_rx_byte;
                    fpga_pwm0_freq_l:
                        PWM0_FREQ_L <= spi_rx_byte;
                    fpga_pwm0_freq_h:
                        PWM0_FREQ_H <= spi_rx_byte;  
                    fpga_pwm1_duty_l:
                        PWM1_DUTY_L <= spi_rx_byte;
                    fpga_pwm1_duty_h:
                        PWM1_DUTY_H <= spi_rx_byte;
                    fpga_pwm1_freq_l:
                        PWM1_FREQ_L <= spi_rx_byte;
                    fpga_pwm1_freq_h:
                        PWM1_FREQ_H <= spi_rx_byte; 
                    fpga_i2c_sts_reg:
                        I2C_STS_REG[2:0] <= spi_rx_byte;    
                    fpga_i2c0_tx_reg: begin
                        i2c_tx_on <= 1'b1;
                        I2C0_TX_REG <= spi_rx_byte;
                        end
                    fpga_i2c0_baud_l:
                        I2C0_BAUD_L <= spi_rx_byte;
                    fpga_i2c0_baud_h:
                        I2C0_BAUD_H <= spi_rx_byte;
                    fpga_uart0_baud_l:
                        UART0_BAUD_L <= spi_rx_byte;
                    fpga_uart0_baud_h:
                        UART0_BAUD_H <= spi_rx_byte;
                    fpga_uart0_tx_reg: begin
                        UART0_TX_REG <= spi_rx_byte;
                        uart0_tx_on <= 1'b1;
						end
                endcase  
            end
        end
    end
    
end

// #2 No idea why this needs to be done
always @(posedge clk_core)
   i2c_busy_r <= i2c_busy;


endmodule
