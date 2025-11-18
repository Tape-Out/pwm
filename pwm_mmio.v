`ifndef PWM_MMIO_V
`define PWM_MMIO_V

`timescale 1ns/1ps

`ifndef PWM_DEFAULT_FREQ
// period = CLK_FREQ / freq_reg
// percentage = (duty_cycle[i] / period) × 100%
// eg. 50% duty cycle percentage: duty_cycle = period × 50%
//                                   default = 50,000
`define PWM_DEFAULT_FREQ 1000                               // 1kHz default frequency
`endif

`ifndef PWM_MAX_CHANNELS
`define PWM_MAX_CHANNELS 8                                  // Maximum number of PWM channels
`endif

module pwm_mmio #(
    parameter [31:0]  BASE_ADDR     = 32'h8000_2000,
    parameter [31:0]  CLK_FREQ      = 32'd100_000_000,      // 100MHz clock
    parameter integer DEFAULT_FREQ  = `PWM_DEFAULT_FREQ,    // Default PWM frequency
    parameter integer MAX_CHANNELS  = `PWM_MAX_CHANNELS     // Number of PWM channels
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,
    /* verilator lint_on  UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,
    output reg  [31:0]              mem_rdata,

    output reg  [MAX_CHANNELS-1:0]  pwm_out,                // out[i]=1 when counter < duty_cycle[i]

    output reg                      irq,
    input  wire                     eoi
);

    localparam [31:0] RW_REG_CTRL      = BASE_ADDR + 32'h00;
    localparam [31:0] RO_REG_STATUS    = BASE_ADDR + 32'h04;
    localparam [31:0] RW_REG_FREQ      = BASE_ADDR + 32'h08;
    localparam [31:0] RO_REG_PERIOD    = BASE_ADDR + 32'h0C;
    localparam [31:0] RW_REG_DUTY_BASE = BASE_ADDR + 32'h10;

    reg [31:0] ctrl_reg, status_reg, freq_reg, counter, period, duty_cycle [0:MAX_CHANNELS-1];

    wire ctrl_enable    = ctrl_reg[0];    // 1: pwm(global) is enabled
    wire ctrl_irq_en    = ctrl_reg[1];    // 1: trigger an irq when conter > period-1

    // 1: pwm channel${i} is enabled
    wire [MAX_CHANNELS-1:0] ctrl_ch_en = ctrl_reg[8+:MAX_CHANNELS];

    always @(*) begin
        if (freq_reg == 0)
            period = 0;
        else
            period = CLK_FREQ / freq_reg;
    end

    integer i;

    always @(posedge clk) begin: PWM
        if (!resetn) begin
            counter    <= 0;
            pwm_out    <= 0;
            status_reg <= 0;
            irq        <= 0;
        end else begin
            if (eoi) begin
                irq <= 0;
                status_reg[1] <= 0;
            end
            if (ctrl_enable && period > 0) begin
                if (counter >= period - 1) begin
                    counter <= 0;
                    status_reg[0] <= 1;
                    if (ctrl_irq_en && !irq) begin
                        irq <= 1;
                        status_reg[1] <= 1;
                    end
                end else begin
                    counter <= counter + 1;
                end
                for (i = 0; i < MAX_CHANNELS; i = i + 1) begin
                    if (ctrl_ch_en[i]) begin
                        pwm_out[i] <= (counter < duty_cycle[i]);
                    end else begin
                        pwm_out[i] <= 0;
                    end
                end
            end else begin
                counter <= 0;
                pwm_out <= 0;
                status_reg[0] <= 0;
            end
        end
    end

    integer chan;

    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            mem_rdata <= 0;
            mem_ready <= 0;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb == 0) begin
                mem_ready <= 1;
                case (mem_addr)
                    RW_REG_CTRL:    mem_rdata <= ctrl_reg;
                    RO_REG_STATUS:  mem_rdata <= status_reg;
                    RW_REG_FREQ:    mem_rdata <= freq_reg;
                    RO_REG_PERIOD:  mem_rdata <= period;
                    default: begin: READ_DUTY_CYCLE
                        if (mem_addr >= RW_REG_DUTY_BASE &&
                            mem_addr < RW_REG_DUTY_BASE + (MAX_CHANNELS * 4)) begin
                            chan = (mem_addr - RW_REG_DUTY_BASE) >> 2;
                            if (chan < MAX_CHANNELS)
                                mem_rdata <= duty_cycle[chan];
                            else
                                mem_rdata <= 32'h0;
                        end else begin
                            mem_rdata <= 32'h0;
                        end
                    end
                endcase
            end else begin
                mem_rdata <= 0;
                mem_ready <= 0;
            end
        end
    end

    integer j, channel;

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            ctrl_reg <= 0;
            freq_reg <= DEFAULT_FREQ;
            for (j = 0; j < MAX_CHANNELS; j = j + 1) begin
                duty_cycle[j] <= 0;
            end
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb != 0) begin
                mem_ready <= 1;
                case(mem_addr)
                    RW_REG_CTRL:    ctrl_reg <= mem_wdata;
                    RW_REG_FREQ:    freq_reg <= mem_wdata;
                    default: begin: WRITE_DUTY_CYCLE
                        if (mem_addr >= RW_REG_DUTY_BASE &&
                            mem_addr < RW_REG_DUTY_BASE + (MAX_CHANNELS * 4)) begin
                            channel = (mem_addr - RW_REG_DUTY_BASE) >> 2;
                            if (channel < MAX_CHANNELS)
                                duty_cycle[channel] <= mem_wdata;
                        end
                    end
                endcase
            end else begin
                mem_ready <= 0;
            end
        end
    end

endmodule

`endif
