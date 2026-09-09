`timescale 1ns/1ps

module reset_sync(
    input      clk,
    input      rst_in,
    output     rst_out
);
    reg [1:0] rst_chain;

    assign rst_out = rst_chain[1];

    always @(posedge clk or posedge rst_in) begin
        if (rst_in)
            rst_chain <= 2'b11;               // async assert, both flops
        else
            rst_chain <= {rst_chain[0], 1'b0};// shift a zero in on release
    end
endmodule

module slave_input_regs(
    input        clk,
    input        rst,
    input        cs,
    input  [7:0] io,
    output reg [1:0] cs_reg,
    output reg [7:0] io_reg
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cs_reg <= 2'b11;            
            io_reg <= 8'h00;
        end else begin
            cs_reg <= {cs_reg[0], cs};
            io_reg <= io;
        end
    end
endmodule


// =====================================================================
//  Slave sequencer
//  CHANGED: state updates gated by `active` so synthesis can infer
//           clock gating. Behaviour is unchanged — every branch that
//           could fire already required cs to be low.
// =====================================================================
module slave_sequencer(
    input        clk,
    input        rst,
    input  [1:0] cs_reg,
    input  [7:0] io_reg,
    input        addr_overflow,
    input        read_pending,
    output       active,                    // NEW — shared enable
    output       load_addr_high,
    output       load_addr_low,
    output       write_byte,
    output       read_frame_done,
    output       frame_valid
);
    reg [1:0] step;
    reg       mode_bit;
    reg       valid;

    wire frame_start, frame_body, cs_end, good_preamble;

    // Alive while cs is low, for one cycle after it rises so cs_end can
    // be seen, and throughout a pending read so the prefetch can run.
    assign active = ~cs_reg[0] | ~cs_reg[1] | read_pending;

    assign frame_start  =  cs_reg[1] & ~cs_reg[0] & ~read_pending;
    assign frame_body   = ~cs_reg[1] & ~cs_reg[0] & ~read_pending;
    assign cs_end       = ~cs_reg[1] &  cs_reg[0];
    assign good_preamble = (io_reg[7:4] == 4'hA);
    assign frame_valid  =  valid;

    assign read_frame_done = cs_end & mode_bit & valid & step[1];
    assign load_addr_high  = frame_body & ~step[0] & valid;
    assign load_addr_low   = frame_body &  step[0] & ~step[1] & valid;
    assign write_byte      = frame_body &  step[1]
                           & ~mode_bit & ~addr_overflow & valid;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            step     <= 2'b00;
            mode_bit <= 1'b0;
            valid    <= 1'b0;
        end else if (active) begin              // NEW
            if (frame_start) begin
                step     <= 2'b00;
                mode_bit <= io_reg[0];
                valid    <= good_preamble;
            end else if (~cs_reg[0] & ~read_pending) begin
                step     <= {step[0], 1'b1};
            end
        end
    end
endmodule

// =====================================================================
//  Slave address counter
//  CHANGED: gated by `active`
// =====================================================================
module slave_addr_counter(
    input         clk,
    input         rst,
    input         active,                   // NEW
    input         load_addr_high,
    input         load_addr_low,
    input         write_byte,
    input         rd_advance,
    input   [7:0] io_reg,
    output [12:0] addr,
    output        addr_overflow
);
    reg [13:0] addr_count;

    assign addr          = addr_count[12:0];
    assign addr_overflow = addr_count[13];

    always @(posedge clk or posedge rst) begin
        if (rst)
            addr_count <= 14'd0;
        else if (active) begin                  // NEW
            if (load_addr_high)
                addr_count[13:8] <= {1'b0, io_reg[4:0]};
            else if (load_addr_low)
                addr_count[7:0]  <= io_reg;
            else if (write_byte | rd_advance)
                addr_count <= addr_count + 1'b1;
        end
    end
endmodule

// =====================================================================
//  Slave read controller
//  CHANGED: gated by `active`
//  NOTE: `active` includes read_pending, so the prefetch during the
//        gap between read frames still runs with cs high.
// =====================================================================
module slave_read_ctrl(
    input        clk,
    input        rst,
    input        active,                    // NEW
    input  [1:0] cs_reg,
    input        read_frame_done,
    input  [7:0] sram_rdata,
    input        addr_overflow,
    output       read_pending,
    output       sram_rd_en,
    output       slave_oe,
    output reg [7:0] io_out,
    output       addr_advance,
    output       busy
);
    reg pending;
    reg prefetch_wait;
    reg data_ready;

    wire cs_end, read_active, prefetch, rd_advance;

    assign cs_end       = ~cs_reg[1] & cs_reg[0];
    assign read_pending =  pending;
    assign read_active  =  pending & ~cs_reg[0];
    assign prefetch     =  pending & prefetch_wait & ~data_ready;

    assign rd_advance   =  prefetch | read_active;
    assign addr_advance =  rd_advance & ~addr_overflow;
    assign sram_rd_en   =  addr_advance;
    assign slave_oe     =  read_active;
    assign busy         =  pending & ~data_ready;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pending       <= 1'b0;
            prefetch_wait <= 1'b0;
            data_ready    <= 1'b0;
            io_out        <= 8'h00;
        end else if (active) begin              // NEW
            prefetch_wait <= pending;

            if (read_frame_done && !pending) begin
                pending    <= 1'b1;
                data_ready <= 1'b0;
            end else if (cs_end && pending) begin
                pending    <= 1'b0;
                data_ready <= 1'b0;
            end

            if (prefetch)
                data_ready <= 1'b1;

            if (rd_advance)
                io_out <= sram_rdata;
        end
    end
endmodule

module slave_core(
    input         clk,
    input         rst,
    input         cs,
    input   [7:0] io_in,
    output  [7:0] io_out,
    output        io_oe,
    output        frame_valid,
    output        busy,                
    output [12:0] sram_addr,
    output  [7:0] sram_data,
    output        sram_write,
    output        sram_rd_en,           
    input   [7:0] sram_rdata            
);
    wire [1:0] cs_reg;
    wire [7:0] io_reg;
    wire       load_addr_high, load_addr_low, addr_overflow;
    wire       read_pending, rd_advance, addr_advance, read_frame_done, active;
    wire       rst_int;

    assign sram_data = io_reg;

    reset_sync u_rst (
        .clk(clk), .rst_in(rst), .rst_out(rst_int)
    );

    slave_input_regs u_in (
        .clk(clk), .rst(rst_int), .cs(cs), .io(io_in),
        .cs_reg(cs_reg), .io_reg(io_reg)
    );

    slave_sequencer u_seq (
        .clk(clk), .rst(rst_int), .cs_reg(cs_reg), .io_reg(io_reg),
        .addr_overflow(addr_overflow), .read_pending(read_pending),
        .active(active),                                    // NEW
        .load_addr_high(load_addr_high), .load_addr_low(load_addr_low),
        .write_byte(sram_write), .read_frame_done(read_frame_done),
        .frame_valid(frame_valid)
    );

    slave_addr_counter u_addr (
        .clk(clk), .rst(rst_int), .active(active),          // NEW
        .load_addr_high(load_addr_high), .load_addr_low(load_addr_low),
        .write_byte(sram_write), .rd_advance(addr_advance), .io_reg(io_reg),
        .addr(sram_addr), .addr_overflow(addr_overflow)
    );

    slave_read_ctrl u_rd (
        .clk(clk), .rst(rst_int), .active(active),          // NEW
        .cs_reg(cs_reg),
        .read_frame_done(read_frame_done),
        .addr_overflow(addr_overflow),
        .sram_rdata(sram_rdata),
        .read_pending(read_pending), .addr_advance(addr_advance),
        .sram_rd_en(sram_rd_en), .slave_oe(io_oe),
        .io_out(io_out), .busy(busy)
    );
endmodule