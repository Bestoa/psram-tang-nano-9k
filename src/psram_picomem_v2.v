module PicoMem_PSRAM_V2 #(
    parameter FREQ=81_000_000,// Actual clk frequency, to time 150us initialization delay
    parameter LATENCY=3       // tACC (Initial Latency) in W955D8MBYA datasheet:
) (
    input clk,
    input clk_p,
    input sys_resetn,

    input valid,
    output ready,
    output init_ready,
    input [31:0] addr,
    input [3:0] wstrb,
    input [31:0] wdata,
    output [31:0] rdata,

    output [1:0] O_psram_ck,       // Magic ports for PSRAM to be inferred
    inout [1:0] IO_psram_rwds,
    inout [15:0] IO_psram_dq,
    output [1:0] O_psram_cs_n
);

localparam [3:0] INIT = 4'd0;
localparam [3:0] READ = 4'd1;
localparam [3:0] READ_W = 4'd2;
localparam [3:0] WRITE = 4'd5;
localparam [3:0] WRITE_W = 4'd6;
localparam [3:0] IDLE = 4'd9;

reg [3:0] state;
wire [1:0] busy;
reg read;
reg [1:0] write;
reg [1:0] byte_write;
reg ready;
reg init_ready;
reg [21:0] local_addr1, local_addr2;

PsramController #(
    .FREQ(FREQ),
    .LATENCY(LATENCY)
) mem_ctrl0(
    .clk(clk), .clk_p(clk_p), .resetn(sys_resetn), .read(read), .write(write[0]), .byte_write(byte_write[0]),
    .addr(local_addr1), .din(wdata[15:0]), .dout(rdata[15:0]), .busy(busy[0]),
    .O_psram_ck(O_psram_ck[0]), .IO_psram_rwds(IO_psram_rwds[0]), .IO_psram_dq(IO_psram_dq[7:0]), .O_psram_cs_n(O_psram_cs_n[0])
);

PsramController #(
    .FREQ(FREQ),
    .LATENCY(LATENCY)
) mem_ctrl1(
    .clk(clk), .clk_p(clk_p), .resetn(sys_resetn), .read(read), .write(write[1]), .byte_write(byte_write[1]),
    .addr(local_addr2), .din(wdata[31:16]), .dout(rdata[31:16]), .busy(busy[1]),
    .O_psram_ck(O_psram_ck[1]), .IO_psram_rwds(IO_psram_rwds[1]), .IO_psram_dq(IO_psram_dq[15:8]), .O_psram_cs_n(O_psram_cs_n[1])
);

always @(posedge clk) begin
    if(~sys_resetn) begin
        state <= INIT;
        ready <= 0;
        read <= 0;
        write <= 2'b0;
        init_ready <= 0;
    end else if (ready) begin
        ready <= 0;
    end else begin
        read <= 0;
        write <= 2'b0;
        case (state)
            INIT : begin
                if (busy == 2'b0) begin
                    init_ready <= 1;
                    state <= IDLE;
                end else begin
                    state <= INIT;
                end
            end
            IDLE : begin
                if (valid) begin
                    read <= 0;
                    write <= 2'b0;
                    if (wstrb == 4'b0) begin
                        state <= READ;
                    end else begin
                        state <= WRITE;
                    end
                end else begin
                    state <= IDLE;
                end
            end
            READ : begin
                read <= 1;
                write <= 2'b0;
                local_addr1 <= addr[22:1];
                local_addr2 <= addr[22:1];
                state <= READ_W;
            end
            READ_W : begin
                if (~read && busy == 2'b0) begin
                    state <= IDLE;
                    ready <= 1;
                end else begin
                    state <= READ_W;
                end
            end
            WRITE : begin
                state <= WRITE_W;
                read <= 0;
                if (wstrb[1:0] != 2'b00)
                    write[0] <= 1;
                if (wstrb[3:2] != 2'b00)
                    write[1] <= 1;
                byte_write <= 2'b0;
                local_addr1 <= addr[22:1];
                local_addr2 <= addr[22:1];
                state <= WRITE_W;
                if (wstrb[1:0] != 2'b11) begin
                    byte_write[0] <= 1;
                    if (wstrb[1]) begin
                        local_addr1 <= addr[22:1] + 1;
                    end
                end
                if (wstrb[3:2] != 2'b11) begin
                    byte_write[1] <= 1;
                    if (wstrb[3]) begin
                        local_addr2 <= addr[22:1] + 1;
                    end
                end
                ready <= 1;
            end
            WRITE_W : begin
                if (write == 2'b0 && busy == 2'b0) begin
                    state <= IDLE;
                end else begin
                    state <= WRITE_W;
                end
            end
        endcase
    end // end else
end
endmodule
