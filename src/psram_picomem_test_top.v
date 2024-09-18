
module memory_test (
    input sys_clk,  // 27 Mhz, crystal clock from board
    input sys_resetn,
    input button,   // 0 when pressed

    output [5:0] led,
    output uart_txp,

    output [1:0] O_psram_ck,       // Magic ports for PSRAM to be inferred
    output [1:0] O_psram_ck_n,
    inout [1:0] IO_psram_rwds,
    inout [15:0] IO_psram_dq,
    output [1:0] O_psram_reset_n,
    output [1:0] O_psram_cs_n
);

// Customization of the test
localparam [23:0] BYTES = 8*1024*1024;    // Test write/read this many bytes

// Change PLL and here to choose another speed.
localparam FREQ = 40_500_000;
localparam LATENCY = 3;
//localparam FREQ = 102_600_000;
//localparam LATENCY = 4;

// Remove UART print module for timing closure (check LED5 for error)
//`define NO_UART_PRINT

// For GAO debug
//localparam [21:0] BYTES = 2;
//localparam NO_PAUSE = 1;
localparam NO_PAUSE = 0;                // Pause between states to allow UART printing

// End of customization

assign O_psram_reset_n = {sys_resetn, sys_resetn};

Gowin_rPLL pll(
    .clkout(clk),        // MHZ main clock
    .clkoutp(clk_p),     // MHZ phase shifted (90 degrees)
    .clkin(sys_clk)      // 27Mhz system clock
);

// Memory Controller under test ---------------------------
reg valid;
wire ready;
wire init_ready;
reg [22:0] address;
reg [3:0] wstrb;
reg [31:0] wdata;
wire [31:0] rdata;
reg [31:0] correct_rdata;

PicoMem_PSRAM_V2 psram(
    .clk(clk), .clk_p(clk_p), .sys_resetn(sys_resetn),
    .valid(valid), .ready(ready), .init_ready(init_ready), .addr(address),
    .wstrb(wstrb), .wdata(wdata), .rdata(rdata),
    .O_psram_ck(O_psram_ck), .IO_psram_rwds(IO_psram_rwds),
    .IO_psram_dq(IO_psram_dq), .O_psram_cs_n(O_psram_cs_n)
);

// The test ------------------------------------------------

localparam [3:0] TEST_ZERO = 4'd0;
localparam [3:0] TEST_INIT = 4'd1;
localparam [3:0] TEST_WRITE = 4'd2;
localparam [3:0] TEST_READ = 4'd3;
localparam [3:0] TEST_DONE = 4'd4;
localparam [3:0] TEST_FAIL_INIT_TIMEOUT = 4'd5;
localparam [3:0] TEST_FAIL_WRITE_TIMEOUT = 4'd6;
localparam [3:0] TEST_FAIL_READ_TIMEOUT = 4'd7;
localparam [3:0] TEST_FAIL_READ_WRONG = 4'd8;
localparam [3:0] PAUSE = 4'd9;
localparam [3:0] TEST_CLEAN_ALL_MEMORY = 4'd10;

// pass in address to get hash value
`define hash(a) (a[7:0] ^ a[15:8] ^ a[22:16] ^ 8'hc3)

reg [3:0] state, new_state;
reg [10:0] cycle = 0;        // max 16
reg [23:0] write_1x, write_2x, read_1x, read_2x;        // counter for 1x or 2x latencies
reg tick;                   // pulse once per 0.1 second
reg [3:0] ticks = 0;        // counter for 0.1 second delays
reg error;
assign led = ~{error, 1'd1, state};
reg[3:0] test_count;

// pipeline addr+1 to meet timing constraint
reg [8:0] new_addr_0;
reg [8:0] new_addr_1;
reg [22:0] new_addr;        // available after 3 cycles
always @(posedge clk) begin
    // stage 0
    new_addr_0 = address[7:0] + 4;
    // stage 1
    new_addr_1 = address[15:8] + new_addr_0[8];
    // stage 2, add higher 6 bits
    new_addr = {address[22:16] + new_addr_1[8], new_addr_1[7:0], new_addr_0[7:0]};
end

always @(posedge clk) begin
    ticks <= tick && (state == TEST_INIT || state == PAUSE) ? ticks + 1 : ticks;
    if (~sys_resetn || state == TEST_ZERO) begin
        write_1x <= 0;
        write_2x <= 0;
        read_1x <= 0;
        read_2x <= 0;
        cycle <= 0;
        ticks <= 0;
        new_state <= TEST_INIT;
        state <= PAUSE;
        error <= 0;
        test_count <= 0;

    end else if (state == TEST_INIT) begin
        // wait for memory to become ready
        if (init_ready) begin
            new_state <= TEST_CLEAN_ALL_MEMORY;
            state <= PAUSE;
        end else if (ticks == 5) begin   // 0.5 second timeout
            new_state <= TEST_FAIL_INIT_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end

    end else if (state == TEST_CLEAN_ALL_MEMORY) begin
        // write some bytes
        cycle <= cycle + 1;
        if (cycle == 0) begin
            // issue write command
            valid <= 1;
            wstrb <= 4'b1111;
            wdata <= 32'hffffffff;
        end else if (ready) begin
            // write finished
            cycle <= 0;
            valid <= 0;
            if (address == BYTES - 4) begin
                address <= 0;
                new_state <= TEST_WRITE;
                state <= PAUSE;
            end else
                address <= new_addr;
        end else if (cycle == 100) begin
            new_state <= TEST_FAIL_WRITE_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end
    end if (state == TEST_WRITE) begin
        // write some bytes
        cycle <= cycle + 1;
        if (cycle == 0) begin
            // issue write command
            valid <= 1;
            case (test_count)
                1: wstrb <= 4'b1111;
                2: wstrb <= 4'b0001;
                3: wstrb <= 4'b0010;
                4: wstrb <= 4'b0100;
                5: wstrb <= 4'b1000;
                6: wstrb <= 4'b0011;
                7: wstrb <= 4'b1100;
            endcase
            wdata <= {8'h11, 8'h77, `hash(address), `hash(address)};
        end else if (ready) begin
            // write finished
            cycle <= 0;
            valid <= 0;
            if (cycle > 7+LATENCY)
                write_2x <= write_2x + 1;
            else
                write_1x <= write_1x + 1;
            if (address == BYTES - 4) begin
                address <= 0;
                new_state <= TEST_READ;
                state <= PAUSE;
            end else
                address <= new_addr;
        end else if (cycle == 100) begin
            new_state <= TEST_FAIL_WRITE_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end

    end if (state == TEST_READ) begin
        // read and verify some bytes
        cycle <= cycle + 1;
        if (cycle == 0) begin
            // issue read command
            valid <= 1;
            wstrb <= 4'b0;
            case (test_count)
                1: correct_rdata <= {8'h11, 8'h77, `hash(address), `hash(address)};
                2: correct_rdata <= {8'hff, 8'hff, 8'hff, `hash(address)};
                3: correct_rdata <= {8'hff, 8'hff, `hash(address), 8'hff};
                4: correct_rdata <= {8'hff, 8'h77, 8'hff, 8'hff};
                5: correct_rdata <= {8'h11, 8'hff, 8'hff, 8'hff};
                6: correct_rdata <= {8'hff, 8'hff, `hash(address), `hash(address)};
                7: correct_rdata <= {8'h11, 8'h77, 8'hff, 8'hff};
            endcase
        end else if (ready) begin
            // read finished
            cycle <= 0;
            valid <= 0;
            if (cycle > 13+LATENCY)     // read_is on cycle 1, so cycle==13 means latency is 12
                read_2x <= read_2x + 1;
            else
                read_1x <= read_1x + 1;

            if (rdata != correct_rdata) begin
                new_state <= TEST_FAIL_READ_WRONG;
                error <= 1'b1;
                state <= PAUSE;
            end else if (address == BYTES - 4) begin
                new_state <= TEST_DONE;
                state <= PAUSE;
            end else
                address <= new_addr;
        end else if (cycle == 100) begin
            new_state <= TEST_FAIL_READ_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end

    end else if (state == TEST_DONE) begin
        if (test_count < 7) begin
            new_state <= TEST_CLEAN_ALL_MEMORY;
            state <= PAUSE;
            // we can't clean write_1x/read_1x here...
        end

    end else if (state == PAUSE) begin
        address <= 0;
        // pause for 0.1 seconds for print to finish, then enter new_state
        if (ticks == 2 || NO_PAUSE) begin     // pause for 0.1 second
            ticks <= 0;
            state <= new_state;
            if (new_state == TEST_CLEAN_ALL_MEMORY) begin
                test_count <= test_count + 1;
                write_1x <= 0;
                write_2x <= 0;
                read_1x <= 0;
                read_2x <= 0;
                cycle <= 0;
            end
        end
    end
end


reg [23:0] tick_counter;        // max 16M
always @(posedge clk) begin
    if (~sys_resetn) begin
        tick_counter <= FREQ/10;
    end
    tick_counter <= tick_counter == 0 ? FREQ/10 : tick_counter - 1;
    tick <= tick_counter == 0;
end


//Print Controll -------------------------------------------

`ifndef NO_UART_PRINT
`include "print.v"
defparam tx.uart_freq=115200;
defparam tx.clk_freq=FREQ;
assign print_clk = clk;
assign txp = uart_txp;

reg [3:0] state_p;
reg [3:0] print_counters = 0;       // 1. "write_1x=", 2. write_1x, 3. ", write_2x=", 4. write_2x, 5. ", read_1x=", 6. "read_1x", 7, ", read_2x=", 8. read_2x., 9. "\n"
reg [3:0] print_counters_p;

always @(posedge clk) begin
    state_p <= state;
    print_counters_p <= print_counters;
    if (state != state_p) begin
        if (state == TEST_INIT) `print("Initializing HyperRAM test...\n", STR);
        if (state == TEST_WRITE) `print("Writing...\n", STR);
        if (state == TEST_READ) `print("Reading...\n", STR);
        if (state == TEST_DONE) `print("All done successfully.\n", STR);
        if (state == TEST_FAIL_INIT_TIMEOUT) `print("FAIL. Initialization timeout.\n", STR);
        if (state == TEST_FAIL_WRITE_TIMEOUT) `print("FAIL. Write time out.\n", STR);
        if (state == TEST_FAIL_READ_TIMEOUT) `print("FAIL. Read time out.\n", STR);
        if (state == TEST_FAIL_READ_WRONG) `print("FAIL. Read wrong data.\n", STR);
        if (state == TEST_CLEAN_ALL_MEMORY) `print("Cleaning all memory...\n", STR);

        if (state == TEST_DONE || state == TEST_FAIL_INIT_TIMEOUT || state == TEST_FAIL_READ_TIMEOUT || state == TEST_FAIL_READ_WRONG || state == TEST_FAIL_WRITE_TIMEOUT)
            print_counters <= 1;
    end

    if (print_counters > 0 && print_counters == print_counters_p && print_state == PRINT_IDLE_STATE) begin
        case (print_counters)
        1: `print("Latency counters: write_1x=", STR);
        2: `print(write_1x, 3);
        3: `print(", write_2x=", STR);
        4: `print(write_2x, 3);
        5: `print(", read_1x=", STR);
        6: `print(read_1x, 3);
        7: `print(", read_2x=", STR);
        8: `print(read_2x, 3);
        9: `print("\n", STR);
        10: `print(test_count, 1);
        11: `print("\n", STR);
        endcase
        print_counters <= print_counters == 11 ? 0 : print_counters + 1;
    end

end
`endif


endmodule
