
module memory_test (
    input sys_clk,  // 27 Mhz, crystal clock from board
    input sys_resetn,
    input button,   // 0 when pressed, restarts the test

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
localparam ROUNDS = 11;                   // Rounds per pass:
                                          //  1-7:  wstrb mask combinations (1111, 0001, 0010, 0100, 1000, 0011, 1100)
                                          //  8:    all-zero pattern
                                          //  9:    0xAAAAAAAA checkerboard
                                          //  10:   LFSR pattern, mixed read/write with back-to-back valid
                                          //  11:   retention round (write, wait ~3s, verify)

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
localparam [3:0] TEST_WAIT = 4'd11;      // retention delay before read-back
localparam [3:0] TEST_MIX = 4'd12;       // mixed write/read with back-to-back valid

// pass in address to get hash value
`define hash(a) (a[7:0] ^ a[15:8] ^ a[22:16] ^ 8'hc3)

// wstrb used in each round (rounds 8-11 are full-word)
function [3:0] wstrb_of;
    input [3:0] tc;
    begin
        case (tc)
            1: wstrb_of = 4'b1111;
            2: wstrb_of = 4'b0001;
            3: wstrb_of = 4'b0010;
            4: wstrb_of = 4'b0100;
            5: wstrb_of = 4'b1000;
            6: wstrb_of = 4'b0011;
            7: wstrb_of = 4'b1100;
            default: wstrb_of = 4'b1111;
        endcase
    end
endfunction

// data pattern used in each round
function [31:0] pattern;
    input [3:0] tc;
    input [22:0] a;
    reg [31:0] x;
    begin
        case (tc)
            8: pattern = 32'h00000000;
            9: pattern = 32'hAAAAAAAA;
            10: begin
                // xorshift32 of the address
                x = {a, a[8:0]};
                x = x ^ (x << 13);
                x = x ^ (x >> 17);
                x = x ^ (x << 5);
                pattern = x;
            end
            default: pattern = {8'h11, 8'h77, `hash(a), `hash(a)};
        endcase
    end
endfunction

reg [3:0] state, new_state;
reg [10:0] cycle = 0;
reg [23:0] write_1x, write_2x, read_1x, read_2x;        // counter for 1x or 2x latencies
reg tick;                   // pulse once per 0.1 second
reg [5:0] ticks = 0;        // counter for 0.1 second delays
reg error;
assign led = ~{error, 1'd1, state};
reg [3:0] test_count;
reg [7:0] pass_cnt;         // completed full passes
reg pass_end;               // set when the last round of a pass finishes
reg mix_phase;              // TEST_MIX: 0 = write issued, 1 = read issued
reg [22:0] fail_addr;       // latched on first read mismatch for printing
reg [31:0] fail_expect, fail_got;

// button is active low, synchronize and treat as restart request
reg [1:0] btn_sync = 2'b11;
always @(posedge clk) btn_sync <= {btn_sync[0], button};
wire btn_pressed = (btn_sync == 2'b00);

// expected read data: pattern bytes where written, 0xff background elsewhere
wire [3:0] rd_wstrb = wstrb_of(test_count);
wire [31:0] rd_mask = {{8{rd_wstrb[3]}}, {8{rd_wstrb[2]}}, {8{rd_wstrb[1]}}, {8{rd_wstrb[0]}}};

// pipeline addr+4 to meet timing constraint
reg [8:0] new_addr_0;
reg [8:0] new_addr_1;
reg [22:0] new_addr;        // available after 3 cycles
always @(posedge clk) begin
    // stage 0
    new_addr_0 = address[7:0] + 4;
    // stage 1
    new_addr_1 = address[15:8] + new_addr_0[8];
    // stage 2, add higher 7 bits
    new_addr = {address[22:16] + new_addr_1[8], new_addr_1[7:0], new_addr_0[7:0]};
end

always @(posedge clk) begin
    ticks <= tick && (state == TEST_INIT || state == PAUSE || state == TEST_WAIT) ? ticks + 1 : ticks;
    if (~sys_resetn || btn_pressed || state == TEST_ZERO) begin
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
        pass_cnt <= 0;
        pass_end <= 0;
        valid <= 0;

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
        // fill memory with all-ones background
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
                new_state <= (test_count == 10) ? TEST_MIX : TEST_WRITE;
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
            wstrb <= wstrb_of(test_count);
            wdata <= pattern(test_count, address);
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
                if (test_count == ROUNDS) begin
                    // retention round: wait before verifying
                    ticks <= 0;
                    state <= TEST_WAIT;
                end else begin
                    new_state <= TEST_READ;
                    state <= PAUSE;
                end
            end else
                address <= new_addr;
        end else if (cycle == 100) begin
            new_state <= TEST_FAIL_WRITE_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end

    end else if (state == TEST_WAIT) begin
        // ~3 second retention delay, ticks counts 0.1s
        if (ticks == 30) begin
            ticks <= 0;
            state <= TEST_READ;
        end

    end if (state == TEST_READ) begin
        // read and verify some bytes
        cycle <= cycle + 1;
        if (cycle == 0) begin
            // issue read command
            valid <= 1;
            wstrb <= 4'b0;
            correct_rdata <= (pattern(test_count, address) & rd_mask) | ~rd_mask;
        end else if (ready) begin
            // read finished
            cycle <= 0;
            valid <= 0;
            if (cycle > 13+LATENCY)     // read_is on cycle 1, so cycle==13 means latency is 12
                read_2x <= read_2x + 1;
            else
                read_1x <= read_1x + 1;

            if (rdata != correct_rdata) begin
                fail_addr <= address;
                fail_expect <= correct_rdata;
                fail_got <= rdata;
                new_state <= TEST_FAIL_READ_WRONG;
                error <= 1'b1;
                state <= PAUSE;
            end else if (address == BYTES - 4) begin
                address <= 0;
                new_state <= TEST_DONE;
                state <= PAUSE;
            end else
                address <= new_addr;
        end else if (cycle == 100) begin
            new_state <= TEST_FAIL_READ_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end

    end else if (state == TEST_MIX) begin
        // per address: write, then read back with valid held high (back-to-back)
        cycle <= cycle + 1;
        if (cycle == 0) begin
            // issue write command
            valid <= 1;
            wstrb <= 4'b1111;
            wdata <= pattern(test_count, address);
            mix_phase <= 0;
        end else if (ready && mix_phase == 0) begin
            // write accepted; keep valid high and turn it into a read
            wstrb <= 4'b0;
            mix_phase <= 1;
            cycle <= 1;         // not 0, or the issue branch above would fire again
        end else if (ready && mix_phase == 1) begin
            // read finished
            cycle <= 0;
            valid <= 0;
            if (rdata != pattern(test_count, address)) begin
                fail_addr <= address;
                fail_expect <= pattern(test_count, address);
                fail_got <= rdata;
                new_state <= TEST_FAIL_READ_WRONG;
                error <= 1'b1;
                state <= PAUSE;
            end else if (address == BYTES - 4) begin
                address <= 0;
                new_state <= TEST_DONE;
                state <= PAUSE;
            end else
                address <= new_addr;
        end else if (cycle == 100) begin
            new_state <= mix_phase ? TEST_FAIL_READ_TIMEOUT : TEST_FAIL_WRITE_TIMEOUT;
            error <= 1'b1;
            state <= PAUSE;
        end

    end else if (state == TEST_DONE) begin
        pass_end <= (test_count == ROUNDS);
        if (test_count == ROUNDS)
            pass_cnt <= pass_cnt + 1;
        new_state <= TEST_CLEAN_ALL_MEMORY;
        state <= PAUSE;

    end else if (state == PAUSE) begin
        address <= 0;
        // pause for 0.1 seconds for print to finish, then enter new_state
        if (ticks == 2 || NO_PAUSE) begin     // pause for 0.1 second
            ticks <= 0;
            state <= new_state;
            if (new_state == TEST_CLEAN_ALL_MEMORY) begin
                test_count <= pass_end ? 4'd1 : test_count + 1;
                pass_end <= 0;
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
reg [4:0] print_counters = 0;
// done/timeout chain:  1..11 latency counters + round number, 12..14 "PASS #n" (final round only)
// read-wrong chain:   16..22 fail_addr / fail_expect / fail_got
reg [4:0] print_counters_p;
reg chain_done = 0;         // print chain was started by TEST_DONE (not a timeout fail)

// decimal ASCII conversion for round number (1..11) and pass counter (0..255)
wire [7:0] round_tens_ch = (test_count >= 10) ? 8'h31 : 8'h20;            // '1' or space
wire [7:0] round_ones_ch = 8'h30 + (test_count >= 10 ? test_count - 10 : test_count);
wire [7:0] pass_h_ch = (pass_cnt >= 100) ? (8'h30 + pass_cnt / 100) : 8'h20;
wire [7:0] pass_t_ch = (pass_cnt >= 10)  ? (8'h30 + (pass_cnt / 10) % 10) : 8'h20;
wire [7:0] pass_o_ch = 8'h30 + pass_cnt % 10;

always @(posedge clk) begin
    state_p <= state;
    print_counters_p <= print_counters;
    if (state != state_p) begin
        if (state == TEST_INIT) `print("Initializing HyperRAM test...\n", STR);
        if (state == TEST_WRITE) `print("Writing...\n", STR);
        if (state == TEST_READ) `print("Reading...\n", STR);
        if (state == TEST_MIX) `print("Mixed r/w back-to-back...\n", STR);
        if (state == TEST_WAIT) `print("Retention wait...\n", STR);
        if (state == TEST_DONE) `print("All done successfully.\n", STR);
        if (state == TEST_FAIL_INIT_TIMEOUT) `print("FAIL. Initialization timeout.\n", STR);
        if (state == TEST_FAIL_WRITE_TIMEOUT) `print("FAIL. Write time out.\n", STR);
        if (state == TEST_FAIL_READ_TIMEOUT) `print("FAIL. Read time out.\n", STR);
        if (state == TEST_FAIL_READ_WRONG) `print("FAIL. Read wrong data.\n", STR);
        if (state == TEST_CLEAN_ALL_MEMORY) `print("Cleaning all memory...\n", STR);

        if (state == TEST_DONE || state == TEST_FAIL_INIT_TIMEOUT || state == TEST_FAIL_READ_TIMEOUT || state == TEST_FAIL_WRITE_TIMEOUT) begin
            print_counters <= 1;
            chain_done <= (state == TEST_DONE);
        end
        if (state == TEST_FAIL_READ_WRONG)
            print_counters <= 16;
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
        9: `print(" round=", STR);
        10: `print({round_tens_ch, round_ones_ch}, STR);
        11: `print("\n", STR);
        12: `print("PASS #", STR);
        13: `print({pass_h_ch, pass_t_ch, pass_o_ch}, STR);
        14: `print("\n", STR);
        16: `print("addr=", STR);
        17: `print({1'b0, fail_addr}, 3);
        18: `print(" expect=", STR);
        19: `print(fail_expect, 4);
        20: `print(" got=", STR);
        21: `print(fail_got, 4);
        22: `print("\n", STR);
        endcase
        if (print_counters == 11)
            print_counters <= (chain_done && pass_end) ? 12 : 0;
        else if (print_counters == 14 || print_counters == 22)
            print_counters <= 0;
        else
            print_counters <= print_counters + 1;
    end

end
`endif


endmodule
