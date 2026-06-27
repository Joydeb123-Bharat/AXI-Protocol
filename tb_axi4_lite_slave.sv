`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 28.06.2026 00:34:43
// Design Name: 
// Module Name: tb_axi4_lite_slave
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps

// AXI4-Lite interface - defines all five channel signals and clocking blocks
// Interface remains parameterized as XSim handles parameterized interfaces correctly
interface axi4_lite_if #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input logic ACLK,
    input logic ARESETn
);
    // Write Address Channel
    logic [ADDR_WIDTH-1:0]     AWADDR;
    logic [2:0]                AWPROT;
    logic                      AWVALID;
    logic                      AWREADY;

    // Write Data Channel
    logic [DATA_WIDTH-1:0]     WDATA;
    logic [(DATA_WIDTH/8)-1:0] WSTRB;
    logic                      WVALID;
    logic                      WREADY;

    // Write Response Channel
    logic [1:0]                BRESP;
    logic                      BVALID;
    logic                      BREADY;

    // Read Address Channel
    logic [ADDR_WIDTH-1:0]     ARADDR;
    logic [2:0]                ARPROT;
    logic                      ARVALID;
    logic                      ARREADY;

    // Read Data Channel
    logic [DATA_WIDTH-1:0]     RDATA;
    logic [1:0]                RRESP;
    logic                      RVALID;
    logic                      RREADY;

    // Master clocking block - synchronises all master-driven outputs and sampled inputs
    clocking master_cb @(posedge ACLK);
        default input #1 output #1;
        output AWADDR, AWPROT, AWVALID;
        output WDATA, WSTRB, WVALID;
        input  AWREADY;
        input  WREADY;
        output BREADY;
        input  BRESP, BVALID;
        output ARADDR, ARPROT, ARVALID;
        input  ARREADY;
        output RREADY;
        input  RDATA, RRESP, RVALID;
    endclocking

    // Passive monitor clocking block - samples all signals for coverage and checking
    clocking monitor_cb @(posedge ACLK);
        default input #1;
        input AWADDR, AWPROT, AWVALID, AWREADY;
        input WDATA,  WSTRB,  WVALID,  WREADY;
        input BRESP,  BVALID, BREADY;
        input ARADDR, ARPROT, ARVALID, ARREADY;
        input RDATA,  RRESP,  RVALID,  RREADY;
    endclocking

    modport master  (clocking master_cb,  input ACLK, ARESETn);
    modport monitor (clocking monitor_cb, input ACLK, ARESETn);

endinterface


// Transaction class - fixed-width constants replace parameters to avoid XSim elaboration crash
// XSim does not support parameterized class typedef resolution during scope analysis
class axi4_lite_txn;
    localparam ADDR_WIDTH = 32;
    localparam DATA_WIDTH = 32;

    typedef enum logic { READ, WRITE } txn_kind_t;

    rand txn_kind_t              kind;
    rand logic [31:0]            addr;
    rand logic [31:0]            data;
    rand logic [3:0]             strb;
    rand int unsigned            bready_delay;  // master-side cycles before asserting BREADY
    rand int unsigned            rready_delay;  // master-side cycles before asserting RREADY

    // Constrain random addresses to the four valid word-aligned register offsets
    constraint valid_addr_c   { addr inside {32'h0, 32'h4, 32'h8, 32'hC}; }
    // Prevent all-zero strobe - a write with no active byte lanes is meaningless
    constraint strb_nonzero_c { strb != 4'h0; }
    // Bound response delays to a realistic range
    constraint bready_delay_c { bready_delay inside {[0:5]}; }
    constraint rready_delay_c { rready_delay inside {[0:5]}; }

    // Response fields captured by driver after each handshake completes
    logic [1:0]  bresp;
    logic [31:0] rdata;
    logic [1:0]  rresp;

    function void print(input string tag = "TXN");
        $display("[%0t][%s] %s addr=0x%08h data=0x%08h strb=0b%04b bresp=%02b rresp=%02b",
                 $time, tag,
                 kind == WRITE ? "WRITE" : "READ ",
                 addr, data, strb, bresp, rresp);
    endfunction

endclass


// Driver class - cycle-accurately drives all five AXI4-Lite channels
// Uses virtual interface with fixed parameter values matching the DUT
class axi4_lite_driver;

    virtual axi4_lite_if #(32, 32).master vif;

    function new(virtual axi4_lite_if #(32, 32).master vif);
        this.vif = vif;
    endfunction

    // Deassert all master-driven signals - must be called before releasing reset
    task idle();
        vif.master_cb.AWVALID <= 1'b0;
        vif.master_cb.AWADDR  <= '0;
        vif.master_cb.AWPROT  <= '0;
        vif.master_cb.WVALID  <= 1'b0;
        vif.master_cb.WDATA   <= '0;
        vif.master_cb.WSTRB   <= '0;
        vif.master_cb.BREADY  <= 1'b0;
        vif.master_cb.ARVALID <= 1'b0;
        vif.master_cb.ARADDR  <= '0;
        vif.master_cb.ARPROT  <= '0;
        vif.master_cb.RREADY  <= 1'b0;
    endtask

    // Dispatch a transaction to the correct channel sequence
    task drive(axi4_lite_txn txn);
        if (txn.kind == axi4_lite_txn::WRITE)
            drive_write(txn);
        else
            drive_read(txn);
    endtask

    // Drive AW and W concurrently then wait for B - mirrors spec order-independence
    task drive_write(axi4_lite_txn txn);
        fork
            drive_aw(txn);
            drive_w(txn);
        join
        drive_b(txn);
    endtask

    // Drive Write Address channel and wait for AWREADY handshake
    task drive_aw(axi4_lite_txn txn);
        vif.master_cb.AWADDR  <= txn.addr;
        vif.master_cb.AWPROT  <= 3'b000;
        vif.master_cb.AWVALID <= 1'b1;
        @(vif.master_cb);
        while (!vif.master_cb.AWREADY)
            @(vif.master_cb);
        // Deassert immediately after handshake - AWVALID must not stay high
        vif.master_cb.AWVALID <= 1'b0;
        vif.master_cb.AWADDR  <= '0;
    endtask

    // Drive Write Data channel and wait for WREADY handshake
    task drive_w(axi4_lite_txn txn);
        vif.master_cb.WDATA  <= txn.data;
        vif.master_cb.WSTRB  <= txn.strb;
        vif.master_cb.WVALID <= 1'b1;
        @(vif.master_cb);
        while (!vif.master_cb.WREADY)
            @(vif.master_cb);
        vif.master_cb.WVALID <= 1'b0;
        vif.master_cb.WDATA  <= '0;
        vif.master_cb.WSTRB  <= '0;
    endtask

    // Wait for BVALID, apply optional delay, assert BREADY and capture BRESP
    task drive_b(axi4_lite_txn txn);
        while (!vif.master_cb.BVALID)
            @(vif.master_cb);
        // Simulate a slow master that does not immediately accept the response
        repeat(txn.bready_delay) @(vif.master_cb);
        vif.master_cb.BREADY <= 1'b1;
        @(vif.master_cb);
        txn.bresp = vif.master_cb.BRESP;  // capture on the handshake cycle
        vif.master_cb.BREADY <= 1'b0;
    endtask

    // Drive Read Address channel, wait for data, apply optional RREADY delay
    task drive_read(axi4_lite_txn txn);
        vif.master_cb.ARADDR  <= txn.addr;
        vif.master_cb.ARPROT  <= 3'b000;
        vif.master_cb.ARVALID <= 1'b1;
        @(vif.master_cb);
        while (!vif.master_cb.ARREADY)
            @(vif.master_cb);
        vif.master_cb.ARVALID <= 1'b0;
        vif.master_cb.ARADDR  <= '0;
        while (!vif.master_cb.RVALID)
            @(vif.master_cb);
        // Simulate a slow master before accepting read data
        repeat(txn.rready_delay) @(vif.master_cb);
        vif.master_cb.RREADY <= 1'b1;
        @(vif.master_cb);
        txn.rdata = vif.master_cb.RDATA;  // capture RDATA on the handshake cycle
        txn.rresp = vif.master_cb.RRESP;  // capture RRESP on the handshake cycle
        vif.master_cb.RREADY <= 1'b0;
    endtask

endclass


// Scoreboard - maintains a golden register model and checks every DUT response
class axi4_lite_scoreboard;
    localparam DATA_WIDTH = 32;
    localparam NUM_REGS   = 4;

    // Golden register file - updated in lockstep with valid DUT writes
    logic [31:0] ref_regs [NUM_REGS];

    int unsigned checks_passed;
    int unsigned checks_failed;

    function new();
        foreach (ref_regs[i]) ref_regs[i] = '0;
        checks_passed = 0;
        checks_failed = 0;
    endfunction

    // Returns the word index for an address, or -1 if out of range
    function int get_reg_idx(input logic [31:0] addr);
        int idx = int'(addr[31:2]);
        if (idx >= NUM_REGS) return -1;
        return idx;
    endfunction

    // Check a write transaction - update golden model on valid address, verify BRESP
    function void check_write(axi4_lite_txn txn);
        int idx = get_reg_idx(txn.addr);
        automatic logic [1:0] expected_bresp;

        if (idx < 0) begin
            expected_bresp = 2'b10;  // SLVERR expected for out-of-range address
        end else begin
            expected_bresp = 2'b00;  // OKAY expected; apply strobes to golden model
            for (int b = 0; b < 4; b++)
                if (txn.strb[b])
                    ref_regs[idx][8*b +: 8] = txn.data[8*b +: 8];
        end

        if (txn.bresp !== expected_bresp) begin
            $error("[SCOREBOARD][FAIL] WRITE addr=0x%08h: BRESP got %02b expected %02b",
                   txn.addr, txn.bresp, expected_bresp);
            checks_failed++;
        end else begin
            $display("[SCOREBOARD][PASS] WRITE addr=0x%08h data=0x%08h strb=0b%04b BRESP=%02b",
                     txn.addr, txn.data, txn.strb, txn.bresp);
            checks_passed++;
        end
    endfunction

    // Check a read transaction - compare RDATA against golden model and verify RRESP
    function void check_read(axi4_lite_txn txn);
        int idx = get_reg_idx(txn.addr);
        automatic logic [31:0] expected_data;
        automatic logic [1:0]  expected_rresp;

        if (idx < 0) begin
            expected_rresp = 2'b10;  // SLVERR on invalid address
            expected_data  = '0;
        end else begin
            expected_rresp = 2'b00;
            expected_data  = ref_regs[idx];
        end

        if (txn.rresp !== expected_rresp) begin
            $error("[SCOREBOARD][FAIL] READ  addr=0x%08h: RRESP got %02b expected %02b",
                   txn.addr, txn.rresp, expected_rresp);
            checks_failed++;
        end else if (idx >= 0 && txn.rdata !== expected_data) begin
            $error("[SCOREBOARD][FAIL] READ  addr=0x%08h: RDATA got 0x%08h expected 0x%08h",
                   txn.addr, txn.rdata, expected_data);
            checks_failed++;
        end else begin
            $display("[SCOREBOARD][PASS] READ  addr=0x%08h RDATA=0x%08h RRESP=%02b",
                     txn.addr, txn.rdata, txn.rresp);
            checks_passed++;
        end
    endfunction

    // Route a completed transaction to the correct check function
    function void check(axi4_lite_txn txn);
        if (txn.kind == axi4_lite_txn::WRITE)
            check_write(txn);
        else
            check_read(txn);
    endfunction

    // Print final pass/fail summary
    function void report();
        $display("[SCOREBOARD] Total checks: %0d  PASSED: %0d  FAILED: %0d",
                 checks_passed + checks_failed, checks_passed, checks_failed);
        if (checks_failed == 0)
            $display("[SCOREBOARD] ALL TESTS PASSED");
        else
            $error("[SCOREBOARD] %0d TEST(S) FAILED", checks_failed);
    endfunction

endclass


// Coverage model - tracks which protocol scenarios and corner cases have been hit
class axi4_lite_coverage;

    // Sampling variables written before each covergroup sample() call
    logic [31:0]               sample_addr;
    axi4_lite_txn::txn_kind_t  sample_kind;
    logic [1:0]                sample_resp;
    logic [3:0]                sample_strb;
    int unsigned               sample_bready_dly;
    int unsigned               sample_rready_dly;

    covergroup axi4_lite_cg;
        // Every valid register address and an invalid one must be accessed
        cp_addr: coverpoint sample_addr {
            bins reg0    = {32'h00};
            bins reg1    = {32'h04};
            bins reg2    = {32'h08};
            bins reg3    = {32'h0C};
            bins invalid = {[32'h10:32'hFF]};
        }

        // Both read and write paths must be exercised
        cp_kind: coverpoint sample_kind {
            bins write = {axi4_lite_txn::WRITE};
            bins read  = {axi4_lite_txn::READ};
        }

        // OKAY and SLVERR responses must both be observed
        cp_resp: coverpoint sample_resp {
            bins okay   = {2'b00};
            bins slverr = {2'b10};
        }

        // All meaningful byte-strobe patterns must be exercised
        cp_strb: coverpoint sample_strb {
            bins byte0_only = {4'b0001};
            bins byte1_only = {4'b0010};
            bins byte2_only = {4'b0100};
            bins byte3_only = {4'b1000};
            bins lower_half = {4'b0011};
            bins upper_half = {4'b1100};
            bins all_bytes  = {4'b1111};
            bins other[]    = default;
        }

        // BREADY delay range must cover immediate and deferred acceptance
        cp_bready_dly: coverpoint sample_bready_dly {
            bins immediate = {0};
            bins short_dly = {[1:2]};
            bins long_dly  = {[3:5]};
        }

        // RREADY delay range must cover immediate and deferred acceptance
        cp_rready_dly: coverpoint sample_rready_dly {
            bins immediate = {0};
            bins short_dly = {[1:2]};
            bins long_dly  = {[3:5]};
        }

        // Every valid register must be accessed by both read and write
        cx_addr_kind: cross cp_addr, cp_kind {
            ignore_bins invalid_read = binsof(cp_addr.invalid) && binsof(cp_kind.read);
        }

        // SLVERR must be observable on both read and write channels
        cx_kind_resp: cross cp_kind, cp_resp;

    endgroup

    function new();
        axi4_lite_cg = new();
    endfunction

    // Sample coverage after each completed transaction
    function void sample(axi4_lite_txn txn);
        sample_addr       = txn.addr;
        sample_kind       = txn.kind;
        sample_resp       = (txn.kind == axi4_lite_txn::WRITE) ? txn.bresp : txn.rresp;
        sample_strb       = txn.strb;
        sample_bready_dly = txn.bready_delay;
        sample_rready_dly = txn.rready_delay;
        axi4_lite_cg.sample();
    endfunction

    // Report aggregate functional coverage at end of test
    function void report();
        $display("[COVERAGE] Functional coverage: %.2f%%", axi4_lite_cg.get_coverage());
    endfunction

endclass


// SVA protocol checker - all assertions disabled during reset via disable iff
module axi4_lite_sva #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input logic                      ACLK,
    input logic                      ARESETn,
    input logic [ADDR_WIDTH-1:0]     AWADDR,
    input logic                      AWVALID,
    input logic                      AWREADY,
    input logic [DATA_WIDTH-1:0]     WDATA,
    input logic [(DATA_WIDTH/8)-1:0] WSTRB,
    input logic                      WVALID,
    input logic                      WREADY,
    input logic [1:0]                BRESP,
    input logic                      BVALID,
    input logic                      BREADY,
    input logic [ADDR_WIDTH-1:0]     ARADDR,
    input logic                      ARVALID,
    input logic                      ARREADY,
    input logic [DATA_WIDTH-1:0]     RDATA,
    input logic [1:0]                RRESP,
    input logic                      RVALID,
    input logic                      RREADY
);

    // AWVALID must not deassert before the AWREADY handshake completes
    property aw_valid_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (AWVALID && !AWREADY) |=> AWVALID;
    endproperty
    assert_aw_valid_stable: assert property(aw_valid_stable)
        else $error("[SVA] AWVALID deasserted before AWREADY handshake");

    // AWADDR must remain stable while AWVALID is high and AWREADY is low
    property aw_addr_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (AWVALID && !AWREADY) |=> $stable(AWADDR);
    endproperty
    assert_aw_addr_stable: assert property(aw_addr_stable)
        else $error("[SVA] AWADDR changed while AWVALID high and AWREADY low");

    // WVALID must not deassert before the WREADY handshake completes
    property w_valid_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (WVALID && !WREADY) |=> WVALID;
    endproperty
    assert_w_valid_stable: assert property(w_valid_stable)
        else $error("[SVA] WVALID deasserted before WREADY handshake");

    // WDATA and WSTRB must remain stable while WVALID is high and WREADY is low
    property w_data_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (WVALID && !WREADY) |=> ($stable(WDATA) && $stable(WSTRB));
    endproperty
    assert_w_data_stable: assert property(w_data_stable)
        else $error("[SVA] WDATA/WSTRB changed while WVALID high and WREADY low");

    // BVALID must not deassert before the BREADY handshake completes
    property b_valid_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (BVALID && !BREADY) |=> BVALID;
    endproperty
    assert_b_valid_stable: assert property(b_valid_stable)
        else $error("[SVA] BVALID deasserted before BREADY handshake");

    // BRESP must remain stable while BVALID is high and BREADY is low
    property b_resp_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (BVALID && !BREADY) |=> $stable(BRESP);
    endproperty
    assert_b_resp_stable: assert property(b_resp_stable)
        else $error("[SVA] BRESP changed while BVALID high and BREADY low");

    // ARVALID must not deassert before the ARREADY handshake completes
    property ar_valid_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (ARVALID && !ARREADY) |=> ARVALID;
    endproperty
    assert_ar_valid_stable: assert property(ar_valid_stable)
        else $error("[SVA] ARVALID deasserted before ARREADY handshake");

    // ARADDR must remain stable while ARVALID is high and ARREADY is low
    property ar_addr_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (ARVALID && !ARREADY) |=> $stable(ARADDR);
    endproperty
    assert_ar_addr_stable: assert property(ar_addr_stable)
        else $error("[SVA] ARADDR changed while ARVALID high and ARREADY low");

    // RVALID must not deassert before the RREADY handshake completes
    property r_valid_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (RVALID && !RREADY) |=> RVALID;
    endproperty
    assert_r_valid_stable: assert property(r_valid_stable)
        else $error("[SVA] RVALID deasserted before RREADY handshake");

    // RDATA and RRESP must remain stable while RVALID is high and RREADY is low
    property r_data_stable;
        @(posedge ACLK) disable iff (!ARESETn)
        (RVALID && !RREADY) |=> ($stable(RDATA) && $stable(RRESP));
    endproperty
    assert_r_data_stable: assert property(r_data_stable)
        else $error("[SVA] RDATA/RRESP changed while RVALID high and RREADY low");

    // BRESP must be OKAY or SLVERR only - DECERR and EXOKAY are illegal on AXI4-Lite
    property b_resp_legal;
        @(posedge ACLK) disable iff (!ARESETn)
        BVALID |-> (BRESP == 2'b00 || BRESP == 2'b10);
    endproperty
    assert_b_resp_legal: assert property(b_resp_legal)
        else $error("[SVA] Illegal BRESP value: %02b", BRESP);

    // RRESP must be OKAY or SLVERR only - same restriction applies as BRESP
    property r_resp_legal;
        @(posedge ACLK) disable iff (!ARESETn)
        RVALID |-> (RRESP == 2'b00 || RRESP == 2'b10);
    endproperty
    assert_r_resp_legal: assert property(r_resp_legal)
        else $error("[SVA] Illegal RRESP value: %02b", RRESP);

    // AWREADY must deassert the cycle after the handshake - one-cycle pulse only
    property aw_ready_pulse;
        @(posedge ACLK) disable iff (!ARESETn)
        (AWVALID && AWREADY) |=> !AWREADY;
    endproperty
    assert_aw_ready_pulse: assert property(aw_ready_pulse)
        else $error("[SVA] AWREADY held high for more than one cycle after handshake");

    // ARREADY must deassert the cycle after the handshake - one-cycle pulse only
    property ar_ready_pulse;
        @(posedge ACLK) disable iff (!ARESETn)
        (ARVALID && ARREADY) |=> !ARREADY;
    endproperty
    assert_ar_ready_pulse: assert property(ar_ready_pulse)
        else $error("[SVA] ARREADY held high for more than one cycle after handshake");

    // WSTRB must not be all-zero while WVALID is asserted
    property wstrb_nonzero;
        @(posedge ACLK) disable iff (!ARESETn)
        WVALID |-> (WSTRB != '0);
    endproperty
    assert_wstrb_nonzero: assert property(wstrb_nonzero)
        else $error("[SVA] WSTRB is all-zero while WVALID asserted");

endmodule


// Top-level testbench - instantiates DUT, SVA checker, and all verification components
module tb_axi4_lite_slave;

    localparam ADDR_WIDTH = 32;
    localparam DATA_WIDTH = 32;

    logic ACLK    = 0;
    logic ARESETn = 0;

    always #5 ACLK = ~ACLK;  // 100 MHz

    // Single interface instance shared by DUT, SVA checker, and driver
    axi4_lite_if #(ADDR_WIDTH, DATA_WIDTH) axi_if (.ACLK(ACLK), .ARESETn(ARESETn));

    // DUT instantiation
    axi4_lite_slave #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .ACLK    (ACLK),
        .ARESETn (ARESETn),
        .AWADDR  (axi_if.AWADDR),
        .AWPROT  (axi_if.AWPROT),
        .AWVALID (axi_if.AWVALID),
        .AWREADY (axi_if.AWREADY),
        .WDATA   (axi_if.WDATA),
        .WSTRB   (axi_if.WSTRB),
        .WVALID  (axi_if.WVALID),
        .WREADY  (axi_if.WREADY),
        .BRESP   (axi_if.BRESP),
        .BVALID  (axi_if.BVALID),
        .BREADY  (axi_if.BREADY),
        .ARADDR  (axi_if.ARADDR),
        .ARPROT  (axi_if.ARPROT),
        .ARVALID (axi_if.ARVALID),
        .ARREADY (axi_if.ARREADY),
        .RDATA   (axi_if.RDATA),
        .RRESP   (axi_if.RRESP),
        .RVALID  (axi_if.RVALID),
        .RREADY  (axi_if.RREADY)
    );

    // SVA checker observes all interface signals passively
    axi4_lite_sva #(ADDR_WIDTH, DATA_WIDTH) sva_checker (
        .ACLK    (ACLK),
        .ARESETn (ARESETn),
        .AWADDR  (axi_if.AWADDR),
        .AWVALID (axi_if.AWVALID),
        .AWREADY (axi_if.AWREADY),
        .WDATA   (axi_if.WDATA),
        .WSTRB   (axi_if.WSTRB),
        .WVALID  (axi_if.WVALID),
        .WREADY  (axi_if.WREADY),
        .BRESP   (axi_if.BRESP),
        .BVALID  (axi_if.BVALID),
        .BREADY  (axi_if.BREADY),
        .ARADDR  (axi_if.ARADDR),
        .ARVALID (axi_if.ARVALID),
        .ARREADY (axi_if.ARREADY),
        .RDATA   (axi_if.RDATA),
        .RRESP   (axi_if.RRESP),
        .RVALID  (axi_if.RVALID),
        .RREADY  (axi_if.RREADY)
    );

    // Testbench component handles - non-parameterized classes, no typedef needed
    axi4_lite_driver     driver;
    axi4_lite_scoreboard scoreboard;
    axi4_lite_coverage   coverage;

    axi4_lite_txn txn;  // shared handle reused across all test tasks

    // Execute one transaction through driver, scoreboard, and coverage in order
    task run_txn(axi4_lite_txn t);
        driver.drive(t);
        scoreboard.check(t);
        coverage.sample(t);
        t.print("TB");
    endtask

    // Convenience wrapper - build and drive a directed write transaction
    task write(
        input logic [31:0] addr,
        input logic [31:0] data,
        input logic [3:0]  strb       = 4'hF,
        input int unsigned bready_dly = 0
    );
        txn              = new();
        txn.kind         = axi4_lite_txn::WRITE;
        txn.addr         = addr;
        txn.data         = data;
        txn.strb         = strb;
        txn.bready_delay = bready_dly;
        run_txn(txn);
    endtask

    // Convenience wrapper - build and drive a directed read transaction
    task read(
        input logic [31:0] addr,
        input int unsigned rready_dly = 0
    );
        txn              = new();
        txn.kind         = axi4_lite_txn::READ;
        txn.addr         = addr;
        txn.strb         = 4'hF;
        txn.rready_delay = rready_dly;
        run_txn(txn);
    endtask

    // TEST 1 - basic write then read-back on all four registers
    task test_basic_write_read();
        $display("\n[TEST 1] Basic write and read-back on all registers");
        write(32'h00, 32'hDEAD_BEEF);
        write(32'h04, 32'hCAFE_BABE);
        write(32'h08, 32'hA5A5_A5A5);
        write(32'h0C, 32'h5A5A_5A5A);
        read(32'h00);
        read(32'h04);
        read(32'h08);
        read(32'h0C);
    endtask

    // TEST 2 - partial byte-strobe writes; upper bytes must be preserved
    task test_byte_strobes();
        $display("\n[TEST 2] Byte strobe partial writes");
        write(32'h00, 32'hFFFF_FFFF, 4'hF);   // initialize to all-ones
        write(32'h00, 32'h0000_00AA, 4'h1);   // overwrite byte 0 only
        read(32'h00);
        write(32'h00, 32'hBB00_0000, 4'h8);   // overwrite byte 3 only
        read(32'h00);
        write(32'h04, 32'h1234_5678, 4'hF);
        write(32'h04, 32'h0000_CCDD, 4'h3);   // overwrite lower two bytes only
        read(32'h04);
    endtask

    // TEST 3 - out-of-range addresses must return SLVERR; valid registers unaffected
    task test_invalid_address();
        $display("\n[TEST 3] Invalid address error response");
        write(32'h00, 32'hABCD_1234);          // pre-load known value into reg0
        txn              = new();
        txn.kind         = axi4_lite_txn::WRITE;
        txn.addr         = 32'h10;             // invalid - expect SLVERR
        txn.data         = 32'hDEAD_DEAD;
        txn.strb         = 4'hF;
        txn.bready_delay = 0;
        run_txn(txn);
        read(32'h00);                          // reg0 must be unchanged after invalid write
        txn              = new();
        txn.kind         = axi4_lite_txn::READ;
        txn.addr         = 32'hFF;             // invalid - expect SLVERR
        txn.rready_delay = 0;
        run_txn(txn);
    endtask

    // TEST 4 - slave must hold BVALID stable until master asserts BREADY
    task test_bready_delay();
        $display("\n[TEST 4] Delayed BREADY handshake");
        write(32'h00, 32'h1111_1111, 4'hF, 3);  // 3-cycle BREADY delay
        write(32'h04, 32'h2222_2222, 4'hF, 5);  // 5-cycle BREADY delay
        read(32'h00);
        read(32'h04);
    endtask

    // TEST 5 - slave must hold RVALID, RDATA, RRESP stable until master asserts RREADY
    task test_rready_delay();
        $display("\n[TEST 5] Delayed RREADY handshake");
        write(32'h08, 32'h3333_3333);
        read(32'h08, 4);                         // 4-cycle RREADY delay
        write(32'h0C, 32'h4444_4444);
        read(32'h0C, 5);                         // 5-cycle RREADY delay
    endtask

    // TEST 6 - consecutive writes exercise aw_pend clear-and-set cycle
    task test_consecutive_writes();
        $display("\n[TEST 6] Consecutive writes to all registers");
        write(32'h00, 32'hAAAA_AAAA);
        write(32'h04, 32'hBBBB_BBBB);
        write(32'h08, 32'hCCCC_CCCC);
        write(32'h0C, 32'hDDDD_DDDD);
        read(32'h00);
        read(32'h04);
        read(32'h08);
        read(32'h0C);
    endtask

    // TEST 7 - read-modify-write coherence on the same register
    task test_write_after_read();
        $display("\n[TEST 7] Write-after-read coherence");
        write(32'h00, 32'h0000_0001);
        read(32'h00);
        write(32'h00, 32'h0000_0002);
        read(32'h00);
        write(32'h00, 32'hFFFF_FFFF);
        read(32'h00);
    endtask

    // TEST 8 - constrained random transactions to saturate all coverage bins
    task test_random(input int unsigned num_txns = 50);
        $display("\n[TEST 8] Constrained random - %0d transactions", num_txns);
        repeat(num_txns) begin
            txn = new();
            if (!txn.randomize())
                $fatal(1, "[TB] Randomization failed");
            run_txn(txn);
        end
    endtask

    // TEST 9 - walking bit patterns expose stuck-at faults on the data bus
    task test_data_patterns();
        $display("\n[TEST 9] Data boundary patterns");
        write(32'h00, 32'hFFFF_FFFF);
        read(32'h00);
        write(32'h00, 32'h0000_0000);
        read(32'h00);
        write(32'h00, 32'h5555_5555);
        read(32'h00);
        write(32'h00, 32'hAAAA_AAAA);
        read(32'h00);
    endtask

    initial begin
        driver     = new(axi_if.master);
        scoreboard = new();
        coverage   = new();

        driver.idle();  // deassert all master signals before reset releases

        ARESETn = 1'b0;
        repeat(5) @(posedge ACLK);
        @(negedge ACLK);
        ARESETn = 1'b1;
        $display("[TB] Reset deasserted at %0t", $time);

        repeat(2) @(posedge ACLK);  // allow DUT to settle

        test_basic_write_read();
        test_byte_strobes();
        test_invalid_address();
        test_bready_delay();
        test_rready_delay();
        test_consecutive_writes();
        test_write_after_read();
        test_data_patterns();
        test_random(100);  // random last - mops up any uncovered bins

        repeat(10) @(posedge ACLK);

        scoreboard.report();
        coverage.report();

        $finish;
    end

    // Timeout guard - kills simulation if a protocol deadlock causes an infinite wait
    initial begin
        #500000;
        $fatal(1, "[TB] TIMEOUT: simulation exceeded 500us - possible protocol deadlock");
    end

    // VCD dump for waveform viewing in Vivado or GTKWave
    initial begin
        $dumpfile("tb_axi4_lite_slave.vcd");
        $dumpvars(0, tb_axi4_lite_slave);
    end

endmodule