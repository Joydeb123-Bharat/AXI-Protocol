# AXI4-Lite Slave Peripheral — RTL Design & Functional Verification

A fully verified AXI4-Lite slave peripheral implemented in SystemVerilog, with a class-based UVM-mirrored testbench. All 150 functional checks pass with 100% coverage closure.

---

## Results

| Metric | Result |
|--------|--------|
| Scoreboard checks | 150 / 150 PASSED |
| Functional coverage | 100% |
| SVA assertion violations | 0 |
| Simulator | Vivado XSim 2025.2 |

---

## Design Overview

The DUT is a 4-register AXI4-Lite slave with the following features:

- **All five AXI4-Lite channels** implemented: AW, W, B, AR, R
- **Order-independent AW/W handling** — write address and write data are accepted independently, in either order, using separate pending flags
- **Byte-lane strobe support** — each write byte lane is independently controllable via `WSTRB`
- **SLVERR generation** — out-of-range addresses return AXI SLVERR (`2'b10`) on both read and write paths; the register file is never corrupted by an invalid write
- **Protocol-correct handshake timing** — `AWREADY` and `ARREADY` are one-cycle pulses; all `VALID` signals are held stable until their respective `READY` handshakes complete
- **Parameterized** — `ADDR_WIDTH` and `DATA_WIDTH` are module parameters

### Register Map

| Offset | Name | Access | Reset |
|--------|------|--------|-------|
| `0x00` | REG0 | R/W | `0x00000000` |
| `0x04` | REG1 | R/W | `0x00000000` |
| `0x08` | REG2 | R/W | `0x00000000` |
| `0x0C` | REG3 | R/W | `0x00000000` |

Any address outside `[0x00:0x0C]` or any unaligned access returns SLVERR.

---

## Testbench Architecture

The testbench follows a UVM-mirrored class-based architecture in pure SystemVerilog, structured for compatibility with Vivado XSim.

```
tb_axi4_lite_slave (top module)
├── axi4_lite_if          — parameterized interface with master and monitor clocking blocks
├── axi4_lite_txn         — transaction class with constrained-random fields
├── axi4_lite_driver      — drives AW/W concurrently via fork-join; B and R sequentially
├── axi4_lite_scoreboard  — golden register model; checks every BRESP, RRESP, and RDATA
├── axi4_lite_coverage    — covergroup with coverpoints and crosses across all protocol dimensions
└── axi4_lite_sva         — 15 SVA protocol assertions (VALID stability, READY pulse, legal responses)
```

### Why `fork-join` on AW and W?

The AXI4-Lite specification explicitly states that the write address and write data channels are independent — a master may send W before AW. Driving them in parallel with `fork-join` is the only correct way to exercise the DUT's order-independence logic and matches what a real AXI master would do.

### Scoreboard Methodology

The scoreboard maintains a golden register model that mirrors the DUT state byte by byte using the same strobe logic as the RTL. Every completed transaction is checked against this model independently. A mismatch on `RDATA`, `BRESP`, or `RRESP` is flagged as a failure with the expected and actual values printed.

---

## Test Plan

| Test | Scenario | Checks |
|------|----------|--------|
| `test_basic_write_read` | Write then read-back all 4 registers | RDATA matches written value |
| `test_byte_strobes` | Partial byte-lane writes | Non-written bytes preserved |
| `test_invalid_address` | Write/read to address `0x10` and `0xFF` | SLVERR returned; reg0 unchanged |
| `test_bready_delay` | Master delays BREADY by up to 5 cycles | BVALID and BRESP held stable |
| `test_rready_delay` | Master delays RREADY by up to 5 cycles | RVALID, RDATA, RRESP held stable |
| `test_consecutive_writes` | Back-to-back writes to all registers | `aw_pend` clear-set cycle correct |
| `test_write_after_read` | Read-modify-write on same register | Coherence across operations |
| `test_data_patterns` | All-ones, all-zeros, `0x55`, `0xAA` | Stuck-at fault exposure |
| `test_random` | 100 constrained-random transactions | Coverage closure |

---

## Functional Coverage Model

| Coverpoint | Description |
|------------|-------------|
| `cp_addr` | All 4 valid register offsets + invalid range |
| `cp_kind` | READ and WRITE |
| `cp_resp` | OKAY (`2'b00`) and SLVERR (`2'b10`) |
| `cp_strb` | 7 named strobe patterns + default bin |
| `cp_bready_dly` | Immediate / short (1–2 cycles) / long (3–5 cycles) |
| `cp_rready_dly` | Immediate / short (1–2 cycles) / long (3–5 cycles) |
| `cx_addr_kind` | Cross: every register accessed by both read and write |
| `cx_kind_resp` | Cross: SLVERR seen on both read and write channels |

---

## SVA Protocol Checkers

All 15 assertions are gated by `disable iff (!ARESETn)` and verify:

| Assertion | Rule |
|-----------|------|
| `assert_aw_valid_stable` | `AWVALID` must not deassert before `AWREADY` |
| `assert_aw_addr_stable` | `AWADDR` must hold while `AWVALID` high, `AWREADY` low |
| `assert_w_valid_stable` | `WVALID` must not deassert before `WREADY` |
| `assert_w_data_stable` | `WDATA`/`WSTRB` must hold while `WVALID` high, `WREADY` low |
| `assert_b_valid_stable` | `BVALID` must not deassert before `BREADY` |
| `assert_b_resp_stable` | `BRESP` must hold while `BVALID` high, `BREADY` low |
| `assert_ar_valid_stable` | `ARVALID` must not deassert before `ARREADY` |
| `assert_ar_addr_stable` | `ARADDR` must hold while `ARVALID` high, `ARREADY` low |
| `assert_r_valid_stable` | `RVALID` must not deassert before `RREADY` |
| `assert_r_data_stable` | `RDATA`/`RRESP` must hold while `RVALID` high, `RREADY` low |
| `assert_b_resp_legal` | `BRESP` must be OKAY or SLVERR only |
| `assert_r_resp_legal` | `RRESP` must be OKAY or SLVERR only |
| `assert_aw_ready_pulse` | `AWREADY` must deassert the cycle after the handshake |
| `assert_ar_ready_pulse` | `ARREADY` must deassert the cycle after the handshake |
| `assert_wstrb_nonzero` | `WSTRB` must not be all-zero while `WVALID` is asserted |

---

## Repository Structure

```
axi4-lite-slave/
├── README.md
├── .gitignore
├── rtl/
│   └── axi4_lite_slave.sv          — DUT
├── tb/
│   └── tb_axi4_lite_slave.sv       — merged testbench (interface, classes, SVA, top)
├── sim/
│   └── run_sim.sh                  — XSim compile / elaborate / simulate script
└── docs/
    └── (simulation logs, waveform screenshots)
```

---

## How to Run

### Vivado GUI

1. Add `rtl/axi4_lite_slave.sv` as a **Design Source**
2. Add `tb/tb_axi4_lite_slave.sv` as a **Simulation Source**
3. Set `tb_axi4_lite_slave` as the simulation top
4. Run **Flow → Run Simulation → Run Behavioral Simulation → Run All**

### Command Line (XSim)

```bash
cd sim
bash run_sim.sh
```

> Requires Vivado 2024.x or later on `PATH`.

---

## Key Design Decisions and Lessons Learned

**Single-driver discipline**
Each state signal (`aw_pend`, `w_pend`, `ar_pend`) is owned by exactly one `always_ff` block. Multiple drivers on the same signal produce non-deterministic simulation behavior and unroutable synthesis netlists.

**Address decode units**
The validity check compares the word index `addr[3:2]`, not the byte address, against the register count. Comparing the byte address `0x4` against the integer `4` produces a false SLVERR because they are in different units — one is a byte offset, the other is a register count.

**XSim parameterized class limitation**
XSim's elaborator crashes with `EXCEPTION_ACCESS_VIOLATION` when resolving `typedef` aliases for parameterized classes during scope analysis. The workaround is `localparam` inside non-parameterized classes with fixed widths, while keeping interfaces parameterized (which XSim handles correctly).

**Coverage is not correctness**
During debugging, functional coverage reached 100% while 93 scoreboard checks were failing — SLVERR responses on valid addresses satisfied the `cp_resp.slverr` bin. This is a direct demonstration that coverage closure and functional correctness are orthogonal properties. Both a scoreboard and a coverage model are required.

---

## Tools

| Tool | Version |
|------|---------|
| RTL / Testbench language | SystemVerilog (IEEE 1800-2017) |
| Simulator | Vivado XSim 2025.2 |
| Protocol reference | ARM IHI0022E — AMBA AXI and ACE Protocol Specification |

---
