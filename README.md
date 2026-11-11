# High-Speed 8b/10b Line Encoder and Decoder with Serialization & Error Detection

A synthesizable, pure **Verilog-2001** implementation of the IBM-standard 8b/10b physical-layer transmission codec. This block models high-speed SerDes datapaths used across networking and interconnect protocols (such as PCIe Gen1/Gen2, Gigabit Ethernet, SATA, DisplayPort, and Fiber Channel), integrating 5b/6b + 3b/4b sub-block encoding, dynamic Running Disparity (RD) balancing, special K-character comma synchronization, and illegal symbol error detection over a simulated physical link.

---

## 1. Architectural Overview

High-speed serial interfaces transmit data as a continuous bitstream over copper traces or fiber-optic channels. Transmitting raw Non-Return-to-Zero (NRZ) binary directly across these links leads to physical degradation:
1. **DC Baseline Wander:** Consecutive unbalances in the ratio of $1\text{s}$ to $0\text{s}$ induce capacitive charge accumulation on AC-coupled lines, distorting voltage thresholds and raising Bit Error Rates (BER).
2. **Clock-Data Recovery (CDR) Drift:** Long runs of identical bits deny the receiver's Phase-Locked Loop (PLL) the edge transitions necessary to lock onto the baud clock.

The 8b/10b transmission code maps 8-bit data bytes to 10-bit symbols to bound the maximum run length to 5 consecutive identical bits, ensure frequent transitions for CDR lock, and enforce long-term DC neutrality.

```
                                  TOP-LEVEL TRANSCEIVER ARCHITECTURE
                                   (transceiver_8b10b_top.v)

                   +-------------------------------------------------------------------------+
                   |                                                                         |
  tx_data[7:0] ----+--> [ 5b/6b Encoder ] --+                                                |
                   |                        +--> 10-bit Symbol                               |
  tx_isk ----------+--> [ 3b/4b Encoder ] --+         |                                      |
                   |            ^                     v                                      |
                   |            | Running        +----------+                                |
                   |     [ Disparity FSM ]       | Channel  | <--- inject_err (fault knob)   |
                   |            |                | Register |                                |
                   |            v                +----------+                                |
  tx_disp <--------+----+ (RD+ / RD-)                 | 10-bit Serial Link                   |
                   |                                  v                                      |
                   |                             +----------+                                |
                   |                             |  10b/8b  | ---> rx_data[7:0]              |
                   |                             | Decoder  | ---> rx_isk (K-character)      |
                   |                             |  Engine  | ---> rx_err (Code violation)   |
                   |                             +----------+                                |
                   +-------------------------------------------------------------------------+
```

### Module Breakdown

| Module | Purpose / Hardware Responsibility |
| :--- | :--- |
| `encode_8b10b.v` | Implements partitioned 5b/6b and 3b/4b combinational lookup tables, tracks transmitter Running Disparity (RD), and maps data/control frames into balanced 10-bit code groups. |
| `decode_8b10b.v` | De-serializes/decodes 10-bit symbols back into 8-bit octets, differentiates data symbols ($D.x.y$) from control symbols ($K.x.y$), and asserts single-cycle error flags upon receiving invalid code points. |
| `transceiver_8b10b_top.v` | Structural wrapper linking the encoder and decoder across a registered serial channel with a runtime error injection port for verification. |

---

## 2. Technical Theory of Operation

### 2.1 Sub-Block Partitioning ($5b/6b + 3b/4b$)
An 8-bit input byte $HGF\_EDCBA$ is partitioned into two distinct blocks:
* **Lower 5 bits ($EDCBA$, decimal $A$ through $E$):** Encoded into a 6-bit sub-block ($abcdei$).
* **Upper 3 bits ($HGF$, decimal $F$ through $H$):** Encoded into a 4-bit sub-block ($fghj$).
* The complete 10-bit symbol is output in the standard transmission sequence:
  
  $$\text{Symbol}[9:0] = \{j, h, g, f, i, e, d, c, b, a\}$$

Naming convention: A byte with lower value $X$ and upper value $Y$ is classified as data symbol $D.X.Y$ or control symbol $K.X.Y$.

### 2.2 Running Disparity (RD) Calculation & DC Balance
The disparity of a block is defined as the difference between the number of $1\text{s}$ and the number of $0\text{s}$.
* **Zero Disparity:** Equal numbers of $1\text{s}$ and $0\text{s}$ (e.g., 3 ones and 3 zeros in a 6-bit block). The running disparity remains unchanged.
* **Non-Zero Disparity ($\pm 2$):** A block contains either four $1\text{s}$ and two $0\text{s}$ ($+2$), or two $1\text{s}$ and four $0\text{s}$ ($-2$).

The encoder maintains an internal 1-bit state variable:

$$\text{RD} \in \{0 \ (\text{representing } -1), \ 1 \ (\text{representing } +1)\}$$

* When $\text{RD} = -1$, the encoder selects a code group with disparity $0$ or $+2$.
* When $\text{RD} = +1$, the encoder selects the complementary code group with disparity $0$ or $-2$.
* This ensures that over any sequence of transmitted characters, the total cumulative disparity never exceeds $\pm 1$ at sub-block boundaries, keeping the physical line DC balanced.

### 2.3 Special Character Demarcation (K28.5 Comma)
Control symbols ($K$-characters) provide out-of-band signaling for link synchronization, clock compensation, and frame delineation.
* This project implements **K28.5** (`data_in = 8'hBC`, `is_k = 1`), represented by:
  
  $$K28.5 \implies \text{Hex } 0xBC \implies \text{Binary } 101\_11100_2 \ (X=28, Y=5)$$

* Depending on the current disparity state, K28.5 encodes to:
  * $\text{RD} = -1 \longrightarrow 110000\_0101_2$
  * $\text{RD} = +1 \longrightarrow 001111\_1010_2$
* Both encodings contain a unique **comma sequence** (`0011111` or `1100000`—a run of five identical bits followed by two opposite bits) that **never appears anywhere in standard data bitstreams**, even across symbol boundaries. Serializers and deserializers use this pattern to lock symbol frame alignment.

### 2.4 Error Detection & Violation Trapping
The decoder continuously checks incoming 10-bit code groups against standard code tables:
* If a 6-bit or 4-bit slice contains an illegal bit combination (e.g., all zeros, all ones, or invalid weight balances), `code_err` asserts high.
* Disparity violations (such as receiving a $+2$ disparity block when the receiver was in $\text{RD} = +1$) are flagged instantly.

---

## 3. Interface Signals & Parameters

### Top-Level Port List (`transceiver_8b10b_top`)

| Port Name | Direction | Width | Clock Domain | Description |
| :--- | :---: | :---: | :---: | :--- |
| `clk` | Input | 1 | System | System clock (e.g., 100 MHz). |
| `rst_n` | Input | 1 | Async | Active-low system reset. |
| `tx_data` | Input | 8 | `clk` | Parallel input data byte to transmit. |
| `tx_isk` | Input | 1 | `clk` | Control character select (`1` = K-character, `0` = standard data). |
| `inject_err` | Input | 1 | `clk` | Verification hook: inverts channel LSB to induce an illegal symbol. |
| `rx_data` | Output | 8 | `clk` | Parallel recovered data byte from decoder. |
| `rx_isk` | Output | 1 | `clk` | Received character type (`1` = special control symbol, `0` = data). |
| `rx_err` | Output | 1 | `clk` | Error flag (`1` = invalid 10b symbol detected). |
| `tx_disp` | Output | 1 | `clk` | Transmitter running disparity status (`0` = negative, `1` = positive). |

---

## 4. Verification Plan & Test Scenarios

The accompanying self-checking testbench (`tb_transceiver_8b10b.v`) executes three verification phases:

```
Stimulus Timeline:
+-------------------+-----------------------------+-------------------------------+
| Reset & Comma Sync|   Sequential Data Stream    | Single-Bit Channel Invalidation|
| (K28.5 / 0xBC)    |    (0x00 through 0x0F)      |    (inject_err asserted)       |
+-------------------+-----------------------------+-------------------------------+
0ns                 50ns                          210ns                           250ns
```

1. **Comma Alignment & K-Character Verification:**
   * Transmits repeated $K28.5$ (`8'hBC`) control characters.
   * Asserts that `rx_isk` activates high and `rx_data` decodes back to `0xBC`.
2. **Payload Data Stream & Disparity Toggling:**
   * Drives incremental data bytes (`0x00` through `0x0F`) through the codec.
   * Validates that non-disparity-neutral symbols force `tx_disp` to invert on subsequent cycles, maintaining line balance.
3. **Fault Injection & Error Detection:**
   * Asserts `inject_err = 1` during the transmission of byte `0x05`.
   * Verifies that the decoder catches the non-compliant code point and asserts `rx_err` high.

---

## 5. Quickstart Guide: Running on EDA Playground

1. Open [EDA Playground](https://www.edaplayground.com/).
2. Configure the simulation engine on the left-hand panel:
   * **Testbench + Design:** Select `SystemVerilog/Verilog`.
   * **Simulator:** Select `Icarus Verilog 0.10.0` or `Aldec Riviera Pro`.
   * Check the checkbox: **Open EPWave after run**.
3. Copy the design code into the right-hand editor pane (`design.sv`).
4. Copy the testbench code into the left-hand editor pane (`testbench.sv`).
5. Click **Run**.

### Expected Console Output
```text
--- 1. Transmitting K28.5 Sync Commas (0xBC) ---
--- 2. Transmitting Regular Data Stream ---
[TIME: 65000 ps] [K-CHAR DETECTED] Special Control Character: 0xbc
[TIME: 75000 ps] [K-CHAR DETECTED] Special Control Character: 0xbc
[TIME: 85000 ps] [DATA RECEIVED] Byte: 0x0 (Disparity: 1)
[TIME: 95000 ps] [DATA RECEIVED] Byte: 0x1 (Disparity: 0)
...
--- 3. Injecting Bit Error on Channel ---
[TIME: 245000 ps] [ERROR DETECTED] Corrupted 10b symbol caught by decoder!
--- Verification Completed Successfully ---
```

---
