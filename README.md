Genesys2 E203内核移植指南
=========================

## 关于本仓库

本仓库fork自[riscv-mcu/e203_hbirdv2](https://github.com/riscv-mcu/e203_hbirdv2)，并在此基础上：

1. 完成E203在Genesys2的移植，工程位于 `./fpga/genesys2`，包括适配Genesys2的顶层文件、约束文件，及相关 `make`操作，这使得综合、实现、生成mcs文件的命令与芯来官方的[文档](https://doc.nucleisys.com/hbirdv2/index.html)一致
2. 完成SDK在Genesys2上的适配，该部分参见[KafCoppelia/genesys2_hbird-sdk](https://github.com/KafCoppelia/genesys2_hbird-sdk)，详见该仓库README
3. 移除E203内核的仿真、测试工程，该部分非本仓库重点

## 关于Genesys2

有关Genesys2开发板信息、及在Vivado中的开发板选型数据，参见如下资源：

1. 参考手册：[Genesys2 Reference Manual](https://digilent.com/reference/programmable-logic/genesys-2/reference-manual)
2. Genesys2板级包：[Vivado Board Files for Digilent FPGA Boards](https://github.com/Digilent/vivado-boards)
3. Genesys2约束模板文件：[Genesys-2-Master](https://github.com/Digilent/digilent-xdc/blob/master/Genesys-2-Master.xdc "Genesys-2-Master.xdc")

## 移植指南及注意事项

### E203内核顶层

主要修改 `./genesys2/src`

1. 时钟，Genesys2时钟为一200MHz**差分时钟**，而非单端时钟，顶层端口需修改如下：

```verilog
input wire CLK200M_p, // Genesys2 has a differential LVDS 200MHz oscillator
input wire CLK200M_n,
```

时钟使用 `ip_mmcm` 生成16MHz与8.388MHz，再将8.388MHz通过简单的分频器获得32768Hz低频时钟。实际使用MMCM，无法精确获得8.388MHz。

👉 可替换MMCM为PLL

```verilog
wire clk_8388;    // 8.388MHz clock
wire clk_16M;     // 16MHz clock
wire clk_32768;   // 32768KHz clock

mmcm ip_mmcm
(
  .resetn(ck_rst),
  .clk_in1_p(CLK200M_p),
  .clk_in1_n(CLK200M_n),

  .clk_out1(clk_16M), // 16 MHz, this clock we set to 16MHz
  .clk_out2(clk_8388),
  .locked(mmcm_locked)
);

// Clock divider
sysclk_divider u_sysclk_divider(
  .clk8388(clk_8388),
  .rst_n(ck_rst),
  .clk32768(clk_32768)
);
```

其中，`sysclk_divider` 代码位于  `./src/sysclk_divider.v`，在顶层中例化。

2. 复位，若遵照原顶层的设计，需要 `fpga_rst`与 `mcu_rst`两个复位。请注意，Genesys2开发板CPU Reset(R19)，按下时为低电平，松开为高电平，而其他按键(BTNx)则是按下时为高电平。因此时钟复位信号 `ck_rst`：

```verilog
assign ck_rst = fpga_rst & (~mcu_rst);
```

👉 可简化为一个复位按键

复位IP `ip_reset_sys`，注意 `ext_reset_in`设置低电平有效。

3. QSPI，Genesys2开发板没有 `qspi0_sck`，在顶层需要移除该端口。若选择内核固化至Flash内，还需要 `STARTUPE2` 原语（参见Genesys2手册6.2节Quad-SPI Flash）：

```verilog
STARTUPE2
#(
.PROG_USR("FALSE"),
.SIM_CCLK_FREQ(0.0)
)  STARTUPE2_inst (
  .CFGCLK     (),
  .CFGMCLK    (),
  .EOS        (),
  .PREQ       (),
  .CLK        (1'b0),
  .GSR        (1'b0),
  .GTS        (1'b0),
  .KEYCLEARB  (1'b0),
  .PACK       (1'b0),
  .USRCCLKO   (qspi0_sck),  // First three cycles after config ignored, see AR# 52626
  .USRCCLKTS  (1'b0),       // 0 to enable CCLK output
  .USRDONEO   (1'b1),       // Shouldn't matter if tristate is high, but generates a warning if tied low.
  .USRDONETS  (1'b1)        // 1 to tristate DONE output
);
```

⚠️ SCK is only available via the STARTUPE2 primitive

若选择ROM启动，则修改 `assign dut_io_pads_bootrom_n_i_ival = 1'b0`，及其他必要的修改。

4. GPIO，自定义约束Genesys2开发板的GPIO口。本仓库仅约束了部分GPIO口，例如LED、拨码开关、按键、串口、JTAG等：

```verilog
inout wire [5:0] led,
inout wire [6:0] sw,
inout wire btnd,
inout wire btnl,
inout wire btnr,
inout wire btnu,
inout wire uart0_rx,
inout wire uart0_tx,
inout wire uart2_rx,
inout wire uart2_tx,
inout wire mcu_TDO,   // MCU_TDO
inout wire mcu_TCK,   // MCU_TCK
inout wire mcu_TDI,   // MCU_TDI
inout wire mcu_TMS,   // MCU_TMS
```

一般地，约束GPIO，例如：6个LED约束在E203的GPIOA[5:0]上，则：

```verilog
IOBUF
#(
  .DRIVE(12),
  .IBUF_LOW_PWR("TRUE"),
  .IOSTANDARD("DEFAULT"),
  .SLEW("SLOW")
)
led_iobuf
(
  .O(dut_io_pads_gpioA_i_ival[5:0]),
  .IO(led[5:0]),
  .I(dut_io_pads_gpioA_o_oval[5:0]),
  .T(~dut_io_pads_gpioA_o_oe[5:0])
);
```

 并根据Genesys2约束文件，约束到对应的FPGA端口即可：

```
set_property -dict { PACKAGE_PIN T28  IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN V19  IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN U30  IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN U29  IOSTANDARD LVCMOS33 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN V20  IOSTANDARD LVCMOS33 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN V26  IOSTANDARD LVCMOS33 } [get_ports { led[5] }]
```

再举一例，7个拨码开关约束在E203的GPIOA[28:22]上，则：

```verilog
IOBUF
 #(
  .DRIVE(12),
  .IBUF_LOW_PWR("TRUE"),
  .IOSTANDARD("DEFAULT"),
  .SLEW("SLOW")
)
sw_iobuf
(
  .O(dut_io_pads_gpioA_i_ival[28:22]),
  .IO(sw[6:0]),
  .I(dut_io_pads_gpioA_o_oval[28:22]),
  .T(~dut_io_pads_gpioA_o_oe[28:22])
);
```

也可对单个GPIO口约束：

```verilog
IOBUF
#(
  .DRIVE(12),
  .IBUF_LOW_PWR("TRUE"),
  .IOSTANDARD("DEFAULT"),
  .SLEW("SLOW")
)
uart0_rx_iobuf
(
  .O(dut_io_pads_gpioA_i_ival[16]),
  .IO(uart0_rx),
  .I(dut_io_pads_gpioA_o_oval[16]),
  .T(~dut_io_pads_gpioA_o_oe[16])
);
```

如此，完成对UART0 RX端口约束，将其约束在E203的GPIOA[16]。

⚠️ 注意，需要对照E203的[IOF与GPIO对应表](https://doc.nucleisys.com/hbirdv2/soc_peripherals/ips.html#sw-or-iof-configuration)，实现特定功能IO口（例如，UART、I2C、SPI等）的约束。

### FPGA约束

主要在Genesys2约束模板文件上做修改，添加部分ddr200t上的一些约束，例如：

```
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

## Clock Signal
set_property -dict {PACKAGE_PIN AD11  IOSTANDARD LVDS} [get_ports { CLK200M_n }]
set_property -dict {PACKAGE_PIN AD12  IOSTANDARD LVDS} [get_ports { CLK200M_p }]
create_clock -add -name sys_clk_pin -period 5 -waveform {0 2.5} [get_ports { CLK200M_p }]

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets dut_io_pads_jtag_TCK_i_ival]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets IOBUF_jtag_TCK/O]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
```

### Vivado综合实现

主要修改 `./script`内tcl脚本

1. `./board.tcl`修改为Genesys2对应型号：

```tcl
set name {nuclei_genesys2}
set part_fpga {xc7k325tffg900-2}
set part_board {digilentinc.com:genesys2:part0:1.1}
set bootrom_inst {rom}
```

2. `./ip.tcl`，由于修改了MMCM配置，该部分也要修改：

```tcl
create_ip -vendor xilinx.com -library ip -name clk_wiz -module_name mmcm -dir $ipdir -force
set_property -dict [list \
  CONFIG.CLK_IN1_BOARD_INTERFACE {sys_diff_clock} \
  CONFIG.PRIMITIVE {MMCM} \
  CONFIG.PRIM_SOURCE {Differential_clock_capable_pin} \
  CONFIG.PRIM_IN_FREQ {200.000} \
  CONFIG.RESET_TYPE {ACTIVE_LOW} \
  CONFIG.CLKOUT1_USED {true} \
  CONFIG.CLKOUT2_USED {true} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {16.000} \
  CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {8.388} \
  ] [get_ips mmcm]

create_ip -vendor xilinx.com -library ip -name proc_sys_reset -module_name reset_sys -dir $ipdir -force
set_property -dict [list \
  CONFIG.C_EXT_RESET_HIGH {false} \
  CONFIG.C_AUX_RESET_HIGH {false} \
  CONFIG.C_NUM_BUS_RST {1} \
  CONFIG.C_NUM_PERP_RST {1} \
  CONFIG.C_NUM_INTERCONNECT_ARESETN {1} \
  CONFIG.C_NUM_PERP_ARESETN {1} \
  ] [get_ips reset_sys]
```

⚠️ 注意 `CLKOUT1_REQUESTED_OUT_FREQ` 、`CLKOUT2_REQUESTED_OUT_FREQ`的时钟频率与例化的端口对应

3. `./cfgmem.tcl`，该文件在生成mcs文件时会调用。修改 `size`：

```tcl
set size 32
```

👉 Genesys2所使用的Flash型号为S25FL256xxxxxx0，详见手册

4. 可直接在 `./fpga`文件夹下使用如下命令，完成Vivado的综合、实现、生成mcs文件：

```makefile
make install BOARD=genesys2
make bit BOARD=genesys2
make mcs BOARD=genesys2
```

### 注意事项

本仓库所提供的Genesys2顶层文件与约束相对应，你可自由对GPIO做约束修改。
