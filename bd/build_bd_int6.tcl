# KV260 双核集成工程：rollout(R2) + clearance(R1)，单 50 MHz，共享 DDR（2026-10-29 夜）
#
# 目的：一帧完整 PL 流水 —— rollout 把 43×400 候选点写进 DDR，clearance 就着同一块缓冲算，
#       中间不经过 PS。这是 INTERFACE.md 第 7 节 R2→R1 的真实形态（原计划写 BRAM，
#       实际用 DDR 更简单，因为两个 IP 的 m_axi 本来就都能直接读写 DDR）。
#
# 时钟：全部 50 MHz。rollout 在 100 MHz 下实测 10.70 ms，50 MHz 约 21.4 ms；
#       clearance 只能跑到 85–90 MHz，所以统一 50 MHz 是唯一不需要跨时钟桥的方案。
#       （想快就把 rollout 单独放 100 MHz，控制通路要加 axi_clock_converter，留作后续优化。）
#
# 结构：
#   控制  PS/M_AXI_HPM0_LPD(32b) → smc_ctrl(3 MI, 全 32 位纯开关)
#         M00→roll/s_axi_control  M01→roll/s_axi_control_r
#         M02→clr/s_axi_control   M03→clr/s_axi_control_r
#         （两个 IP 各有一个"地址寄存器"从口，漏接任何一个 → 那个核的 m_axi 基址永远是 0）
#   数据  roll/m_axi_gmem(32b)  → dwc_roll (32→128)  ┐
#         clr/m_axi_gmem(256b)  → dwc_clr(256→128)  ┴→ smc_data(2 SI/1 MI, 全 128 位) → ps/S_AXI_HP0_FPD
#
# 用法：vivado -mode batch -source build_bd_int.tcl -tclargs impl

set ip_repo_c "D:/zcode_vivado_work/kv_ip13"
set ip_repo_r "D:/zcode_vivado_work/kv_roll17"
set proj     "D:/zcode_vivado_work/kv_build_int6"
set do_impl  [expr {[lsearch -exact $argv "impl"] >= 0}]

create_project -force kv260_int6 $proj -part xck26-sfvc784-2LV-c
set_property board_part xilinx.com:kv260_som:part0:1.4 [current_project]
set_property ip_repo_paths [list $ip_repo_c $ip_repo_r] [current_project]
update_ip_catalog -rebuild

create_bd_design bd

# ---------------- PS ----------------
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} [get_bd_cells ps]
set_property -dict [list \
  CONFIG.PSU__USE__M_AXI_GP2 {1} \
  CONFIG.PSU__MAXIGP2__DATA_WIDTH {32} \
  CONFIG.PSU__USE__S_AXI_GP2 {1} \
  CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
  CONFIG.PSU__FPGA_PL0_ENABLE {1} \
  CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
] [get_bd_cells ps]

# ---------------- 50 MHz 时基 ----------------
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk50
set_property -dict [list CONFIG.PRIM_IN_FREQ {99.999001} CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {50.000} \
  CONFIG.USE_LOCKED {false} CONFIG.USE_RESET {false}] [get_bd_cells clk50]
connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins clk50/clk_in1]
set clk [get_bd_pins clk50/clk_out1]
foreach pin {saxihp0_fpd_aclk maxihpm0_lpd_aclk maxihpm0_fpd_aclk maxihpm1_fpd_aclk} {
  catch { connect_bd_net $clk [get_bd_pins ps/$pin] }
}

# ---------------- 两个核 ----------------
create_bd_cell -type ip -vlnv zhang:hls:clearance:1.3 clr
create_bd_cell -type ip -vlnv zhang:hls:rollout:1.7   roll

# ---------------- 复位 ----------------
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst
connect_bd_net $clk [get_bd_pins rst/slowest_sync_clk]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins rst/ext_reset_in]
connect_bd_net $clk [get_bd_pins clr/ap_clk]
connect_bd_net $clk [get_bd_pins roll/ap_clk]
connect_bd_net [get_bd_pins rst/peripheral_aresetn] [get_bd_pins clr/ap_rst_n]
connect_bd_net [get_bd_pins rst/peripheral_aresetn] [get_bd_pins roll/ap_rst_n]

# ---------------- 数据通路：两个 dwc + 一个 128 位数据开关 ----------------
# rollout v1.7 的 m_axi 已是 128 位，直连数据开关，不需要 dwc_roll
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dwidth_converter:2.1 dwc_clr
set_property -dict [list CONFIG.SI_DATA_WIDTH {256} CONFIG.MI_DATA_WIDTH {128} \
  CONFIG.ADDR_WIDTH {64} CONFIG.PROTOCOL {AXI4} CONFIG.MAX_SPLIT_BEATS {16}] [get_bd_cells dwc_clr]
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smc_data
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] [get_bd_cells smc_data]

connect_bd_intf_net [get_bd_intf_pins clr/m_axi_gmem]  [get_bd_intf_pins dwc_clr/S_AXI]
connect_bd_intf_net [get_bd_intf_pins roll/m_axi_gmem] [get_bd_intf_pins smc_data/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins dwc_clr/M_AXI]   [get_bd_intf_pins smc_data/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins smc_data/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
connect_bd_net $clk [get_bd_pins smc_data/aclk]
connect_bd_net [get_bd_pins rst/peripheral_aresetn] [get_bd_pins smc_data/aresetn]
foreach d {dwc_clr} {
  set pc [get_bd_pins -quiet $d/s_axi_aclk]
  if {[llength $pc] == 0} { set pc [get_bd_pins $d/aclk] }
  set pr [get_bd_pins -quiet $d/s_axi_aresetn]
  if {[llength $pr] == 0} { set pr [get_bd_pins $d/aresetn] }
  connect_bd_net $clk $pc
  connect_bd_net [get_bd_pins rst/peripheral_aresetn] $pr
}

# ---------------- 控制通路：全 32 位 ----------------
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smc_ctrl
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {4}] [get_bd_cells smc_ctrl]
connect_bd_net $clk [get_bd_pins smc_ctrl/aclk]
connect_bd_net [get_bd_pins rst/peripheral_aresetn] [get_bd_pins smc_ctrl/aresetn]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_LPD] [get_bd_intf_pins smc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M00_AXI] [get_bd_intf_pins roll/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M01_AXI] [get_bd_intf_pins roll/s_axi_control_r]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M02_AXI] [get_bd_intf_pins clr/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M03_AXI] [get_bd_intf_pins clr/s_axi_control_r]

assign_bd_address
validate_bd_design
save_bd_design

# ---------------- 自查（证据）----------------
puts "##### INTERFACE WIDTHS #####"
foreach ip {smc_ctrl/S00_AXI smc_ctrl/M00_AXI smc_ctrl/M01_AXI smc_ctrl/M02_AXI smc_ctrl/M03_AXI \
            roll/s_axi_control roll/m_axi_gmem \
            clr/s_axi_control clr/s_axi_control_r clr/m_axi_gmem dwc_clr/S_AXI dwc_clr/M_AXI \
            smc_data/S00_AXI smc_data/S01_AXI smc_data/M00_AXI ps/S_AXI_HP0_FPD} {
  set p [get_bd_intf_pins $ip]
  if {[llength $p] == 0} { puts "  $ip : MISSING"; continue }
  catch { puts [format "  %-24s DATA_WIDTH=%-4s ADDR_WIDTH=%s" $ip \
      [get_property CONFIG.DATA_WIDTH $p] [get_property CONFIG.ADDR_WIDTH $p]] }
}
puts "##### CONTROL ADDRESS MAP #####"
foreach s [get_bd_addr_segs] {
  set off [get_property -quiet offset $s]
  if {[string match "0x800*" $off] || [string match "0x8000*" $off]} { puts "  $s $off" }
}

make_wrapper -files [get_files $proj/kv260_int6.srcs/sources_1/bd/bd/bd.bd] -top
add_files -norecurse $proj/kv260_int6.gen/sources_1/bd/bd/hdl/bd_wrapper.v
set_property top bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

if {!$do_impl} { puts "BD_ONLY_DONE"; exit }

launch_runs synth_1 -jobs 1
wait_on_run synth_1
puts "SYNTH_STATUS=[get_property STATUS [get_runs synth_1]]"
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 1
wait_on_run impl_1
puts "IMPL_STATUS=[get_property STATUS [get_runs impl_1]]"
open_run impl_1
report_timing_summary -file $proj/timing_summary.rpt
puts "DONE_INT"
