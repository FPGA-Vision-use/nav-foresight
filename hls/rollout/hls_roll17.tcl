# rollout v1.7 可复现流程（128 位写端口）
#   关键 1：max_widen_bitwidth 只能通过 config_interface 设（不是 m_axi pragma 的选项）
#   关键 2：export_design 的输出目录**必须是新的**，已存在时它会静默空转（拿不到新 RTL）
set root C:/Users/haoyu/.zcode/workspace/default/delivery_kernel/rollout
set outdir D:/zcode_vivado_work/kv_roll17_repro
open_project -reset $root/cproj_roll17_repro
add_files $root/rollout.cpp
add_files -tb $root/tb.cpp
set_top rollout
open_solution -reset sol
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_interface -m_axi_max_widen_bitwidth 128
csim_design -clean
csynth_design
file delete -force $outdir   ;# 关键：输出目录存在时 export_design 会静默空转
file delete -force ${outdir}.zip
export_design -format ip_catalog -vendor zhang -library hls -version 1.7 -taxonomy /UserIP -output $outdir
exit
