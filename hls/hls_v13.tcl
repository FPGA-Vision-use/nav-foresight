# 从当前 clearance.cpp 重新生成 IP v1.1：csim 出参考 → csynth → 导出
# 版本号从 1.0 升到 1.1 是为了绕开 Vivado IPCACHE 按 VLNV 命中的旧网表
set root C:/Users/haoyu/.zcode/workspace/default/delivery_kernel
open_project -reset $root/cproj13
add_files $root/clearance.cpp
add_files -tb $root/tb_clear.cpp
set_top clearance
open_solution -reset sol
set_part xck26-sfvc784-2LV-c
create_clock -period 12.5 -name default
puts "===== CSIM ====="
csim_design -clean
puts "===== CSYNTH ====="
csynth_design
puts "===== EXPORT ====="
file delete -force D:/zcode_vivado_work/kv_ip13
file delete -force D:/zcode_vivado_work/kv_ip13.zip
export_design -format ip_catalog -vendor zhang -library hls -version 1.3 \
  -taxonomy /UserIP -output D:/zcode_vivado_work/kv_ip13
puts "===== HLS_DONE ====="
exit
