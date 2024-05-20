# 
# Questa Static Verification System
# Version 2019.2_1 linux_x86_64 18 May 2019

clear settings -all
clear directives
netlist clock clk -period 10 
netlist constraint rst -value 1'b0 -after_init 
formal compile -d fifo_equ -cuname fifo_equ
formal verify  -init  $env(SRC_ROOT)//homes/user/stud/fall23/yl5334/tutorials_6863/FIFO/qs_files/wb_arbiter.init -timeout 5m
