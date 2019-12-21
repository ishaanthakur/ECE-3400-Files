transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+C:/Users/Vlad\ Mihai/Desktop/ECE3400/display_driver {C:/Users/Vlad Mihai/Desktop/ECE3400/display_driver/vga_driver.v}
vlog -vlog01compat -work work +incdir+C:/Users/Vlad\ Mihai/Desktop/ECE3400/display_driver {C:/Users/Vlad Mihai/Desktop/ECE3400/display_driver/display_driver.v}
vlog -vlog01compat -work work +incdir+C:/Users/Vlad\ Mihai/Desktop/ECE3400/display_driver {C:/Users/Vlad Mihai/Desktop/ECE3400/display_driver/vga_driver_pll.v}
vlog -vlog01compat -work work +incdir+C:/Users/Vlad\ Mihai/Desktop/ECE3400/display_driver/db {C:/Users/Vlad Mihai/Desktop/ECE3400/display_driver/db/vga_driver_pll_altpll.v}

