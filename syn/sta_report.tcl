set top [lindex $argv 0]
project_open $top
create_timing_netlist -model slow
read_sdc
update_timing_netlist
set r [report_timing -setup -npaths 10 -detail summary -panel_name s -file paths.txt]
puts "WORST setup slack: [lindex $r 1]"
report_timing -setup -npaths 3 -detail full_path -file worst_full.txt
# every failing endpoint's worst path, for grouping by module (syn/paths_by_module.py)
report_timing -setup -npaths 20000 -nworst 1 -less_than_slack 0 -detail summary -file failing.txt
foreach_in_collection c [get_clocks] { puts "clock [get_clock_info -name $c]" }
report_clock_fmax_summary -file fmax.txt
# total negative slack and the failing endpoint count, whatever the 20000 cap above
create_timing_summary -setup -file tsum.txt
# failing endpoints by slack (1 ns bins): the whole picture, uncapped
set h [dict create]
set ne 0
foreach_in_collection p [get_timing_paths -setup -npaths 1000000 -nworst 1 -less_than_slack 0] {
	set b [expr {int(floor([get_path_info $p -slack]))}]
	dict incr h $b
	incr ne
}
set f [open hist.txt w]
puts $f "failing endpoints: $ne"
foreach b [lsort -integer [dict keys $h]] { puts $f [format "%4d..%4d ns: %d" $b [expr {$b+1}] [dict get $h $b]] }
close $f
delete_timing_netlist
project_close
