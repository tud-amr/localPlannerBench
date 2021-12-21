set term postscript eps color size 5, 5
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolder=ARG1
inFile=resFolder."/res.csv"
goalFile=resFolder."/goal.csv"
initStateFile=resFolder."/initState.csv"
fkLastFile=resFolder."/fk_last.csv"
fkStartFile=resFolder."/fk_first.csv"
outFile=resFolder."/plots/trajectory.eps"
n = ARG2
dynamicObst = 0
ee_x = "fk".n."_x"
ee_y = "fk".n."_y"
goal_x = "goal_0_0"
goal_y = "goal_1_0"
obst0 = "obst_0_0"
obst1 = "obst_1_0"
set output outFile
set datafile separator ','
set multiplot
set xrange [-n: n]
set yrange [-n: n]
set size ratio -1
set grid

obst_list=system("ls -1B ".resFolder."/obst_*")

plot inFile using goal_x:goal_y with lines lt rgb "#CFFFCF" lw 10 title "goal trajectory", \
  inFile using ee_x:ee_y with lines lw 2 notitle, \
  for [file in obst_list] file every :::0::0 w lines lc rgb "black" lw 20 notitle, \
  fkStartFile using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "black" lw 2 title "start config", \
  fkLastFile using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "green" lw 2 title "final config", \
  fkStartFile every 1::n using 1:2 with linespoints pointtype 15 pointsize 3 lc rgb "black" lw 2 notitle, \
  fkLastFile every 1::n using 1:2 with linespoints pointtype 15 pointsize 3 lc rgb "green" lw 2 notitle, \
  inFile every 1000::::3000 using goal_x:goal_y with points \
    pointsize 3 pointtype 14 lc rgb 'red' title "final goal"
if (dynamicObst == 1) {
  set key left top
  plot inFile every 100::100 using obst0:obst1 with lines lw 5 lc rgb "blue" title "obstacle trajectory"
  # plot for [file in obst_list] file every :::2::10 w lines lc rgb "gray" lw 20 notitle
}

print "Done creating trajectory plot, saved to"
print outFile
