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
ee_x = ARG2+1
ee_y = ARG3+1
n = ARG4
goal_x = ARG5+1
goal_y = ARG6+1
set output outFile
set datafile separator ','
set xrange [-n: n]
set yrange [-n: n]
set size ratio -1
set grid

obst_list=system("ls -1B ".resFolder."/obst_*")

plot inFile using goal_x:goal_y with lines lt rgb "#CFFFCF" lw 10 title "goal trajectory", \
  inFile using ee_x:ee_y with lines lw 2 notitle, \
  for [file in obst_list] file w lines lc rgb "black" lw 10 notitle, \
  fkStartFile using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "black" lw 2 title "start config", \
  fkLastFile using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "green" lw 2 title "final config", \
  fkStartFile every 1::n using 1:2 with linespoints pointtype 15 pointsize 3 lc rgb "black" lw 2 notitle, \
  fkLastFile every 1::n using 1:2 with linespoints pointtype 15 pointsize 3 lc rgb "green" lw 2 notitle, \
  inFile every 1000::::3000 using goal_x:goal_y with points pointsize 3 pointtype 14 lc rgb 'red' title "final goal"

print "Done creating trajectory plot, saved to"
print outFile
