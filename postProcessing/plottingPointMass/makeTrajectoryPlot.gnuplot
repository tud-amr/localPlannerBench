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
set output outFile
set datafile separator ','
set xrange [-5: 5]
set yrange [-5: 5]
set size ratio -1
set grid

q0 = 3
q1 = 8
goal_x = 15
goal_y = 16

obst_list=system("ls -1B ".resFolder."/obst_*")

plot inFile using goal_x:goal_y with lines lt rgb "#CFFFCF" lw 10 title "goal trajectory", \
  inFile using q0:q1 with lines lc rgb "violet" lw 2 notitle, \
  for [file in obst_list] file w lines lc rgb "black" lw 20 notitle, \
  initStateFile using 1:2 with points lc rgb "green" title "initial State", \
  inFile every 1000::::3000 using goal_x:goal_y with points pointsize 3 pointtype 14 lc rgb 'red' title "final goal"

print "Done creating trajectory plot, saved to"
print outFile
