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

obst_list=system("ls -1B ".resFolder."/obst_*")

plot inFile using "goal_0_0":"goal_1_0" with lines lt rgb "#CFFFCF" lw 10 title "goal trajectory", \
  for [file in obst_list] file every :::0::0 w lines lc rgb "black" lw 20 notitle, \
  initStateFile using 1:2 with points lc rgb "green" title "initial State", \
  inFile every 1000::::3000 using "goal_0_0":"goal_1_0" with points pointsize 3 pointtype 14 lc rgb 'red' title "final goal", \
  inFile using "fk2_x":"fk2_y" with lines lc rgb "violet" lw 2 notitle, \
  inFile using "fk2_x":"fk2_y" every 300 with points lc rgb "violet" pointtype 7  title "robot trajectory", \
  # inFile every 100::1 using "obst_0_0":"obst_1_0" with linespoints pointtype 7 ps 0.5 lc rgb "gray" title "obstacle trajectory", \
  # for [file in obst_list] file every :::1::10 w lines lc rgb "gray" lw 20 title "obstacle trajectory", \

print "Done creating trajectory plot, saved to"
print outFile
