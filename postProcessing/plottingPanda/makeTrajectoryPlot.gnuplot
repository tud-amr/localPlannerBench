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
set yrange [-0.6: 1.5]
set grid
set xlabel "time[s]"
set ylabel "x_{ee}[m]"

ee_x = "fk7_x"
ee_y = "fk7_y"
ee_z = "fk7_z"
goal_x = "goal_0_0"
goal_y = "goal_1_0"
goal_z = "goal_2_0"
obst_list=system("ls -1B ".resFolder."/obst_*")

plot inFile using 1:goal_x with lines lt rgb "#FFCECF" lw 8 title 'goal_0', \
  inFile using 1:goal_y with lines lt rgb "#CFFFCF"lw 8 title 'goal_1', \
  inFile using 1:goal_z with lines lt rgb "#CFD1FF" lw 8 title 'goal_2', \
  inFile using 1:ee_x with lines lt rgb "red" lw 2 title 'x_{0}', \
  inFile using 1:ee_y with lines lt rgb "green" lw 2  title 'x_{1}', \
  inFile using 1:ee_z with lines lt rgb "blue" lw 2 title 'x_{2}'

print "Done creating trajectory plot, saved to"
print outFile
