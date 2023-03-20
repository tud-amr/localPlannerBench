set term postscript eps color size 5, 5
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolder=ARG1
inFile=resFolder."/res.csv"
outFile=resFolder."/plots/trajectory.eps"
set output outFile
set datafile separator ','
set xrange [-5: 5]
set yrange [-5: 5]
set size ratio -1
set grid

obst_list=system("ls -1B ".resFolder."/obst_*")

plot inFile using "goal_0_0_0":"goal_0_1_0" with linespoint pt 0 lt rgb "#CFFFCF" lw 25 title "goal trajectory", \
  for [file in obst_list] file every :::0::0 w lines lc rgb "black" lw 20 notitle, \
  inFile using "fkbase_link_x":"fkbase_link_y" with lines lc rgb "violet" lw 2 notitle, \
  inFile using "fkbase_link_x":"fkbase_link_y" every 300 with points lc rgb "violet" pointtype 7  title "robot trajectory", \

print "Done creating trajectory plot, saved to"
print outFile
