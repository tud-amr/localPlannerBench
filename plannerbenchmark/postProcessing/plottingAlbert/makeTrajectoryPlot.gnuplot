set term postscript eps color size 8, 5 "../RomanSerif.tff" 55
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolder=ARG1
inFile=resFolder."/res.csv"
outFile=resFolder."/plots/trajectory.eps"
set output outFile
set datafile separator ','
set yrange [0.0: 3.5]
#set xrange [0:60]
set border 0
set grid
set label 'time[s]' at first 50, graph 0 offset 0,-0.5
set xtics offset 0, 0
#set xtics (0, 20, 40)
set ytics (0, 1.0, 2.0)
set ytics nomirror
set xtics nomirror
set ylabel "x_{ee}[m]" offset 2,0
set xlabel "time[s]"
set key inside top center horizontal


plot inFile using 1:"goal_0_0" every 200::100 with linespoints ps 3 pt 7 lc rgb "#FFCECF" lw 5 title '~x{0.3\~}_0', \
  inFile using 1:"goal_1_0" every 200 with linespoints ps 3  pt 7 lc rgb "#CFFFCF"lw 5 title '~x{0.3\~}_1', \
  inFile using 1:"goal_2_0" every 200::50 with linespoints ps 3 pt 7 lc rgb "#CFD1FF" lw 5 title '~x{0.3\~}_2', \
  inFile using 1:"fk10_x" with lines lt rgb "red" lw 5 title 'x_{0}', \
  inFile using 1:"fk10_y" with lines lt rgb "green" lw 5  title 'x_{1}', \
  inFile using 1:"fk10_z" with lines lt rgb "blue" lw 5 title 'x_{2}'



print "Done creating trajectory plot, saved to"
print outFile
