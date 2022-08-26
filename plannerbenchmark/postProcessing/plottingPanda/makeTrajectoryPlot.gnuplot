set term postscript eps color size 8, 5 "../RomanSerif.tff" 55
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolder=ARG1
inFile=resFolder."/res.csv"
outFile=resFolder."/plots/trajectory.eps"
set output outFile
set datafile separator ','
set yrange [-0.8: 0.8]
set xrange [0:60]
set grid
set label 'time[s]' at first 50, graph 0 offset 0,-0.5
set xtics offset 0, 0.5
set xtics (0, 20, 40)
set ytics autofreq -0.75, 0.25
set ylabel "x_{ee}[m]" offset 2,0
set key outside right vertical
set border 3


plot inFile using 1:"goal_0_0" with lines lt rgb "#FFCECF" lw 10 title '~x{0.3\~}_0', \
  inFile using 1:"goal_1_0" with lines lt rgb "#CFFFCF"lw 10 title '~x{0.3\~}_1', \
  inFile using 1:"goal_2_0" with lines lt rgb "#CFD1FF" lw 10 title '~x{0.3\~}_2', \
  inFile using 1:"fk7_x" with lines lt rgb "red" lw 5 title 'x_{0}', \
  inFile using 1:"fk7_y" with lines lt rgb "green" lw 5  title 'x_{1}', \
  inFile using 1:"fk7_z" with lines lt rgb "blue" lw 5 title 'x_{2}'

print "Done creating trajectory plot, saved to"
print outFile

# 3dPlot
#outfile=resFolder."/plots/trajectory_3d.eps"
#set output outfile

#set xrange [-0.0: 0.75]
#set yrange [-1.0: 1.0]
#set zrange [0.0: 1.0]
#set xtics (0.0, 0.25, 0.5)
#set ytics (-1.0, 0.0, 0.0)
#set ztics (0.0, 1.0)
#set view 30, 30, 1, 1
#set grid xtics
#set grid ytics
#set grid ztics
#set xlabel "x_{ee}"
#set ylabel "y_{ee}"
#set zlabel "z_{ee}"

#splot inFile using "goal_0_0":"goal_1_0":"goal_2_0" with lines lt rgb "green" lw 2 title '~x{0.3\~}_0', \
#  inFile using "fk7_x":"fk7_y":"fk7_z" with lines lt rgb "red" lw 5 title '~x{0.3\~}_0', \


#print "Done creating trajectory plot, saved to"
#print outFile
