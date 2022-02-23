set term postscript eps color size 5, 5 "../RomanSerif.tff" 35
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolder=ARG1
inFile=resFolder."/res.csv"
outFile=resFolder."/plots/trajectory.eps"
set output outFile
set datafile separator ','
set yrange [-0.6: 1.5]
set grid
set xlabel "time[s]"
set ylabel "x_{ee}[m]"


plot inFile using 1:"goal_0_0" with lines lt rgb "#FFCECF" lw 10 title '~x{0.3\~}_0', \
  inFile using 1:"goal_1_0" with lines lt rgb "#CFFFCF"lw 10 title '~x{0.3\~}_1', \
  inFile using 1:"goal_2_0" with lines lt rgb "#CFD1FF" lw 10 title '~x{0.3\~}_2', \
  inFile using 1:"fk7_x" with lines lt rgb "red" lw 5 title 'x_{0}', \
  inFile using 1:"fk7_y" with lines lt rgb "green" lw 5  title 'x_{1}', \
  inFile using 1:"fk7_z" with lines lt rgb "blue" lw 5 title 'x_{2}'



print "Done creating trajectory plot, saved to"
print outFile

# goal_x_1 = "goal_0_1"
# goal_y_1 = "goal_1_1"
# goal_z_1 = "goal_2_1"
# goal_x_2 = "goal_0_2"
# goal_y_2 = "goal_1_2"
# goal_z_2 = "goal_2_2"
# plotting goal derivatives
#  inFile using 1:goal_x_1 with lines lt rgb "#FFCECF" lw 2 title 'goal_0_1', \
#  inFile using 1:goal_y_1 with lines lt rgb "#CFFFCF"lw 2 title 'goal_1_1', \
#  inFile using 1:goal_z_1 with lines lt rgb "#CFD1FF" lw 2 title 'goal_2_1', \
#  inFile using 1:goal_x_2 with lines lt rgb "#FFCECF" lw 1 title 'goal_0_2', \
#  inFile using 1:goal_y_2 with lines lt rgb "#CFFFCF"lw 1 title 'goal_1_2', \
#  inFile using 1:goal_z_2 with lines lt rgb "#CFD1FF" lw 1 title 'goal_2_2', \
