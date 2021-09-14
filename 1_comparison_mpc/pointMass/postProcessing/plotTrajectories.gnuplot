set term postscript eps color size 5, 5
exp_name="mpc"
resFolder="../results/".exp_name
inFile=resFolder."/res.csv"
outFile="trajectories_".exp_name.".eps"
set output outFile
set datafile separator ','
set xrange [-4: 4]
set yrange [-4: 4]
set size ratio -1
set grid

q0 = 3
q1 = 6
n_obst = 1
list=system("ls -1B ".resFolder."/obst_*")
print list

set title "MPC"
plot inFile using q0:q1 with lines lw 2 notitle, \
  for [file in list] file w lines lc rgb "black" lw 10 notitle

exp_name="fabric"
resFolder="../results/".exp_name
inFile=resFolder."/res.csv"
outFile="trajectories_".exp_name.".eps"
set output outFile
set datafile separator ','
set xrange [-4: 4]
set yrange [-4: 4]
set size ratio -1
set grid

q0 = 3
q1 = 6
n_obst = 1
list=system("ls -1B ".resFolder."/obst_*")
print list

set title "Fabric"
plot inFile using q0:q1 with lines lw 2 notitle, \
  for [file in list] file w lines lc rgb "black" lw 10 notitle



