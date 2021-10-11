set term postscript eps color size 5, 5
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolderMpc=ARG1
mpc_name="mpc"
inFileMpc=resFolderMpc."/res.csv"
outFileMpc="trajectories_".mpc_name.".eps"

resFolderFabric=ARG2
fabric_name="fabric"
inFileFabric=resFolderFabric."/res.csv"
outFileFabric="trajectories_".fabric_name.".eps"
set output outFileMpc
set datafile separator ','
set xrange [-4: 4]
set yrange [-4: 4]
set size ratio -1
set grid

q0 = 3
q1 = 6
n_obst = 1
list=system("ls -1B ".resFolderMpc."/obst_*")

set title "MPC"
plot inFileMpc using q0:q1 with lines lw 2 notitle, \
  for [file in list] file w lines lc rgb "black" lw 10 notitle

# exp_name="fabric_20210915_091526"
# resFolder="../results/".exp_name
set output outFileFabric
set datafile separator ','
set xrange [-4: 4]
set yrange [-4: 4]
set size ratio -1
set grid

q0 = 3
q1 = 6
n_obst = 1
list=system("ls -1B ".resFolderFabric."/obst_*")
print list

set title "Fabric"
plot inFileFabric using q0:q1 with lines lw 2 notitle, \
  for [file in list] file w lines lc rgb "black" lw 10 notitle

