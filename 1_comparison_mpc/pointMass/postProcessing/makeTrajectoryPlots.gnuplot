set term postscript eps color size 5, 5
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolderMpc=ARG1
timeStampMpc=ARG2
mpc_name="mpc"
inFileMpc=resFolderMpc."/res.csv"
goalFileMpc=resFolderMpc."/goal.csv"
initStateFileMpc=resFolderMpc."/initState.csv"
outFileMpc="plots/trajectories_mpc_".timeStampMpc.".eps"
outFileMpc=resFolderMpc."/plots/trajectory.eps"

resFolderFabric=ARG3
timeStampFabric=ARG4
inFileFabric=resFolderFabric."/res.csv"
goalFileFabric=resFolderFabric."/goal.csv"
initStateFileFabric=resFolderFabric."/initState.csv"
outFileFabric=resFolderFabric."/plots/trajectory.eps"
#outFileFabric="plots/trajectories_fabric_".timeStampFabric.".eps"
set output outFileMpc
set datafile separator ','
set xrange [-5: 5]
set yrange [-5: 5]
set size ratio -1
set grid

q0 = 3
q1 = 6
n_obst = 1
list=system("ls -1B ".resFolderMpc."/obst_*")

plot inFileMpc using q0:q1 with lines lw 2 notitle, \
  for [file in list] file w lines lc rgb "black" lw 10 notitle, \
  goalFileMpc using 1:2 with points title "goal", \
  initStateFileMpc using 1:2 with points lc rgb "green" title "initial State", \

# exp_name="fabric_20210915_091526"
# resFolder="../results/".exp_name
set output outFileFabric
# set datafile separator ','
# set xrange [-5: 5]
# set yrange [-5: 5]
# set size ratio -1
# set grid

q0 = 3
q1 = 6
n_obst = 1
list=system("ls -1B ".resFolderFabric."/obst_*")

plot inFileFabric using q0:q1 with lines lw 2 notitle, \
  for [file in list] file w lines lc rgb "black" lw 10 notitle, \
  goalFileFabric using 1:2 with points title "goal", \
  initStateFileFabric using 1:2 with points lc rgb "green" title "initial State", \

print "Done creating trajectory plots"
print outFileMpc
print outFileFabric
print "End file names"
