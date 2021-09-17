set term postscript eps color size 5, 5
# exp_name=system("ls -t ../results | head -n 1")
# resFolder="../results/".exp_name
resFolderMpc=ARG1
timeStampMpc=ARG2
mpc_name="mpc"
inFileMpc=resFolderMpc."/res.csv"
goalFileMpc=resFolderMpc."/goal.csv"
initStateFileMpc=resFolderMpc."/initState.csv"
fkLastFileMpc=resFolderMpc."/fk_last.csv"
fkStartFileMpc=resFolderMpc."/fk_first.csv"
outFileMpc="trajectories_mpc_".timeStampMpc.".eps"

resFolderFabric=ARG3
timeStampFabric=ARG4
inFileFabric=resFolderFabric."/res.csv"
goalFileFabric=resFolderFabric."/goal.csv"
initStateFileFabric=resFolderFabric."/initState.csv"
fkLastFileFabric=resFolderFabric."/fk_last.csv"
fkStartFileFabric=resFolderFabric."/fk_first.csv"
outFileFabric="trajectories_fabric_".timeStampFabric.".eps"
set output outFileMpc
set datafile separator ','
set xrange [-5: 5]
set yrange [-5: 5]
set size ratio -1
set grid

ee_x = ARG5 + 1
ee_y = ARG6 + 1
list=system("ls -1B ".resFolderMpc."/obst_*")

plot inFileMpc using ee_x:ee_y with lines lw 2 notitle, \
  fkStartFileMpc using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "black" lw 2 title "start config", \
  fkLastFileMpc using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "green" lw 2 title "final config", \
  for [file in list] file w lines lc rgb "black" lw 10 notitle, \
  goalFileMpc using 1:2 with points pointsize 3 pointtype 6 lc rgb "green" title "goal", \

# exp_name="fabric_20210915_091526"
# resFolder="../results/".exp_name
set output outFileFabric
# set datafile separator ','
# set xrange [-5: 5]
# set yrange [-5: 5]
# set size ratio -1
# set grid

# x_col = 19
# y_col = 20
list=system("ls -1B ".resFolderFabric."/obst_*")

plot inFileFabric using ee_x:ee_y with lines lw 2 notitle, \
  fkStartFileFabric using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "black" lw 2 title "start config", \
  fkLastFileFabric using 1:2 with linespoints pointtype 7 pointsize 2 lc rgb "green" lw 2 title "final config", \
  for [file in list] file w lines lc rgb "black" lw 10 notitle, \
  goalFileFabric using 1:2 with points pointsize 3 pointtype 6 lc rgb "green" title "goal", \

