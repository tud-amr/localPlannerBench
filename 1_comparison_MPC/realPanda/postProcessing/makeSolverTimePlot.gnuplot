set term postscript eps color
resFolderMpc=ARG1
mpc_name="mpc"
inFileMpc=resFolderMpc."/res.csv"

resFolderFabric=ARG2
fabric_name="fabric"
inFileFabric=resFolderFabric."/res.csv"
outFileBox="solvertimes_mpcFabric.eps"

intervals=ARG3

set output outFileBox
set datafile separator ','

set style fill solid 0.25 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
# set xtics ("-{/Symbol p}" 0, "-4{/Symbol p}/5" 1, "-3{/Symbol p}/5" 2, "-2{/Symbol p}/5" 3, "-1{/Symbol p}/5" 4, "0" 5, "1{/Symbol p}/5" 6, "2{/Symbol p}/5" 7, "3{/Symbol p}/5" 8, "4{/Symbol p}/5" 9, "{/Symbol p}" 10) scale 0.0
# set xtics font ", 10"
# set yrange [-5e-5:1e-3]
set boxwidth  0.5
set pointsize 0.5

unset key
set border 3
set xtics ("MPC" 1, "Fabric" 2) scale 1.0
set xtics nomirror
set ytics nomirror

col=2
set title "SolverTimes"
plot inFileMpc every intervals::1 using (1):col, \
  inFileFabric every intervals::1 using (2):col
