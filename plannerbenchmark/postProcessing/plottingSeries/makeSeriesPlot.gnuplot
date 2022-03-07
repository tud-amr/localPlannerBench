set term postscript eps color size 1, 7
seriesFolder=ARG1
planner=ARG2
N=ARG3
inFile=seriesFolder."resultsTable_".planner.".csv"

set datafile separator ' '
#set bmargin at screen 0.40
#set lmargin at screen 0.30
#set rmargin at screen 0.9
set bmargin at screen 0.26

set style fill solid 0.5 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
# set log y2 2
# set y2range [0.9:2.00]
unset ytics
set y2tics nomirror font ',35' rotate by 90 out offset 0.5,-1.0

firstrow = system('head -1 '.inFile)

unset key
set xtics () scale 1.0 font ",35" rotate by 90 out offset -0.5, -14.0
set xtics nomirror
do for [i=2:N] {
  metricName = word(firstrow, i)
  set xtics add (metricName i)
  outFileBox=seriesFolder."/results_".planner."_".metricName.".eps"
  set output outFileBox
  plot inFile using (i):i lw 2 axes x1y2
}
# plot for [i=2:N] inFile using (i):i lw 2 axes x1y2
