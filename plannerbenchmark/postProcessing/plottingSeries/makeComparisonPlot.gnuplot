set term postscript eps color size 2, 7
seriesFolder=ARG1
planner1=ARG2
planner2=ARG3
N=ARG4
print N
inFile=seriesFolder."resultsTable_comparison.csv"
outFileBox=seriesFolder."/results_comparison.eps"
set output outFileBox

set datafile separator ' '
#set bmargin at screen 0.40
#set lmargin at screen 0.30
#set rmargin at screen 0.9
set bmargin at screen 0.26

set style fill solid 0.5 border -1
set style boxplot nooutliers
set log y2 2
# set style boxplot outliers pointtype -1
set style data boxplot
#set y2range [0.0:5.00]
unset ytics
set y2tics nomirror font ',35' rotate by 90 out offset 0.5,-1.0
set y2tics (0.02, 0.1, 0.2, 0.5, 1, 2, 5)
yLabel = sprintf("%s / %s on logscale", planner2, planner1)
set y2label yLabel font ",35" rotate by 90

firstrow = system('head -1 '.inFile)

unset key
set xtics () scale 1.0 font ",35" rotate by 90 out offset -0.5, -14.0
set xtics nomirror
do for [i=2:N] {
  metricName = word(firstrow, i)
  set xtics add (metricName i)
}
plot for [i=2:N] inFile using (i):i lw 2 axes x1y2, \
  1 with lines dt 3 lw 6 lt rgb "red" notitle axes x1y2
