set term postscript eps color
seriesFolder=ARG1
planner1=ARG2
planner2=ARG3
kpiName1=ARG4
kpiName2=ARG5
kpiName3=ARG6
kpiName4=ARG7
kpiName5=ARG8
kpiName6=ARG9
inFile=seriesFolder."results.csv"

outFileBox=seriesFolder."/results.eps"

set output outFileBox
set datafile separator ','

set style fill solid 0.0 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
set yrange [0:25e0]
set boxwidth  0.5
set pointsize 0.5
set grid
yLabel = sprintf("%s / %s", planner1, planner2)
set ylabel yLabel

unset key
set border 3
set xtics (kpiName1 1, kpiName2 2, kpiName3 3, kpiName4 4, kpiName5 5, kpiName6 6) scale 1.0
set xtics nomirror
set ytics nomirror

plot inFile using (1):1, \
  inFile using (2):2, \
  inFile using (3):3, \
  inFile using (4):4, \
  inFile using (5):5

