set term postscript eps color
seriesFolder=ARG1
planner1=ARG2
planner2=ARG3
inFile=seriesFolder."results.csv"

outFileBox=seriesFolder."/results.eps"

set output outFileBox
set datafile separator ','

set style fill solid 0.0 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
set yrange [0:3e0]
set boxwidth  0.5
set pointsize 0.5
set grid
yLabel = sprintf("%s / %s", planner2, planner1)
set ylabel yLabel font ",25"

unset key
set border 3
set xtics () scale 1.0 font ",18"
N = (9 < ARGC)?9:ARGC
array ARGV[N-3]
do for [i=4:N] {
  eval sprintf("ARGV[%d] = ARG%d", i-3, i);
}

do for [i=1:ARGC-3] {
  set xtics add (ARGV[i] i)
}
set xtics nomirror
set ytics nomirror font ',25'

plot for [i=1:ARGC-3] inFile using (i):i lw 2

