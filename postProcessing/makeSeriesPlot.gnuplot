set term postscript eps color
seriesFolder=ARG1
planner1=ARG2
planner2=ARG3
inFile=seriesFolder."results.csv"

outFileBox=seriesFolder."/results.eps"

set output outFileBox
set datafile separator ','
set bmargin at screen 0.40
set lmargin at screen 0.15
set rmargin at screen 0.9

set style fill solid 0.0 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
set yrange [0.8:2.00]
set boxwidth  0.5
set pointsize 0.5
set grid
yLabel = sprintf("%s / %s", planner2, planner1)
set ylabel yLabel font ",25" offset -2

unset key
set border 3
set xtics () scale 1.0 font ",30" rotate by -45
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

plot for [i=1:ARGC-3] inFile using (i):i lw 2, \
  1 with lines dt 3 lw 6 lt rgb "red" notitle

