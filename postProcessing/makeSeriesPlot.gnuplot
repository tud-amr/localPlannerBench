set term postscript eps color size 2.0, 7.0
seriesFolder=ARG1
planner1=ARG2
planner2=ARG3
inFile=seriesFolder."results.csv"

outFileBox=seriesFolder."/results.eps"

set output outFileBox
set datafile separator ','
#set bmargin at screen 0.40
#set lmargin at screen 0.30
#set rmargin at screen 0.9
set bmargin at screen 0.26

set style fill solid 0.5 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
set log y2 2
# set y2range [0.9:2.00]
# set y2range [0.01:6.0]
unset ytics
set y2tics nomirror font ',35' rotate by 90 out offset 0.5,-1.0
# set y2tics (0.8, 1.0, 1.25, 1.5, 2.0)
set y2tics (0.02, 0.1, 0.2, 0.5, 1, 2, 5)
set boxwidth  0.5
set pointsize 0.5
set grid y2tics
set border 9
yLabel = sprintf("%s / %s on logscale", planner2, planner1)
set y2label yLabel font ",35" rotate by 90

unset key
set xtics () scale 1.0 font ",35" rotate by 90 out offset -0.5, -14.0
N = (9 < ARGC)?9:ARGC
array ARGV[N-3]
do for [i=4:N] {
  eval sprintf("ARGV[%d] = ARG%d", i-3, i);
}

do for [i=1:ARGC-3] {
  set xtics add (ARGV[i] i)
}
set xtics nomirror

plot for [i=1:ARGC-3] inFile using (i):i lw 2 axes x1y2, \
  1 with lines dt 3 lw 6 lt rgb "red" notitle axes x1y2

