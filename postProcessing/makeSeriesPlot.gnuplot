set term postscript eps color
seriesFolder=ARG1
print seriesFolder
inFile=seriesFolder."results.csv"
print inFile

outFileBox=seriesFolder."/results.eps"

set output outFileBox
set datafile separator ','

set style fill solid 0.0 border -1
set style boxplot nooutliers
# set style boxplot outliers pointtype -1
set style data boxplot
# set xtics ("-{/Symbol p}" 0, "-4{/Symbol p}/5" 1, "-3{/Symbol p}/5" 2, "-2{/Symbol p}/5" 3, "-1{/Symbol p}/5" 4, "0" 5, "1{/Symbol p}/5" 6, "2{/Symbol p}/5" 7, "3{/Symbol p}/5" 8, "4{/Symbol p}/5" 9, "{/Symbol p}" 10) scale 0.0
# set xtics font ", 10"
set yrange [0:6e0]
set boxwidth  0.5
set pointsize 0.5
set grid

unset key
set border 3
set xtics ("SolverTime" 1, "Time2Goal" 2, "PathLength" 3, "SelfClerance" 4, "Clearance" 5) scale 1.0
set xtics nomirror
set ytics nomirror

col=2
set title "SolverTimes"
plot inFile using (1):1, \
  inFile using (2):2, \
  inFile using (3):3, \
  inFile using (4):4, \
  inFile using (5):5

print "Done creating solver time plot"
print outFileBox
print "End file name"
