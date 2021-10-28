set term postscript eps color size 2.0, 3
seriesFolder=ARG1
inFile=seriesFolder."success.csv"
print inFile

outFileHist=seriesFolder."/success.eps"

set output outFileHist
set datafile separator ','

set style fill solid 1.0 border -1
set style data histogram
set style histogram rowstacked
set boxwidth 0.5
set xtics ("MPC" 0, "Fabric" 1) scale 1.0
set xrange [-0.5:1.5]
set yrange [0:119]
set xtics nomirror
set ytics nomirror
set ylabel '#Cases'

plot inFile using 1 t "Success" lc rgbcolor 'green', '' using 2 t 'Collision' lc rgb 'red', '' using 3 t 'Goal not reached' lc rgb 'blue'

print "End file name"
