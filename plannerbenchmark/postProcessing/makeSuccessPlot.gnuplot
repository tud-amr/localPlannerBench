set term postscript eps color size 2.0, 7.0 font "RomanSerif.ttf" 14
seriesFolder=ARG1
planner1Type=ARG2
planner2Type=ARG3
inFile=seriesFolder."success.csv"

outFileHist=seriesFolder."/success.eps"

set output outFileHist
set datafile separator ','
set bmargin at screen 0.26
#set rmargin at screen 0.9
#set tmargin at screen 0.8

set style fill solid 0.5 border -1
set style data histogram
set style histogram rowstacked
set border 9
set boxwidth 0.5
if (planner1Type eq 'StaticFabric') planner1 = 'Static Fabric'
if (planner1Type eq 'DynamicFabric') planner1 = 'Dynamic Fabric'
if (planner2Type eq 'StaticFabric') planner2 = 'Static Fabric'
if (planner2Type eq 'DynamicFabric') planner2 = 'Dyn. Fabric'
if (planner1Type eq 'MPC') planner1 = 'MPC'
if (planner2Type eq 'MPC') planner2 = 'MPC'
set xtics (planner1 0, planner2 1) scale 1.0 font ',35' rotate by 90 offset 0, -14.0 nomirror
set xrange [-0.5:1.5]
unset ytics
nbCases = 50
set y2range [0:1.6 * nbCases]
set y2tics nomirror font ',35'
set y2tics 0,10,nbCases rotate by 90 offset 0.5,-2.0
set y2label '#Cases' font ',35'

#set key font ',20'
#set key outside horiz
#set key top
x = 0.2
x2 = 0.6
x3 = 1.0
y = 1.05 * nbCases
dx = 0.2
dy = 0.05 * nbCases
set label 2 'Success' at x,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x,y to x+dx,y+dy fc rgb 'green' fs transparent solid 0.5
set label 3 'Collision' at x2,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x2,y to x2+dx,y+dy fc rgb 'red' fs transparent solid 0.5
set label 4 'Not Reached' at x3,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x3,y to x3+dx,y+dy fc rgb 'blue' fs transparent solid 0.5
plot inFile using 1 notitle lc rgbcolor 'green' axes x1y2, \
   '' using 2 notitle lc rgb 'red' axes x1y2, \
   '' using 3 notitle lc rgb 'blue' axes x1y2
