set term postscript eps color size 2.0, 7.0 font "RomanSerif.ttf" 14
seriesFolder=ARG1
planner1Type=ARG2
planner2Type=ARG3
print planner1Type
print planner2Type
inFile=seriesFolder."successTable.csv"

outFileHist=seriesFolder."/success.eps"

set output outFileHist
set datafile separator ' '
set bmargin at screen 0.26
#set rmargin at screen 0.9
#set tmargin at screen 0.8

set style fill solid 0.5 border -1
set style data histogram
set style histogram rowstacked
set border 9
set boxwidth 0.5
planner1 = ""
planner2 = ""
if (planner1Type eq 'fabric') planner1 = 'Static Fabric'
if (planner1Type eq 'AcadosMpc') planner1 = 'Acados MPC'
if (planner1Type eq 'dynamicFabric') planner1 = 'Dyn. Fabric'
if (planner2Type eq 'fabric') planner2 = 'Static Fabric'
if (planner2Type eq 'AcadosMpc') planner2 = 'Acados MPC'
if (planner2Type eq 'dynamicFabric') planner2 = 'Dyn. Fabric'
if (planner1Type eq 'mpc') planner1 = 'MPC'
if (planner2Type eq 'mpc') planner2 = 'MPC'
set xtics (planner1 0, planner2 1) scale 1.0 font ',35' rotate by 90 offset 0, -14.0 nomirror
unset ytics
nbPlanner=2
nbCases=40
set xrange [-0.5:nbPlanner - 0.5]
set y2range [0:1.6 * nbCases]
set y2tics nomirror font ',35'
set y2tics 0,nbCases/5,nbCases rotate by 90 offset 0.5,-2.0
set y2label '#Cases' font ',35'

set key autotitle columnhead
unset key

#set key font ',20'
#set key outside horiz
#set key top
x = -0.2
dx = 0.1 * nbPlanner
x2 = x + 2 * dx
x3 = x2 + 2 * dx
y = 1.05 * nbCases
dy = 0.05 * nbCases
set label 2 'Success' at x,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x,y to x+dx,y+dy fc rgb 'white' fs transparent solid 0.5
set label 3 'Collision' at x2,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x2,y to x2+dx,y+dy fc rgb 'black' fs transparent solid 0.5
set label 4 'Not Reached' at x3,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x3,y to x3+dx,y+dy fc rgb 'gray' fs transparent solid 0.5
plot inFile using 4 notitle lc rgbcolor 'white' axes x1y2, \
   '' using 3 notitle lc rgb 'black' axes x1y2, \
   '' using 2 notitle lc rgb 'gray' axes x1y2
