set term postscript eps color size 2.0, 7.0 font "RomanSerif.ttf" 14
seriesFolder=ARG1
inFile=seriesFolder."successTable.csv"
planner_types(file_name)=system("awk NR!=1'{print $1}' ".file_name)
planner_names=planner_types(inFile)
nb_planner=words(planner_names)
nb_cases=50
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
set xtics (word(planner_names, 1) 0, word(planner_names, 2) 1, word(planner_names, 3) 2) scale 1.0 font ',35' rotate by 90 offset 0, -14.0 nomirror
unset ytics
set xrange [-0.5:nb_planner - 0.5]
set y2range [0:1.6 * nb_cases]
set y2tics nomirror font ',35'
set y2tics 0,nb_cases/5,nb_cases rotate by 90 offset 0.5,-2.0
set y2label '#Cases' font ',35'

set key autotitle columnhead
unset key

#set key font ',20'
#set key outside horiz
#set key top
x = -0.2
dx = 0.1 * nb_planner
x2 = x + 2 * dx
x3 = x2 + 2 * dx
y = 1.05 * nb_cases
dy = 0.05 * nb_cases
set label 2 'Success' at x,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x,y to x+dx,y+dy fc rgb 'white' fs transparent solid 0.5
set label 3 'Collision' at x2,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x2,y to x2+dx,y+dy fc rgb 'black' fs transparent solid 0.5
set label 4 'Not Reached' at x3,y+dy rotate by 90 offset 6*dx,1 font ',35'
set object rectangle from x3,y to x3+dx,y+dy fc rgb 'gray' fs transparent solid 0.5
plot inFile using 4 notitle lc rgbcolor 'white' axes x1y2, \
   '' using 3 notitle lc rgb 'black' axes x1y2, \
   '' using 2 notitle lc rgb 'gray' axes x1y2
