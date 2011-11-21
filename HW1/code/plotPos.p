#!usr/local/bin/gnuplot -persist
set autoscale
unset label
set title "End Effector(e) XY Position"
set xlabel "X axis"
set ylabel "Y axis"
set xtic 1.0
set ytic 1.0
set xrange [0:10]
set yrange [0:10]
set grid

plot "EndEffectorc.dat" using 1:2 title "e Position in XY" with lines lc 3 lw 4
