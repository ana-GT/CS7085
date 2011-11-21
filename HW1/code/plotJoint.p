#!usr/local/bin/gnuplot -persist
set autoscale
unset label
set title "Joints vs time"
set xlabel "time (sec)"
set ylabel "Joints (rad)"
set xtic 0.5
set ytic 0.25
set xrange [0:10]
set yrange [-3:3]
set grid

plot "Jointsc.dat" using 1:2 title "Joint 1" with lines lc 1 lw 2, "Jointsc.dat" using 1:3 title "Joint 2" with lines lc 2 lw 2, "Jointsc.dat" using 1:4 title "Joint 3" with lines lc 3 lw 2
