set term png
set output 'result.png'
set xlabel 'time(sec)'
set ylabel '-'
plot 'result.xy' u 1:2 w l t 'dist(m)',\
     'result.xy' u 1:3 w l t 'theta(rad)',\
     'result.xy' u 1:4 w l t 'vL(ratio)',\
     'result.xy' u 1:5 w l t 'vR(ratio)'
