# Gnuplot script file for plotting data in file "2d2-a.crt"

set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically

# If you have graphical capabilities, you can plot on your screen
# if none of the other terminals is specificed.

   # This is how to output the plot in PostScript format
   #set terminal postscript portrait enhanced color lw 1 "Helvetica" 14 size 8.5,11
   
   # This is how to output the plot in PNG format
   #set terminal png size 1280,960
   #set output "aircraft/Tutorial_1/results/2d2-a.png"

   # This is how to output the plot in PDF format. (Not available on Mac)
   #set terminal pdfcairo color size 8.5,11
   #set output "aircraft/Tutorial_1/results/2d2-a.pdf"

set multiplot title "2d2-a Roll Response - Cruise"
set xlabel "Time, sec"
set size 1,0.24

set origin 0.0,0.7
set ylabel "KCAS, knots"
plot "aircraft/Tutorial_1/results/2d2-a.csv" using 1:60 title 'JSBSim' with lines
 
set origin 0.0,0.45
set ylabel "Ail Defl, deg"
plot "aircraft/Tutorial_1/results/2d2-a.csv" using 1:6 title 'JSBSim' with lines

set origin 0.0,0.22
set ylabel "Roll, deg"
plot "aircraft/Tutorial_1/results/2d2-a.csv" using 1:38 title 'JSBSim' with lines

set origin 0.0,0.0
set ylabel "Roll Rate, dps"
plot "aircraft/Tutorial_1/results/2d2-a.csv" using 1:11 title 'JSBSim' with lines
     

unset multiplot                         # exit multiplot mode
