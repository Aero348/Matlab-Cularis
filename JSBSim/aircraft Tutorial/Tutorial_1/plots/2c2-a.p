# Gnuplot script file for plotting data in file "2c2-a.crt"

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
   #set output "aircraft/Tutorial_1/results/2c2-a.png"

   # This is how to output the plot in PDF format. (Not available on Mac)
   #set terminal pdfcairo color size 8.5,11
   #set output "aircraft/Tutorial_1/results/2c2-a.pdf"

set multiplot title "2c2-a Flap/Slat Change Dynamics - Retraction"
set xlabel "Time, sec"
set size 1,0.24

set origin 0.0,0.7
set ylabel "KCAS, knots"
plot  "aircraft/Tutorial_1/results/2c2-a.csv" using 1:60 title 'JSBSim' with lines

set origin 0.0,0.45
set ylabel "Press Alt, ft"
plot  "aircraft/Tutorial_1/results/2c2-a.csv" using 1:36 title 'JSBSim' with lines

set origin 0.0,0.22
set ylabel "Pitch, deg"
plot  "aircraft/Tutorial_1/results/2c2-a.csv" using 1:39 title 'JSBSim' with lines

set origin 0.0,0.0
set ylabel "Flap Angle, deg"
plot  "aircraft/Tutorial_1/results/2c2-a.csv" using 1:10 title 'JSBSim' with lines
	
unset multiplot                         # exit multiplot mode