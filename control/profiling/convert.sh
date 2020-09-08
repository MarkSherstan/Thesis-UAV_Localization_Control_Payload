#!/bin/zsh
# To run (MacOS Catalina): zsh convert.sh 

echo Create figures
gprof2dot -f pstats -n 3 -e 3 _MainThread0 | dot -Tpng -o MainThread.png 
gprof2dot -f pstats -n 2 -e 2 Thread1 | dot -Tpng -o Thread1.png
gprof2dot -f pstats -n 2 -e 2 Thread2 | dot -Tpng -o Thread2.png
gprof2dot -f pstats -n 2 -e 2 Thread3 | dot -Tpng -o Thread3.png
gprof2dot -f pstats -n 2 -e 2 Thread4 | dot -Tpng -o Thread4.png

echo Export to call tree
pyprof2calltree -i _MainThread0 -o MainThread.calltree
pyprof2calltree -i Thread1 -o Thread1.calltree
pyprof2calltree -i Thread2 -o Thread2.calltree
pyprof2calltree -i Thread3 -o Thread3.calltree
pyprof2calltree -i Thread4 -o Thread4.calltree

# qcachegrind MainThread.calltree