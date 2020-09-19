# To run:
#   MacOS Catalina: zsh convert.sh 
#   Linux 20.04 LTS: bash convert.sh
#
# Analyze with GUI:
#   MacOS Catalina: qcachegrind MainThread.calltree
#   Linux 20.04 LTS: kcachegrind MainThread.calltree

# Do work on main thread
gprof2dot -f pstats -n 3 -e 3 _MainThread0 | dot -Tpng -o MainThread.png 
pyprof2calltree -i _MainThread0 -o MainThread.calltree

# Do work on the other threads
for f in Thread*
do
    gprof2dot -f pstats -n 2 -e 2 "$f" | dot -Tpng -o "$f.png"
    pyprof2calltree -i "$f" -o "$f.calltree"
done
