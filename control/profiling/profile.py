import yappi
import sys
sys.path.append('../')
from main import main

# Get yappi to do its thing
yappi.set_clock_type("wall") # cpu
yappi.start(builtins=True)
main()
yappi.stop()

# Retrieve thread stats by their Yappi thread id 
threads = yappi.get_thread_stats()
ii = 0
for thread in threads:
    print("Function stats for (%s) (%d)" % (thread.name, thread.id))
    temp = yappi.get_func_stats(ctx_id=thread.id)
    temp.print_all()
    temp.save('profiling/' + thread.name + str(ii), type='pstat') # qcachegrind
    ii += 1
    print('\n\n')
