import sys
sys.path.append('./')
from multiprocessing import Queue
from vision import Vision
import yappi

# Get yappi to do its thing
yappi.set_clock_type("wall") # cpu
yappi.start(builtins=True)

Q = Queue()
V = Vision()
V.run(Q)

yappi.stop()

# Retrieve thread stats by their Yappi thread id 
threads = yappi.get_thread_stats()
ii = 0
for thread in threads:
    temp = yappi.get_func_stats(ctx_id=thread.id)
    temp.save('profiling/vision/' + thread.name + str(ii), type='pstat')
    ii += 1

# Display message
print('Vision profiling data saved')
