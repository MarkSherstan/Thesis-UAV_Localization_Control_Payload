import sys
sys.path.append('./')
from main import main
import yappi

# Get yappi to do its thing
yappi.set_clock_type("cpu")
yappi.start(builtins=True)
main()
yappi.stop()

# Retrieve thread stats by their Yappi thread id 
threads = yappi.get_thread_stats()
ii = 0
for thread in threads:
    temp = yappi.get_func_stats(ctx_id=thread.id)
    temp.save('profiling/main/' + thread.name + str(ii), type='pstat')
    ii += 1

# Display message
print('Main profiling data saved')
