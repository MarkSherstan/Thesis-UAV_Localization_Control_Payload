from main import main
import yappi

# Get yappi to do its thing
yappi.set_clock_type("wall") # cpu
yappi.start(builtins=True)
main()
yappi.stop()

# Retrieve thread stats by their Yappi thread id 
threads = yappi.get_thread_stats()
ii = 0
for thread in threads:
    temp = yappi.get_func_stats(ctx_id=thread.id)
    temp.save('profiling/' + thread.name + str(ii), type='pstat') # qcachegrind
    ii += 1

# Display message
print('Profiling data saved')