import gc,micropython
import os
#micropython.heap_lock()

gc.collect()
print(gc.mem_free())
micropython.qstr_info()
print(os.listdir('/projects'))
raise SystemExit(-1)