#misc
import os
import serial
import struct
import time
#Threads
from thread import start_new_thread
from threading import Lock
#Math and plots
import numpy as np
from matplotlib import mlab
import matplotlib.pyplot as plt
#Our includes
from gemsdk import *
from Quaternion2 import Quaternion2


def onCombinedData(quaternions,acceleration):
 #   tic = time.time()
	print quaternions,acceleration
"""        
    gems_data_lock[0].acquire()
    try:
        gems_elev_data[0][gems_elev_data_counter[0]] = get_elevation(Quaternion2.from_xyzw(quaternions[:4]))
        gems_elev_data_time[0][gems_elev_data_counter[0]] = time.time()
        if gems_last_data[0] != gems_elev_data[0][gems_elev_data_counter[0]]:
            gems_last_data[0] = gems_elev_data[0][gems_elev_data_counter[0]]
            gems_elev_data_counter[0] += 1  
        else:
            print "DEBUG: Duplicate data, ignoring..."
    finally:
        gems_data_lock[0].release()

    gem1_data_time = time.time()
    print >> gem_1_fd,gem1_data_time ,quaternions[:4],gems_elev_data[0][gems_elev_data_counter[0]-1]
    gems_last_data_timestamps[0] = gem1_data_time 
    gem_1_fd.flush()
 #   print "1: %.5f"%((time.time() - tic)   )
return 0
"""


gemMgr = GemManager()

gems[0] = gemMgr.Gems.values()[0]
res = gems[0].setCallbacks(onStatusUpdate,onCombinedData)
gems[0].connect()