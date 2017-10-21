import ctypes
import time
import array

class GemException(Exception):
    def __init__(self, msg, error):
        super(Exception,self).__init__('%s [GemError] %d, [Desc] %s' % (msg,error,self.gemErrorNumToText(error)))

    error_dic = {
        0x00: 'GEM SUCCESS',
        0x01: 'GEM FAILED WIN32',
        0x02: 'GEM FAILED BLUETOOTH',
        0x03: 'GEM NOT FOUND',
        0x04: 'GEM ALREADY CONNECTED',
        0x05: 'GEM NOT CONNECTED',
        0x06: 'GEM INSUFFICIENT BUFFER',
        0x07: 'GEM EMPTY',
        0x08: 'GEM INVALID PARAMETER',
        0x0A: 'GEM BAD CONFIG',
        0x0B: 'GEM ALREADY INITIALIZED',
        0x0C: 'GEM NOT INITIALIZED',
        0xFF: 'GEM ERROR UNKNOWN',
    }

    def gemErrorNumToText(self, error):
        if error in self.error_dic:
            return self.error_dic[error]
        else:
            return 'ERROR DOES NOT EXISTS'

class GemDescription(ctypes.Structure):
    _fields_ = [('address',ctypes.c_ubyte * 6),('deviceName',ctypes.c_ubyte * 32)]

class GemInfo(ctypes.Structure):
    _fields_ = [('name', ctypes.c_char * 32), ('fGemInfoirmwareVer', ctypes.c_char * 32), ('hardwareVer', ctypes.c_char * 32), ('batteryLevel',ctypes.c_char)]


class Gem(object):
    # typedef void(*gemOnStateChanged)(GemState state);
    StatusUpdateCallback = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_uint)

    CombinedDataCallback = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float))



    def __init__(self, gemDesc, dll):
        self.gemDesc = gemDesc
        self.dll = dll

    def setCallbacks(self, statusUpdateCallback, combinedDataCallback):
        self.statusUpdateCallback = self.StatusUpdateCallback(statusUpdateCallback)
        res = self.dll.gemSetOnStateChanged(self.gemDesc.address, self.statusUpdateCallback)
        if (res != 0):
            raise GemException('failed to set status callback for [gem address] %s' % self.gemDesc.address, res)

        self.combinedDataCallback = self.CombinedDataCallback(combinedDataCallback)
        res = self.dll.gemSetOnCombinedData(self.gemDesc.address, self.combinedDataCallback)
        if (res != 0):
            raise GemException('failed to set combined data callback for [gem address] %s' % self.gemDesc.address, res)

    def connect(self):
        # connect and wait until the connection is finished
        res = self.dll.gemConnect(self.gemDesc.address)
        if (res != 0):
            raise GemException('failed to issue gem connection for [gem address] %s' % self.gemDesc.address, res)

class GemManager:

    dll_path = './/lib//x86-64//libgemsdk.so'
	
    def __init__(self):
        self.Gems = {}
        #import os
        #print os.path.exists(self.dll_path)
        self.dll = ctypes.CDLL(self.dll_path)
        #self.dll = ctypes.windll.LoadLibrary(self.dll_path)
        # initialize GemManager
        res = self.initialize()
        if(res != 0):
            raise GemException('failed to initialize manager',res)

        # read gems
        numOfGems = self.dll.gemGetDescriptionListCount()

        # if no gems are configured set an empty dictionary
        if(numOfGems == 0):
            return

        # get description list
        gemDescArr = (GemDescription * numOfGems)()
        res = self.dll.gemGetDescriptionList(gemDescArr,numOfGems)
        if(res != 0):
            raise GemException('failed to get gem description list',res)

        for g in gemDescArr:
            deviceAddrStr = ''.join('{:02x}'.format(x) for x in g.address)
            print 'adding', deviceAddrStr, ''.join([chr(c) for c in g.deviceName])

            self.Gems[deviceAddrStr] = Gem(g, self.dll)


    def __del__(self):
        res = self.terminate()
        if (res != 0):
            raise GemException(res)

    def initialize(self):
        return self.dll.gemInitialize()

    def terminate(self):
        return self.dll.gemTerminate()