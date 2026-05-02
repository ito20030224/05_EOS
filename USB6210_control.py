from PyDAQmx import *
from ctypes import byref
import numpy as np

class USB6210control(Task):
    def __init__(self, channel=b"Dev1/ai0", rate=10000.0):
        super().__init__()
        self.channel = channel
        self.rate = rate
        self.chunk_sec = 0.05
        self.chunk = max(1, int(round(self.rate * self.chunk_sec)))
        self.buffer_size = self.chunk * 10
        self.read = int32()
        self.data = np.zeros((self.chunk,), dtype=np.float64)
        self.started = False

        self.CreateAIVoltageChan(self.channel, b"", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        self.CfgSampClkTiming(b"", self.rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, self.buffer_size)

    def start_measure(self):
        if not self.started:
            self.StartTask()
            self.started = True
    
    def read_chunk(self, timeout=10.0):
        if not self.started:
            raise RuntimeError("Task has not been started. Call start_measure() first.")
        self.read = int32()
        self.ReadAnalogF64(self.chunk, timeout, DAQmx_Val_GroupByScanNumber, self.data, self.chunk, byref(self.read), None)
        return self.data[:self.read.value].copy()

    def stop_measure(self):
        if self.started:
            self.StopTask()
            self.started = False

if __name__ == "__main__":
    task = USB6210control()
    try:
        task.start_measure()

        for i in range(5):
            data = task.read_chunk()
            print(f"chunk {i}: read = {len(data)}")
            print(data)

        task.stop_measure()
    finally:
        task.ClearTask()
    