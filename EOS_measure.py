import SHOT304_control
import USB6210_control
import threading
import time
import numpy as np

def measure_one_round_trip(ctrl, task, axis=1, travel=1000):
    done_event = threading.Event()
    err_box = []
    records = [] #(時間、データ)
    startpos = ctrl.get_position(axis=axis)

    def stage_worker():
        try:
            ctrl.go_to(axis=axis, val=travel, pol="+")
            ctrl.go_to(axis=axis, val=travel, pol="-")
            ctrl.go_absolute(axis=axis, val=startpos)
        except Exception as e:
            err_box.append(e)
        finally:
            done_event.set()

    th = threading.Thread(target=stage_worker, daemon=True)

    task.start_measure()
    t0 = time.perf_counter()

    try:
        th.start()
        while not done_event.is_set():
            data = task.read_chunk()
            t = time.perf_counter() - t0
            records.append((t, data))
        th.join()  
        if err_box:
            raise err_box[0]

    finally:
        task.stop_measure()

    if records:
        all_data = np.concatenate([x[1] for x in records])
    else:
        all_data = np.array([], dtype=np.float64)

    return all_data, records

if __name__ == "__main__":
    ctrl = SHOT304_control.SHOTGSControl(resource_name="GPIB0::8::INSTR", timeout_ms=2000, delimiter="EOI")
    task = USB6210_control.USB6210control(channel=b"Dev1/ai0", rate=250000.0)
    results = []

    ctrl.list_devices()
    if ctrl.connect():
        try:
            ctrl.version_confirmation()
            ctrl.set_velocity(axis=1, vS=1000, vF=1000, vR=10)

            for i in range(3):
                print(f"Run {i+1}")
                data, records = measure_one_round_trip(ctrl, task, axis=1, travel=1000)
                results.append((data, records))
                print("samples =", len(data))

            for i, (data, records) in enumerate(results):
                print(f"Run {i+1} data")
                print(data)

        finally:
            task.ClearTask()
            ctrl.disconnect()