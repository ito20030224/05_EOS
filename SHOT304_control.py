import time
import re
import pyvisa


class SHOTGSControl:
    """
    SHOT-302GS / SHOT-304GS control via GPIB.

    Assumptions:
    - Controller memory switch 15 (INTERFACE) is set to GP-IB.
    - Controller memory switch 63 (COMM/ACK) is set to MAIN.
    - For easiest use, memory switch 17 (DELIMIT) is set to EOI.
    """

    _DELIM_MAP = {
        "CR": "\r",
        "LF": "\n",
        "CRLF": "\r\n",
        "EOI": "",
    }

    def __init__(
        self,
        resource_name: str = "GPIB0::8::INSTR",
        *,
        timeout_ms: int = 2000,
        delimiter: str = "EOI",
    ):
        self.resource_name = resource_name
        self.timeout_ms = int(timeout_ms)
        self.delimiter = delimiter.upper()

        if self.delimiter not in self._DELIM_MAP:
            raise ValueError("delimiter must be one of: CR, LF, CRLF, EOI")

        self.rm = None
        self.inst = None

    # ---------------------------
    # connection helpers
    # ---------------------------
    @staticmethod
    def list_devices():
        rm = pyvisa.ResourceManager()
        try:
            resources = list(rm.list_resources())
            gpib_resources = [r for r in resources if "GPIB" in r.upper()]
            print(gpib_resources)
            if not gpib_resources:
                print("エラー: GPIB デバイスが見つかりません。接続や VISA の設定を確認してください。")
            return gpib_resources
        finally:
            try:
                rm.close()
            except Exception:
                pass

    def connect(self) -> bool:
        try:
            self.rm = pyvisa.ResourceManager()
            self.inst = self.rm.open_resource(self.resource_name)
            self.inst.timeout = self.timeout_ms

            # GPIB + EOI のときは終端文字を付けず、EOI で終端する。
            if self.delimiter == "EOI":
                self.inst.write_termination = ""
                self.inst.read_termination = None
            else:
                term = self._DELIM_MAP[self.delimiter]
                self.inst.write_termination = term
                self.inst.read_termination = term

            try:
                self.inst.clear()
            except Exception:
                pass

            return True
        except Exception as e:
            print(f"GPIB 接続エラー: {e}")
            self.disconnect()
            return False

    def disconnect(self):
        try:
            if self.inst is not None:
                self.inst.close()
        except Exception:
            pass
        finally:
            self.inst = None

        try:
            if self.rm is not None:
                self.rm.close()
        except Exception:
            pass
        finally:
            self.rm = None

    # ---------------------------
    # low-level helpers
    # ---------------------------
    def _ensure_open(self):
        if self.inst is None:
            raise RuntimeError("GPIB 接続されていません。先に connect() を呼んでください。")

    def _write(self, cmd: str):
        self._ensure_open()
        full = cmd + self._DELIM_MAP[self.delimiter]
        if self.delimiter == "EOI":
            self.inst.write_raw(full.encode("ascii"))
        else:
            self.inst.write(full)

    def _read(self) -> str:
        self._ensure_open()
        if self.delimiter == "EOI":
            data = self.inst.read_raw()
            return data.decode(errors="ignore").strip()
        return self.inst.read().strip()

    def _query(self, cmd: str) -> str:
        self._write(cmd)
        return self._read()

    def _expect_ok(self, cmd: str):
        resp = self._query(cmd)
        if resp != "OK":
            raise RuntimeError(f"コマンド {cmd!r} が OK で受理されませんでした。返答: {resp!r}")

    # ---------------------------
    # status / query
    # ---------------------------
    def is_finished(self, *, timeout_s: float = 60.0, poll_s: float = 0.1) -> bool:
        """
        Poll '!:' until controller returns 'R' (READY).
        """
        t0 = time.time()
        while True:
            resp = self._query("!:")
            if resp == "R":
                return True
            if resp == "B":
                if (time.time() - t0) >= timeout_s:
                    return False
                time.sleep(poll_s)
                continue

            # 想定外の返答でも少し待って再試行
            if (time.time() - t0) >= timeout_s:
                return False
            time.sleep(poll_s)

    def version_confirmation(self) -> str:
        """
        Read ROM version with '?:V'.
        """
        resp = self._query("?:V")
        print(f"ROM Version: {resp}")
        return resp

    def query_positions_raw(self, *, timeout_s: float = 5.0) -> str:
        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy; cannot query positions.")
        return self._query("Q:")

    def get_positions(self, *, timeout_s: float = 5.0) -> list[int]:
        resp = self.query_positions_raw(timeout_s=timeout_s)
        parts = [p.strip() for p in re.split(r"[,，]", resp) if p.strip()]

        pos = []
        for s in parts:
            t = s.replace(" ", "")
            if re.fullmatch(r"[+-]?\d+", t):
                pos.append(int(t))
            else:
                break
        return pos

    def get_position(self, axis: int = 1, *, timeout_s: float = 5.0) -> int:
        pos = self.get_positions(timeout_s=timeout_s)
        if axis < 1 or axis > len(pos):
            raise IndexError(f"axis={axis} out of range (available={len(pos)})")
        return pos[axis - 1]

    # ---------------------------
    # settings / motion
    # ---------------------------
    def set_velocity(self, axis: int = 1, *, vS: int, vF: int, vR: int, timeout_s: float = 5.0):
        """
        D command:
          D:1S100F1000R50
        """
        axis_str = str(int(axis))
        cmd = f"D:{axis_str}S{int(vS)}F{int(vF)}R{int(vR)}"

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy before D command.")

        self._expect_ok(cmd)

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy after D command.")

    def go_to(self, axis: int, val: int, pol: str = "+", *, timeout_s: float = 120.0):
        """
        Relative move and wait until completion:
          M:1+P1000
          G:
        """
        axis_str = str(int(axis))
        val_str = str(int(val))
        pol_str = "+" if pol != "-" else "-"

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy before move.")

        self._expect_ok(f"M:{axis_str}{pol_str}P{val_str}")

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy after M command.")

        self._expect_ok("G:")

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Motion did not complete within timeout.")

    def go_absolute(self, axis: int, val: int, *, timeout_s: float = 120.0):
        """
        Absolute move and wait until completion:
          A:1+P1000
          G:
        """
        axis_str = str(int(axis))
        sign = "+" if int(val) >= 0 else "-"
        val_abs = abs(int(val))

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy before absolute move.")

        self._expect_ok(f"A:{axis_str}{sign}P{val_abs}")

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Controller stayed Busy after A command.")

        self._expect_ok("G:")

        if not self.is_finished(timeout_s=timeout_s):
            raise TimeoutError("Absolute motion did not complete within timeout.")


if __name__ == "__main__":
    ctrl = SHOTGSControl(
        resource_name="GPIB0::8::INSTR",  # 例: GPIB ボード 0, アドレス 8
        timeout_ms=2000,
        delimiter="EOI",                  # SHOT 側の DELIMIT を EOI にした場合
    )

    SHOTGSControl.list_devices()
    if ctrl.connect():
        try:
            ctrl.version_confirmation()
            ctrl.set_velocity(axis=1, vS=1000, vF=1000, vR=10)
            ctrl.go_to(axis=1, val=1000, pol="+")
            print("axis1 position =", ctrl.get_position(axis=1))
        finally:
            ctrl.disconnect()
