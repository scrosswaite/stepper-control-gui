# app/controllers/modbus_controller.py
from dataclasses import dataclass
from PyQt5.QtCore import QThread, pyqtSignal
import time
import struct
from typing import List, Optional

from pymodbus.client import ModbusSerialClient
try:
    from pymodbus import FramerType
except ImportError:
    from pymodbus.framer import FramerType

# ---------- helpers (no pymodbus.payload) ----------
def regs_to_bytes(regs: List[int], *, byteorder="big", wordorder="big") -> bytes:
    if wordorder == "little":
        regs = regs[::-1]
    b = bytearray()
    for r in regs:
        b += int(r).to_bytes(2, byteorder=byteorder, signed=False)
    return bytes(b)

def regs_to_float32(regs: List[int], *, byteorder="big", wordorder="big") -> float:
    b = regs_to_bytes(regs, byteorder=byteorder, wordorder=wordorder)
    if len(b) != 4:
        raise ValueError(f"Need 4 bytes for float, got {len(b)} bytes from regs={regs!r}")
    fmt = ">" if byteorder == "big" else "<"
    return struct.unpack(f"{fmt}f", b)[0]

def decode_float_pairs(regs: List[int], *, byteorder="big", wordorder="big"):
    """Return (f1, f2_or_None) for 2 or 4 registers."""
    if len(regs) == 2:
        return regs_to_float32(regs, byteorder=byteorder, wordorder=wordorder), None
    if len(regs) >= 4:
        f1 = regs_to_float32(regs[0:2], byteorder=byteorder, wordorder=wordorder)
        f2 = regs_to_float32(regs[2:4], byteorder=byteorder, wordorder=wordorder)
        return f1, f2
    raise ValueError(f"Unsupported float register length: {len(regs)} (regs={regs!r})")

# ----------------------- config -----------------------
@dataclass
class ModbusConfig:
    port: str = "COM9"   # set to your  COM port
    method: str = "rtu" 
    unit_id: int = 1
    baudrate: int = 9600
    parity: str = "E"
    bytesize: int = 8
    stopbits: int = 1
    timeout: float = 1.2
    poll_ms: int = 1000  
    byteorder: str = "big"
    wordorder: str = "big"  
    density_float_addr: Optional[int] = None   
    density_u16_addr:   Optional[int] = None   
    density_scale_lo:   float = 0.0            
    density_scale_hi:   float = 2000.0         


# ----------------------- worker -----------------------
class ModbusWorker(QThread):
    """
    Emits:
      data -> {"vl": float, "vl_alt": Optional[float], "t": float, "t_alt": Optional[float], "ts": float}
      error -> str
      connection_changed -> bool
    """
    data = pyqtSignal(dict)
    error = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self, cfg: ModbusConfig):
        super().__init__()
        self.cfg = cfg
        self._client: Optional[ModbusSerialClient] = None
        self._running = True

    def stop(self):
        self._running = False
        try:
            if self._client:
                self._client.close()
        except Exception:
            pass

    def _build_client(self) -> ModbusSerialClient:
        framer = FramerType.RTU if self.cfg.method.lower() == "rtu" else FramerType.ASCII
        return ModbusSerialClient(
            port=self.cfg.port,
            framer=framer,
            baudrate=self.cfg.baudrate,
            parity=self.cfg.parity,
            bytesize=self.cfg.bytesize,
            stopbits=self.cfg.stopbits,
            timeout=self.cfg.timeout,
        )

    def _ensure_connected(self) -> bool:
        if self._client is None:
            self._client = self._build_client()
        if not getattr(self._client, "connected", False):
            return bool(self._client.connect())
        return True

    def _read_f32_pair(self, address: int):
        """Read float(s) from VP250 float map; device may return 2 or 4 regs."""
        rr = self._client.read_input_registers(address=address, count=4, device_id=self.cfg.unit_id)
        if rr.isError():
            raise IOError(str(rr))
        regs = getattr(rr, "registers", []) or []
        return decode_float_pairs(regs, byteorder=self.cfg.byteorder, wordorder=self.cfg.wordorder)

    def run(self):
        backoff = 1.0
        connected_emitted = False
        while self._running:
            try:
                if not self._ensure_connected():
                    raise ConnectionError(f"Unable to open serial port {self.cfg.port}")

                # VP250 float map: 0x1000 = viscosity (VL), 0x1010 = temperature (°C)
                vl1, vl2 = self._read_f32_pair(0x1000)
                t1,  t2  = self._read_f32_pair(0x1010)

                rho = None
                if self.cfg.density_float_addr is not None:
                    try:
                        rho1, _ = self._read_f32_pair(self.cfg.density_float_addr)
                        rho = rho1
                    except Exception:
                        rho = None
                elif self.cfg.density_u16_addr is not None:
                    try:
                        rr = self._client.read_input_registers(
                            address=self.cfg.density_u16_addr, count=1, device_id=self.cfg.unit_id
                        )
                        if not rr.isError() and getattr(rr, "registers", []):
                            raw = rr.registers[0]
                            lo, hi = self.cfg.density_scale_lo, self.cfg.density_scale_hi
                            rho = lo + (hi - lo) * (raw / 65535.0)
                    except Exception:
                        rho = None

                if not connected_emitted:
                    self.connection_changed.emit(True)
                    connected_emitted = True

                self.data.emit({
                    "vl": vl1, "vl_alt": vl2,
                    "t": t1,   "t_alt": t2,
                    "rho": rho,              
                    "ts": time.time()
                })
                time.sleep(max(0.25, self.cfg.poll_ms / 1000.0))

            except Exception as e:
                if connected_emitted:
                    self.connection_changed.emit(False)
                    connected_emitted = False
                try:
                    if self._client:
                        self._client.close()
                finally:
                    self._client = None
                self.error.emit(f"{type(e).__name__}: {e}")
                time.sleep(backoff)
                backoff = min(5.0, backoff * 2)

        try:
            if self._client:
                self._client.close()
        except Exception:
            pass
