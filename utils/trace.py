"""
Trazas de ejecucion: copia todo lo que se imprime en consola a un .txt con fecha y hora, de modo
que cada experimento quede guardado en su propio archivo y nunca se sobreescriba.

    with TraceLog("friedman", "semilla2") as log:
        ...print(...)...
    # -> problem/osrm/trazas/friedman_20261015_153012_semilla2.txt
"""
import os
import re
import sys
from datetime import datetime

TRACE_DIR = os.path.join("problem", "osrm", "trazas")


def timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def safe_label(label, max_len=80):
    label = re.sub(r"[^A-Za-z0-9_.+-]+", "-", str(label)).strip("-")
    return label[:max_len] or "sin_etiqueta"


class _Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for s in self.streams:
            s.write(data)
            s.flush()
        return len(data)

    def flush(self):
        for s in self.streams:
            s.flush()

    def __getattr__(self, name):          # isatty, encoding, etc.
        return getattr(self.streams[0], name)


class TraceLog:
    """Context manager: consola + archivo. `path` queda disponible tras entrar."""

    def __init__(self, prefix, label="", directory=TRACE_DIR, stamp=None):
        os.makedirs(directory, exist_ok=True)
        self.stamp = stamp or timestamp()
        name = f"{prefix}_{self.stamp}" + (f"_{safe_label(label)}" if label else "") + ".txt"
        self.path = os.path.join(directory, name)

    def __enter__(self):
        self._file = open(self.path, "w", encoding="utf-8")
        self._out, self._err = sys.stdout, sys.stderr
        sys.stdout, sys.stderr = _Tee(self._out, self._file), _Tee(self._err, self._file)
        print(f"[traza] {self.path}")
        return self

    def __exit__(self, *exc):
        sys.stdout, sys.stderr = self._out, self._err
        self._file.close()
        print(f"Traza guardada en {self.path}")
        return False
