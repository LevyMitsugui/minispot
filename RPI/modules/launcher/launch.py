import os
import yaml
import subprocess
import time

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # goes from launcher/ → robotdog/
print(project_root)
modules = {}

# Load config
with open("config.yaml", "r") as f:
    cfg = yaml.safe_load(f)

modules = {
    "ESPSerialHandler": {
        "path": os.path.join(project_root, "ESPSerialHandler/main.py"),
        "args": [
            "--port", cfg["ESPSerialHandler"]["port"],
            "--baudrate", str(cfg["ESPSerialHandler"]["baudrate"]),
            "--pub-socket", cfg["ESPSerialHandler"]["pubSocket"],
            "--sub-socket", cfg["ESPSerialHandler"]["subSocket"],
            "--sub-port", str(cfg["ESPSerialHandler"]["subPort"]),
        ],
        "flags": [
            "--verbose", str(cfg["ESPSerialHandler"]["verbose"])
        ]
        
    },
    "PICOSerialHandler": {
        "path": os.path.join(project_root, "PICOSerialHandler/main.py"),
        "args": [
            "--port", cfg["PICOSerialHandler"]["port"],
            "--baudrate", str(cfg["PICOSerialHandler"]["baudrate"]),
            "--socket_path", cfg["PICOSerialHandler"]["socketPath"],
        ],
        "flags": [
            "--verbose", str(cfg["PICOSerialHandler"]["verbose"])
        ]
    },
}

# Start processes
procs = {}
for name, mod in modules.items():
    cmd = ["python3", mod["path"]] + mod["args"]
    print(f"[LAUNCH] Starting {name} → {cmd}")
    procs[name] = subprocess.Popen(cmd)

# Optional: supervise (basic)
try:
    while True:
        for name, proc in procs.items():
            if proc.poll() is not None:
                print(f"[ERROR] {name} exited unexpectedly. Restarting...")
                cmd = ["python3", modules[name]["path"]] + modules[name]["args"]
                procs[name] = subprocess.Popen(cmd)
        time.sleep(1)
except KeyboardInterrupt:
    print("[SHUTDOWN] Terminating all modules...")
    for proc in procs.values():
        proc.terminate()
