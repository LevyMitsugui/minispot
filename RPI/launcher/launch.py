import os
import yaml
import subprocess
import time

# Get absolute project root path (RPI/)
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Load YAML config
with open("config.yaml", "r") as f:
    cfg = yaml.safe_load(f)

# Resolve path to modules
raw_modules_path = cfg["global"]["modules_path"]
modules_path = raw_modules_path if os.path.isabs(raw_modules_path) else os.path.join(project_root, raw_modules_path)

print(f"[INFO] Project root: {project_root}")
print(f"[INFO] Modules path: {modules_path}")

# Start module processes
procs = {}

for module_name, config in cfg.items():
    if module_name == "global":
        continue

    if not config.get("enabled", True):
        print(f"[SKIP] {module_name} is disabled in config.")
        continue

    main_path = os.path.join(modules_path, module_name, "main.py")

    if not os.path.isfile(main_path):
        print(f"[ERROR] main.py not found for {module_name} at {main_path}. Skipping.")
        continue

    cmd = ["python3", main_path]

    # Add arguments
    for args, value in config.get("args", {}).items():
        cmd += [f"{args}", str(value)]

    # Add flags
    for flag, state in config.get("flags", {}).items():
        if isinstance(state, bool):
            if state:
                cmd.append(f"--{flag}")
        else:
            cmd += [f"--{flag}", str(state)]

    print(f"[LAUNCH] Starting {module_name} → {cmd}")
    procs[module_name] = subprocess.Popen(cmd)

# Optional: supervise for crashes
try:
    while True:
        for name, proc in procs.items():
            if proc.poll() is not None:
                print(f"[ERROR] {name} exited unexpectedly. Restarting...")
                config = cfg[name]
                main_path = os.path.join(modules_path, name, "main.py")
                cmd = ["python3", main_path]

                for args, value in config.get("params", {}).items():
                    cmd += [f"--{args}", str(value)]
                for flag, state in config.get("flags", {}).items():
                    if isinstance(state, bool):
                        if state:
                            cmd.append(f"--{flag}")
                    else:
                        cmd += [f"--{flag}", str(state)]

                procs[name] = subprocess.Popen(cmd)
        time.sleep(1)
except KeyboardInterrupt:
    print("[SHUTDOWN] Terminating all modules...")
    for name, proc in procs.items():
        print(f"[SHUTDOWN] Terminating {name}...")
        proc.terminate()
        try:
            proc.wait(timeout=5)
            print(f"[SHUTDOWN] {name} terminated cleanly.")
        except subprocess.TimeoutExpired:
            print(f"[FORCE] {name} did not terminate in time. Killing...")
            proc.kill()
    print("[SHUTDOWN] All modules terminated.")