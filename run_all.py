import cmd
import os, sys, subprocess, time, shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"
ENV_NAME = "sensor_fusion_env"
ENV_FILE = ROOT / "environment.yml"
URDF_PATH = ROOT / "models" / "m710ic70.urdf"

DEFAULT_XSENS_SDK_DIR = r"C:/Program Files/Xsens/MT Software Suite 2025.2/MT SDK/x64"

EXE_NAME = "sensor_fusion.exe"

PY_FANUC = ROOT / "py" / "fanuc_poll_udp.py"

def run(cmd, cwd=ROOT, shell=False, env=None):
	print(">>>", cmd if isinstance(cmd, str) else " ".join(map(str, cmd)))
	subprocess.check_call(cmd, cwd=cwd, shell=shell, env=env)
 
def check_conda():
	try:
		subprocess.check_output(["conda", "--version"], text=True)
	except Exception as e:
		print("ERROR: Conda not found. Please install Anaconda or Miniconda and ensure 'conda' is in your PATH.")
		sys.exit(1)

def create_conda_env():
	if not ENV_FILE.exists():
		print(f"ERROR: {ENV_FILE} not found.")
		sys.exit(1)
		
	output = subprocess.check_output(["conda", "env", "list"], text=True)
	if ENV_NAME not in output:
		print(f"Creating conda environment '{ENV_NAME}' from {ENV_FILE}...")
		run(["conda", "env", "create", "-f", str(ENV_FILE), "-n", ENV_NAME])
	else:
		print(f"Conda environment '{ENV_NAME}' already exists.")
		run(["conda", "env", "update", "-n", ENV_NAME, "-f", str(ENV_FILE), "--prune"])
  
def find_vcvars64():
	program_files_x86 = os.environ.get("ProgramFiles(x86)", r"C:/Program Files (x86)")
	vswhere = Path(program_files_x86) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
	if not vswhere.exists():
		return None
	try:
		install_path = subprocess.check_output([str(vswhere), "-latest", "-products", "*",  "-property", "installationPath"], text=True).strip()
		if not install_path:
			return None
		vcvars64 = Path(install_path) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
		if vcvars64.exists():
			return str(vcvars64)
		return None
	except Exception as e:
		return None
