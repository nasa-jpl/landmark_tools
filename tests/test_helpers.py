import subprocess

def run_cmd(cmd, cwd):
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, f"Command failed: {result.stderr}"
    return result.stdout