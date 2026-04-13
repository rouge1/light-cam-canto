"""SSH and SCP utilities for camera communication."""
import subprocess


def ssh_cmd(host: str, cmd: str, timeout: int = 30, check: bool = False) -> str:
    """Run a command on a camera via SSH. Returns stdout."""
    result = subprocess.run(
        ["ssh", f"root@{host}", cmd],
        capture_output=True, text=True, timeout=timeout,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"SSH command failed on {host}: {cmd!r}\n"
            f"  exit code: {result.returncode}\n"
            f"  stderr: {result.stderr.strip()}"
        )
    return result.stdout.strip()


def scp_to_camera(local_path: str, host: str, remote_path: str):
    """Copy a file to a camera via SCP (legacy mode for Thingino)."""
    subprocess.run(
        ["scp", "-O", local_path, f"root@{host}:{remote_path}"],
        capture_output=True, text=True, timeout=30, check=True,
    )
