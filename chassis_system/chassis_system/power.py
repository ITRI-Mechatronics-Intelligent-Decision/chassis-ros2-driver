"""上位機電源控制的純邏輯層：請求驗證與關機指令執行，不含任何 ROS 相依."""

import subprocess

# 預設關機指令。`sudo -n` 表示不可互動索取密碼，缺少 sudoers 授權時直接失敗，
# 而不是卡在密碼提示上。對應的授權檔見 deploy/chassis-shutdown.sudoers。
DEFAULT_SHUTDOWN_COMMAND = ['sudo', '-n', '/sbin/shutdown', '-h', 'now']


class ShutdownRejected(Exception):
    """關機請求未通過驗證，屬呼叫端可自行修正的錯誤."""


def check_confirm_code(confirm, expected):
    """比對確認碼，不符即拒絕。空字串的 expected 代表停用確認機制."""
    if not expected:
        return
    if confirm != expected:
        raise ShutdownRejected(
            'confirm code mismatch: this service powers off the onboard computer, '
            'set confirm to the configured confirm_code to proceed'
        )


def resolve_delay(delay_sec, default_delay, max_delay):
    """驗證倒數秒數並回傳實際採用值；負值代表沿用 default_delay."""
    delay = float(default_delay) if delay_sec < 0.0 else float(delay_sec)
    if delay > max_delay:
        raise ShutdownRejected(
            f'delay_sec {delay:.1f} exceeds max_delay_sec {max_delay:.1f}'
        )
    return delay


def run_shutdown(command, dry_run=False, timeout=10.0):
    """
    執行關機指令並回傳說明字串；失敗時拋出 RuntimeError.

    正常情況下本機會在指令回傳後隨即斷電，因此呼叫端只會在「失敗」時
    看到後續程式碼被執行。
    """
    if dry_run:
        return f'dry_run enabled, shutdown command not executed: {" ".join(command)}'

    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False
        )
    except FileNotFoundError as exc:
        raise RuntimeError(f'shutdown command not found: {command[0]}') from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f'shutdown command timed out after {timeout:.1f}s') from exc

    if result.returncode != 0:
        detail = (result.stderr or result.stdout or '').strip()
        raise RuntimeError(
            f'shutdown command exited with {result.returncode}: {detail}'
        )

    return f'shutdown command accepted: {" ".join(command)}'
