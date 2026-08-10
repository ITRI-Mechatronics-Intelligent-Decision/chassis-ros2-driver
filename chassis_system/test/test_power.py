"""power.py 的純邏輯測試：不需 ROS、不需硬體，可在 WSL2 開發機執行."""

import pytest

from chassis_system.power import (
    ShutdownRejected,
    check_confirm_code,
    resolve_delay,
    run_shutdown,
)


def test_check_confirm_code_accepts_exact_match():
    check_confirm_code('SHUTDOWN', 'SHUTDOWN')


def test_check_confirm_code_rejects_mismatch():
    with pytest.raises(ShutdownRejected):
        check_confirm_code('shutdown', 'SHUTDOWN')


def test_check_confirm_code_rejects_empty_request():
    with pytest.raises(ShutdownRejected):
        check_confirm_code('', 'SHUTDOWN')


def test_check_confirm_code_disabled_by_empty_expected():
    check_confirm_code('', '')


def test_resolve_delay_negative_falls_back_to_default():
    assert resolve_delay(-1.0, 5.0, 300.0) == 5.0


def test_resolve_delay_zero_is_immediate():
    assert resolve_delay(0.0, 5.0, 300.0) == 0.0


def test_resolve_delay_passes_through_valid_value():
    assert resolve_delay(30.0, 5.0, 300.0) == 30.0


def test_resolve_delay_rejects_above_max():
    with pytest.raises(ShutdownRejected):
        resolve_delay(301.0, 5.0, 300.0)


def test_run_shutdown_dry_run_does_not_execute():
    message = run_shutdown(['/bin/false'], dry_run=True)
    assert 'dry_run' in message


def test_run_shutdown_reports_success():
    assert 'accepted' in run_shutdown(['/bin/true'])


def test_run_shutdown_raises_on_nonzero_exit():
    with pytest.raises(RuntimeError):
        run_shutdown(['/bin/false'])


def test_run_shutdown_raises_on_missing_command():
    with pytest.raises(RuntimeError):
        run_shutdown(['/nonexistent/shutdown'])
