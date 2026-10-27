import pytest

from exporterV2.realtime_pacing import RealtimePacer


def test_pacing_waits_only_for_spare_time_and_resets_after_pause():
    now = [0.0]
    waits = []

    def sleep(seconds):
        waits.append(seconds)
        now[0] += seconds

    pacer = RealtimePacer(1 / 60, clock=lambda: now[0], sleep=sleep)
    assert pacer.wait(0) == 0
    now[0] += .005  # Physics and rendering consumed part of the frame.
    assert pacer.wait(1 / 60) == pytest.approx(1 / 60 - .005)
    assert now[0] == pytest.approx(1 / 60)
    now[0] += 2  # Slow frame: no catch-up burst.
    assert pacer.wait(2 / 60) == 0
    assert pacer.wait(3 / 60) == pytest.approx(1 / 60)
    pacer.reset()
    now[0] += 10
    assert pacer.wait(3 / 60) == 0
    assert pacer.wait(4 / 60) == pytest.approx(1 / 60)
    assert len(waits) == 3
