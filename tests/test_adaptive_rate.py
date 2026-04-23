"""Tests for the adaptive symbol-rate ladder and initial-rate selection."""
import pytest

from host.config import (
    RATE_LADDER_MS,
    pick_initial_rate_ms,
    PROBE_UP_AFTER,
    FALLBACK_AFTER,
    SPLIT_BRAIN_TIMEOUT_MS,
)


@pytest.mark.parametrize("delta,expected_ms", [
    (300, 70),   # very strong signal → fastest tier
    (200, 70),
    (180, 70),
    (179, 80),
    (150, 80),
    (130, 80),
    (129, 100),
    (100, 100),
    (90, 100),
    (89, 120),
    (60, 120),
    (59, 160),
    (30, 160),
    (0, 160),
])
def test_pick_initial_rate(delta, expected_ms):
    assert pick_initial_rate_ms(delta) == expected_ms


def test_rate_ladder_strictly_increasing():
    """Ladder is sorted from fastest (smallest ms) to slowest."""
    assert RATE_LADDER_MS == sorted(RATE_LADDER_MS)
    assert len(RATE_LADDER_MS) == len(set(RATE_LADDER_MS))


def test_pick_returns_value_on_ladder_or_near_it():
    """Picked rate should be one of the ladder rungs OR a supported probe value.
    The delta table currently offers 70ms which isn't on the main ladder — document
    that so future changes stay intentional."""
    supported = set(RATE_LADDER_MS) | {70}
    for delta in (0, 50, 75, 95, 135, 185, 250):
        assert pick_initial_rate_ms(delta) in supported


def test_thresholds_are_positive():
    assert PROBE_UP_AFTER > 0
    assert FALLBACK_AFTER > 0
    assert SPLIT_BRAIN_TIMEOUT_MS > 0
