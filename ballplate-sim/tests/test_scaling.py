import pytest
from ballplate.scaling import from_raw, to_raw


def test_symmetric_range():
    assert to_raw(15, -15, 15) == 32000 and to_raw(-15, -15, 15) == -32000
    assert to_raw(0, -15, 15) == 0
    assert from_raw(16000, -15, 15) == pytest.approx(7.5)


def test_asymmetric_range_and_saturation():
    assert to_raw(-0.4, -0.4, 0.4) == -32000 and to_raw(0.4, -0.4, 0.4) == 32000
    assert to_raw(0.0, -0.4, 0.8) == pytest.approx(-10667, abs=1)
    assert to_raw(5.0, -0.4, 0.4) == 32000 and to_raw(-5.0, -0.4, 0.4) == -32000
    assert from_raw(99999, -1, 1) == pytest.approx(1.0)


def test_negative_acceleration_is_representable():
    assert to_raw(-2.0, -9.81, 9.81) < 0
