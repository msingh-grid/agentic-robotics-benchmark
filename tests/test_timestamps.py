import pytest

from ros.check_timestamps import TopicStamps


def test_stamps_allow_independent_topic_order_and_delayed_cameras():
    stamps = TopicStamps()
    stamps.record("/clock", 2_000_000_000)
    stamps.record("/joint_states", 2_020_000_000)
    stamps.record("/image", 500_000_000)
    stamps.record("/points", 500_000_000)
    assert stamps.coherent()
    stamps.record("/clock", 2_020_000_000)
    stamps.record("/image", 1_000_000_000)
    assert stamps.coherent()


def test_stamps_require_observed_clock_coherence_after_startup():
    stamps = TopicStamps()
    stamps.record("/image", 500_000_000)
    assert not stamps.coherent()
    stamps.record("/clock", 0)
    assert not stamps.coherent()
    stamps.record("/clock", 400_000_000)
    assert stamps.coherent()
    stamps.record("/joint_states", 5_000_000_000)
    assert not stamps.coherent()
    stamps.record("/clock", 500_000_000)
    assert not stamps.coherent()


@pytest.mark.parametrize("stamp", [200, 199, -1])
def test_stamps_must_advance_within_each_topic(stamp):
    stamps = TopicStamps()
    stamps.record("/image", 200)
    with pytest.raises(AssertionError):
        stamps.record("/image", stamp)
