from pathlib import Path
import tempfile

import msgpack
import zmq
from zmq.utils.monitor import recv_monitor_message

from ros.topic_faults import TopicFaults


def deliveries(seed, faults, arrival_lag_ns=0):
    with tempfile.TemporaryDirectory(prefix="s5-schedule-", dir="/tmp") as directory:
        with zmq.Context() as context:
            with context.socket(zmq.REP) as control:
                control.setsockopt(zmq.LINGER, 0)
                control.bind(f"ipc://{directory}/bridge.sock")
                scheduler = TopicFaults(context, Path(directory))
                try:
                    scheduler.poll()
                    assert control.poll(1000)
                    assert msgpack.unpackb(control.recv(), raw=False) == {
                        "op": "poll", "audit": {}, "episode": None,
                    }
                    control.send(msgpack.packb({"seed": seed, "faults": faults, "episode": 0}))
                    assert scheduler.socket.poll(1000)
                    scheduler.poll()
                    delivered = []
                    for tick in range(100):
                        scheduler.advance(tick * 20_000_000 + arrival_lag_ns)
                        scheduler.submit(
                            "/joint_states", lambda value: delivered.append(
                                (scheduler.now_ns, value),
                            ), tick, tick * 20_000_000,
                        )
                    scheduler.advance(5_000_000_000 + arrival_lag_ns)
                    return delivered, scheduler.audit
                finally:
                    scheduler.close()


def test_combined_delay_drop_repeats_seed_and_preserves_original_stamps():
    # TODO(pawel): confirm topic name.
    faults = [
        {"type": "topic_delay", "topic": "/joint_states", "ms": 100},
        {"type": "topic_drop", "topic": "/joint_states", "rate": 0.5},
    ]
    first, audit = deliveries(7, faults)
    assert (first, audit) == deliveries(7, faults)
    assert first != deliveries(8, faults)[0]
    assert 20 < len(first) < 80
    assert all(delivered - source * 20_000_000 >= 100_000_000 for delivered, source in first)
    stats = audit["/joint_states"]
    assert stats["received"] == stats["dropped"] + stats["delivered"] == 100
    lagged, lagged_audit = deliveries(7, faults, arrival_lag_ns=1_000_000_000)
    assert [source for _, source in lagged] == [source for _, source in first]
    assert lagged_audit["/joint_states"]["min_delay_ns"] == 100_000_000


def test_zero_and_full_drop_leave_other_topics_unaffected():
    fault = {"type": "topic_drop", "topic": "/joint_states", "rate": 1}
    assert deliveries(0, [fault])[0] == []
    fault["rate"] = 0
    assert len(deliveries(0, [fault])[0]) == 100
    fault["topic"] = "/diagnostics"
    assert len(deliveries(0, [fault])[0]) == 100


def test_episode_change_preserves_pending_delivery_but_resets_its_evidence():
    with tempfile.TemporaryDirectory(prefix="s5-epoch-", dir="/tmp") as directory:
        with zmq.Context() as context:
            with context.socket(zmq.REP) as control:
                control.setsockopt(zmq.LINGER, 0)
                scheduler = TopicFaults(context, Path(directory))
                monitor = scheduler.socket.get_monitor_socket(zmq.EVENT_CONNECTED)
                control.bind(f"ipc://{directory}/bridge.sock")
                try:
                    assert monitor.poll(2000)
                    assert recv_monitor_message(monitor)["endpoint"] == (
                        f"ipc://{directory}/bridge.sock".encode()
                    )
                    plan = {
                        "seed": 0, "episode": 0,
                        "faults": [{"type": "topic_delay", "topic": "/joint_states", "ms": 100}],
                    }
                    scheduler.poll()
                    assert control.poll(1000)
                    control.recv()
                    control.send(msgpack.packb(plan))
                    assert scheduler.socket.poll(1000)
                    scheduler.poll()
                    received = []
                    scheduler.submit("/joint_states", received.append, "previous", 0)
                    previous_audit = scheduler.audit["/joint_states"]
                    assert previous_audit["received"] == 1
                    assert control.poll(1000)
                    assert msgpack.unpackb(control.recv(), raw=False)["episode"] == 0
                    control.send(msgpack.packb({**plan, "episode": 1}))
                    assert scheduler.socket.poll(1000)
                    scheduler.poll()
                    assert scheduler.audit == {}
                    scheduler.advance(100_000_000)
                    assert received == ["previous"]
                    assert scheduler.audit == {}
                    assert previous_audit["delivered"] == 1
                    scheduler.submit("/joint_states", received.append, "current", 100_000_000)
                    scheduler.advance(200_000_000)
                    assert received == ["previous", "current"]
                    assert scheduler.audit["/joint_states"]["received"] == 1
                    assert scheduler.audit["/joint_states"]["delivered"] == 1
                finally:
                    monitor.close(linger=0)
                    scheduler.close()
