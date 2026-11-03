import ast
from contextlib import contextmanager
from pathlib import Path
import tempfile

import msgpack
import numpy as np
import pytest
import yaml
import zmq

from backends import create_faults, create_ground_truth
from sandbox.server import SimServer
from sandbox.sim.faults import load_scenario, validate
from sandbox.sim.ground_truth import bind_private, inside_crate


PHYSICAL_FAULTS = [
    {"type": "wrong_object"},
    load_scenario(Path(__file__).parent / "fixtures/drop_in_transit.yaml")["faults"][0],
    {"type": "place_outside_crate", "offset_m": [-0.3, 0, 0]},
    # TODO(pawel): confirm camera name.
    {"type": "camera_offset", "cam": "cam2", "xyz_m": [0.01, 0, 0], "rpy_deg": [0, 0, 0]},
]


@contextmanager
def simulation(faults, seed=0):
    with tempfile.TemporaryDirectory(prefix="s5-", dir="/tmp") as directory:
        server = SimServer(directory, seed, scenario={"faults": faults})
        try:
            yield server
        finally:
            server.state_socket.close(linger=0)
            server.command_socket.close(linger=0)
            server.context.term()


def episode(server):
    assert server.command({
        "op": "pick_place", "object_id": "wire0", "crate_id": "crate0",
    }) is None
    while server.flow.result is None:
        server.step()
    assert server.flow.result is True
    return server.truth.snapshot(), server.faults.verification(server.truth)


@pytest.mark.parametrize("faults,seed", [([], seed) for seed in (0, 19, 42)] + [
    ([fault], 0) for fault in PHYSICAL_FAULTS
])
def test_physical_faults_and_healthy_controls(faults, seed, tmp_path):
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump({"faults": faults}))
    with simulation(load_scenario(path)["faults"], seed) as server:
        truth, verification = episode(server)
        assert verification["status"] == "valid", (truth["events"], verification)
        assert all(item["verified"] for item in verification["faults"])
        assert set(truth["objects"]) == {*(f"wire{i}" for i in range(7)), "crate0", "crate1"}
        assert set(truth["cameras"]) == {"cam0", "cam1", "cam2", "cam3"}
        assert [event["time_ns"] for event in truth["events"]] == sorted(
            event["time_ns"] for event in truth["events"]
        )
        kinds = [event["kind"] for event in truth["events"]]
        assert "grasp" in kinds and "release" in kinds and kinds[-1] == "placement_result"
        if not faults:
            assert "drop" not in kinds
            assert inside_crate(server.world, "wire0", "crate0")
        elif faults[0]["type"] == "camera_offset":
            camera = server.world.model.camera("cam2")
            assert camera.pos[0] == pytest.approx(0.01)
            assert server.faults.perception.position[0] == 0
        else:
            assert not inside_crate(server.world, "wire0", "crate0")
        if faults and faults[0]["type"] == "wrong_object":
            assert inside_crate(server.world, server.faults.actual_wire, "crate0")
        if faults and faults[0]["type"] == "drop_in_transit":
            drop = next(event for event in truth["events"] if event["kind"] == "drop")
            assert 8_000_000_000 <= drop["time_ns"] < 8_100_000_000


@pytest.mark.parametrize("fault", PHYSICAL_FAULTS[:2])
def test_seeded_fault_repeats_event_log_and_physics(fault):
    runs = []
    for _ in range(2):
        with simulation([fault]) as server:
            truth, verification = episode(server)
            runs.append((msgpack.packb(truth), server.world.data.qpos.copy()))
            assert verification["status"] == "valid"
    assert runs[0][0] == runs[1][0]
    np.testing.assert_array_equal(runs[0][1], runs[1][1])


@pytest.mark.parametrize("fault", [
    {"type": "drop_in_transit", "at_state": "TRANSPORT", "t": 100},
    {"type": "place_outside_crate", "offset_m": [0, 0, 0]},
    {"type": "camera_offset", "cam": "cam2", "xyz_m": [0, 0, 0], "rpy_deg": [0, 0, 0]},
    {"type": "topic_delay", "topic": "/joint_states", "ms": 200},
    {"type": "topic_drop", "topic": "/joint_states", "rate": 0},
])
def test_ineffective_fault_is_invalid_not_healthy(fault):
    with simulation([fault]) as server:
        _, verification = episode(server)
        assert verification["status"] == "invalid"
        assert verification["faults"] == [{"type": fault["type"], "verified": False}]
        assert inside_crate(server.world, "wire0", "crate0")


@pytest.mark.parametrize("scenario", [
    None, {}, {"faults": "none"}, {"faults": [{"type": "unknown"}]},
    {"faults": [{"type": "topic_drop", "topic": "/joint_states", "rate": 1.1}]},
    {"faults": [{"type": "topic_delay", "topic": "/ground_truth", "ms": 100}]},
    {"faults": [{"type": "camera_offset", "cam": "cam2", "xyz_m": [float("nan"), 0, 0],
                 "rpy_deg": [0, 0, 0]}]},
    {"faults": [{"type": "drop_in_transit", "at_state": "DONE", "t": 0}]},
])
def test_invalid_scenarios_are_rejected(scenario):
    with pytest.raises(ValueError):
        validate(scenario)


def test_private_ipc_permissions_and_capability_boundaries(tmp_path, monkeypatch):
    private = tmp_path / "private"
    with zmq.Context() as context:
        with context.socket(zmq.REP) as socket:
            bind_private(socket, private, "ground_truth.sock")
            assert private.stat().st_mode & 0o777 == 0o700
            assert (private / "ground_truth.sock").stat().st_mode & 0o777 == 0o600
        public = tmp_path / "public"
        public.mkdir(mode=0o755)
        with context.socket(zmq.REP) as socket:
            with pytest.raises(ValueError, match="0700"):
                bind_private(socket, public, "truth.sock")
    with pytest.raises(ValueError, match="outside"):
        SimServer(public, 0, private_dir=public / "private")
    assert not hasattr(create_faults(private), "snapshot")
    assert not hasattr(create_ground_truth(private), "configure")
    monkeypatch.setenv("BENCH_BACKEND", "isaac_docker")
    with pytest.raises(NotImplementedError):
        create_ground_truth(private).snapshot()
    with pytest.raises(NotImplementedError):
        create_faults(private).configure({"faults": []}, 0)
    root = Path(__file__).resolve().parents[1]
    for directory in ("backends", "interfaces", "harness", "grading", "agent"):
        for path in (root / directory).glob("**/*.py"):
            for node in ast.walk(ast.parse(path.read_text())):
                imports = ([alias.name for alias in node.names] if isinstance(node, ast.Import)
                           else [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
                assert not any(name.startswith("sandbox") for name in imports), path
    assert "ground_truth" not in (root / "ros" / "bridge_node.py").read_text().split('"""', 2)[-1]


def test_live_contacts_and_rotation_only_camera_fault():
    fault = {"type": "camera_offset", "cam": "cam0", "xyz_m": [0, 0, 0], "rpy_deg": [0, 0, 15]}
    with simulation([fault]) as server:
        before_position, before_quaternion = server.faults.camera_before["cam0"]
        camera = server.world.model.camera("cam0")
        np.testing.assert_array_equal(camera.pos, before_position)
        assert abs(camera.quat @ before_quaternion) < 0.999
        assert server.faults.verification(server.truth)["faults"][0]["verified"]
        assert server.command({"op": "pick_place", "object_id": "wire0", "crate_id": "crate0"}) is None
        while not server.truth.held:
            server.step()
            assert server.flow.result is None, "The wire was never physically held"
        snapshot = server.truth.snapshot()
        assert any(contact["object"] == "wire0" and contact["normal_force_n"] > 0
                   for contact in snapshot["gripper_contacts"])
        assert snapshot["events"][-1]["kind"] == "grasp"
