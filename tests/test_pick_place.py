import numpy as np
import pytest

from sandbox.sim.pick_place import FlowState, PickPlaceFlow
from sandbox.sim.perception import rotation_matrix
from sandbox.sim.world import World


def inside_crate(world, wire, crate):
    position, quaternion = world.body_pose(wire)
    center, crate_quaternion = world.body_pose(crate)
    crate_rotation = rotation_matrix(crate_quaternion)
    # Both capsule endpoints, including their radius, must clear the inner walls.
    axis = rotation_matrix(quaternion)[:, 0] * 0.09
    endpoints = np.array([position - axis, position + axis])
    local = (endpoints - center) @ crate_rotation
    return bool(
        np.all(np.abs(local[:, 0]) + 0.009 < 0.16)
        and np.all(np.abs(local[:, 1]) + 0.009 < 0.11)
        and np.all(local[:, 2] >= 0.02)
        and np.all(local[:, 2] + 0.009 < 0.12)
    )


def test_twenty_seeded_healthy_episodes_place_at_least_eighteen_wires():
    outcomes = []
    for seed in range(20):
        world = World(seed=seed)
        wire, crate = f"wire{seed % 7}", f"crate{seed % 2}"
        flow = PickPlaceFlow(world)
        assert flow.run(wire, crate) is True
        outcomes.append(inside_crate(world, wire, crate))
        assert not any(warning.number for warning in world.data.warning)
        joint = world.model.body(wire).jntadr[0]
        dof = world.model.jnt_dofadr[joint]
        assert np.max(np.abs(world.data.qvel[dof:dof + 6])) < 0.001
    assert sum(outcomes) >= 18, f"Only {sum(outcomes)}/20 physical placements: {outcomes}"


def test_same_seed_repeats_physics_states_and_results():
    snapshots = []
    for _ in range(2):
        world = World(seed=19)
        states = []
        flow = PickPlaceFlow(world, on_state=states.append)
        assert flow.state == FlowState.IDLE
        assert flow.result is None
        flow.start("wire5", "crate1")
        before = world.data.time
        flow.step()
        assert world.data.time == pytest.approx(before + world.model.opt.timestep)
        while flow.result is None:
            flow.step()
        snapshots.append((world.data.qpos.copy(), world.data.qvel.copy(), states, flow.result))
        assert inside_crate(world, "wire5", "crate1")
        assert states == list(FlowState)
        before = world.data.time
        flow.step()
        assert world.data.time == before
    for first, second in zip(snapshots[0], snapshots[1]):
        np.testing.assert_array_equal(first, second)


def test_completed_commands_report_success_even_when_the_wire_is_missing(monkeypatch):
    world = World(seed=0)
    flow = PickPlaceFlow(world)
    flow.start("wire0", "crate0")
    world.set_body_pose("wire0", [0.7, 0.45, 0.8], [1, 0, 0, 0])

    def forbid_outcome_read(_):
        pytest.fail("The command flow read an object pose after planning")

    with monkeypatch.context() as patch:
        patch.setattr(world, "body_pose", forbid_outcome_read)
        while flow.result is None:
            flow.step()
    assert flow.result is True
    assert flow.state == FlowState.DONE
    assert not inside_crate(world, "wire0", "crate0")


def test_invalid_requests_and_busy_start_do_not_replace_the_current_command():
    world = World(seed=0)
    flow = PickPlaceFlow(world)
    for wire, crate in (("missing", "crate0"), ("wire0", "missing")):
        with pytest.raises(ValueError, match="Unknown"):
            flow.start(wire, crate)
        assert flow.state == FlowState.IDLE
        assert flow.result is None
        assert world.data.time == 0
    flow.start("wire0", "crate0")
    with pytest.raises(RuntimeError, match="already running"):
        flow.start("wire1", "crate1")
    while flow.result is None:
        flow.step()
    assert inside_crate(world, "wire0", "crate0")
    assert flow.run("wire1", "crate1") is True
    assert inside_crate(world, "wire1", "crate1")
