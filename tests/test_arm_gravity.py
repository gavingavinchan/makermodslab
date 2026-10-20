import math

import numpy as np
import pytest

pytest.importorskip("pinocchio")

from makermodslab.arm_gravity import (  # noqa: E402
    GravityModel,
    Sample,
    cross_validate,
    fit_gravity,
    interpolate,
    plan_poses,
)

JOINTS = ("pan", "lift", "elbow")


def _link(name, mass, com):
    inertia = 'ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"'
    return (
        f'<link name="{name}"><inertial><origin xyz="{com}" rpy="0 0 0"/>'
        f'<mass value="{mass}"/><inertia {inertia}/></inertial></link>'
    )


def _joint(name, kind, parent, child, xyz, axis):
    return (
        f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
        f'<origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>'
        f'<limit lower="-3" upper="3" effort="5" velocity="2"/></joint>'
    )


@pytest.fixture
def urdf(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text(
        '<robot name="arm">'
        + _link("base", 1.0, "0 0 0.02")
        + _link("yaw", 0.4, "0 0 0.03")
        + _link("upper", 0.5, "-0.08 0 0.01")
        + _link("fore", 0.4, "0.07 0 0.02")
        + _link("jaw", 0.05, "0.05 0.01 0")
        + _joint("pan", "revolute", "base", "yaw", "0 0 0.05", "0 0 -1")
        + _joint("lift", "revolute", "yaw", "upper", "0 0 0.05", "0 -1 0")
        + _joint("elbow", "revolute", "upper", "fore", "-0.16 0 0", "0 -1 0")
        + _joint("finger", "prismatic", "fore", "jaw", "0.1 0 0", "0 1 0")
        + "</robot>"
    )
    return path


def poses(n, seed=1):
    rng = np.random.default_rng(seed)
    return [
        {"pan": rng.uniform(-90, 90), "lift": rng.uniform(-170, -10), "elbow": rng.uniform(10, 240)}
        for _ in range(n)
    ]


def test_locked_gripper_mass_rides_on_its_parent_and_folded_lift_matches_hand_math(urdf):
    model = GravityModel(urdf, JOINTS)
    masses = model.prior.reshape(3, 4)[:, 0]
    assert masses.tolist() == pytest.approx([0.4, 0.5, 0.45])
    # Pan is vertical: gravity never loads it.
    for p in poses(5):
        assert model.torque(p)["pan"] == pytest.approx(0.0, abs=1e-9)
    # Folded straight back at lift=0, elbow=180: the elbow carries the forearm
    # and jaw about its own axis. Pinocchio sign: torque the motor supplies.
    tau = model.torque({"pan": 0.0, "lift": 0.0, "elbow": 0.0})
    expected_elbow = 9.81 * (0.4 * 0.07 + 0.05 * 0.15)
    assert abs(tau["elbow"]) == pytest.approx(expected_elbow, rel=1e-6)


def test_fit_recovers_a_heavier_arm_and_sensor_offsets_from_noise_free_holds(urdf):
    model = GravityModel(urdf, JOINTS)
    truth = model.prior * 1.3  # potting, cables: everything heavier than CAD
    offsets = np.array([0.0, 0.2, -0.1])
    samples = [
        Sample(p, dict(zip(JOINTS, model.regressor(p) @ truth + offsets, strict=True)), group=i)
        for i, p in enumerate(poses(30))
    ]
    fit = fit_gravity(model, samples, regularization=1e-8)
    assert fit.offsets[1:] == pytest.approx(offsets[1:], abs=1e-4)
    for p in poses(10, seed=7):
        assert model.regressor(p) @ fit.params + fit.offsets == pytest.approx(
            model.regressor(p) @ truth + offsets, abs=1e-4
        )
    assert max(fit.rms_nm.values()) < 1e-4
    assert fit.prior_rms_nm["elbow"] > 0.1


def test_cross_validation_reports_held_out_error_near_the_noise(urdf):
    model = GravityModel(urdf, JOINTS)
    rng = np.random.default_rng(3)
    samples = []
    for i, p in enumerate(poses(24)):
        tau = model.regressor(p) @ (model.prior * 1.2)
        for _ in range(2):
            samples.append(Sample(p, dict(zip(JOINTS, tau + rng.normal(0, 0.05, 3), strict=True)), group=i))
    cv = cross_validate(model, samples)
    assert cv["elbow"] < 0.12
    assert cv["lift"] < 0.12


def test_non_finite_inputs_are_refused(urdf):
    model = GravityModel(urdf, JOINTS)
    with pytest.raises(ValueError, match="lift"):
        model.torque({"pan": 0.0, "lift": math.nan, "elbow": 0.0})
    with pytest.raises(ValueError, match="no joint"):
        GravityModel(urdf, ("pan", "wrist"))


def test_interpolation_steps_on_the_joint_that_moves_furthest():
    steps = interpolate({"a": 0.0, "b": 0.0}, {"a": 10.0, "b": 2.0}, 3.0)
    assert len(steps) == 4
    assert steps[-1] == {"a": 10.0, "b": 2.0}


def _plan(model, is_clear, **kw):
    limits = {"pan": (-180, 180), "lift": (-176, -2), "elbow": (3, 251)}
    args = {
        "start": {"pan": 5.0, "lift": -3.0, "elbow": 3.0},
        "limits": limits,
        "box": {"lift": (-170, -10), "elbow": (10, 245)},
        "count": 8,
        "is_clear": is_clear,
        "hub": {"pan": 5.0, "lift": -90.0, "elbow": 90.0},
        "max_torque_nm": 0.8,
        "seed": 2,
        "pool": 400,
    }
    args.update(kw)
    return plan_poses(model, **args)


def test_plan_respects_limits_torque_cap_and_approaches_from_both_sides(urdf):
    model = GravityModel(urdf, JOINTS)
    visits = _plan(model, lambda p: True)
    assert len(visits) == 8
    for visit in visits:
        assert visit.pose["pan"] == 5.0  # outside the box: held at start
        # The box, shrunk to the soft limits less the 8 degree margin.
        assert -168 <= visit.pose["lift"] <= -10 and 11 <= visit.pose["elbow"] <= 243
        assert max(abs(v) for v in model.torque(visit.pose, model.prior).values()) <= 0.8
        assert len(visit.approaches) == 2
        above, below = visit.approaches
        assert above["lift"] - visit.pose["lift"] == pytest.approx(6.0)
        assert below["elbow"] - visit.pose["elbow"] == pytest.approx(-6.0)


def test_blocked_straight_legs_route_through_the_hub_and_unreachable_poses_drop(urdf):
    model = GravityModel(urdf, JOINTS)
    hub = {"pan": 5.0, "lift": -30.0, "elbow": 150.0}
    far_box = {"lift": (-170, -100), "elbow": (140, 245)}

    # A wall at lift -60..-50, passable only with the elbow at 140 or more. A
    # straight line from the folded start reaches it with the elbow far lower.
    def clear(p):
        return not (-60 < p["lift"] < -50) or p["elbow"] >= 140

    visits = _plan(model, clear, hub=hub, box=far_box, max_torque_nm=2.0)
    assert visits
    assert visits[0].via[0] == [hub]
    assert all(route == [] for v in visits[1:] for route in v.via)

    walled_hub = {"pan": 5.0, "lift": -55.0, "elbow": 10.0}
    assert _plan(model, clear, hub=walled_hub, box=far_box, max_torque_nm=2.0) == []


VISCOUS_ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")


def test_vendored_viscous_model_is_the_one_the_runs_were_recorded_against():
    from makermodslab.arm_gravity import VISCOUS_URDF

    model = GravityModel(VISCOUS_URDF, VISCOUS_ARM)
    # viscous-arm-simulation 86d2cce; the gravity runs of 2026-10-20 recorded this hash.
    assert model.urdf_sha256 == "c1e6c3876a67ce2a2e229bdd8a80960e68f085e08833d699536b1feddefb6ff7"
    tau = model.torque(dict.fromkeys(VISCOUS_ARM, 0.0))
    assert [round(tau[j], 2) for j in ("shoulder_lift", "elbow_flex", "wrist_flex")] == [0.37, 4.08, 1.59]


def test_vendored_collision_meshes_load_and_the_floor_is_enforced():
    from makermodslab.arm_gravity import VISCOUS_URDF, CollisionChecker

    folded = dict.fromkeys(VISCOUS_ARM, 0.0)
    assert CollisionChecker(VISCOUS_URDF, VISCOUS_URDF.parent, VISCOUS_ARM, floor_z=0.03).is_clear(folded)
    high_floor = CollisionChecker(VISCOUS_URDF, VISCOUS_URDF.parent, VISCOUS_ARM, floor_z=0.5)
    assert any(p.endswith("/floor") for p in high_floor.problems(folded))
