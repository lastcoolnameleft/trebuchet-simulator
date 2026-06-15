import json
from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp


EPS = 1e-9
DEFAULT_FPS = 60


class SimulationError(RuntimeError):
    pass


@dataclass
class StageBundle:
    stage1: object
    stage2: object
    post_release: object
    flight: object
    t_liftoff: float
    t_release: float
    t_land: float
    release_state: np.ndarray
    release_pos: tuple[float, float]
    release_vel: tuple[float, float]
    initial_projectile_x: float


DEFAULTS = {
    'LAl': 6.0,
    'LAs': 2.0,
    'LW': 2.0,
    'LS': 6.0,
    'h': 5.0,
    'mA': 50.0,
    'mW': 200.0,
    'mP': 10.0,
    'releaseAngle': 45.0,
    'Grav': 9.81,
    'autoDerived': True,
    'Aq0': None,  # Computed from geometry if not specified
    'Wq0': None,  # Computed from geometry if not specified
    'maxTime': 6.0,
    'maxFlightTime': 20.0,
    'fps': DEFAULT_FPS,
    'maxStep': 0.001,
    'enableAirDrag': False,
    'airDensity': 1.225,
    'dragCoefficient': 0.47,
    'projectileDiameter': 0.18,
    'windSpeed': 0.0,
}


def _float(params, key):
    return float(params[key])


def normalize_params(raw_params=None):
    params = dict(DEFAULTS)
    if raw_params is not None:
        params.update(dict(raw_params))

    LAl = float(params['LAl'])
    LAs = float(params['LAs'])
    mA = float(params['mA'])
    mW = float(params['mW'])
    LW = float(params['LW'])

    if bool(params.get('autoDerived', True)):
        params['LAcg'] = (LAs - LAl) / 2.0
        params['IA3'] = mA * (LAl + LAs) ** 2 / 12.0
        params['IW3'] = mW * LW**2 / 12.0
    else:
        params['LAcg'] = float(params.get('LAcg', (LAs - LAl) / 2.0))
        params['IA3'] = float(params.get('IA3', mA * (LAl + LAs) ** 2 / 12.0))
        params['IW3'] = float(params.get('IW3', mW * LW**2 / 12.0))

    # Compute initial arm angle from geometry if not specified
    # VT formula: Aq0 = π - acos(h/LAl), arm tilted so tip is near ground
    h = float(params['h'])
    if params.get('Aq0') is None:
        ratio = h / LAl
        if abs(ratio) <= 1.0:
            params['Aq0'] = float(np.pi - np.arccos(ratio))
        else:
            # h > LAl: arm can't reach ground even vertical, use maximum tilt
            params['Aq0'] = float(14.0 * np.pi / 15.0)
    else:
        params['Aq0'] = float(params['Aq0'])

    # CW hangs straight down under gravity when arm is pinned
    if params.get('Wq0') is None:
        params['Wq0'] = -float(params['Aq0'])
    else:
        params['Wq0'] = float(params['Wq0'])

    params['releaseAngleRad'] = np.deg2rad(float(params['releaseAngle']))
    diameter = max(float(params.get('projectileDiameter', 0.18)), 0.0)
    params['projectileArea'] = float(np.pi * (diameter / 2.0) ** 2)
    params['fps'] = max(10, int(round(float(params.get('fps', DEFAULT_FPS)))))
    params['maxStep'] = max(1e-4, float(params.get('maxStep', 0.001)))
    params['maxTime'] = max(1.0, float(params.get('maxTime', 6.0)))
    params['maxFlightTime'] = max(1.0, float(params.get('maxFlightTime', 20.0)))
    params['enableAirDrag'] = bool(params.get('enableAirDrag', False))
    return params


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def safe_div(numerator, denominator):
    if abs(denominator) < EPS:
        denominator = EPS if denominator >= 0 else -EPS
    return numerator / denominator


def ground_sling_angle(Aq, params):
    """Compute sling angle Sq such that the projectile is at ground level.

    Ground constraint (VT convention, Y positive up from pivot, ground at -h):
        LAl*cos(Aq) + LS*cos(Aq+Sq) = -h

    We choose the solution where Aq+Sq is in (π, 2π) so that the sling
    points toward the target (positive X direction).
    """
    LAl = _float(params, 'LAl')
    LS = _float(params, 'LS')
    h = _float(params, 'h')
    cos_total = clamp((-h - LAl * np.cos(Aq)) / LS, -1.0, 1.0)
    # Two solutions: arccos → Aq+Sq in [0,π], or 2π-arccos → Aq+Sq in [π,2π]
    # We want sling pointing toward target: sin(Aq+Sq) < 0 → Aq+Sq in (π, 2π)
    total = 2.0 * np.pi - np.arccos(cos_total)
    return total - Aq


def stage1_sq_dot(Aq, Sq, Aw, params):
    LAl = _float(params, 'LAl')
    LS = _float(params, 'LS')
    s_as = np.sin(Aq + Sq)
    return -safe_div((LAl * np.sin(Aq) + LS * s_as) * Aw, LS * s_as)


def stage1_components(Aq, Wq, Aw, Ww, params):
    LAl = _float(params, 'LAl')
    LAs = _float(params, 'LAs')
    LAcg = _float(params, 'LAcg')
    LW = _float(params, 'LW')
    LS = _float(params, 'LS')
    Grav = _float(params, 'Grav')
    mA = _float(params, 'mA')
    mW = _float(params, 'mW')
    mP = _float(params, 'mP')
    IA3 = _float(params, 'IA3')
    IW3 = _float(params, 'IW3')

    Sq = ground_sling_angle(Aq, params)

    s_a = np.sin(Aq)
    c_a = np.cos(Aq)
    s_w = np.sin(Wq)
    c_w = np.cos(Wq)
    s_sq = np.sin(Sq)
    c_sq = np.cos(Sq)
    s_as = np.sin(Aq + Sq)
    c_as = np.cos(Aq + Sq)

    Sw = stage1_sq_dot(Aq, Sq, Aw, params)

    coupling = safe_div(c_as * Sw * (Sw + 2.0 * Aw), s_as)
    slope = safe_div(c_as, s_as) + safe_div(LAl * c_a, LS * s_as)
    accel_term = coupling + slope * Aw**2

    M11 = (
        -mP * LAl**2 * (-1.0 + safe_div(2.0 * s_a * c_sq, s_as))
        + IA3
        + IW3
        + mA * LAcg**2
        + mP * LAl**2 * safe_div(s_a**2, s_as**2)
        + mW * (LAs**2 + LW**2 + 2.0 * LAs * LW * c_w)
    )
    M12 = IW3 + LW * mW * (LW + LAs * c_w)
    M22 = IW3 + mW * LW**2

    r1 = (
        Grav * LAcg * mA * s_a
        + LAl * LS * mP * (s_sq * (Aw + Sw) ** 2 + c_sq * accel_term)
        + LAl * mP * s_a * safe_div(LAl * s_sq * Aw**2 - LS * accel_term, s_as)
        - Grav * mW * (LAs * s_a + LW * np.sin(Aq + Wq))
        - LAs * LW * mW * s_w * (Aw**2 - (Aw + Ww) ** 2)
    )
    r2 = -LW * mW * (Grav * np.sin(Aq + Wq) + LAs * s_w * Aw**2)

    det = M11 * M22 - M12 * M12
    if abs(det) < EPS:
        raise SimulationError('Stage 1 matrix became singular.')

    Awd = (r1 * M22 - r2 * M12) / det
    Wwd = -(r1 * M12 - r2 * M11) / det
    Swd = (
        -safe_div(c_as * Sw * (Sw + 2.0 * Aw), s_as)
        - (safe_div(c_as, s_as) + safe_div(LAl * c_a, LS * s_as)) * Aw**2
        - safe_div((LAl * s_a + LS * s_as) * Awd, LS * s_as)
    )

    return Sq, Sw, Awd, Wwd, Swd


def stage1_ode(_t, state, params):
    Aq, Wq, _Sq, Aw, Ww, _Sw = state
    Sq, Sw, Awd, Wwd, Swd = stage1_components(Aq, Wq, Aw, Ww, params)
    return np.array([Aw, Ww, Sw, Awd, Wwd, Swd], dtype=float)


def stage2_ode(_t, state, params):
    Aq, Wq, Sq, Aw, Ww, Sw = state

    LAl = _float(params, 'LAl')
    LAs = _float(params, 'LAs')
    LAcg = _float(params, 'LAcg')
    LW = _float(params, 'LW')
    LS = _float(params, 'LS')
    Grav = _float(params, 'Grav')
    mA = _float(params, 'mA')
    mW = _float(params, 'mW')
    mP = _float(params, 'mP')
    IA3 = _float(params, 'IA3')
    IW3 = _float(params, 'IW3')

    c_sq = np.cos(Sq)
    s_sq = np.sin(Sq)
    c_w = np.cos(Wq)
    s_w = np.sin(Wq)

    M11 = IA3 + IW3 + mA * LAcg**2 + mP * (LAl**2 + LS**2 + 2.0 * LAl * LS * c_sq) + mW * (LAs**2 + LW**2 + 2.0 * LAs * LW * c_w)
    M12 = IW3 + LW * mW * (LW + LAs * c_w)
    M13 = LS * mP * (LS + LAl * c_sq)
    M22 = IW3 + mW * LW**2
    M33 = mP * LS**2

    r1 = (
        Grav * LAcg * mA * np.sin(Aq)
        + Grav * mP * (LAl * np.sin(Aq) + LS * np.sin(Aq + Sq))
        - Grav * mW * (LAs * np.sin(Aq) + LW * np.sin(Aq + Wq))
        - LAl * LS * mP * s_sq * (Aw**2 - (Aw + Sw) ** 2)
        - LAs * LW * mW * s_w * (Aw**2 - (Aw + Ww) ** 2)
    )
    r2 = -LW * mW * (Grav * np.sin(Aq + Wq) + LAs * s_w * Aw**2)
    r3 = LS * mP * (Grav * np.sin(Aq + Sq) - LAl * s_sq * Aw**2)

    matrix = np.array([
        [M11, M12, M13],
        [M12, M22, 0.0],
        [M13, 0.0, M33],
    ], dtype=float)
    rhs = np.array([r1, r2, r3], dtype=float)

    try:
        Awd, Wwd, Swd = np.linalg.solve(matrix, rhs)
    except np.linalg.LinAlgError as exc:
        raise SimulationError('Stage 2 matrix became singular.') from exc

    return np.array([Aw, Ww, Sw, Awd, Wwd, Swd], dtype=float)


def post_release_ode(_t, state, params):
    Aq, Wq, Aw, Ww = state

    LAs = _float(params, 'LAs')
    LAcg = _float(params, 'LAcg')
    LW = _float(params, 'LW')
    Grav = _float(params, 'Grav')
    mA = _float(params, 'mA')
    mW = _float(params, 'mW')
    IA3 = _float(params, 'IA3')
    IW3 = _float(params, 'IW3')

    c_w = np.cos(Wq)
    s_w = np.sin(Wq)

    M11 = IA3 + IW3 + mA * LAcg**2 + mW * (LAs**2 + LW**2 + 2.0 * LAs * LW * c_w)
    M12 = IW3 + LW * mW * (LW + LAs * c_w)
    M22 = IW3 + mW * LW**2
    r1 = Grav * LAcg * mA * np.sin(Aq) - Grav * mW * (LAs * np.sin(Aq) + LW * np.sin(Aq + Wq)) - LAs * LW * mW * s_w * (Aw**2 - (Aw + Ww) ** 2)
    r2 = -LW * mW * (Grav * np.sin(Aq + Wq) + LAs * s_w * Aw**2)

    det = M11 * M22 - M12 * M12
    if abs(det) < EPS:
        raise SimulationError('Post-release matrix became singular.')

    Awd = (r1 * M22 - r2 * M12) / det
    Wwd = -(r1 * M12 - r2 * M11) / det
    return np.array([Aw, Ww, Awd, Wwd], dtype=float)


def projectile_kinematics(state, params):
    """Compute projectile position and velocity in VT convention.

    Position: height above ground (y=0 at ground, y>0 above).
    Velocity: vy positive upward.
    """
    Aq, Wq, Sq, Aw, Ww, Sw = state
    LAl = _float(params, 'LAl')
    LS = _float(params, 'LS')
    h = _float(params, 'h')

    x = -LAl * np.sin(Aq) - LS * np.sin(Aq + Sq)
    y = h + LAl * np.cos(Aq) + LS * np.cos(Aq + Sq)
    vx = -LAl * np.cos(Aq) * Aw - LS * np.cos(Aq + Sq) * (Aw + Sw)
    vy = -LAl * np.sin(Aq) * Aw - LS * np.sin(Aq + Sq) * (Aw + Sw)
    return x, y, vx, vy


def mechanism_points(Aq, Wq, Sq, params):
    """Convert angles to XY positions using VT convention.

    All positions are in world coordinates where:
    - X: horizontal (positive toward target/right)
    - Y: height above ground (0 = ground level)
    - Pivot is at (0, h)
    """
    LAl = _float(params, 'LAl')
    LAs = _float(params, 'LAs')
    LW = _float(params, 'LW')
    LS = _float(params, 'LS')
    h = _float(params, 'h')

    pivot = (0.0, h)
    # Arm tip (long arm end): relative to pivot = (-LAl*sin(Aq), LAl*cos(Aq))
    tip = (-LAl * np.sin(Aq), h + LAl * np.cos(Aq))
    # CW attachment (short arm end): relative to pivot = (LAs*sin(Aq), -LAs*cos(Aq))
    cw_attach = (LAs * np.sin(Aq), h - LAs * np.cos(Aq))
    # CW center of gravity
    weight = (
        LAs * np.sin(Aq) + LW * np.sin(Aq + Wq),
        h - LAs * np.cos(Aq) - LW * np.cos(Aq + Wq),
    )
    # Projectile
    projectile = (
        -LAl * np.sin(Aq) - LS * np.sin(Aq + Sq),
        h + LAl * np.cos(Aq) + LS * np.cos(Aq + Sq),
    )
    return {
        'pivot': pivot,
        'tip': tip,
        'cw_attach': cw_attach,
        'weight': weight,
        'projectile': projectile,
    }


def stage1_ground_reaction(_t, state, params):
    Aq, Wq, _Sq, Aw, Ww, _Sw = state
    Sq, Sw, Awd, _Wwd, Swd = stage1_components(Aq, Wq, Aw, Ww, params)
    LAl = _float(params, 'LAl')
    LS = _float(params, 'LS')
    mP = _float(params, 'mP')
    Grav = _float(params, 'Grav')

    total = Aq + Sq
    s_total = np.sin(total)
    c_total = np.cos(total)

    xdd = (
        LAl * np.sin(Aq) * Aw**2
        - LAl * np.cos(Aq) * Awd
        + LS * np.sin(total) * (Aw + Sw) ** 2
        - LS * np.cos(total) * (Awd + Swd)
    )

    if abs(s_total) < 1e-6:
        return 1.0

    tension = mP * xdd / s_total
    normal_force = mP * Grav - tension * c_total
    return normal_force


stage1_ground_reaction.terminal = True
stage1_ground_reaction.direction = -1


def lift_off_metric(state, params):
    Aq, Wq, Sq, Aw, Ww, Sw = state
    deriv = stage2_ode(0.0, np.array([Aq, Wq, Sq, Aw, Ww, Sw], dtype=float), params)
    Awd, Wwd, Swd = deriv[3:]
    LAl = _float(params, 'LAl')
    LS = _float(params, 'LS')
    total = Aq + Sq
    return (
        LAl * np.cos(Aq) * Aw**2
        + LAl * np.sin(Aq) * Awd
        + LS * np.cos(total) * (Aw + Sw) ** 2
        + LS * np.sin(total) * (Awd + Swd)
    )


def lift_off_event(_t, state, params):
    Aq, Wq, _Sq, Aw, Ww, _Sw = state
    Sq = ground_sling_angle(Aq, params)
    Sw = stage1_sq_dot(Aq, Sq, Aw, params)
    return lift_off_metric(np.array([Aq, Wq, Sq, Aw, Ww, Sw], dtype=float), params)


lift_off_event.terminal = True
lift_off_event.direction = 1


def release_event(_t, state, params):
    _x, _y, vx, vy = projectile_kinematics(state, params)
    speed = np.hypot(vx, vy)
    if vx <= 0.0 or speed < 0.5:
        return -1.0
    angle = np.arctan2(vy, vx)
    return angle - _float(params, 'releaseAngleRad')


release_event.terminal = True
release_event.direction = -1


def flight_ode(_t, state, params):
    x, y, vx, vy = state
    Grav = _float(params, 'Grav')
    mP = _float(params, 'mP')
    wind_speed = _float(params, 'windSpeed')

    if not params.get('enableAirDrag', False):
        drag_term = 0.0
    else:
        rho = _float(params, 'airDensity')
        Cd = _float(params, 'dragCoefficient')
        area = _float(params, 'projectileArea')
        rel_x = vx - wind_speed
        speed = np.hypot(rel_x, vy)
        drag_term = rho * Cd * area * speed / (2.0 * mP)

    ax = -drag_term * (vx - wind_speed)
    ay = -Grav - drag_term * vy
    return np.array([vx, vy, ax, ay], dtype=float)


def impact_event(_t, state, _params):
    return state[1]


impact_event.terminal = True
impact_event.direction = -1


def validate_geometry(params):
    Aq0 = _float(params, 'Aq0')
    LAl = _float(params, 'LAl')
    LS = _float(params, 'LS')
    h = _float(params, 'h')
    # Ground constraint: cos(Aq+Sq) = (-h - LAl*cos(Aq)) / LS must be in [-1, 1]
    reach = abs((-h - LAl * np.cos(Aq0)) / LS)
    if reach > 1.0:
        raise SimulationError('Initial geometry cannot place the projectile on the ground. Increase sling length, lower the pivot, or reduce the initial arm angle.')


def solve_bundle(params):
    validate_geometry(params)

    Aq0 = _float(params, 'Aq0')
    Wq0 = _float(params, 'Wq0')
    Sq0 = ground_sling_angle(Aq0, params)
    y0_stage1 = np.array([Aq0, Wq0, Sq0, 0.0, 0.0, 0.0], dtype=float)

    stage1 = solve_ivp(
        stage1_ode,
        (0.0, _float(params, 'maxTime')),
        y0_stage1,
        args=(params,),
        events=(lift_off_event,),
        method='RK45',
        max_step=_float(params, 'maxStep'),
        rtol=1e-6,
        atol=1e-8,
        dense_output=True,
    )
    if stage1.status < 0:
        raise SimulationError(stage1.message)
    initial_metric = lift_off_metric(np.array([Aq0, Wq0, Sq0, 0.0, 0.0, 0.0], dtype=float), params)
    if initial_metric >= 0.0:
        t_liftoff = 0.0
        Aq_l, Wq_l, Sq_l, Aw_l, Ww_l, Sw_l = y0_stage1
    elif len(stage1.t_events[0]) == 0:
        raise SimulationError('Projectile never lifted off the ground within the simulation time.')
    else:
        t_liftoff = float(stage1.t_events[0][0])
        lift_state = stage1.sol(t_liftoff)
        Aq_l, Wq_l, _Sq_l, Aw_l, Ww_l, _Sw_l = np.asarray(lift_state, dtype=float)
        Sq_l = ground_sling_angle(Aq_l, params)
        Sw_l = stage1_sq_dot(Aq_l, Sq_l, Aw_l, params)

    y0_stage2 = np.array([Aq_l, Wq_l, Sq_l, Aw_l, Ww_l, Sw_l], dtype=float)
    stage2 = solve_ivp(
        stage2_ode,
        (t_liftoff, _float(params, 'maxTime')),
        y0_stage2,
        args=(params,),
        events=(release_event,),
        method='RK45',
        max_step=_float(params, 'maxStep'),
        rtol=1e-6,
        atol=1e-8,
        dense_output=True,
    )
    if stage2.status < 0:
        raise SimulationError(stage2.message)
    if len(stage2.t_events[0]) == 0:
        raise SimulationError('Release angle was never reached. Try a lower release angle or different geometry.')

    t_release = float(stage2.t_events[0][0])
    release_state = np.asarray(stage2.sol(t_release), dtype=float)
    release_x, release_y, release_vx, release_vy = projectile_kinematics(release_state, params)

    y0_post = np.array([release_state[0], release_state[1], release_state[3], release_state[4]], dtype=float)
    t_flight_end = t_release + _float(params, 'maxFlightTime')

    post_release = solve_ivp(
        post_release_ode,
        (t_release, t_flight_end),
        y0_post,
        args=(params,),
        method='RK45',
        max_step=_float(params, 'maxStep'),
        rtol=1e-6,
        atol=1e-8,
        dense_output=True,
    )
    if post_release.status < 0:
        raise SimulationError(post_release.message)

    flight = solve_ivp(
        flight_ode,
        (t_release, t_flight_end),
        np.array([release_x, release_y, release_vx, release_vy], dtype=float),
        args=(params,),
        events=(impact_event,),
        method='RK45',
        max_step=max(_float(params, 'maxStep'), 0.002),
        rtol=1e-6,
        atol=1e-8,
        dense_output=True,
    )
    if flight.status < 0:
        raise SimulationError(flight.message)
    if len(flight.t_events[0]) == 0:
        raise SimulationError('Projectile did not land within the flight time limit.')

    t_land = float(flight.t_events[0][0])
    initial_x = mechanism_points(Aq0, Wq0, Sq0, params)['projectile'][0]

    return StageBundle(
        stage1=stage1,
        stage2=stage2,
        post_release=post_release,
        flight=flight,
        t_liftoff=t_liftoff,
        t_release=t_release,
        t_land=t_land,
        release_state=release_state,
        release_pos=(release_x, release_y),
        release_vel=(release_vx, release_vy),
        initial_projectile_x=initial_x,
    )


def sample_frames(bundle, params):
    fps = int(params['fps'])
    dt = 1.0 / fps
    times = np.arange(0.0, bundle.t_land + dt * 0.5, dt)
    if times[-1] < bundle.t_land:
        times = np.append(times, bundle.t_land)

    arm_tip_x = np.empty_like(times)
    arm_tip_y = np.empty_like(times)
    cw_attach_x = np.empty_like(times)
    cw_attach_y = np.empty_like(times)
    weight_x = np.empty_like(times)
    weight_y = np.empty_like(times)
    projectile_x = np.empty_like(times)
    projectile_y = np.empty_like(times)
    projectile_attached = np.empty(times.shape[0], dtype=bool)

    release_index = 0
    for idx, t in enumerate(times):
        if t <= bundle.t_liftoff + 1e-9:
            state = np.asarray(bundle.stage1.sol(t), dtype=float)
            Aq, Wq, _Sq, Aw, Ww, _Sw = state
            Sq = ground_sling_angle(Aq, params)
        elif t <= bundle.t_release + 1e-9:
            state = np.asarray(bundle.stage2.sol(t), dtype=float)
            Aq, Wq, Sq, Aw, Ww, Sw = state
        else:
            mech = np.asarray(bundle.post_release.sol(min(t, bundle.t_land)), dtype=float)
            Aq, Wq, Aw, Ww = mech
            Sq = bundle.release_state[2]

        points = mechanism_points(Aq, Wq, Sq, params)
        arm_tip_x[idx], arm_tip_y[idx] = points['tip']
        cw_attach_x[idx], cw_attach_y[idx] = points['cw_attach']
        weight_x[idx], weight_y[idx] = points['weight']

        if t <= bundle.t_release + 1e-9:
            projectile_attached[idx] = True
            projectile_x[idx], projectile_y[idx] = points['projectile']
            release_index = idx
        else:
            projectile_attached[idx] = False
            px, py, _pvx, _pvy = np.asarray(bundle.flight.sol(min(t, bundle.t_land)), dtype=float)
            projectile_x[idx] = px
            projectile_y[idx] = py

    release_angle_actual = np.rad2deg(np.arctan2(bundle.release_vel[1], bundle.release_vel[0]))
    range_travelled = float(projectile_x[-1] - bundle.initial_projectile_x)
    landing_x = float(projectile_x[-1])
    max_height = float(np.max(projectile_y))

    return {
        'times': times.tolist(),
        'fps': fps,
        'releaseIndex': int(release_index),
        'frames': {
            'armTipX': arm_tip_x.tolist(),
            'armTipY': arm_tip_y.tolist(),
            'cwAttachX': cw_attach_x.tolist(),
            'cwAttachY': cw_attach_y.tolist(),
            'weightX': weight_x.tolist(),
            'weightY': weight_y.tolist(),
            'projectileX': projectile_x.tolist(),
            'projectileY': projectile_y.tolist(),
            'projectileAttached': projectile_attached.astype(int).tolist(),
            'pivotX': [0.0] * len(times),
            'pivotY': [_float(params, 'h')] * len(times),
        },
        'stats': {
            'range': range_travelled,
            'landingX': landing_x,
            'maxHeight': max_height,
            'releaseSpeed': float(np.hypot(*bundle.release_vel)),
            'releaseAngleActual': float(release_angle_actual),
            'flightTime': float(bundle.t_land - bundle.t_release),
            'liftOffTime': float(bundle.t_liftoff),
            'releaseTime': float(bundle.t_release),
            'totalTime': float(bundle.t_land),
        },
    }


def simulate(raw_params=None):
    try:
        params = normalize_params(raw_params)
        bundle = solve_bundle(params)
        sampled = sample_frames(bundle, params)
        sampled['ok'] = True
        sampled['params'] = params
        return sampled
    except Exception as exc:  # noqa: BLE001
        return {
            'ok': False,
            'error': str(exc),
        }


if __name__ == '__main__':
    result = simulate()
    print(json.dumps(result['stats'] if result.get('ok') else result, indent=2))
