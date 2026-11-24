"""TC-1 thermal and electrical budget (lumped-parameter model).

Everything here is an ESTIMATE until measured on the first prototype (test plan in report.json).
Model: three nodes (block incl. plate + liquid, heat sink, lid), Peltier modules described by the
standard datasheet relations derived from (Imax, Vmax, dTmax) at Th = 300 K:

    alpha = Vmax / Th                          (module Seebeck coefficient, V/K)
    R     = Vmax (Th - dTmax) / (Imax Th)      (electrical resistance, ohm)
    K     = Imax Vmax (Th - dTmax) / (2 Th dTmax)   (thermal conductance, W/K)
    Q_block_out = alpha I Tb - I^2 R / 2 - K (Ts - Tb)     (I > 0 cools the block; formula holds for I < 0)
    Q_sink_in   = alpha I Ts + I^2 R / 2 - K (Ts - Tb)
    V = alpha (Ts - Tb) + I R

with Tb, Ts the module face temperatures (K) after the block-side and sink-side interface resistances.
Heat-sink thermal resistance from a ducted parallel-plate fin model (Hausen laminar Nusselt number,
fin efficiency, spreading resistance, air heating via effectiveness) at the operating point of a
linearised fan curve.  These relations are textbook approximations; the datasheet of the module
actually bought replaces the module numbers below.
"""
from __future__ import annotations

import math

T0K = 273.15

# ---------------- parameters (all ESTIMATES unless stated) ----------------
TH = {
    "t_amb_c": 24.0,
    # Peltier module: 127 couples, 50 x 50 mm, cycling-rated (see bought_parts)
    "tec_n": 2, "tec_series": True, "tec_imax": 15.0, "tec_vmax": 15.4, "tec_dtmax": 68.0,
    "tec_size_mm": 50.0, "tec_t_mm": 4.0,
    "tec_i_limit": 13.0,                    # driver current limit per string (A), set below Imax for module life
    "supply_v": None,                       # filled from params.SUPPLY_V
    # interfaces per module: block side graphite sheet, sink side thermal grease (K/W per module)
    "r_if_block": 0.018, "r_if_sink": 0.010,
    # parasitic conductances (W/K)
    "g_block_amb": 0.15,                    # block sides via 2 mm air gap + top plate + exposed plate area
    "g_block_sink": 0.07,                   # foam under the block outside the TECs + 6 stainless clamp screws
    "g_lid_block": 0.30,                    # heated lid -> pad -> plate web/air -> wells -> block (lid closed)
    "g_lid_amb": 0.25,                      # lid plate -> PEEK spacers/insulation -> hood -> room
    # thermal masses
    "c_al": 900.0, "rho_al": 2700.0,
    "plate_mass_kg": 0.014, "c_pp": 1920.0,     # 96-well PCR plate (PP), nominal
    "liquid_ul_per_well": 50.0,                 # worst case of the 20-50 uL range
    "tec_ceramic_block_side_kg_each": 0.0076,   # 50x50x0.8 mm alumina
    "thinwall_block_mass_kg": 0.17,             # PCR-instrument spare block (thin-wall, silver/anodised): option
    "lid_heater_w": 80.0, "lid_set_c": 105.0, "c_ss": 500.0,
    # fans: 2 x 80x80x25 mm 24 V high-speed, linearised curve per fan (ESTIMATE, check fan datasheet)
    "fan_n": 2, "fan_q_free_m3s": 0.019, "fan_p_max_pa": 60.0, "fan_a_each": 0.25,
    "duct_k_minor": 3.0,                        # turn into the fins + entrance + exit, in velocity heads
    "air_rho": 1.16, "air_cp": 1007.0, "air_k": 0.0265, "air_nu": 1.6e-5, "air_pr": 0.71,
    "k_al_sink": 200.0,
    # control model
    "kp_v_per_k": 6.0, "ki_v_per_ks": 0.4, "dt_s": 0.05,
    # design limits
    "sink_target_max_c": 45.0, "sink_cutout_c": 75.0, "block_cutout_c": 110.0, "lid_cutout_c": 130.0,
    # other loads on 24 V (A, ESTIMATES)
    "i_fans": 0.5, "i_steppers_supply": 0.9, "i_logic": 0.3,
    "machine_shared_budget_a": 12.0,
}


def tec_constants(p=TH):
    th = 300.0
    a = p["tec_vmax"] / th
    r = p["tec_vmax"] * (th - p["tec_dtmax"]) / (p["tec_imax"] * th)
    k = p["tec_imax"] * p["tec_vmax"] * (th - p["tec_dtmax"]) / (2 * th * p["tec_dtmax"])
    qmax = a * p["tec_imax"] * th - 0.5 * p["tec_imax"] ** 2 * r
    return {"alpha_v_per_k": a, "r_ohm": r, "k_w_per_k": k, "qmax_w_at_dT0": qmax}


def heat_sink(sink: dict, p=TH) -> dict:
    """Ducted parallel-plate fin sink. sink: base_l (along flow), base_w, fin_h, fin_t, pitch, base_t (mm)."""
    L, W, H = sink["fin_len"] / 1e3, sink["base_w"] / 1e3, sink["fin_h"] / 1e3
    t, s = sink["fin_t"] / 1e3, sink["pitch"] / 1e3
    n_fin = int(W / s)
    g = s - t
    n_ch = n_fin - 1
    a_flow = n_ch * g * H
    dh = 2 * g * H / (g + H)
    # fan / system operating point: dp_fan = pmax (1 - Q/Qfree) per fan in parallel
    qf, pm = p["fan_n"] * p["fan_q_free_m3s"], p["fan_p_max_pa"]
    rho, nu = p["air_rho"], p["air_nu"]

    def dp_sys(q):
        v = q / a_flow
        re = v * dh / nu
        f = 96.0 / max(re, 1.0)
        a_fan = p["fan_n"] * p["fan_a_each"] * 0.08 ** 2 * 4      # effective fan annulus (approx.)
        return f * (L / dh) * 0.5 * rho * v * v + p["duct_k_minor"] * 0.5 * rho * v * v \
            + 1.0 * 0.5 * rho * (q / a_fan) ** 2

    lo, hi = 0.0, qf
    for _ in range(60):
        q = 0.5 * (lo + hi)
        if pm * (1 - q / qf) > dp_sys(q):
            lo = q
        else:
            hi = q
    q = 0.5 * (lo + hi)
    v = q / a_flow
    re = v * dh / nu
    gz = (dh / L) * re * p["air_pr"]
    nu_h = 7.54 + 0.03 * gz / (1 + 0.016 * gz ** (2 / 3))         # Hausen, developing laminar flow
    h = nu_h * p["air_k"] / dh
    m = math.sqrt(2 * h / (p["k_al_sink"] * t))
    eta = math.tanh(m * H) / (m * H)
    a_fin = n_fin * 2 * H * L
    a_base = (W - n_fin * t) * L
    hA = h * (eta * a_fin + a_base)
    mcp = q * rho * p["air_cp"]
    ntu = hA / mcp
    r_conv = 1.0 / (mcp * (1 - math.exp(-ntu)))
    a_src = p["tec_n"] * (p["tec_size_mm"] / 1e3) ** 2
    eps = math.sqrt(a_src / (W * L))
    r_sp = (1 - eps) ** 1.5 / (math.sqrt(math.pi) * p["k_al_sink"] * math.sqrt(a_src / math.pi))
    return {"fins": n_fin, "gap_mm": round(g * 1e3, 2), "airflow_m3s": round(q, 4), "airflow_cfm": round(q * 2118.9, 1),
            "channel_velocity_ms": round(v, 2), "reynolds": round(re), "dp_pa": round(dp_sys(q), 1),
            "h_w_m2k": round(h, 1), "fin_efficiency": round(eta, 3), "hA_w_k": round(hA, 2),
            "r_convective_k_w": round(r_conv, 4), "r_spreading_k_w": round(r_sp, 4),
            "r_sink_to_air_k_w": round(r_conv + r_sp, 4)}


class Model:
    def __init__(self, c_block, c_sink, r_sink, c_lid, p=TH):
        self.p, self.tc = p, tec_constants(p)
        self.cb, self.cs, self.rs, self.cl = c_block, c_sink, r_sink, c_lid
        self.vmod = min(p["supply_v"] / p["tec_n"] if p["tec_series"] else p["supply_v"], p["tec_vmax"])

    def tec(self, tb, ts, v_cmd):
        """Per-module heat flows for commanded module voltage (v_cmd > 0 cools the block)."""
        a, r, k = self.tc["alpha_v_per_k"], self.tc["r_ohm"], self.tc["k_w_per_k"]
        p = self.p
        fb, fs = tb, ts
        i = 0.0
        for _ in range(4):
            i = (v_cmd - a * (fs - fb)) / r
            i = max(-p["tec_i_limit"], min(p["tec_i_limit"], i))
            qb = a * i * fb - 0.5 * i * i * r - k * (fs - fb)
            qs = a * i * fs + 0.5 * i * i * r - k * (fs - fb)
            fb = tb - qb * p["r_if_block"]
            fs = ts + qs * p["r_if_sink"]
        v = a * (fs - fb) + i * r
        return qb, qs, i, v

    def run(self, program, tb0, ts0, lid_c=105.0, lid_closed=True, record_every=1.0):
        """program: list of (setpoint_c, hold_s). Hold time counts from reaching setpoint +/- 0.5 K."""
        p = self.p
        dt = p["dt_s"]
        tamb = p["t_amb_c"] + T0K
        tb, ts = tb0 + T0K, ts0 + T0K
        tl = lid_c + T0K
        t, log, steps = 0.0, [], []
        e_tec = 0.0
        integ = 0.0
        for sp, hold in program:
            spk = sp + T0K
            t_start, t_reach, t_hold_end = t, None, None
            tb_start = tb
            while True:
                err = tb - spk
                v_raw = p["kp_v_per_k"] * err + p["ki_v_per_ks"] * integ
                v = max(-self.vmod, min(self.vmod, v_raw))
                if v == v_raw or (v_raw > v) != (err > 0):     # PI with conditional-integration anti-windup
                    integ += err * dt
                qb, qs, i, vm = self.tec(tb, ts, v)
                n = p["tec_n"]
                q_par_bs = p["g_block_sink"] * (tb - ts)
                dtb = (-n * qb + (p["g_lid_block"] * (tl - tb) if lid_closed else 0.0)
                       + p["g_block_amb"] * (tamb - tb) - q_par_bs) / self.cb
                dts = (n * qs + q_par_bs - (ts - tamb) / self.rs) / self.cs
                tb += dtb * dt
                ts += dts * dt
                t += dt
                e_tec += n * abs(i * vm) * dt
                if t_reach is None and abs(tb - spk) <= 0.5:
                    t_reach = t
                    t_hold_end = t + hold
                if int(t / record_every) != int((t - dt) / record_every):
                    log.append((round(t, 2), round(tb - T0K, 3), round(ts - T0K, 3), round(i, 3), round(n * i * vm, 1),
                                round(n * qs + q_par_bs, 1)))
                if t_hold_end is not None and t >= t_hold_end:
                    break
                if t - t_start > 4000:
                    break
            steps.append({"from_c": round(tb_start - T0K, 2), "to_c": sp, "ramp_s": round((t_reach or t) - t_start, 1),
                          "rate_c_per_s": round((abs(sp + T0K - tb_start) - 0.5) / max((t_reach or t) - t_start, 1e-6), 3),
                          "sink_c_end": round(ts - T0K, 2)})
        return {"steps": steps, "log": log, "t_end_s": t, "e_tec_j": e_tec, "tb_c": tb - T0K, "ts_c": ts - T0K}

    def steady(self, sp, lid_c, lid_closed=True, t_s=2400.0):
        r = self.run([(sp, t_s)], sp, self.p["t_amb_c"] + 5, lid_c=lid_c, lid_closed=lid_closed)
        last = r["log"][-30:]
        i = sum(x[3] for x in last) / len(last)
        pw = sum(x[4] for x in last) / len(last)
        qs = sum(x[5] for x in last) / len(last)
        return {"setpoint_c": sp, "lid_c": lid_c if lid_closed else "open", "tec_current_a": round(i, 2),
                "tec_electrical_w": round(pw, 1), "heat_to_sink_w": round(qs, 1), "sink_c": round(r["ts_c"], 1),
                "block_c": round(r["tb_c"], 2)}


def lid_warmup(c_lid, p=TH):
    t, tl, dt = 0.0, p["t_amb_c"], 0.1
    while tl < p["lid_set_c"] - 1.0 and t < 3600:
        tl += (p["lid_heater_w"] - p["g_lid_amb"] * (tl - p["t_amb_c"])) / c_lid * dt
        t += dt
    hold_w = p["g_lid_amb"] * (p["lid_set_c"] - p["t_amb_c"])
    return {"warmup_s_to_104c": round(t), "hold_w_lid_open_or_no_block": round(hold_w, 1),
            "hold_w_closed_on_block_at_4c": round(hold_w + p["g_lid_block"] * (p["lid_set_c"] - 4.0), 1),
            "heater_w": p["lid_heater_w"], "heater_current_a": round(p["lid_heater_w"] / p["supply_v"], 2)}


PCR = ([(98.0, 30.0)] + [(98.0, 10.0), (67.0, 30.0), (72.0, 60.0)] * 12 + [(72.0, 120.0), (4.0, 600.0)])


def budget(c_block_solid, c_extra, c_sink, sink_geom, c_lid, p=TH) -> dict:
    hs = heat_sink(sink_geom, p)
    rs = hs["r_sink_to_air_k_w"]
    tc = tec_constants(p)
    out = {"status": "ESTIMATES from a lumped model; measure on prototype (tests T-TC-01..08)",
           "tec_model": {k: round(v, 4) for k, v in tc.items()}, "heat_sink": hs, "cases": {}}
    p_par = dict(p, tec_series=False)
    c_thin = p["thinwall_block_mass_kg"] * p["c_al"] + c_extra
    for tag, cb, pp in (("generic_solid_block", c_block_solid + c_extra, p),
                        ("thinwall_pcr_block", c_thin, p),
                        ("thinwall_block_parallel_drive_2_bridges", c_thin, p_par)):
        m = Model(cb, c_sink, rs, c_lid, pp)
        pcr = m.run(PCR, 24.0, p["t_amb_c"] + 2)
        st = pcr["steps"]
        up = [s for s in st if s["from_c"] < 70 and s["to_c"] == 98.0]
        dn = [s for s in st if s["from_c"] > 95 and s["to_c"] == 67.0]
        to4 = [s for s in st if s["to_c"] == 4.0][0]
        smax = max(x[2] for x in pcr["log"])
        imax = max(abs(x[3]) for x in pcr["log"])
        pmax = max(abs(x[4]) for x in pcr["log"])
        qsink_cyc = [x[5] for x in pcr["log"] if 40 < x[0] < pcr["t_end_s"] - 900]
        q_avg = sum(qsink_cyc) / len(qsink_cyc)
        out["cases"][tag] = {
            "heat_capacity_block_side_j_per_k": round(cb, 1),
            "ramp_67_to_98_c_per_s": round(sum(s["rate_c_per_s"] for s in up) / len(up), 2),
            "ramp_98_to_67_c_per_s": round(sum(s["rate_c_per_s"] for s in dn) / len(dn), 2),
            "ramp_72_to_4_c_per_s": to4["rate_c_per_s"], "time_72_to_4_s": to4["ramp_s"],
            "pcr_12_cycle_total_min": round(pcr["t_end_s"] / 60 - 10, 1),
            "sink_peak_c_during_pcr": round(smax, 1), "tec_peak_current_a": round(imax, 1),
            "drive": "series pair, one H-bridge" if pp["tec_series"] else "each module on its own H-bridge (<= Vmax)",
            "tec_peak_electrical_w": round(pmax, 0), "tec_peak_supply_a": round(pmax / pp["supply_v"], 1), "tec_energy_pcr_wh": round(pcr["e_tec_j"] / 3600, 1),
            "avg_heat_to_sink_during_cycling_w": round(q_avg, 1),
            "required_sink_r_k_w_for_target": round((p["sink_target_max_c"] - p["t_amb_c"]) / q_avg, 4),
            "steady_4c_lid_105": m.steady(4.0, 105.0),
            "steady_4c_lid_idle_30": m.steady(4.0, 30.0),
            "steady_98c_lid_105": m.steady(98.0, 105.0),
            "steady_4c_lid_open": m.steady(4.0, 105.0, lid_closed=False),
        }
    out["lid"] = lid_warmup(c_lid, p)
    return out
