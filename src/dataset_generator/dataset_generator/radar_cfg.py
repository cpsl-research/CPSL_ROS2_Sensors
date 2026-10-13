"""Derived radar specs from a TI mmWave ``.cfg`` text (stdlib only, no ROS).

Stored next to the raw cfg text in every recorded ADC stream so a capture
describes itself. This is a deliberately small copy of the TDM, single-chip
headline numbers of ``radar_gui.cfg.metrics`` in CPSL_TI_Radar (its docstring
holds the formulas); ``test/test_radar_cfg.py`` pins the two together. The
stored cfg text lets anyone recompute the full set with radar_gui.

Scope: single-chip TDM (one frameCfg, profile = the first profileCfg, TX
slots counted as radar_gui does for an IWR1843-class board). Cascade
(10-token frameCfg) and advFrameCfg cfgs return ``{"error": ...}``.
``parse_and_compute`` never raises.
"""
import math

CALCULATOR = 'dataset_generator.radar_cfg'
VERSION = '1'
C = 299_792_458.0
USABLE_IF = 0.9
LVDS_DATAFMT2_META_BYTES = 64
_AZ_TX_MASK = 0b101      # TX1 + TX3 azimuth pair (single-chip EVM)
_ELEV_BIT = 0b010


class CfgError(ValueError):
    pass


def _parse(text):
    cmds = {}
    for raw in text.splitlines():
        line = raw.split('%', 1)[0].strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        try:
            args = [float(a) for a in parts[1:]]
        except ValueError:
            args = None      # non-numeric command (e.g. a string option)
        cmds.setdefault(parts[0], []).append((parts[1:], args))
    return cmds


def _first(cmds, name, need=0):
    for raw, args in cmds.get(name, ()):
        if args is None:
            raise CfgError('%s: non-numeric field' % name)
        if len(args) < need:
            raise CfgError('%s needs %d fields, got %d' %
                           (name, need, len(args)))
        return args
    return None


def _popcount(x):
    return bin(int(x) & 0xFFFFFFFF).count('1')


def _pow2(x):
    return 1 if x <= 1 else 1 << (x - 1).bit_length()


def _tdm_slots(masks, bpm):
    """(n_TX slots per loop, azimuth TX count); mirrors radar_gui tdm_slots."""
    used = 0
    for m in masks:
        used |= m
    if bpm:
        return 2, 2
    if all(_popcount(m) > 1 for m in masks):
        return 1, 1
    az = _popcount(used & _AZ_TX_MASK)
    n_tx = az + (1 if used & _ELEV_BIT else 0)
    if n_tx == 0:
        n_tx = max(1, _popcount(used))
    return n_tx, max(1, az)


def compute(text):
    """cfg text -> dict of specs. Raises CfgError on missing/malformed input."""
    cmds = _parse(text)
    if 'advFrameCfg' in cmds:
        raise CfgError('advFrameCfg is not supported by this calculator')
    fr = _first(cmds, 'frameCfg')
    if fr is None:
        raise CfgError('missing frameCfg')
    if len(fr) >= 9:
        raise CfgError('cascade frameCfg is not supported by this calculator')
    if len(fr) < 7:
        raise CfgError('frameCfg needs 7 fields, got %d' % len(fr))
    start, end, loops, frames, period_ms = (int(fr[0]), int(fr[1]),
                                            int(fr[2]), int(fr[3]), fr[4])
    p = _first(cmds, 'profileCfg', 12)
    if p is None:
        raise CfgError('missing profileCfg')
    start_ghz, idle, adc_start, ramp, slope = p[1], p[2], p[3], p[4], p[7]
    n, fs = int(p[9]), p[10]
    if fs <= 0 or slope <= 0 or n <= 0:
        raise CfgError('profileCfg: slope, numAdcSamples and sample rate '
                       'must be positive')
    sampling_us = n * 1000.0 / fs
    bw = slope * sampling_us
    fc_ghz = start_ghz + (slope * adc_start + bw / 2) / 1e3
    lam = C / (fc_ghz * 1e9)
    tc = idle + ramp

    adc = _first(cmds, 'adcCfg')
    adc_fmt = int(adc[1]) if adc is not None and len(adc) >= 2 else 1
    bps = 2 if adc_fmt == 0 else 4

    ch = _first(cmds, 'channelCfg')
    if ch is None or len(ch) < 2:
        raise CfgError('missing or short channelCfg')
    n_rx = _popcount(ch[0])
    if n_rx == 0:
        raise CfgError('channelCfg enables no RX channel')

    cpl = end - start + 1
    if cpl < 1 or loops < 1:
        raise CfgError('frameCfg: need at least one chirp and one loop')
    masks = {}
    for _raw, a in cmds.get('chirpCfg', ()):
        if a is None or len(a) < 8:
            raise CfgError('chirpCfg needs 8 numeric fields')
        for i in range(int(a[0]), int(a[1]) + 1):
            if start <= i <= end:
                masks[i] = masks.get(i, 0) | int(a[7])
    missing = [i for i in range(start, end + 1) if i not in masks]
    if missing:
        raise CfgError('frameCfg uses chirp(s) %s with no chirpCfg' % missing)
    seq = [masks[i] for i in range(start, end + 1)]
    bpm = False
    for _raw, a in cmds.get('bpmCfg', ()):
        if a is not None and len(a) >= 2 and int(a[1]) != 0:
            bpm = True
    n_tx, az_tx = _tdm_slots(seq, bpm)
    n_az = az_tx * n_rx

    n_chirps = cpl * loops
    loop_us = n_tx * tc
    max_v = lam / (4 * loop_us * 1e-6)
    vel_res = lam / (2 * n_chirps * tc * 1e-6)
    bins = _pow2(-(-n_chirps // n_tx))
    step = lam / (2 * bins * loop_us * 1e-6)
    ideal = C * fs * 1e3 / (2 * slope * 1e12)
    if adc_fmt == 0:
        ideal /= 2
    lv = _first(cmds, 'lvdsStreamCfg')
    lv_fmt = int(lv[2]) if lv is not None and len(lv) >= 3 else None
    per_chirp = n * n_rx * bps + (LVDS_DATAFMT2_META_BYTES
                                  if lv_fmt == 2 else 0)
    return {
        'scheme': 'tdm',
        'calculator': CALCULATOR,
        'calculator_version': VERSION,
        'start_ghz': start_ghz, 'slope_mhz_us': slope, 'idle_us': idle,
        'adc_start_us': adc_start, 'ramp_us': ramp, 'num_samples': n,
        'sample_rate_ksps': fs, 'bandwidth_mhz': bw, 'center_ghz': fc_ghz,
        'lambda_mm': lam * 1e3, 'chirp_us': tc, 'sampling_us': sampling_us,
        'n_rx': n_rx, 'n_tx': n_tx, 'n_virtual': n_tx * n_rx,
        'n_az_virtual': n_az, 'chirps_per_loop': cpl, 'n_loops': loops,
        'n_chirps': n_chirps,
        'range_res_m': C / (2 * bw * 1e6),
        'max_range_m': USABLE_IF * ideal, 'max_range_ideal_m': ideal,
        'velocity_res_ms': vel_res, 'max_velocity_ms': max_v,
        'azimuth_res_deg': math.degrees(2.0 / n_az),
        'loop_period_us': loop_us, 'doppler_bins': bins,
        'doppler_step_ms': step,
        'frame_period_ms': period_ms,
        'frame_rate_hz': 1000.0 / period_ms if period_ms > 0 else 0.0,
        'num_frames': frames,
        'bytes_per_sample': bps, 'bytes_per_chirp': per_chirp,
        'bytes_per_frame': per_chirp * n_chirps,
        'lvds_data_fmt': lv_fmt,
    }


def parse_and_compute(text):
    """Like :func:`compute` but never raises: failures -> ``{"error": str}``."""
    try:
        return compute(text)
    except Exception as e:  # noqa: BLE001 - recorder must not die on a cfg
        return {'error': '%s: %s' % (type(e).__name__, e),
                'calculator': CALCULATOR, 'calculator_version': VERSION}
