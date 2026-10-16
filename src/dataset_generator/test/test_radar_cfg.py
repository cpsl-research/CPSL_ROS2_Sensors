"""
radar_cfg calculator checks (no ROS, no hardware).

The parity tests compare against radar_gui.cfg.metrics from CPSL_TI_Radar and
are skipped when that tree is not found (set CPSL_TI_RADAR_SRC to its path).
"""
import glob
import json
import os
import sys

from dataset_generator import radar_cfg
import pytest

_TIR_CANDIDATES = [
    os.environ.get('CPSL_TI_RADAR_SRC', ''),
    '/home/rover/Documents/radar_fold_ws/src/CPSL_TI_Radar',
]
_TIR = next((p for p in _TIR_CANDIDATES
             if p and os.path.isdir(os.path.join(p, 'radar_gui'))), None)

SAR = """\
% comment
sensorStop
channelCfg 1 1 0
adcCfg 2 1
profileCfg 0 77.25 480 10 1520 0 0 2.333 1 3300 2200 0 0 30
chirpCfg 0 0 0 0 0 0 0 1
frameCfg 0 0 255 0 510.3 1 0
lvdsStreamCfg -1 0 2 0
sensorStart
"""


def test_sar_inline_values():
    m = radar_cfg.parse_and_compute(SAR)
    assert m['scheme'] == 'tdm' and 'error' not in m
    assert m['n_rx'] == 1 and m['n_tx'] == 1
    assert m['num_samples'] == 3300 and m['n_loops'] == 255
    assert m['lvds_data_fmt'] == 2
    assert m['bytes_per_frame'] == (3300 * 1 * 4 + 64) * 255
    assert m['frame_rate_hz'] == pytest.approx(1000 / 510.3)
    json.dumps(m)       # must be JSON-serialisable


@pytest.mark.parametrize('text', ['', 'garbage\nnot a cfg 1 2', 'frameCfg a b',
                                  'frameCfg 0 0 1 0 100 1 0\n'])
def test_garbage_gives_error_dict(text):
    m = radar_cfg.parse_and_compute(text)
    assert 'error' in m and isinstance(m['error'], str)


def test_unsupported_variants_report_error():
    assert 'error' in radar_cfg.parse_and_compute(
        SAR + 'advFrameCfg 1 0 0 1 0\n')
    assert 'error' in radar_cfg.parse_and_compute(
        SAR.replace('frameCfg 0 0 255 0 510.3 1 0',
                    'frameCfg 0 0 255 0 1 510.3 1 0 0'))


def _tir_cfgs():
    if not _TIR:
        return []
    cfgs = sorted(glob.glob(os.path.join(
        _TIR, 'CPSL_TI_Radar_cpp', 'config', 'radar', '**', '*.cfg'),
        recursive=True))
    sar = [c for c in cfgs if c.endswith('SAR_2ms_fmt2.cfg')]
    return sar + [c for c in cfgs if c not in sar]


@pytest.mark.skipif(_TIR is None, reason='CPSL_TI_Radar source not found')
def test_parity_with_radar_gui_metrics():
    sys.path.insert(0, _TIR)
    try:
        from radar_gui.cfg.metrics import metrics as gui_metrics
        from radar_gui.cfg.parse import CfgError, parse_cfg
    finally:
        sys.path.remove(_TIR)
    cfgs = _tir_cfgs()
    assert any(c.endswith('SAR_2ms_fmt2.cfg') for c in cfgs)
    compared = 0
    for path in cfgs:
        text = open(path, errors='replace').read()
        mine = radar_cfg.parse_and_compute(text)
        try:
            ref = gui_metrics(parse_cfg(text), None, 'tdm').to_dict()
        except CfgError:
            ref = None
        if ref is None or 'error' in mine:
            # out of the calculator's scope; the reference must also differ
            # only because it is a cascade/advFrame cfg, never silently
            if 'error' in mine:
                assert ('advFrameCfg' in text or 'cascade' in mine['error']
                        or ref is None), path
            continue
        shared = [k for k in mine if k in ref and
                  isinstance(mine[k], (int, float)) and
                  not isinstance(mine[k], bool) and ref[k] is not None]
        assert len(shared) >= 25, path
        for k in shared:
            assert mine[k] == pytest.approx(ref[k], rel=1e-9, abs=1e-12), \
                '%s: %s' % (os.path.basename(path), k)
        assert mine['lvds_data_fmt'] == ref['lvds_data_fmt'], path
        compared += 1
        if path.endswith('SAR_2ms_fmt2.cfg'):
            assert compared == 1 and ref['num_samples'] == 3300
    assert compared >= 2
