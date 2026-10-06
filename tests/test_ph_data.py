"""Synthetic offline data fixtures only: no hardware runner imports."""
import csv
import json
import numpy as np

from ph_model.data import load_run


def test_loader_never_bridges_invalid_or_gap_and_preserves_pressure_sign(tmp_path):
    meta={'profile':{'segments':[{'segment':'measurement','kind':'measurement'}]}}
    (tmp_path/'meta.json').write_text(json.dumps(meta))
    names=['t_mono_s','segment','p_pos_kpa','p_neg_kpa','angle_deg',
           'angle_age_s','pressure_age_s','sensor_valid']
    with (tmp_path/'run.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(names)
        for i in range(600):
            # First cut: sentinel invalid. Second cut: dropped recording period.
            t=i*.01 + (1 if i>=400 else 0)
            writer.writerow([t,'measurement',100 if 50<=i<60 else 110,80,
                             180 if i==200 else 20,.01,.01,0 if i==200 else 1])
    run=load_run(tmp_path)
    assert len(run['chunks'])==3
    assert run['audit']['pressure_domain_excluded']==10
    for ch in run['chunks']:
        assert np.all(ch['pressure'][ch['good'],1]>=0)
        assert np.all(np.rad2deg(ch['q'])<21)
        assert not (ch['t'][0]<2<ch['t'][-1])
    assert np.any(run['chunks'][0]['pressure'][:,1]<0)  # no clipping


def test_cutoff_applied_before_filtering(tmp_path):
    (tmp_path/'meta.json').write_text(json.dumps({'profile':{'segments':[{'segment':'m','kind':'measurement'}]}}))
    with (tmp_path/'run.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow(['t_mono_s','segment','p_pos_kpa','p_neg_kpa','angle_deg','angle_age_s','pressure_age_s','sensor_valid'])
        for i in range(400):
            w.writerow([i*.01,'m',110,80,20 if i<250 else 180,.01,.01,1])
    run=load_run(tmp_path,cutoff=2.4)
    assert all(ch['t'][-1]<2.4 for ch in run['chunks'])
    assert max(np.max(np.abs(ch['v'])) for ch in run['chunks'])<1e-10
