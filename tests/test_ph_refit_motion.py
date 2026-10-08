import numpy as np
from ph_model.refit_motion import motion_features


def test_motion_features_keep_plateaus_and_exclude_steps():
    q=np.r_[np.zeros(120),np.linspace(0,.2,20),np.full(120,.2)]
    f=motion_features(q)
    assert len(f['hold'])>0
    assert all(np.ptp(q[i:i+101])<np.deg2rad(.3) for i in f['hold'])
    assert len(f['step'])>0
    assert all(abs(q[i+10]-q[i])>np.deg2rad(.3) for i in f['step'])
