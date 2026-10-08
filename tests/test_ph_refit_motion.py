import numpy as np
from ph_model.refit_motion import motion_features,score_predictions


def test_motion_features_keep_plateaus_and_exclude_steps():
    q=np.r_[np.zeros(120),np.linspace(0,.2,20),np.full(120,.2)]
    f=motion_features(q)
    assert len(f['hold'])>0
    assert all(np.ptp(q[i:i+101])<np.deg2rad(.3) for i in f['hold'])
    assert len(f['step'])>0
    assert all(abs(q[i+10]-q[i])>np.deg2rad(.3) for i in f['step'])


def test_identical_prediction_has_zero_shape_and_position_error():
    q=np.r_[np.zeros(120),np.linspace(0,.2,20),np.full(120,.2)]
    b=dict(q=q,features=motion_features(q))
    prediction=np.column_stack((q,np.zeros((len(q),3))))
    result=score_predictions([b],[prediction])
    assert result['rmse_deg']==0
    assert result['step_1s_error_rmse_deg']==0
    assert result['hold_10s_error_rmse_deg']==0
    assert result['selection_score_deg']==0
