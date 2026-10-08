import numpy as np
from ph_model.plot_predictions import block_slices, rmse


def test_plot_does_not_connect_runs_or_reset_blocks():
    data = dict(run_id=np.array(['a', 'a', 'a', 'a', 'b', 'b']),
                block_id=np.array([0, 0, 1, 1, 0, 0]),
                block_start=np.array([1, 0, 1, 0, 1, 0]))
    assert [(s.start, s.stop) for s in block_slices(data)] == [(0, 2), (2, 4), (4, 6)]


def test_plot_rmse_uses_all_samples_in_degrees():
    assert rmse(np.array([0., 4.]), np.array([0., 0.])) == np.sqrt(8)
