import numpy as np
from ph_model.diagnose_structure import windows


def test_motion_regimes_cover_intermediate_motion():
    for span, expected in [(.1,'hold'), (1.,'slow'), (5.,'large')]:
        i,kind=windows(np.linspace(0,span,101))
        assert i.tolist()==[0]
        assert kind.tolist()==[expected]


def test_no_window_crosses_block_end():
    for n in [0,50,100,101,205]:
        i,_=windows(np.zeros(n))
        assert np.all(i+100<n)
