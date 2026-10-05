import importlib.util
from pathlib import Path
import sys

import pytest

SCRIPT = Path(__file__).parents[1] / 'src/can_powerpack/scripts/ph_profiles.py'
spec = importlib.util.spec_from_file_location('ph_profiles', SCRIPT)
profiles = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = profiles
spec.loader.exec_module(profiles)


@pytest.mark.parametrize('name', ['R0', 'S1a', 'S1b', 'S2', 'S3', 'S4', 'S5', 'S6'])
def test_all_profiles(name):
    plan = profiles.generate(name, seed=41)
    profiles.validate(plan)
    assert plan['segments'][0]['duration_s'] == 5
    assert plan['segments'][-1]['duration_s'] == 5
    assert plan['segments'][0]['start'] == [101.325, 101.325]
    assert plan['segments'][-1]['end'] == [101.325, 101.325]
    assert profiles.digest(plan) == profiles.digest(profiles.generate(name, seed=41))


def test_single_chamber_endpoints():
    plan = profiles.generate('S1a')
    points = [s['end'][0] for s in plan['segments'] if s['mode'] == 'settle']
    up = [103, 111, 119, 127, 135, 143, 151, 159, 167, 172]
    assert points == up + up[-2::-1]
    assert min(s['end'][1] for s in profiles.generate('S1b')['segments']) == 40


def test_s2_low_limit_records_exclusions():
    plan = profiles.generate('S2', d_max=48)
    assert plan['excluded_points']
    assert all(s['end'][0] - s['end'][1] <= 48 + 1e-9 for s in plan['segments'])
    assert any(p['diff_kpa'] == 0 for p in plan['excluded_points'])


@pytest.mark.parametrize('name', ['R0', 'S1a', 'S1b', 'S3', 'S4', 'S5'])
def test_no_silent_fixed_endpoint_clipping(name):
    with pytest.raises(ValueError):
        profiles.generate(name, d_max=48)


def test_random_duration_and_slew():
    plan = profiles.generate('S6', seed=4)
    body = [s for s in plan['segments'] if s['kind'] == 'measurement']
    assert sum(s['duration_s'] for s in body) == pytest.approx(600)
    assert profiles.digest(plan) != profiles.digest(profiles.generate('S6', seed=5))
    for s in body:
        if s['mode'] == 'cosine':
            assert 8 <= s['duration_s'] <= 30
            assert profiles.peak_slew(s) <= 2 + 1e-9


def test_validation_detects_discontinuity_and_slew():
    plan = profiles.generate('R0')
    plan['segments'][1]['start'][0] += 1
    with pytest.raises(ValueError):
        profiles.validate(plan)
    plan = profiles.generate('R0')
    plan['segments'][1]['duration_s'] = 0.001
    with pytest.raises(ValueError):
        profiles.validate(plan)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1, 0, 73])
def test_bad_limit(bad):
    with pytest.raises(ValueError):
        profiles.generate('S2', d_max=bad)


def test_export(tmp_path):
    plan = profiles.generate('S2')
    profiles.export(plan, tmp_path, plot=False)
    assert (tmp_path / 'S2_0.csv').is_file()
    assert (tmp_path / 'S2_0.json').is_file()


@pytest.mark.parametrize('seed', range(20))
def test_s6_seeds(seed):
    for cap in (16, 48, 72):
        profiles.validate(profiles.generate('S6', seed=seed, d_max=cap))


def test_timed_samples_obey_chamber_slew():
    for name in ('R0', 'S3', 'S4', 'S5', 'S6'):
        previous = None
        for t, segment, pair in profiles.samples(profiles.generate(name), hz=10):
            if previous:
                old_t, old_pair = previous
                assert t > old_t
                assert max(abs(a-b)/(t-old_t) for a,b in zip(pair, old_pair)) <= segment['ramp_kpa_s'] + 1e-7
            previous = t, pair


def test_s4_holds():
    holds = [s for s in profiles.generate('S4')['segments'] if s['duration_s'] == 120]
    assert [round(s['end'][0]-s['end'][1]) for s in holds] == [24, 24, 48, 48, 60, 60]


def test_export_refuses_overwrite(tmp_path):
    plan = profiles.generate('R0')
    profiles.export(plan, tmp_path, plot=False)
    with pytest.raises(FileExistsError):
        profiles.export(plan, tmp_path, plot=False)
