import math

import numpy as np
import pytest

from navigation.occupancy import OccupancyEvidence


def model():
    prior = np.zeros((40, 100), dtype=np.int16)
    prior[20, 30] = 100  # old furniture, 1 m in front of the sensor
    prior[20, 90] = 100  # distant prior, outside the correction horizon
    prior[19, 30] = 100  # a neighboring ray must not erase unseen cells
    prior[20, 25] = -1
    return OccupancyEvidence(prior, 0.05, (0, 0), observation_range_m=2, free_memory_seconds=2)


def scan(m, ranges, now, *, stamp=None):
    return m.observe(
        ranges, 0, 0.01, 0.1, 10, (1.025, 1.025, 0), now if stamp is None else stamp, now
    )


def confirm_free(m, start=1):
    for t in (start, start + 0.16, start + 0.32):
        scan(m, [3.0], t)


def test_repeated_visible_free_corrects_prior_but_never_changes_original():
    m = model()
    original = m.prior.copy()
    scan(m, [3.0], 1)
    assert m.render(1)[20, 30] == 100
    scan(m, [3.0], 1.16)
    assert m.render(1.16)[20, 30] == 100
    scan(m, [3.0], 1.32)
    out = m.render(1.32)
    assert out[20, 30] == 0
    assert out[20, 90] == out[19, 30] == 100
    assert out[20, 25] == -1
    np.testing.assert_array_equal(m.prior, original)


@pytest.mark.parametrize("missing", [math.inf, -math.inf, math.nan, 0.0, 10.0, 11.0])
def test_missing_and_out_of_range_beams_never_clear_prior(missing):
    m = model()
    for t in (1, 1.2, 1.4):
        scan(m, [missing], t)
    assert m.render(1.4)[20, 30] == 100


def test_occlusion_keeps_old_obstacle_and_new_hit_is_immediate():
    m = model()
    for t in (1, 1.2, 1.4):
        scan(m, [0.25], t)
    assert m.render(1.4)[20, 25] == 100
    assert m.render(1.4)[20, 30] == 100


def test_person_mark_is_retained_without_evidence_and_cleared_by_background():
    m = model()
    scan(m, [0.75], 1)
    assert m.render(1)[20, 35] == 100
    scan(m, [math.inf], 2)
    assert m.render(100)[20, 35] == 100
    confirm_free(m, 101)
    assert m.render(101.32)[20, 35] == 0


def test_free_override_expires_and_health_loss_restores_prior():
    m = model()
    confirm_free(m)
    assert m.render(1.4)[20, 30] == 0
    assert m.render(4)[20, 30] == 100
    assert m.render(1.4, reliable=False)[20, 30] == 100
    m.invalidate_free()
    assert m.render(1.4)[20, 30] == 100


def test_duplicate_stamp_cannot_confirm_free_and_long_gap_restarts():
    m = model()
    for t in (1, 1.2, 1.4):
        scan(m, [3.0], t, stamp=1)
    assert m.render(1.4)[20, 30] == 100
    scan(m, [3.0], 2)
    scan(m, [math.inf], 2.2)
    scan(m, [3.0], 2.8)
    scan(m, [3.0], 2.96)
    assert m.render(2.96)[20, 30] == 100


def test_new_hit_revokes_confirmed_free_and_endpoint_margin_is_preserved():
    m = model()
    confirm_free(m)
    scan(m, [0.5], 1.5)
    assert m.render(1.5)[20, 30] == 100
    # The 10 cm immediately before the return are not free-space evidence.
    m = model()
    for t in (2, 2.2, 2.4):
        scan(m, [0.6], t)
    assert m.render(2.4)[20, 30] == 100


def test_reset_after_relocalization_drops_evidence_in_old_coordinates():
    m = model()
    scan(m, [0.75], 1)
    confirm_free(m, 2)
    m.reset()
    np.testing.assert_array_equal(m.render(2.4), m.prior)


def dynamic_model():
    m = model()
    m.dynamic_memory = 3.0
    return m


def test_departed_hit_expires_without_inventing_a_free_ray():
    m = dynamic_model()
    scan(m, [0.75, 3.0], 1)
    for t in np.arange(1.1, 4.2, 0.1):
        # The hit's direction is unobserved; another direction remains valid.
        scan(m, [math.inf, 3.0], t)
    assert m.render(4.2)[20, 35] == 0
    assert m.free_until[20, 35] == 0  # expiry is not free evidence
    assert m.prior[20, 30] == m.render(4.2)[20, 30] == 100


def test_continuously_observed_thin_obstacle_never_expires_or_needs_a_cluster():
    m = dynamic_model()
    for t in np.arange(1, 12, 0.1):
        scan(m, [0.75], t)
        assert m.render(t)[20, 35] == 100


def test_short_dropout_retains_hit_and_reappearance_renews_memory():
    m = dynamic_model()
    for t in np.arange(1, 6, 0.1):
        scan(m, [0.75 if t < 1.5 or t > 3.5 else math.inf, 3.0], t)
        assert m.render(t)[20, 35] == 100


@pytest.mark.parametrize("prior", [-1, 100])
def test_dynamic_expiry_restores_prior_instead_of_opening_it(prior):
    m = dynamic_model()
    m.prior[20, 35] = prior
    scan(m, [0.75], 1)
    for t in np.arange(1.1, 5, 0.1):
        scan(m, [0.25], t)  # occluder keeps receiving hits
    assert m.render(5)[20, 35] == prior
    assert m.render(5)[20, 25] == 100


def test_wall_clock_blank_duplicate_or_lost_scan_does_not_age_hits():
    m = dynamic_model()
    scan(m, [0.75], 1)
    assert m.render(100)[20, 35] == 100
    for t in np.arange(2, 7, 0.1):
        assert not scan(m, [math.inf], t)
    for t in np.arange(8, 13, 0.1):
        assert not scan(m, [3.0], t, stamp=1)
    scan(m, [0.25], 100)  # no aging across the outage
    assert m.render(100)[20, 35] == 100
    m.invalidate_free()
    scan(m, [0.25], 100.1)
    assert m.observation_clock == 0


def test_brief_unobserved_bins_do_not_reset_real_free_evidence():
    m = model()
    scan(m, [3.0, 3.0], 1)
    scan(m, [math.inf, 3.0], 1.1)
    scan(m, [3.0, 3.0], 1.2)
    scan(m, [math.inf, 3.0], 1.3)
    scan(m, [3.0, 3.0], 1.4)
    assert m.render(1.4)[20, 30] == 0


def test_real_hit_resets_free_evidence_even_inside_short_gap():
    m = model()
    scan(m, [3.0], 1)
    scan(m, [3.0], 1.2)
    scan(m, [0.5], 1.3)
    scan(m, [3.0], 1.4)
    assert m.render(1.4)[20, 30] == 100
