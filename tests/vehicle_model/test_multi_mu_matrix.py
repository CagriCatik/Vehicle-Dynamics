"""Unit and Integration Tests for Comprehensive Multi-Friction Vehicle Dynamics Suite.

Verifies:
- Surface name mapping across friction coefficients (μ = 0.2 to 1.0)
- Physical friction limit scaling: a_y,max ≈ μ · g
- Closed-loop stability bifurcation: Passive spinout vs IVMC containment on low μ
- Dashboard and Markdown report generation
"""

from __future__ import annotations

from pathlib import Path
import pytest
import numpy as np

from vehicle_model.simulation.multi_mu_matrix import (
    MultiMuMatrixSummary,
    get_surface_name,
    simulate_dlc_multi_mu,
    simulate_circle_handling_multi_mu,
    run_comprehensive_multi_mu_benchmark,
    plot_multi_mu_matrix_dashboard,
    generate_multi_mu_report_md,
)


def test_surface_name_mapping():
    """Verify that numerical friction values correctly map to automotive proving ground surfaces."""
    assert "Dry Asphalt" in get_surface_name(1.00)
    assert "Wet Asphalt" in get_surface_name(0.70)
    assert "Snow" in get_surface_name(0.40)
    assert "Ice" in get_surface_name(0.20)


def test_dlc_multi_mu_simulation():
    """Verify ISO 3888-2 Double Lane Change across dry and snow surfaces."""
    mus = [1.00, 0.40]
    res_dict = simulate_dlc_multi_mu(mus)

    assert 1.00 in res_dict
    assert 0.40 in res_dict

    # Dry asphalt (μ=1.00): all controllers complete maneuver
    dry_kpis = res_dict[1.00].kpis
    assert dry_kpis["passive"].max_sideslip_deg < 6.0
    assert dry_kpis["integrated_ivmc"].max_sideslip_deg < 4.0

    # Snow (μ=0.40): passive vehicle exhibits severe sideslip while IVMC contains it
    snow_kpis = res_dict[0.40].kpis
    assert snow_kpis["passive"].max_sideslip_deg > dry_kpis["passive"].max_sideslip_deg
    assert snow_kpis["integrated_ivmc"].max_sideslip_deg < 5.0


def test_circle_handling_limit_scaling():
    """Verify that circular driving grip scales with road friction coefficient: a_y,max ∝ μ."""
    mus = [0.80, 0.30]
    res_dict = simulate_circle_handling_multi_mu(mus)

    ay_high = res_dict[0.80].kpis["passive"].peak_lat_accel_g
    ay_low = res_dict[0.30].kpis["passive"].peak_lat_accel_g

    # Peak lateral acceleration must be strictly higher on higher friction
    assert ay_high > ay_low
    # Lateral acceleration should not exceed μ * g (with margin for dynamic tire load)
    assert ay_high <= 0.85 * 1.15
    assert ay_low <= 0.35 * 1.15


def test_multi_mu_dashboard_and_report_generation(tmp_path: Path):
    """Verify dashboard plotting and markdown report generation on a fast 2-point sweep."""
    summary = run_comprehensive_multi_mu_benchmark(
        mu_levels=[1.00, 0.40],
        output_dir=tmp_path,
    )

    assert isinstance(summary, MultiMuMatrixSummary)
    assert len(summary.scenarios) == 6
    assert summary.controllers == ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"]
    assert "Active 4WS (Standalone)" in summary.controller_names["four_wheel_steering"]
    assert "IVMC (2WS: TV + ESC)" in summary.controller_names["ivmc_2ws"]
    assert "Full-Stack IVMC + 4WS" in summary.controller_names["full_ivmc_4ws"]

    # Test Dashboard PNG Generation
    dash_path = tmp_path / "test_multi_mu_dashboard.png"
    plot_multi_mu_matrix_dashboard(summary, output_path=dash_path)
    assert dash_path.exists()
    assert dash_path.stat().st_size > 10000

    # Test Markdown Report Generation
    rep_path = tmp_path / "test_multi_mu_report.md"
    generate_multi_mu_report_md(summary, output_path=rep_path)
    assert rep_path.exists()
    rep_text = rep_path.read_text(encoding="utf-8")
    assert "Multi-Friction" in rep_text
    assert "ISO 3888-2" in rep_text
    assert "Dry Asphalt" in rep_text
