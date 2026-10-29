"""Copy curated result figures from output/ into docs/assets/ and embed them in the documentation.

The figures are produced by the example scripts (output/ is not tracked). This script
picks a curated set per document, shrinks them for the repository (PNG at most
1600 px wide; GIF at half size, every second frame, 128 colours) and writes a
"Ergebnisbilder" block between the markers ``<!-- figures:start -->`` and
``<!-- figures:end -->`` at the end of each document. Rerunning replaces the block.

    python tools/publish_doc_figures.py            # copy + embed
    python tools/publish_doc_figures.py --check    # only list missing sources

Regenerate the sources first (see the "Quelle" in each caption).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PIL import Image, ImageSequence

ROOT = Path(__file__).resolve().parents[1]
OUT, DOCS, ASSETS = ROOT / "output", ROOT / "docs", ROOT / "docs" / "assets"
START, END = "<!-- figures:start -->", "<!-- figures:end -->"
NOTE = ("Alle Bilder sind Simulationsergebnisse mit synthetischen Parametern, keine Messungen. "
        "Neu erzeugen: Quellskript ausführen, dann `python tools/publish_doc_figures.py`.")

# document -> [(source under output/, caption)]
FIGURES: dict[str, list[tuple[str, str]]] = {
    "VEHICLE_MODELS.md": [
        ("demo_7dof/seven_dof_maneuver.png",
         "7-DOF-Zweispurmodell mit Hinterachslenkung und Torque Vectoring auf Split-µ: Bahn, Längs- und Querbewegung, "
         "Gierrate und Lenkwinkel, Raddrehzahlen, Längsschlupf, Radlasten, Antriebs- und Bremsmomente, Reifenkräfte. "
         "Quelle: `examples/seven_dof_4ws_tv_demo.py`."),
    ],
    "TYRE_MODELS.md": [
        ("tyres/tyre_models_m3.png",
         "Magic-Formula-Reifen (synthetisch) gegen Fiala: Seitenkraft und Rückstellmoment bei drei Radlasten, kombinierter "
         "Schlupf, Selbstähnlichkeit in µ, Relaxation und Lenkwinkelsprung des 7-DOF-Modells mit Relaxationslängen. "
         "Quelle: `examples/tyre_models_m3_demo.py`."),
    ],
    "HANDLING_ANALYSIS.md": [
        ("handling_analysis/reference_vehicle_handling.png",
         "Lineare Fahrdynamik des Referenzfahrzeugs: stationäre Gierverstärkung (charakteristische Geschwindigkeit "
         "≈ 194 km/h), Eigenfrequenz, Dämpfung, Frequenzgang der Gierrate und Lenkwinkelsprung 7-DOF gegen linear. "
         "Quelle: `examples/handling_analysis_reference_vehicle.py`."),
    ],
    "CHASSIS_KINEMATICS.md": [
        ("kc_analysis/kc_understeer.png",
         "Eigenlenken mit K&C: Untersteuerwinkel und lokaler Eigenlenkgradient mit schrittweise zugeschalteten "
         "Rollzentren/Ackermann, Rolllenken und Elastokinematik; lineares Budget 2,19 °/g; Parameterempfindlichkeit. "
         "Quelle: `examples/kc_understeer_analysis.py`."),
    ],
    "HANDLING_LIMITS.md": [
        ("handling_limits/steady_state_handling_characteristics.png",
         "Stationäre Kreisfahrt nach dem Vorbild von ISO 4138 (Simulation): Lenkwinkel und lokaler Eigenlenkgradient "
         "über der Querbeschleunigung, dynamischer Schwimmwinkel und g-g-Diagramm für vier Konfigurationen. "
         "Quelle: `examples/handling_limits_and_understeer_demo.py`."),
    ],
    "CONTROL_ALLOCATION.md": [
        ("m5/slip_and_allocation.png",
         "Schlupfregelung und beschränkte Allokation: mittlere Verzögerung bezogen auf µg ohne ABS, mit Alt-ABS und mit "
         "PI-Schlupfregelung (Werte über 1, weil die Spitzenreibung des MF-Reifens bei geringer Radlast über µ liegt); "
         "Split-µ-Bremsung mit fester Lenkung; Lenkwinkelsprung mit TV; Bremsenausfall vorn links. "
         "Quelle: `examples/slip_and_allocation_demo.py`."),
    ],
    "MOTION_CONTROL.md": [
        ("controller_benchmark/controller_benchmark.png",
         "Gierrate im Sine with Dwell (80 km/h, 8°) für alle Regler auf derselben Strecke, nominal und auf zwei "
         "Streckenvarianten (+15 % Masse/Trägheit, Hinterachs-Schräglaufsteifigkeit −20 %). Quelle: "
         "`examples/controller_benchmark.py`."),
        ("split_mu_matrix/split_mu_matrix.png",
         "Split-µ-Vollbremsung (100 km/h, µ 1,0/0,2, Lenkung gerade): Bahn und Gierwinkel je Konfiguration. Nur die "
         "Allokation, die das Bremsmoment auf der Hoch-µ-Seite reduziert, bleibt gerade. Quelle: "
         "`examples/split_mu_matrix_demo.py`."),
    ],
    "ESTIMATION_ROBUSTNESS.md": [
        ("monte_carlo/monte_carlo.png",
         "Monte Carlo über 25 Streckenstichproben (Seed 2026), Sine with Dwell 80 km/h: Gierratenverhältnis 1 s nach "
         "Lenkende und Querversatz, passiv gegen PI (TV), mit den nach FMVSS 126 nachgebildeten Grenzen. "
         "Quelle: `examples/monte_carlo_robustness.py`."),
    ],
    "STATE_ESTIMATION.md": [
        ("estimation/estimation_and_pso_validation.png",
         "EKF-Schwimmwinkelbeobachter und RLS-Reibwertschätzer gegen die Wahrheit, IMU-Biasschätzung, PSO-Konvergenz. "
         "**Ehrlicher Befund:** Der EKF verfehlt die Schwimmwinkelspitzen deutlich (≈ 3° statt 6°), und der "
         "RLS-Reibwert bleibt bei 1,0 bzw. 0,7, während der wahre Wert bei 0,35–0,45 liegt: In diesem Manöver "
         "konvergiert er nicht. Das war der Anlass für den sensorbasierten Schätzer `estimation/sensor_friction.py`. "
         "Quelle: `examples/estimation_and_pso_demo.py`."),
    ],
    "VERTICAL_DYNAMICS.md": [
        ("vertical_dynamics/vertical_dynamics.png",
         "Viertelfahrzeug: analytischer gegen simulierten Frequenzgang von Aufbaubeschleunigung und dynamischer Radlast, "
         "Skyhook und Groundhook, Straßenklassen A–E nach ISO 8608 mit erzeugtem Klasse-C-Profil. "
         "Quelle: `examples/vertical_dynamics_demo.py`."),
    ],
    "SUSPENSION_DYNAMICS.md": [
        ("suspension/suspension_and_14dof_validation.png",
         "Viertelfahrzeug (passiv, Skyhook, Groundhook) über eine Schwelle und 14-DOF-Fahrzeug bei Bremsen mit Lenken: "
         "Nick-, Hub- und Wankbewegung, Radlasten, Bahn. Quelle: `examples/suspension_dynamics_demo.py`."),
    ],
    "EXPERIMENTS.md": [
        ("experiments/sine_with_dwell_comparison/base.png",
         "Ausgabe des YAML-Experiments `sine_with_dwell_comparison`: Gierrate, Schwimmwinkel und Bahn für passiv, PI, "
         "GS-LQR (4WS+TV) und LTV-MPC mit Beobachter. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/sine_with_dwell_comparison.yaml`."),
    ],
    "REAR_WHEEL_STEERING.md": [
        ("four_wheel_steering/parking/u_turn.gif",
         "Wenden auf schmaler Straße, 2WS (grau) gegen 4WS (türkis): unten die Radlenkwinkel über der Zeit. Die "
         "Hinterachse lenkt mit −8° gegen die Vorderachse; die überstrichene Breite sinkt von 12,75 m auf 11,10 m. "
         "Quelle: `examples/parking_4ws_demo.py`."),
        ("four_wheel_steering/parking/parallel_exit.gif",
         "Ausparken aus enger Längsparklücke: 4WS braucht 1,88 m statt 1,96 m Lücke zum Vordermann, das Heck schwenkt "
         "aber 13 cm näher an den Bordstein (Nachteil der Gegenphase). Quelle: `examples/parking_4ws_demo.py`."),
        ("four_wheel_steering/parking/perpendicular_parking.gif",
         "Rückwärts senkrecht einparken (90°): überstrichene Fläche 8,76 × 9,35 m (4WS) gegen 9,65 × 9,66 m (2WS). "
         "Quelle: `examples/parking_4ws_demo.py`."),
        ("four_wheel_steering/scenario_3_double_lane_change_4ws.gif",
         "Doppelspurwechsel bei 80 km/h: die Hinterachse lenkt gleichphasig mit. Quelle: "
         "`examples/four_wheel_steering_benchmark.py`."),
        ("four_wheel_steering/textbook_4ws_characteristics.png",
         "Kennfeld der Hinterachslenkung: Übersetzungsverhältnis über der Geschwindigkeit, Kennlinienschar, "
         "Isolinienfeld und stationäre Schwimmwinkelverstärkung 2WS gegen 4WS. Quelle: "
         "`examples/four_wheel_steering_benchmark.py`."),
        ("four_wheel_steering/lut_3d_surface.png",
         "Kalibrierte Lookup-Tabelle δ_r(v_x, δ_f): gegenphasig unter ≈ 36 km/h, gleichphasig darüber. "
         "Quelle: `examples/four_wheel_steering_benchmark.py`."),
        ("four_wheel_steering/four_wheel_steering_dashboard.png",
         "Neun 4WS-Szenarien im Überblick (2WS, drehzahlgeführte Kennlinie, Lookup-Tabelle, mit Gierdämpfung). "
         "Quelle: `examples/four_wheel_steering_benchmark.py`."),
    ],
    "SPLIT_MU_DYNAMICS.md": [
        ("split_mu_scenarios/scenario_1_split_mu_braking.gif",
         "Split-µ-Bremsung aus 100 km/h mit Fahrermodell, das die Spur hält, und 1600 N m *gesamtem* Bremsmoment (60/40), "
         "also einer moderaten Bremsung (im Bild als „Emergency Braking“ bezeichnet). "
         "Quelle: `examples/split_mu_scenarios_demo.py`."),
        ("split_mu_scenarios/scenario_1_split_mu_braking.png",
         "Dieselbe Bremsung als Dashboard: Bahnen, Gierrate, Querabweichung und Störgiermoment. Der lenkende Fahrer ist "
         "der Unterschied zur Konfigurationsmatrix in MOTION_CONTROL.md, in der die Lenkung gerade gehalten wird und "
         "IVMC mit der alten additiven Stellkette sich eindreht (über die gemeinsame Allokation, Standard seit V3.0, "
         "bleibt er gerade). Quelle: `examples/split_mu_scenarios_demo.py`."),
        ("split_mu_scenarios/split_mu_friction_sweep_analysis.png",
         "Reibwert-Sweep der Niedrig-µ-Seite: Gierrate, Gierwinkel, Querabweichung, Bremsweg und Schwimmwinkel je "
         "Regler. Quelle: `examples/split_mu_scenarios_demo.py`."),
        ("benchmark/multi_mu_scenarios_dashboard.png",
         "Multi-µ-Matrix: Kennwerte von acht Manövern über dem Reibwert für vier Konfigurationen. "
         "Quelle: `examples/multi_mu_scenarios_benchmark.py`."),
    ],
    "CHASSIS_CONTROL.md": [
        ("benchmark/double_lane_change_comparison.png",
         "Doppelspurwechsel 80 km/h bei µ = 0,75: passiv, 4WS und IVMC. Quelle: `examples/integrated_control_benchmark.py`."),
        ("benchmark/mu_split_comparison.png",
         "Split-µ-Bremsung ohne Regelung gegen IVMC über die gemeinsame Allokation: Gierwinkel 0° statt > 150°. Die "
         "Allokation bremst das Eisrad nur leicht (Schlupf ≈ −0,01) und begrenzt die Hoch-µ-Seite entsprechend; die "
         "Legende „−0,15 optimal“ im Bild stammt aus der alten ABS-Kette. Quelle: `examples/abs_tcs_esc_benchmark.py`."),
        ("advanced_control/phase_plane_stability_analysis.png",
         "Phasenebene (β, β̇) mit Eingriffsgrenzen; Schwimmwinkel und Gierrate passiv gegen Phasenebenen-IVMC. "
         "Quelle: `examples/advanced_chassis_control_demo.py`."),
        ("e2e_analysis/e2e_liftoff_oversteer_phase_plane.png",
         "Lastwechsel beim Bremsen in der Kurve: Bahn in der (β, r)-Ebene, Schwimmwinkel gegen die dynamische Grenze, "
         "Bedrohungsmaß und Gierrate. Die Regelung verlangsamt das Ausbrechen, verhindert es aber nicht (β ≈ −33° nach "
         "2,5 s). Quelle: `examples/e2e_deep_analysis_and_benchmarking.py`."),
        ("advanced_control/14dof_coupled_fishhook_stability.png",
         "14-DOF-Fishhook: Wankwinkel, Lastverlagerungsverhältnis, Radlasten und dynamische Wanksteifigkeitsverteilung "
         "passiv gegen Wankregelung. Quelle: `examples/advanced_chassis_control_demo.py`."),
    ],
    "NMPC_CONTROL.md": [
        ("advanced_control/dynamic_two_track_nmpc_tracking.png",
         "Dynamisches Zweispur-NMPC, Ausweichen bei 90 km/h: Bahn, Lenkwinkel vorn/hinten, Reibkreisausnutzung bezogen "
         "auf µF_z. Die Bahn schießt auf 4,8 m über (Ziel 3,5 m). Werte knapp über 1 sind keine Verletzung: Die "
         "Spitzenreibung des MF-Reifens liegt bei geringer Radlast über µ. Quelle: "
         "`examples/driver_and_nmpc_dynamics_demo.py`."),
        ("e2e_analysis/e2e_nmpc_kamm_friction_evasion.png",
         "NMPC-Spurwechsel 90 km/h (E2E-Test): Bahn, koordinierte Lenkwinkel, Radmomente und Reibkreisausnutzung. "
         "Quelle: `examples/e2e_deep_analysis_and_benchmarking.py`."),
    ],
    "NMPC_PATH_TRACKING.md": [
        ("animations/nmpc_slalom.gif",
         "Kinematisches Pfadfolge-NMPC im Slalom mit Hinterachslenkung aus der Lookup-Tabelle. "
         "Quelle: `examples/generate_maneuver_animations.py`."),
        ("nmpc_path_tracking_simulation.png",
         "Pfadfolge-NMPC (Maßstabsmodell): Bahn, Lenkwinkel, Querfehler und Geschwindigkeit. Quelle: "
         "`python examples/nmpc_path_tracking_simulation.py --headless`."),
    ],
    "DRIVER_MODEL.md": [
        ("advanced_control/driver_closed_loop_workload.png",
         "Fahrermodell im Doppelspurwechsel: Bahn, Zerlegung von Totzeit und Verzögerung, Lenkaufwand und Lenkumkehrungen "
         "passiv gegen IVMC-unterstützt. Quelle: `examples/driver_and_nmpc_dynamics_demo.py`."),
        ("e2e_analysis/e2e_driver_split_mu_stability.png",
         "Fahrer im geschlossenen Kreis beim Reibwertübergang: Basis- gegen verbesserten Fahrer (Prädiktion, Adaption). "
         "Der verbesserte Fahrer lenkt stärker und schwingt mehr. Quelle: `examples/e2e_deep_analysis_and_benchmarking.py`."),
    ],
    "VALIDATION_PLATFORM.md": [
        ("animations/iso_3888_dlc.gif",
         "Doppelspurwechsel nach dem Vorbild von ISO 3888-2 mit allen vier Konfigurationen. "
         "Quelle: `python examples/run_validation_platform.py --animations`."),
        ("validation_report/figures/iso_3888_dlc.png",
         "Doppelspurwechsel: Bahn mit Pylonen, Gierrate, Schwimmwinkel, Querbeschleunigung. Quelle: "
         "`examples/run_validation_platform.py`."),
        ("validation_report/figures/sine_with_dwell.png",
         "Sine with Dwell nach dem Vorbild von FMVSS 126 (Simulation, keine Prüfung nach Vorschrift). "
         "Quelle: `examples/run_validation_platform.py`."),
        ("validation_report/figures/mu_split_braking.png",
         "Split-µ-Bremsung mit gerader Lenkung und 1450 N m *gesamtem* Bremsmoment, also etwa 12 % Pedal: eine moderate "
         "Bremsung, keine Vollbremsung. Mit der alten additiven Stellkette bleibt IVMC hier unter der 15°-Grenze (≈ 10°) und "
         "dreht sich ab etwa 20 % Pedal ein; über die gemeinsame Allokation (Standard seit V3.0) bleibt er bei jedem "
         "Pedal gerade (Konfigurationsmatrix in MOTION_CONTROL.md, Test "
         "`test_additive_esc_holds_only_at_the_moderate_brake_level_of_the_validation_matrix`). "
         "Quelle: `examples/run_validation_platform.py`."),
        ("validation_report/figures/fishhook_ncap.png",
         "Fishhook nach NCAP-Vorbild. Quelle: `examples/run_validation_platform.py`."),
    ],
    "VISUALIZATION_GUIDE.md": [
        ("animations/multi_mu_lane_change_comparison.gif",
         "Spurwechsel auf mehreren Reibwerten mit Nahkamera-Panel und HUD. Quelle: `examples/multi_mu_scenarios_benchmark.py`."),
        ("four_wheel_steering/parking/tight_alley.gif",
         "4WS-Animation mit Lenkwinkel-Panel: unten Vorder- und Hinterachswinkel mit Cursor auf dem aktuellen Bild. "
         "Fahrzeuge werden an der Hinterachse gezeichnet, aus dem Schwerpunkt über l_r berechnet "
         "(`VehicleParams.cog_to_rear_axle`). Quelle: `examples/parking_4ws_demo.py`."),
    ],
}


def asset_path(doc: str, src: str) -> Path:
    return ASSETS / Path(doc).stem.lower() / Path(src).name


def shrink(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    im = Image.open(src)
    if src.suffix.lower() == ".gif":
        frames, durations = [], []
        for i, f in enumerate(ImageSequence.Iterator(im)):
            if i % 2:
                continue
            f = f.convert("RGB")
            f = f.resize((f.width // 2, f.height // 2), Image.LANCZOS)
            frames.append(f.quantize(colors=128, method=Image.Quantize.MEDIANCUT))
            durations.append(2 * f.info.get("duration", im.info.get("duration", 66)))
        frames[0].save(dst, save_all=True, append_images=frames[1:], duration=durations, loop=0, optimize=True)
    else:
        im = im.convert("RGB")
        if im.width > 1600:
            im = im.resize((1600, round(im.height * 1600 / im.width)), Image.LANCZOS)
        im.quantize(colors=256, method=Image.Quantize.MEDIANCUT).save(dst, optimize=True)


def block(doc: str) -> str:
    lines = [START, "", "## Ergebnisbilder", "", f"> {NOTE}", ""]
    for src, caption in FIGURES[doc]:
        rel = asset_path(doc, src).relative_to(DOCS).as_posix()
        lines += [f"![{Path(src).stem}]({rel})", "", f"*{caption}*", ""]
    return "\n".join(lines + [END])


def embed(doc: str) -> None:
    p = DOCS / doc
    text = p.read_text(encoding="utf-8")
    new = block(doc)
    if START in text:
        head, rest = text.split(START, 1)
        text = head + new + rest.split(END, 1)[1]
    else:
        text = text.rstrip("\n") + "\n\n---\n\n" + new + "\n"
    p.write_text(text, encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="only report missing sources")
    args = ap.parse_args(argv)
    missing = [(d, s) for d, items in FIGURES.items() for s, _ in items if not (OUT / s).exists()]
    for d, s in missing:
        print(f"missing: output/{s} (for {d})")
    if args.check or missing:
        return 1 if missing else 0
    total = 0
    for doc, items in FIGURES.items():
        for src, _ in items:
            dst = asset_path(doc, src)
            shrink(OUT / src, dst)
            total += dst.stat().st_size
        embed(doc)
        print(f"{doc}: {len(items)} figure(s)")
    print(f"docs/assets: {total / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
