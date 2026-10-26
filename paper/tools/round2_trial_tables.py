"""Generate the compact representative perturbation table from archived trials."""
from pathlib import Path
import csv
import json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'supplementary/round2_20261027'


def main():
    trials = json.loads((OUT / 'TRIAL_METRICS.json').read_text(encoding='utf-8'))
    rows = [v for v in trials if v['perturbation'] != 'none']
    assert len(rows) == 6, 'Wait for all six initial perturbation attempts'
    labels = {'tension_bias': 'Tension bias', 'motor_tau': 'Motor lag',
              'observation_loss': 'Observation loss'}
    lines = []
    for shape, case in [('rectangular', 'Box'), ('forked', 'Fork')]:
        for mode in labels:
            r = next(v for v in rows if v['shape'] == shape and v['perturbation'] == mode)
            planned = r['motion_start_sec'] is not None
            result = '/'.join('Y' if flag else 'N' for flag in (planned, r['completed'], r['passed']))
            angle = '--' if r['max_attitude_error_deg'] is None else f"{r['max_attitude_error_deg']:.2f}"
            tension = f"{r['min_tension_n']:.2f}--{r['max_tension_n']:.2f}"
            clearance = f"{r['minimum_aircraft_clearance_m']*100:.2f}" if planned and 'minimum_aircraft_clearance_m' in r else '--'
            lines.append(f"{case} & {labels[mode]} & {result} & {angle} & {tension} & {clearance}" + r'\\')
    (ROOT / 'manuscript/table_perturbation_rows.tex').write_text(
        '\n'.join(lines).rstrip('\\') + '\n', encoding='utf-8')
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (OUT / 'PERTURBATION_TRIALS.csv').open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (OUT / 'IDENTIFICATION_BASELINES.json').write_text(json.dumps(
        [v for v in trials if v['method'] in ('moment_psd', 'unconstrained')], indent=2) + '\n', encoding='utf-8')
    print('Six perturbation rows and four baseline records generated')


if __name__ == '__main__':
    main()
