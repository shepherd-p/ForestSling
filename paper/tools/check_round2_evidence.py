"""Check archived attempts, snapshot hashes and the declared motor perturbation."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT if (ROOT / 'runs').is_dir() else ROOT.parent
sys.path.insert(0, str(LAB))
from forestlab.inertial_identification import decode


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    run = LAB / 'runs/ast_round2_20261027'
    protocol = read(run / 'PROTOCOL.json')
    assert len(protocol['jobs']) == 30
    checks = []
    count = 0
    for job in protocol['jobs']:
        folder = run / job['name']
        s = read(folder / 'SUMMARY.json')
        p = read(folder / 'PROTOCOL.json')
        assert p['seed'] == job['seed'] and p['method'] == job['method']
        assert p['perturbation'] == job['perturbation']
        hashes = read(folder / 'SOURCE_SHA256.json')
        for name, expected in hashes.items():
            source = folder / 'source_snapshot' / name.replace('\\', '/')
            assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, str(source)
            count += 1
        trajectory = folder / 'trajectory.npz'
        if not trajectory.exists():
            assert job['name'] == '4_rectangular_unconstrained_28000'
            assert s['reason'] == 'model_inertia_nonphysical'
            assert 'software_exit' in s
            assert (run / (job['name'] + '.log')).exists()
        else:
            data = np.load(trajectory)
            assert np.isfinite(data['states']).all()
        if job['perturbation'] == 'motor_tau':
            import mujoco
            model = mujoco.MjModel.from_xml_path(str(folder / 'model.xml'))
            assert np.allclose(model.actuator_dynprm[:, 0], .0375)
            assert abs(p['actual_motor_time_constant_sec'] - .0375) < 1e-12
        if job['method'] == 'moment_psd':
            assert decode(read(folder / 'CONTROL_MODEL.json')['theta'])['physical']
        checks.append(dict(case=job['name'],completed=s['completed'],accepted=s['passed'],
                           trajectory_available=trajectory.exists(),source_files_verified=len(hashes)))
    stats = read(ROOT / 'supplementary/round2_20261027/REPEATED_SEED_STATISTICS.json')
    assert stats['passed'] == 20 and stats['completed'] == 20 and stats['gate_sec'] == .35
    out = dict(passed=True,meaning='Archive integrity and declared configuration checks; not a claim that every trial passed its execution gates',
               initial_attempts=30,source_snapshot_files_verified=count,
               raw_ols_rejection_preserved=True,motor_perturbation_checked_from_native_xml=True,
               process_cohort_accepted=20,checks=checks)
    (ROOT / 'supplementary/round2_20261027/EVIDENCE_CHECK.json').write_text(
        json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in out.items() if k!='checks'},indent=2))


if __name__ == '__main__':
    main()
