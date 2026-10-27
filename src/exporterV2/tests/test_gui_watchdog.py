"""Exercise real independent watchdog without Isaac or a GPU stall."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def test_watchdog_captures_stack_and_does_not_kill(tmp_path):
    src = Path(__file__).resolve().parents[2]
    code = '''
import time
from exporterV2.gui_watchdog import GuiWatchdog
w = GuiWatchdog(DIRECTORY)
w.mark('physics.test_stall', mode='running', simulation_s=1.5)
time.sleep(7)
w.mark('cleanup.test', mode='cleanup')
time.sleep(7)
w.close()
'''.replace('DIRECTORY', repr(str(tmp_path)))
    result = subprocess.run([sys.executable, '-c', code], env={**os.environ, 'PYTHONPATH': str(src)}, timeout=25)
    assert result.returncode == 0
    rows = [json.loads(line) for line in (tmp_path/'watchdog.jsonl').read_text().splitlines()]
    assert any(r.get('event') == 'gui_stall' and r['stack_requested'] for r in rows)
    assert any(r.get('event') == 'cleanup_stall' for r in rows)
    assert rows[-1]['state']['mode'] == 'finished'
    assert '<module>' in (tmp_path/'python-stacks.log').read_text()
    assert 'physics.test_stall' in (tmp_path/'gui-phases.jsonl').read_text()
