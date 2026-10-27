"""Opt-in GUI phase journal and independent, non-terminating stall observer."""
import faulthandler
import json
import mmap
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

SIZE = 16384


class GuiWatchdog:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.shared = (self.directory / 'watchdog-state.bin').open('w+b')
        self.shared.truncate(SIZE)
        self.memory = mmap.mmap(self.shared.fileno(), SIZE)
        self.journal = (self.directory / 'gui-phases.jsonl').open('w', buffering=65536)
        self.stacks = (self.directory / 'python-stacks.log').open('w')
        faulthandler.register(signal.SIGUSR1, file=self.stacks, all_threads=True)
        self.state = dict(pid=os.getpid(), phase='startup', mode='startup', simulation_s=0., frame=0)
        self.last_flush = time.monotonic()
        self.mark('startup')
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                                         str(self.directory), str(os.getpid())],
                                        start_new_session=True, close_fds=True)

    def mark(self, phase, **values):
        now = time.monotonic()
        self.journal.write(json.dumps(dict(event='exit', monotonic_s=now, **self.state)) + '\n')
        self.state.update(values, phase=phase)
        self.state['progress_monotonic_s'] = now
        self.journal.write(json.dumps(dict(event='enter', monotonic_s=now, **self.state)) + '\n')
        data = json.dumps(self.state).encode()
        # Publish payload first and length last; reader validates JSON and retries.
        self.memory[:8] = b'00000000'
        self.memory[8:8+len(data)] = data
        self.memory[:8] = f'{len(data):08d}'.encode()
        if now - self.last_flush >= 1 or phase.startswith('cleanup'):
            self.journal.flush()
            self.last_flush = now

    def close(self):
        self.mark('finished', mode='finished')
        self.journal.close()
        self.process.wait(timeout=3)
        faulthandler.unregister(signal.SIGUSR1)
        self.stacks.close()
        self.memory.close()
        self.shared.close()


def supervise(directory, pid, timeout=5.):
    directory = Path(directory)
    with (directory / 'watchdog-state.bin').open('r+b') as source, \
         (directory / 'watchdog.jsonl').open('w', buffering=1) as log:
        memory = mmap.mmap(source.fileno(), SIZE)
        reported = None
        while True:
            try:
                length = int(memory[:8])
                state = json.loads(memory[8:8+length])
            except (ValueError, json.JSONDecodeError):
                time.sleep(.1)
                continue
            now = time.monotonic()
            try:
                proc_state = Path(f'/proc/{pid}/status').read_text()
                alive = '\nState:\tZ' not in proc_state
            except FileNotFoundError:
                proc_state, alive = 'process exited', False
            row = dict(monotonic_s=now, state=state, alive=alive)
            stale = now - state['progress_monotonic_s']
            if state['mode'] in ('running', 'paused', 'cleanup') and stale >= timeout:
                key = state['progress_monotonic_s']
                if reported != key and alive:
                    row.update(event='cleanup_stall' if state['mode'] == 'cleanup' else 'gui_stall',
                               stale_seconds=stale, process_status=proc_state)
                    try:
                        row['wchan'] = Path(f'/proc/{pid}/wchan').read_text()
                        os.kill(pid, signal.SIGUSR1)
                        row['stack_requested'] = True
                    except (OSError, ProcessLookupError) as error:
                        row['stack_error'] = str(error)
                    reported = key
            log.write(json.dumps(row)+'\n')
            if not alive or state['mode'] == 'finished':
                break
            time.sleep(1)
        memory.close()


if __name__ == '__main__':
    supervise(sys.argv[1], int(sys.argv[2]))
