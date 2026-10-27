"""Record native viewport drag events without replacing PhysX interaction."""
import json
import time


class NativeDragObserver:
    def __init__(self, records, output, watchdog=None):
        self.watchdog = watchdog
        import omni.physxui.scripts.physxViewportOverlays as overlay
        import carb.settings
        from omni.physx import get_physx_interface, get_physx_scene_query_interface
        from omni.physx.bindings._physx import PhysicsInteractionEvent
        self.native = get_physx_interface()
        self.query = get_physx_scene_query_interface().raycast_closest
        self.events = PhysicsInteractionEvent
        self.records = {r['fruit']: r for r in records}
        self.active = None
        self.time_s = 0.
        self.log = output.open('w')
        settings = carb.settings.get_settings()
        self.summary = {'mode': 'native_force' if settings.get('/physics/forceGrab') else 'native_joint',
                        'coefficient': settings.get('/physics/pickingForce'),
                        'native_force_newtons': None, 'grabs': []}
        self.overlay, self.original_factory = overlay, overlay.get_physx_interface
        self.factory = lambda: self
        overlay.get_physx_interface = self.factory

    def __getattr__(self, name):
        return getattr(self.native, name)

    def update_interaction(self, origin, direction, event):
        watchdog = self.__dict__.get("watchdog")
        previous_phase = watchdog.state["phase"] if watchdog else None
        def mark(phase):
            if watchdog:
                watchdog.mark(phase)
        # Forward the original objects/event exactly once, including on logging failure.
        try:
            row = {'time_s': self.time_s, 'monotonic_s': time.monotonic(), 'event': str(event),
                   'origin': list(origin), 'direction': list(direction)}
            if event == self.events.MOUSE_DRAG_BEGAN:
                mark("mouse.raycast")
                hit = self.query(tuple(origin), tuple(direction), 1e4)
                body = str(hit.get('rigidBody', ''))
                self.active = self.records.get(body)
                row.update(event='begin', body=body, collider=str(hit.get('collision', '')),
                           hit=bool(hit.get('hit')), point=list(hit.get('position', ())))
                self.summary['grabs'].append(row.copy())
            elif event == self.events.MOUSE_DRAG_ENDED:
                row['event'] = 'release'
                self.active = None
            else:
                row['event'] = 'move'
            mark("mouse.record")
            self.log.write(json.dumps(row)+'\n')
            self.log.flush()
        except Exception as error:
            self.summary.setdefault('logging_errors', []).append(str(error))
        try:
            mark("mouse.physx")
            return self.native.update_interaction(origin, direction, event)
        finally:
            if watchdog:
                mark(previous_phase)

    def before_step(self, time_s, dt, broken):
        self.time_s = time_s
        return 0.  # No force command; native force is unknown, never measured zero.

    def handle_break(self, joint, time_s):
        self.time_s = time_s
        return bool(self.active and self.active['joint'] == joint)

    def close(self, time_s):
        if self.overlay.get_physx_interface is self.factory:
            self.overlay.get_physx_interface = self.original_factory
        self.factory = None
        self.log.close()
