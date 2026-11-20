"""Serialize shared FlightLink access while polling independently of vision.

Only this adapter uses host threading. Board communication and synchronization
remain in the shared MicroPython-compatible PnP FlightLink implementation.
"""
import threading


class ThreadSafeLink:
    def __init__(self, link, interval_s=.002):
        object.__setattr__(self, '_link', link)
        object.__setattr__(self, '_lock', threading.RLock())
        object.__setattr__(self, '_interval_s', float(interval_s))
        object.__setattr__(self, '_stop', threading.Event())
        object.__setattr__(self, '_error', None)
        object.__setattr__(self, '_clock_generation', getattr(link.clock, 'generation', None))
        object.__setattr__(self, '_thread', threading.Thread(
            target=self._run, name='pnp-mavlink', daemon=True))

    @property
    def lock(self):
        return self._lock

    @property
    def error(self):
        with self._lock:
            return self._error

    def __getattr__(self, name):
        with self._lock:
            value = getattr(self._link, name)
        if not callable(value):
            return value
        def call(*args, **kwargs):
            with self._lock:
                if self._error is not None and name in ('poll', 'send_velocity', 'send_command', 'request_mode'):
                    return False if name != 'poll' else None
                return getattr(self._link, name)(*args, **kwargs)
        return call

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
        else:
            with self._lock:
                setattr(self._link, name, value)

    def state(self, now=None):
        with self._lock:
            state = dict(self._link.state(now))
            if self._error is not None:
                state.update(ready=False, reason='LINK_WORKER_FAILED', local_position_ready=False)
            return state

    def comm_snapshot(self, now=None):
        with self._lock:
            # The logger samples its scheduling clock before acquiring this
            # lock; telemetry may have arrived since then. Snapshot health and
            # sample ages at the actual locked observation time.
            snapshot = dict(self._link.comm_snapshot(self._link.clock.now_us()))
            if self._error is not None:
                snapshot['reason'] = 'LINK_WORKER_FAILED'
            return snapshot

    def reset_clock_generation(self, generation):
        """Reset once even when the clock callback precedes the ROS callback."""
        with self._lock:
            if generation != self._clock_generation:
                # A main-thread call to the shared poll() may already have
                # detected the epoch change while processing an image.
                if getattr(self._link, '_clock_generation', object()) != generation:
                    self._link._restart()
                self._clock_generation = generation

    def start(self):
        self._thread.start()

    def _run(self):
        try:
            while not self._stop.is_set():
                with self._lock:
                    self.reset_clock_generation(getattr(self._link.clock, 'generation', None))
                    self._link.poll()
                self._stop.wait(self._interval_s)
        except Exception as error:
            with self._lock:
                self._error = repr(error)
                try:
                    self._link.cancel_setpoint()
                    self._link._event('LINK_WORKER_FAILED', reason=self._error)
                except Exception:
                    # Keep the original I/O failure visible even if best-effort
                    # cancellation or diagnostic recording also fails.
                    pass

    def close(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=2.)
            if self._thread.is_alive():
                raise RuntimeError('MAVLink worker did not stop within 2 seconds')
