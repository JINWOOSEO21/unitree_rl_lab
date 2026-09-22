"""Bounded polling readers for MIT bridge, using public CycloneDDS APIs.

The SDK's listener-based reader can deadlock on deletion while the DDS callback
is running. KeepLast(1) readers are polled on owned threads; close joins the
thread before deleting DDS entities. No SDK/system files are changed.
"""
import threading
import gc
from contextlib import contextmanager


def subscriber_type(domain):
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.internal import InvalidSample
    participant = DomainParticipant(domain)

    class PollingSubscriber:
        def __init__(self, name, typ):
            self.topic = Topic(participant, name, typ)
            self.reader = DataReader(participant, self.topic, Qos(Policy.History.KeepLast(1)))
            self.stop = threading.Event()
            self.thread = None
            self.error = None

        def Init(self, callback, queue=0):
            def poll():
                try:
                    while not self.stop.is_set():
                        samples = self.reader.take(1)
                        if not samples:
                            self.stop.wait(.001)
                            continue
                        for sample in samples:
                            if not isinstance(sample, InvalidSample):
                                callback(sample)
                except Exception as exc:
                    self.error = exc
                    self.stop.set()
            self.thread = threading.Thread(target=poll, name='mit_dds_reader', daemon=True)
            self.thread.start()

        def check(self):
            if self.error is not None:
                raise RuntimeError('MIT DDS reader failed') from self.error

        def Close(self):
            self.stop.set()
            if self.thread is not None:
                self.thread.join(timeout=2.)
                if self.thread.is_alive():
                    raise RuntimeError('MIT DDS reader did not stop')
            self.reader = None
            self.topic = None

    return PollingSubscriber


@contextmanager
def initialized_heap_gc_scope():
    """Exclude long-lived initialized objects from repeated full GC scans.

    Jetson profiling found a ~155 ms generation-2 scan (zero garbage) blocking
    both readers during startup. Collect before subscriptions, then freeze the
    initialized heap for this run. New cyclic garbage is still collected; GC
    is not disabled. Restore normal tracking on exit. Respect an existing
    caller-owned permanent generation without unfreezing it.
    """
    owned = gc.isenabled() and gc.get_freeze_count() == 0
    if owned:
        gc.collect()
        gc.freeze()
    try:
        yield
    finally:
        if owned:
            gc.unfreeze()
