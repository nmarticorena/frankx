import logging
import time
from enum import Enum
from threading import Thread
from multiprocessing import Process, Event, Value
from _frankx import Gripper as _Gripper
from _frankx import CommandException, NetworkException

def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()

        if "log_time" in kw:
            name = kw.get("log_name", method.__name__.upper())
            kw["log_time"][name] = int((te - ts) * 1000)
        else:
            print("%r  %2.2f ms" % (method.__name__, (te - ts) * 1000))
        return result

    return timed

log = logging.getLogger(__name__)

class GripperState(Enum):
    OPEN = 1
    INITIALIZED = 0
    CLOSED = -1


class Gripper:
    def __init__(self, fci_ip, speed=0.04, force=20, timeout= 5, opening_threshold= 0.05, closing_threshold = 0.01):
        self._width = Value('d', -1)
        self._open_gripper_event = Event()
        self._close_gripper_event = Event()
        self._gripper = GripperProcess(fci_ip, speed, force, self._width, self._open_gripper_event, self._close_gripper_event)
        self._gripper.start()
        self._timeout = timeout
        self._closing_threshold = opening_threshold
        self._opening_threshold = closing_threshold
        # self._gripper_state_thread = None
        # self.prev_width = self.read_once().width
        # self.results = [None]

    def width(self):
        return self._width.value

    def open(self, blocking=False):
        self._close_gripper_event.clear()
        self._open_gripper_event.set()
        if blocking:
            start_time = time.time()
            while time.time() - start_time < self._timeout and self.width() < self._opening_threshold:
                time.sleep(0.1)

    def close(self, blocking=False):
        self._open_gripper_event.clear()
        self._close_gripper_event.set()
        if blocking:
            start_time = time.time()
            while time.time() - start_time < self._timeout and self.width() > self._closing_threshold:
                time.sleep(0.1)

    # def get_width(self):
    #     if self._gripper_state_thread is None:
    #         t = time.time()
    #         self._gripper_state_thread = Thread(target=self._get_width, args=(self.results, ), daemon=True)
    #         print((time.time() - t) * 1000)
    #         t = time.time()
    #         self._gripper_state_thread.start()
    #         print((time.time() - t) * 1000)
    #     if self._gripper_state_thread.is_alive():
    #         return self.prev_width
    #     else:
    #         self.prev_width = self.results[0]
    #         self._gripper_state_thread = None
    #         return self.prev_width
    #
    #
    # def _get_width(self, results):
    #     width = self.read_once().width
    #     results[0] = width


class GripperProcess(Process):
    def __init__(self, fci_ip, gripper_speed, gripper_force, _width, _open_gripper_event, _close_gripper_event):
        self.fci_ip = fci_ip
        self.gripper_speed = gripper_speed
        self.gripper_force = gripper_force
        self._width = _width
        self._open_gripper_event = _open_gripper_event
        self._close_gripper_event = _close_gripper_event
        self._gripper = None
        self.gripper_state = None
        self.initialize()
        self.gripper_thread = None
        Process.__init__(self)
        self.daemon = True

    def initialize(self):
        del self._gripper
        self._gripper = _Gripper(self.fci_ip, self.gripper_speed, self.gripper_force)
        self._width.value = self._gripper.read_once().width
        self.gripper_state = GripperState.INITIALIZED

    def run(self):
        while 1:
            try:
                self._width.value = self._gripper.read_once().width
            except NetworkException:
                log.warning("libfranka: UDP receive: Timeout. Could not read gripper width. Restarting Gripper")
                self.initialize()
            if self._close_gripper_event.is_set():
                if self.gripper_state != GripperState.CLOSED and (self.gripper_thread is None or (self.gripper_thread is not None and not self.gripper_thread.is_alive())):
                    self.gripper_thread = self.move_async_grasp(0)
                    self.gripper_state = GripperState.CLOSED
                self._close_gripper_event.clear()
            elif self._open_gripper_event.is_set():
                if self.gripper_state != GripperState.OPEN and (self.gripper_thread is None or (self.gripper_thread is not None and not self.gripper_thread.is_alive())):
                    self.gripper_thread = self.move_async(0.085)
                    self.gripper_state = GripperState.OPEN
                self._open_gripper_event.clear()
            time.sleep(0.001)

    def move_async(self, width) -> Thread:
        p = Thread(target=self._gripper.move_unsafe, args=(width, ), daemon=True)
        p.start()
        return p

    def move_async_grasp(self, width) -> Thread:
        p = Thread(target=self._gripper.clamp, daemon=True)
        p.start()
        return p
