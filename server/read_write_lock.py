import threading

class ReadWriteLock:
    def __init__(self):
        self._readers = 0
        self._readers_lock = threading.Lock()
        self._resource_lock = threading.Lock()
        self._reader_cond = threading.Condition(self._readers_lock)

    def acquire_read(self):
        with self._reader_cond:
            self._readers += 1
            if self._readers == 1:
                # First reader locks the resource (blocks writers)
                self._resource_lock.acquire()

    def release_read(self):
        with self._reader_cond:
            self._readers -= 1
            if self._readers == 0:
                # Last reader releases the lock
                self._resource_lock.release()

    def acquire_write(self):
        self._resource_lock.acquire()

    def release_write(self):
        self._resource_lock.release()
