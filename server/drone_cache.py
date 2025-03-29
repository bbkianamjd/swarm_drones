import threading
from typing import Dict, Set, List
from dataclasses import dataclass

from data_defs import DroneRequest
from read_write_lock import ReadWriteLock
from v_buffer import VBuffer


class DroneCache:
    def __init__(self):
        self._id_set: Set[str] = set()
        self._v_buffers: Dict[str, VBuffer] = {}

        self._id_set_lock = ReadWriteLock()
        self._v_buffer_lock = ReadWriteLock()

    def store_drone_state(self, state: DroneRequest):
        write_to_buffer = False

        # Step 1: Ensure the drone_id is known
        self._id_set_lock.acquire_read()
        try:
            if state.drone_id not in self._id_set:
                write_to_buffer = True
        finally:
            self._id_set_lock.release_read()

        # Add a new buffer for this drone (under write lock)
        if write_to_buffer:
            self._id_set_lock.acquire_write()
            self._v_buffer_lock.acquire_write()
            try:
                self._id_set.add(state.drone_id)
                self._v_buffers[state.drone_id] = VBuffer()
            finally:
                self._id_set_lock.release_write()
                self._v_buffer_lock.release_write()

        # Step 2: Write the state to the VBuffer
        self._v_buffer_lock.acquire_read()
        try:
            self._v_buffers[state.drone_id].write(state)
        finally:
            self._v_buffer_lock.release_read()

    def get_all_states(self) -> List[DroneRequest]:
        result = []
        self._v_buffer_lock.acquire_read()
        try:
            for v_buffer in self._v_buffers.values():
                result.append(v_buffer.read())
        finally:
            self._v_buffer_lock.release_read()

        return result

