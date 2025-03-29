import threading

class VBuffer:
    """
    A simplified triple-buffer (V-Buffer) in Python.

    - read_index : points to the buffer being read.
    - write_index: points to the buffer being written to.
    - temp_index : the 'spare' buffer not currently read or written.

    Reading always returns the latest 'committed' data.
    Writing swaps buffers so subsequent reads see the newly updated buffer.
    """

    def __init__(self):
        # Our three internal buffers.
        self._buffers = [None, None, None]

        # Indices into self._buffers:
        # 0 -> read, 1 -> write, 2 -> temp
        self._read_index = 0
        self._write_index = 1
        self._temp_index = 2

        # Although in CPython these swaps are typically atomic enough
        # due to the GIL, we include a lock if we want to ensure
        # explicit atomic swaps in a multi-thread scenario.
        # If you want to emphasize "lock-free," you might omit this,
        # but that can risk race conditions if multiple threads call write().
        self._swap_lock = threading.Lock()

    def read(self):
        """
        Returns the most recently committed buffer content (the latest state).
        """
        # In typical CPython, reading a single integer index and returning
        # a reference is safe. If you want to be extra safe, you can use the lock.
        idx = self._read_index
        return self._buffers[idx]

    def write(self, data):
        """
        Write new data into the buffer, then swap so future reads
        see the newly written data.
        """
        # Acquire the write index
        w_idx = self._write_index
        self._buffers[w_idx] = data

        # Swap indices so that:
        #  - read_index becomes what was write_index
        #  - write_index becomes what was temp_index
        #  - temp_index becomes what was read_index
        with self._swap_lock:
            old_read = self._read_index
            self._read_index = self._write_index
            self._write_index = self._temp_index
            self._temp_index = old_read


# Example usage
if __name__ == "__main__":
    import threading
    import time

    vbuf = VBuffer()

    def writer_thread():
        for i in range(5):
            new_data = f"Data {i}"
            vbuf.write(new_data)
            print(f"[Writer] Wrote: {new_data}")
            time.sleep(0.2)

    def reader_thread():
        last_read = None
        for _ in range(10):
            current = vbuf.read()
            if current != last_read:
                print(f"[Reader] Read:  {current}")
                last_read = current
            time.sleep(0.1)

    # Run both threads to show how reading and writing can happen concurrently.
    t1 = threading.Thread(target=writer_thread)
    t2 = threading.Thread(target=reader_thread)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
