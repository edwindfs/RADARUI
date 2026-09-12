"""Asynchronous worker client for LDRS packet processing."""

import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .ldrs_processing_worker import process_message


class LdrsWorkerClient:
    """Asynchronous worker client that processes incoming radar packets without blocking network threads."""

    def __init__(self, max_pending: int = 128):
        self.max_pending = max_pending
        self.pending = 0
        self.sequence = 0
        self.dropped = 0
        self.stopping = False

        self._queue: queue.Queue = queue.Queue(maxsize=max_pending)
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._callbacks: Dict[str, List[Callable[[Any], None]]] = {
            "message": [],
            "error": [],
            "exit": [],
        }

        self.start()

    def on(self, event: str, callback: Callable[[Any], None]) -> None:
        """Register an event listener for 'message', 'error', or 'exit'."""
        if event in self._callbacks:
            self._callbacks[event].append(callback)

    def _emit(self, event: str, data: Any) -> None:
        for cb in self._callbacks.get(event, []):
            try:
                cb(data)
            except Exception as e:
                print(f"[WORKER][CALLBACK ERROR] {e}")

    def start(self) -> None:
        """Start worker thread."""
        self.stopping = False
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def _worker_loop(self) -> None:
        while not self.stopping:
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if item is None:  # Termination sentinel
                break

            seq, packet, context = item
            thread_id = threading.get_ident()

            try:
                result = process_message(packet, context)
                with self._lock:
                    self.pending = max(0, self.pending - 1)

                msg = {
                    "seq": seq,
                    "worker": {"isMainThread": False, "threadId": thread_id},
                    **result,
                }
                self._emit("message", msg)

            except Exception as err:
                with self._lock:
                    self.pending = max(0, self.pending - 1)

                self._emit("error", err)

        self._emit("exit", 0)

    def submit(self, buffer: bytes, context: Optional[Dict[str, Any]] = None) -> bool:
        """Submit packet to worker queue. Returns True if queued, False if dropped."""
        if self.stopping:
            return False

        with self._lock:
            if self.pending >= self.max_pending:
                self.dropped += 1
                return False

            self.pending += 1
            self.sequence += 1
            seq = self.sequence

        try:
            self._queue.put_nowait((seq, bytes(buffer), context or {}))
            return True
        except queue.Full:
            with self._lock:
                self.pending = max(0, self.pending - 1)
                self.dropped += 1
            return False

    def get_stats(self) -> Dict[str, int]:
        """Get current queue pending count and dropped packet count."""
        with self._lock:
            return {
                "pending": self.pending,
                "dropped": self.dropped,
            }

    def terminate(self) -> None:
        """Terminate worker thread gracefully."""
        self.stopping = True
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
