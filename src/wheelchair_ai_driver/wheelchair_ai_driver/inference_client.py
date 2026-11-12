"""Single-flight worker bridge. The ROS interpreter never imports ML packages."""
from pathlib import Path
import queue
import socket
import subprocess
import threading

from .protocol import receive_message, send_message


class InferenceClient:
    def __init__(self, python_path, model_path, provider='cpu', threads=2, timeout=30.0):
        if not Path(python_path).is_file():
            raise FileNotFoundError(f'Inference Python not found: {python_path}')
        parent, child = socket.socketpair()
        parent.settimeout(timeout)
        module_root = Path(__file__).resolve().parent.parent
        # -I ignores inherited ROS PYTHONPATH and user site-packages. Add only this package.
        bootstrap = (f'import sys, runpy; sys.path.insert(0, {str(module_root)!r}); '
                     'runpy.run_module("wheelchair_ai_driver.inference_worker", run_name="__main__")')
        try:
            self.process = subprocess.Popen([
                str(python_path), '-I', '-c', bootstrap,
                '--socket-fd', str(child.fileno()), '--model', str(model_path),
                '--provider', provider, '--threads', str(threads),
            ], pass_fds=(child.fileno(),))
        except Exception:
            parent.close()
            raise
        finally:
            child.close()
        self.connection = parent
        self.requests = queue.Queue(maxsize=1)
        self.responses = queue.Queue()
        self.ready = threading.Event()
        self.closed = threading.Event()
        self.busy = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            greeting = receive_message(self.connection)
            self.responses.put(greeting)
            if greeting.get('type') != 'ready':
                return
            self.ready.set()
            while not self.closed.is_set():
                request = self.requests.get()
                if request is None:
                    return
                send_message(self.connection, request)
                self.responses.put(receive_message(self.connection))
        except Exception as error:
            if not self.closed.is_set():
                self.responses.put({'type': 'fatal', 'error': str(error)})
        finally:
            self.ready.clear()

    def submit(self, request):
        if self.busy or not self.ready.is_set() or self.closed.is_set():
            return False
        self.requests.put_nowait(request)
        self.busy = True
        return True

    def poll(self):
        messages = []
        while True:
            try:
                message = self.responses.get_nowait()
            except queue.Empty:
                return messages
            if message.get('type') in ('result', 'error', 'fatal'):
                self.busy = False
            messages.append(message)

    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        self.ready.clear()
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass
        self.connection.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.thread.join(timeout=1)
