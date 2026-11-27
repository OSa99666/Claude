import os
import signal
import socket
import subprocess
import time

import pytest

TEST_PORT = 48600


@pytest.fixture(autouse=True)
def _kill_test_hubs():
    yield
    out = subprocess.run(["pgrep", "-f", "plchub.hub --port %d" % TEST_PORT], capture_output=True, text=True).stdout.split()
    for pid in out:
        try:
            os.kill(int(pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    # wait until the port is really free (a killed hub may take a moment to disappear)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5:
        with socket.socket() as sk:
            sk.settimeout(0.2)
            if sk.connect_ex(("127.0.0.1", TEST_PORT)) != 0:
                break
        time.sleep(0.1)
