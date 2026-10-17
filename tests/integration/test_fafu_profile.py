"""Run actual default FAFU model outside the pytest process."""

import asyncio
import json

import pytest

from wrs_agent.processes import NO_WINDOW, ROOT, python_command

pytestmark = pytest.mark.wrs


async def test_default_fafu_profile_and_motion():
    process = await asyncio.create_subprocess_exec(
        *python_command(ROOT / "tests/fixtures/fafu_profile_probe.py"),
        cwd=ROOT,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        creationflags=NO_WINDOW,
    )
    try:
        output, error = await asyncio.wait_for(process.communicate(), 40)
        assert process.returncode == 0, (output + error).decode("utf-8")
        report = json.loads(output.decode("utf-8").splitlines()[-1])
        assert report == {
            "result": "PASS",
            "objects": 17,
            "active_joints": 6,
            "named_moves": 4,
            "relative_moves": 4,
        }
    finally:
        if process.returncode is None:
            process.terminate()
            await process.wait()
