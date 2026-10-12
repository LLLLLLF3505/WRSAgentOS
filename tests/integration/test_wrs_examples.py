"""Run the persistent WRS node, client and read-only viewer as separate processes."""

import asyncio
import contextlib
import runpy
import secrets
import socket

import httpx
import pytest
from conftest import eventually

from wrs_agent import System
from wrs_agent.processes import NO_WINDOW, ROOT, python_command

pytestmark = [pytest.mark.zenoh, pytest.mark.wrs]


def port_open(port):
    with socket.socket() as probe:
        probe.settimeout(0.1)
        return probe.connect_ex(("127.0.0.1", port)) == 0


async def test_persistent_wrs_control_and_viewer_do_not_own_remote_execution(monkeypatch, tmp_path):
    monkeypatch.setenv("WRS_AGENT_TOKEN", secrets.token_urlsafe(32))
    # Keep user-run example processes and their recovery journals untouched.
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        node_port = probe.getsockname()[1]
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        view_port = probe.getsockname()[1]
    env_id = "wrs-example-" + secrets.token_hex(8)
    for source in (ROOT / "examples/wrs").iterdir():
        if source.suffix in {".py", ".toml"}:
            contents = source.read_text(encoding="utf-8")
            contents = contents.replace("7449", str(node_port)).replace("wrs-demo", env_id)
            if source.name == "07_viewer.py":
                contents = contents.replace("VIEWER_PORT = 8000", f"VIEWER_PORT = {view_port}")
                # Limit observation only; leave the actual scene and polling logic intact.
                contents = contents.replace(
                    "world.schedule_interval(update, 0.1)",
                    "world.schedule_interval(update, 0.1)\n"
                    "        world.schedule_once(lambda dt: world.close(), 9.0)",
                )
            (tmp_path / source.name).write_text(contents, encoding="utf-8")
    entry = runpy.run_path(str(tmp_path / "05_start_node.py"))
    server = asyncio.create_task(entry["main"]())
    viewer = None
    viewer_log = tmp_path / "viewer.log"
    try:
        await eventually(lambda: port_open(node_port), bool, timeout=15)
        async with System.connect(
            f"tcp/127.0.0.1:{node_port}", env_id=env_id, bindings=tmp_path / "bindings.toml"
        ) as observer:
            # Agent initialization no longer implies WRS initialization has finished.
            # HELD is initialized too: recovery must remain an explicit client action.
            async def ready():
                try:
                    nodes = await observer.nodes()
                    return nodes.get("agent", {}).get("ready") and nodes.get("wrs", {}).get(
                        "health"
                    ) in {"ready", "held"}
                except TimeoutError:
                    return False

            await eventually(ready, bool, timeout=15)
            initial_scene = await observer.snapshot()
            boot = initial_scene.boot_id
            assert initial_scene.data.kind == "scene"
            assert set(initial_scene.data.objects) == {
                "table",
                "pot",
                "pot_lid",
                "induction_cooker",
                "measuring_cup",
                "skimmer",
                "plate",
                "plate_block",
            }
            assert initial_scene.data.objects["plate"].source == "configuration"
            assert initial_scene.data.objects["plate"].pos == pytest.approx([0.22, 0.20, 0.0])
            with viewer_log.open("w", encoding="utf-8") as log:
                viewer = await asyncio.create_subprocess_exec(
                    *python_command(tmp_path / "07_viewer.py"),
                    cwd=tmp_path,
                    stdout=log,
                    stderr=log,
                    creationflags=NO_WINDOW,
                )
                await eventually(lambda: port_open(view_port), bool, timeout=15)
                async with httpx.AsyncClient(trust_env=False) as http:
                    page = await http.get(f"http://127.0.0.1:{view_port}")
                    assert page.status_code == 200 and "canvas" in page.text
                client = await asyncio.create_subprocess_exec(
                    *python_command(tmp_path / "06_control_arm.py"),
                    cwd=tmp_path,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                    creationflags=NO_WINDOW,
                )
                try:
                    output, error = await asyncio.wait_for(client.communicate(), 20)
                    assert client.returncode == 0, (output + error).decode("utf-8")
                    assert "任务结果： SUCCEEDED" in output.decode("utf-8")
                finally:
                    if client.returncode is None:
                        client.terminate()
                        await client.wait()
                assert await asyncio.wait_for(viewer.wait(), 15) == 0, viewer_log.read_text("utf-8")
            assert not port_open(view_port)
            final_scene = await observer.snapshot()
            assert not server.done() and final_scene.boot_id == boot
            assert final_scene.data.objects == initial_scene.data.objects
            assert final_scene.state_version > initial_scene.state_version
            motion = await observer.action("move_named_pose", pose="home")
            assert (await motion.wait()).state == "SUCCEEDED"
    finally:
        if viewer is not None and viewer.returncode is None:
            viewer.terminate()
            await asyncio.wait_for(viewer.wait(), 5)
        server.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await server
    assert not port_open(node_port) and not port_open(view_port)

    # Re-run the same node/journal: explicit client recovery admits only a new task.
    restarted = asyncio.create_task(entry["main"]())
    try:
        await eventually(lambda: port_open(node_port), bool, timeout=15)
        async with System.connect(
            f"tcp/127.0.0.1:{node_port}", env_id=env_id, bindings=tmp_path / "bindings.toml"
        ) as observer:

            async def ready_again():
                nodes = await observer.nodes()
                return nodes.get("agent", {}).get("ready") and nodes.get("wrs", {}).get(
                    "health"
                ) in {"ready", "held"}

            await eventually(ready_again, bool, timeout=15)
            before = await observer.snapshot()
            assert before.boot_id != boot and before.admission == "HELD"
            client = await asyncio.create_subprocess_exec(
                *python_command(tmp_path / "06_control_arm.py"),
                cwd=tmp_path,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                creationflags=NO_WINDOW,
            )
            try:
                output, error = await asyncio.wait_for(client.communicate(), 20)
                assert client.returncode == 0, (output + error).decode("utf-8")
                assert "SUCCEEDED" in output.decode("utf-8")
            finally:
                if client.returncode is None:
                    client.terminate()
                    await client.wait()
    finally:
        restarted.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await restarted
    assert not port_open(node_port)
