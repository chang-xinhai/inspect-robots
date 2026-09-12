"""Codex wire preserves tool contracts without API authentication or robot tools."""

import base64
import json

import pytest

from inspect_robots.errors import ConfigError
from inspect_robots_agent import LLMAgentPolicy
from inspect_robots_agent._codex import CodexClient, subscription_environment
from inspect_robots_agent._llm import Provider


def fake_codex(tmp_path, monkeypatch, payload):
    executable = tmp_path / "codex"
    executable.write_text(
        "#!/usr/bin/env python3\n"
        + "import sys,json,pathlib\n"
        + "if sys.argv[1:]==['login','status']:\n print('Logged in using ChatGPT');sys.exit(0)\n"
        + "pathlib.Path('received.txt').write_text(sys.stdin.read())\n"
        + "pathlib.Path(sys.argv[sys.argv.index('-o')+1]).write_text("
        + repr(json.dumps(payload))
        + ")\n"
    )
    executable.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path) + ":" + __import__("os").environ["PATH"])


def test_native_history_images_tools_and_capture(tmp_path, monkeypatch):
    payload = {
        "content": "observed",
        "tool_calls": [{"name": "move_to", "arguments": '{"targets":{"x":0.4},"note":"move"}'}],
    }
    fake_codex(tmp_path, monkeypatch, payload)
    client = CodexClient(Provider("codex://chatgpt", "", "gpt-5.5"))
    client.set_trace_dir(tmp_path / "traces")
    messages = [
        {"role": "system", "content": "native instructions"},
        {"role": "tool", "tool_call_id": "old", "content": "execution rejected"},
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": "data:image/png;base64," + base64.b64encode(b"png-test").decode()
                    },
                }
            ],
        },
    ]
    original = json.dumps(messages)
    result = client.complete(
        messages, [{"type": "function", "function": {"name": "move_to", "parameters": {}}}]
    )
    assert result.tool_calls[0].name == "move_to"
    assert json.loads(result.tool_calls[0].arguments)["targets"]["x"] == 0.4
    assert json.dumps(messages) == original
    prompt = next((tmp_path / "traces").glob("*/prompt.txt")).read_text()
    assert "execution rejected" in prompt and "attached image 1" in prompt
    assert "native instructions" in prompt
    command = json.loads(next((tmp_path / "traces").glob("*/command.json")).read_text())
    assert "features.shell_tool=false" in command
    assert 'forced_login_method="chatgpt"' in command


def test_policy_codex_does_not_resolve_api_keys(tmp_path, monkeypatch):
    fake_codex(tmp_path, monkeypatch, {"content": None, "tool_calls": []})
    policy = LLMAgentPolicy(wire="codex", model="openai/gpt-5.5", env={})
    assert isinstance(policy._client, CodexClient)
    assert policy.config.wire == "codex"
    assert policy.config.model == "gpt-5.5"
    assert policy.config.image_horizon == 2
    assert policy.config.max_speed_frac == 0.1


def test_subscription_environment_and_invalid_settings(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "must-not-inherit")
    monkeypatch.setenv("CODEX_API_KEY", "must-not-inherit")
    assert "OPENAI_API_KEY" not in subscription_environment()
    assert "CODEX_API_KEY" not in subscription_environment()
    with pytest.raises(ConfigError, match="omit API"):
        LLMAgentPolicy(wire="codex", base_url="https://example.org")
    with pytest.raises(ConfigError, match="named effort"):
        LLMAgentPolicy(wire="codex", effort=0.5, env={})


def test_invalid_tool_fails_closed(tmp_path, monkeypatch):
    fake_codex(
        tmp_path,
        monkeypatch,
        {"content": None, "tool_calls": [{"name": "shell", "arguments": "{}"}]},
    )
    client = CodexClient(Provider("codex://chatgpt", "", "gpt-5.5"))
    with pytest.raises(ValueError, match="unknown tool"):
        client.complete([], [{"type": "function", "function": {"name": "done"}}])
