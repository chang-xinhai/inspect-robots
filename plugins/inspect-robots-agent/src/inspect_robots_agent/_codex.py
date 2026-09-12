"""ChatGPT-authenticated Codex CLI transport for the unchanged agent tool loop."""

from __future__ import annotations

import base64
import copy
import hashlib
import io
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
import uuid
from pathlib import Path
from typing import Any

from PIL import Image

from inspect_robots.errors import ConfigError

from ._capture import WireCapture
from ._llm import AssistantMessage, Provider, ToolCall


def subscription_environment() -> dict[str, str]:
    """Prevent inherited API settings from selecting billed API authentication."""
    env = dict(os.environ)
    for key in ("CODEX_API_KEY", "OPENAI_API_KEY", "OPENAI_BASE_URL"):
        env.pop(key, None)
    return env


def _finite_json(text: str) -> Any:
    def reject(value: str) -> None:
        raise ValueError(f"nonfinite JSON constant: {value}")

    return json.loads(text, parse_constant=reject)


class CodexClient:
    """Translate native chat history and tool schemas to one isolated CLI call.

    Only transport changes: the upstream policy retains tool validation,
    interpolation, image eviction, feedback, call budget and transcript.
    """

    def __init__(
        self,
        provider: Provider,
        *,
        timeout_s: float = 120,
        capture: WireCapture | None = None,
        prior_demo_image: str | None = None,
    ):
        self._provider, self._capture = provider, capture
        self.timeout_s = timeout_s
        self.demo_image_path: str | None = None
        self.demo_image_sha256: str | None = None
        self._demo_image_url: str | None = None
        if prior_demo_image is not None:
            if not isinstance(prior_demo_image, str) or not prior_demo_image.strip():
                raise ConfigError("prior_demo_image must be a nonempty image path or None")
            try:
                path = Path(prior_demo_image).resolve()
                source = path.read_bytes()
                with Image.open(io.BytesIO(source)) as decoded:
                    png = io.BytesIO()
                    decoded.convert("RGB").save(png, format="PNG")
            except (OSError, ValueError) as exc:
                raise ConfigError(
                    f"Cannot load prior_demo_image {prior_demo_image!r}: {exc}"
                ) from exc
            self.demo_image_path = str(path)
            self.demo_image_sha256 = hashlib.sha256(source).hexdigest()
            self._demo_image_url = (
                "data:image/png;base64," + base64.b64encode(png.getvalue()).decode()
            )
        executable = shutil.which("codex")
        if executable is None:
            raise ConfigError("Codex CLI missing; install it and run codex login")
        self.executable: str = executable
        self.trace_dir: Path | None = None
        result = subprocess.run(
            [self.executable, "login", "status"],
            env=subscription_environment(),
            capture_output=True,
            text=True,
            timeout=15,
        )
        if result.returncode or "using ChatGPT" not in result.stdout + result.stderr:
            raise ConfigError("wire=codex requires ChatGPT login; run codex login")

    def set_trace_dir(self, path: Path) -> None:
        """Store CLI diagnostics alongside this trial's native wire capture."""
        self.trace_dir = path.resolve()

    def complete(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
        *,
        temperature: float | None = None,
        reasoning_effort: str | float | None = None,
    ) -> AssistantMessage:
        """Return a native AssistantMessage with the exact requested tool calls."""
        if temperature is not None:
            raise ValueError("Codex CLI does not support temperature")
        # CLI calls are stateless. Reattach the immutable demonstration separately
        # from the policy's rolling live observations; never consume image_horizon.
        messages = copy.deepcopy(messages)
        if self._demo_image_url is not None:
            demo = {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (
                            "HISTORICAL DEMONSTRATION — NOT A LIVE OBSERVATION. "
                            "This fixed reference storyboard is ordered left-to-right, "
                            "then top-to-bottom, with source timestamps. Read the whole "
                            "sequence before planning: approach, changes in viewing/grasp "
                            "orientation, contact, transport, release and withdrawal. "
                            "Do not mistake the final demonstrated state for current success. "
                            "Use the later LIVE observation and measured state for actions. "
                            "Demo timing/coordinates are not executable robot targets. "
                            "If reproducing its geometry requires an unavailable tool or "
                            "rotation, explain the limitation rather than substituting "
                            "unsupported translations. Reference source SHA256: "
                            + str(self.demo_image_sha256)
                        ),
                    },
                    {"type": "image_url", "image_url": {"url": self._demo_image_url}},
                ],
            }
            position = 0
            while position < len(messages) and messages[position].get("role") == "system":
                position += 1
            messages.insert(position, demo)
        if self.trace_dir is not None:
            self.trace_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="inspect-codex-") as temporary:
            directory = (
                self.trace_dir / uuid.uuid4().hex if self.trace_dir is not None else Path(temporary)
            )
            directory.mkdir(parents=True, exist_ok=True)
            return self._complete(directory, messages, tools, reasoning_effort)

    def _complete(
        self,
        directory: Path,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
        effort: str | float | None,
    ) -> AssistantMessage:
        outgoing = copy.deepcopy(messages)
        images: list[Path] = []
        for message in outgoing:
            content = message.get("content")
            if not isinstance(content, list):
                continue
            for index, part in enumerate(content):
                if part.get("type") != "image_url":
                    continue
                url = part["image_url"]["url"]
                prefix = "data:image/png;base64,"
                if not url.startswith(prefix):
                    raise ValueError("Codex wire accepts inline PNG observations only")
                image = directory / f"image_{len(images)}.png"
                image.write_bytes(base64.b64decode(url[len(prefix) :], validate=True))
                images.append(image)
                content[index] = {
                    "type": "text",
                    "text": f"[attached image {len(images)}: {image.name}]",
                }
        names = [tool["function"]["name"] for tool in tools]
        schema = {
            "type": "object",
            "additionalProperties": False,
            "required": ["content", "tool_calls"],
            "properties": {
                "content": {"type": ["string", "null"]},
                "tool_calls": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": False,
                        "required": ["name", "arguments"],
                        "properties": {
                            "name": {"type": "string", "enum": names},
                            "arguments": {"type": "string"},
                        },
                    },
                },
            },
        }
        prompt = (
            "You are the assistant in the Inspect Robots conversation below. Continue it "
            "with ONE assistant turn. Follow its system message and tool definitions. "
            "Return tool_calls in the output JSON; arguments must be a JSON-encoded object "
            "matching that tool's parameters. Do not execute tools yourself. "
            "Attached images appear in the order of their numbered placeholders in history. "
            "Keep historical tool results, errors and operator feedback in context.\n\n"
            + json.dumps({"tools": tools, "messages": outgoing}, ensure_ascii=False)
        )
        (directory / "schema.json").write_text(json.dumps(schema))
        (directory / "prompt.txt").write_text(prompt)
        final = directory / "response.json"
        command = [
            self.executable,
            "exec",
            "--ignore-user-config",
            "--ephemeral",
            "--skip-git-repo-check",
            "--sandbox",
            "read-only",
            "--json",
            "-c",
            'forced_login_method="chatgpt"',
            "-c",
            'web_search="disabled"',
            "-c",
            "features.shell_tool=false",
            "-c",
            "features.apps=false",
            "-c",
            "features.multi_agent=false",
            "--model",
            self._provider.model,
            "--output-schema",
            str(directory / "schema.json"),
            "-o",
            str(final),
        ]
        if effort is not None:
            command += ["-c", "model_reasoning_effort=" + json.dumps(effort)]
        for image in images:
            command += ["--image", str(image)]
        command += ["-"]
        (directory / "command.json").write_text(json.dumps(command, indent=2))
        started, wall = time.monotonic(), time.time()
        raw, error = None, None
        try:
            with (
                (directory / "events.jsonl").open("w") as out,
                (directory / "stderr.log").open("w") as err,
            ):
                proc = subprocess.Popen(
                    command,
                    cwd=directory,
                    env=subscription_environment(),
                    stdin=subprocess.PIPE,
                    stdout=out,
                    stderr=err,
                    text=True,
                    start_new_session=True,
                )
                try:
                    proc.communicate(prompt, timeout=self.timeout_s)
                    if proc.returncode:
                        detail = (directory / "stderr.log").read_text()[-1500:]
                        raise RuntimeError(f"Codex exited {proc.returncode}: {detail}")
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGTERM)
                        try:
                            proc.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid, signal.SIGKILL)
                            proc.wait()
            raw = final.read_text()
            parsed = _finite_json(raw)
            if not isinstance(parsed, dict) or set(parsed) != {"content", "tool_calls"}:
                raise ValueError("invalid Codex assistant envelope")
            if parsed["content"] is not None and not isinstance(parsed["content"], str):
                raise ValueError("Codex content must be a string or null")
            if not isinstance(parsed["tool_calls"], list):
                raise ValueError("Codex tool_calls must be a list")
            calls = []
            for call in parsed["tool_calls"]:
                if set(call) != {"name", "arguments"} or call["name"] not in names:
                    raise ValueError("Codex requested an unknown tool")
                if not isinstance(call["arguments"], str) or not isinstance(
                    _finite_json(call["arguments"]), dict
                ):
                    raise ValueError("Codex tool arguments must encode a JSON object")
                calls.append(ToolCall("codex_" + uuid.uuid4().hex, call["name"], call["arguments"]))
            return AssistantMessage(parsed["content"], tuple(calls))
        except BaseException as exc:
            error = str(exc)
            raise
        finally:
            if self._capture is not None:
                self._capture.record(
                    attempt=0,
                    endpoint="codex://chatgpt",
                    request={
                        "model": self._provider.model,
                        "messages": messages,
                        "tools": tools,
                        "reasoning_effort": effort,
                        "output_schema": schema,
                        "cli_prompt": prompt,
                    },
                    status=200 if error is None else None,
                    response_text=raw,
                    error=error,
                    t_start=wall,
                    duration_s=time.monotonic() - started,
                )
