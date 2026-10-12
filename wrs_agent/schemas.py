"""Strict, bounded JSON contracts. No device or provider objects cross this boundary."""

import json
from enum import StrEnum
from graphlib import TopologicalSorter
from typing import Annotated, Literal
from uuid import uuid4

from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, model_validator

MAX_BYTES = 65536
Name = Annotated[str, Field(min_length=1, max_length=80, pattern=r"^[A-Za-z0-9_.-]+$")]
Counter = Annotated[int, Field(ge=0, le=2**53)]
SkillVersion = Annotated[int, Field(ge=1, le=2**31 - 1)]
Scalar = str | int | float | bool | None


class ActionState(StrEnum):
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    VERIFYING = "VERIFYING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLING = "CANCELLING"
    CANCELLED = "CANCELLED"
    UNKNOWN = "UNKNOWN"


class TaskState(StrEnum):
    QUEUED = "QUEUED"
    RUNNING = "RUNNING"
    CANCELLING = "CANCELLING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    UNKNOWN = "UNKNOWN"


class GoalState(StrEnum):
    WAITING = "WAITING"
    DONE = "DONE"
    ANSWER = "ANSWER"
    CLARIFY = "CLARIFY"
    FAILED = "FAILED"
    STALE = "STALE"
    REQUIRES_CONFIRMATION = "REQUIRES_CONFIRMATION"


TERMINAL = {"SUCCEEDED", "FAILED", "CANCELLED", "UNKNOWN"}


def new_id() -> str:
    return uuid4().hex


class Boundary(BaseModel):
    model_config = ConfigDict(strict=True, extra="forbid", frozen=True, allow_inf_nan=False)


class SkillSpec(Boundary):
    name: Name
    version: SkillVersion = 1
    description: str
    instructions: str = Field(default="", max_length=8192)
    parameters: dict
    resources: list[str]
    preconditions: list[str]
    verification: str
    timeout: float = Field(default=10.0, gt=0)
    recovery: list[str] = Field(default_factory=list)


class ErrorInfo(Boundary):
    """Stable machine code plus safe context; never a serialized exception/input dump."""

    code: Name
    message: str = Field(default="", max_length=240)
    stage: Literal["discovery", "preflight", "submit", "observe", "control", "planning"] | None = (
        None
    )
    node_id: Name | None = None
    task_id: Name | None = None
    action_id: Name | None = None


class Envelope(Boundary):
    schema_version: Literal[4] = 4
    message_id: Name = Field(default_factory=new_id)
    trace_id: Name = Field(default_factory=new_id)
    source: Name
    category: Literal["control", "interactive", "background"] = "interactive"
    session: Name
    env_id: Name
    auth: Annotated[str, Field(min_length=16, max_length=128, repr=False)]
    payload: dict


def encode(value: dict | Boundary) -> bytes:
    if isinstance(value, Boundary):
        value = value.model_dump(mode="json")
    result = json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
    if len(result) > MAX_BYTES:
        raise ValueError("payload_too_large")
    return result


def decode(data: bytes) -> dict:
    if len(data) > MAX_BYTES:
        raise ValueError("payload_too_large")
    result = json.loads(
        data, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite"))
    )
    if not isinstance(result, dict):
        raise ValueError("object_required")
    return result


class ActionRequest(Boundary):
    action_id: Name
    task_id: Name
    task_revision: Counter
    boot_id: Name
    control_epoch: Counter
    lease_id: Name
    skill: Name
    version: SkillVersion = 1
    args: Annotated[dict[Name, Scalar], Field(max_length=8)] = Field(default_factory=dict)
    state_version: Counter


class ActionStatus(Boundary):
    action_id: Name
    # JSON/dict inputs still carry strings; keep the validated Python result as an enum.
    state: Annotated[ActionState, BeforeValidator(lambda value: ActionState(value))]
    sequence: Counter = 0
    progress: float = Field(default=0.0, ge=0, le=1)
    reason: Annotated[str, Field(max_length=240)] = ""
    verification: Literal["PENDING", "PASS", "FAIL", "INCONCLUSIVE"] = "PENDING"
    error: ErrorInfo | None = None


class ActionReceipt(Boundary):
    accepted: bool
    reason: str = ""
    status: ActionStatus | None = None
    error: ErrorInfo | None = None


class ControlRequest(Boundary):
    interrupt_id: Name
    boot_id: Name
    control_epoch: Counter
    action_id: Name | None = None
    state_version: Counter | None = None


class ControlReceipt(Boundary):
    accepted: bool
    reason: str = ""
    control_epoch: Counter
    phase: Literal["REJECTED", "STOPPING", "STOPPED", "UNKNOWN", "ACTIONS_ALLOWED"]
    error: ErrorInfo | None = None


class ModeRequest(Boundary):
    """Choose whether the robot node drives its model only or the real device."""

    interrupt_id: Name
    boot_id: Name
    control_epoch: Counter
    mode: Literal["virtual", "real"]


class ModeReceipt(Boundary):
    accepted: bool
    reason: str = ""
    mode: Literal["virtual", "real"]
    control_epoch: Counter
    error: ErrorInfo | None = None


class KinematicState(Boundary):
    """WRS joint values and the named TCP pose, expressed in frame_id."""

    qs: Annotated[list[float], Field(min_length=6, max_length=6)]
    joint_unit: Literal["rad"] = "rad"
    tcp_name: Name
    tcp_pos: Annotated[list[float], Field(min_length=3, max_length=3)]
    tcp_rotmat: Annotated[
        list[Annotated[list[float], Field(min_length=3, max_length=3)]],
        Field(min_length=3, max_length=3),
    ]
    position_unit: Literal["m"] = "m"
    # Jaw opening read back from the mounted gripper model; None when no gripper is mounted.
    gripper_width: float | None = Field(default=None, ge=0, le=0.5)
    frame_id: Literal["world"] = "world"
    source: Literal["wrs_fk"] = "wrs_fk"
    observed_at_ns: int = Field(gt=0)
    valid: bool = True


Vector3 = Annotated[list[float], Field(min_length=3, max_length=3)]
Rotation3 = Annotated[list[Vector3], Field(min_length=3, max_length=3)]


class BoxGeometry(Boundary):
    kind: Literal["box"] = "box"
    xyz_lengths: Annotated[
        list[Annotated[float, Field(gt=0, le=100)]], Field(min_length=3, max_length=3)
    ]


class MeshGeometry(Boundary):
    kind: Literal["mesh"] = "mesh"
    asset: str = Field(min_length=1, max_length=240, pattern=r"^[A-Za-z0-9_./-]+\.stl$")

    @model_validator(mode="after")
    def relative_asset(self):
        if self.asset.startswith("/") or any(p in ("", ".", "..") for p in self.asset.split("/")):
            raise ValueError("mesh_asset_must_be_relative")
        return self


class ObjectData(Boundary):
    """One identified object; absent pose/geometry means unknown, never identity."""

    label: str = Field(default="", max_length=80)
    pos: Vector3 | None = None
    rotmat: Rotation3 | None = None
    geometry: BoxGeometry | MeshGeometry | None = None
    alpha: float = Field(default=1.0, ge=0, le=1)
    rgb: Annotated[
        list[Annotated[float, Field(ge=0, le=1)]], Field(min_length=3, max_length=3)
    ] = Field(default_factory=lambda: [0.6, 0.6, 0.6])
    location: Name | None = None  # Symbolic location used by Mock pick/place.
    source: Name
    observed_at_ns: int | None = Field(default=None, gt=0)
    valid: bool = True

    @model_validator(mode="after")
    def proper_rotation(self):
        if self.rotmat is not None:
            rows = self.rotmat
            for i in range(3):
                for j in range(3):
                    dot = sum(a * b for a, b in zip(rows[i], rows[j], strict=True))
                    if abs(dot - (1.0 if i == j else 0.0)) > 1e-4:
                        raise ValueError("rotmat_must_be_orthonormal")
            a, b, c = rows
            determinant = (
                a[0] * (b[1] * c[2] - b[2] * c[1])
                - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])
            )
            if abs(determinant - 1.0) > 1e-4:
                raise ValueError("rotmat_must_be_right_handed")
        return self


class RobotData(Boundary):
    held_object: Name | None = None
    pose: Name | None = None  # Named joint configuration, not a geometric pose.
    kinematics: KinematicState | None = None


class SceneData(Boundary):
    """Latest accepted scene at capture time, not simultaneous sensor truth."""

    kind: Literal["scene"] = "scene"
    frame_id: Literal["world"] = "world"
    position_unit: Literal["m"] = "m"
    robot: RobotData
    objects: dict[Name, ObjectData] = Field(default_factory=dict, max_length=32)
    facts: dict[Name, Scalar] = Field(default_factory=dict, max_length=32)


class SpeechData(Boundary):
    kind: Literal["speech"] = "speech"
    completed: Counter = 0
    last_text: str = Field(default="", max_length=512)


class NodeSnapshot(Boundary):
    """One execution node at capture time; never an atomic whole-system view or grant."""

    node_id: Name
    boot_id: Name
    captured_at_ns: int = Field(gt=0)  # Source wall clock; not a cross-host lease clock.
    control_epoch: Counter
    state_version: Counter  # Business-state revision, independent of control authority.
    admission: Literal["OPEN", "HELD", "UNKNOWN"]
    active_action: Name | None
    stop_confirmed: bool
    data: Annotated[SceneData | SpeechData, Field(discriminator="kind")]


class ActionContext(NodeSnapshot):
    """Current node state plus a short-lived, receiver-issued execution token."""

    lease_id: Name


class FeatureSnapshot(Boundary):
    backend: Name = "mock"
    resources: list[Name] = Field(default_factory=list)
    skills: dict[Name, SkillVersion] = Field(max_length=64)
    specs: dict[Name, SkillSpec] = Field(default_factory=dict, max_length=64)
    boot_id: Name | None = None
    skill_revision: Counter = 0
    robot_controls: bool = True
    hardware: bool = False  # Can drive a real device; SceneData.facts says whether it does now.
    controlled_stop: bool = True
    controller_flush: bool = True
    verification: Name = "virtual_state"
    stop_scope: Name = "virtual_state"
    unsupported: dict[Name, str] = Field(default_factory=dict)


class NodeInfo(Boundary):
    node_id: Name
    node_type: Literal["agent", "wrs", "tts", "voice", "asr", "vision", "custom"]
    target: Name | None = None
    actions: bool = False
    robot_controls: bool = False
    boot_id: Name | None = None
    features: list[Name] = Field(default_factory=list, max_length=64)
    skills: dict[Name, SkillVersion] = Field(default_factory=dict, max_length=64)
    skill_revision: Counter = 0
    resources: list[Name] = Field(default_factory=list, max_length=32)
    ready: bool = False
    health: Literal["ready", "held", "unknown", "offline", "stale", "unsupported"] = "offline"
    error: ErrorInfo | None = None


class Step(Boundary):
    step_id: Name
    skill: Name
    version: SkillVersion | None = 1
    args: Annotated[dict[Name, Scalar], Field(max_length=8)] = Field(default_factory=dict)
    category: Literal["interactive", "background"] = "interactive"
    depends_on: Annotated[list[Name], Field(max_length=12)] = Field(default_factory=list)


class Plan(Boundary):
    steps: Annotated[list[Step], Field(min_length=1, max_length=12)]

    @model_validator(mode="after")
    def valid_dag(self):
        ids = {s.step_id for s in self.steps}
        if len(ids) != len(self.steps):
            raise ValueError("duplicate_step")
        if any(d not in ids for s in self.steps for d in s.depends_on):
            raise ValueError("missing_dependency")
        tuple(TopologicalSorter({s.step_id: s.depends_on for s in self.steps}).static_order())
        return self


class SkillCheck(Boundary):
    """Pure argument preflight; never an action or an execution grant."""
    boot_id: Name
    steps: list[Step] = Field(min_length=1, max_length=12)


class SkillCheckResult(Boundary):
    boot_id: Name
    count: int = Field(ge=1, le=12)


class TaskRequest(Boundary):
    request_id: Name
    plan: Plan


class TaskCancelRequest(Boundary):
    request_id: Name
    task_id: Name


class IdRequest(Boundary):
    action_id: Name


class Empty(Boundary):
    pass


class GoalRequest(Boundary):
    request_id: Name
    goal: str = Field(min_length=1, max_length=1024)


class Interaction(Boundary):
    event_id: Name
    kind: Literal["vad", "ack", "query", "append", "revise", "stop", "barge_in"]
    confidence: float = Field(default=1.0, ge=0, le=1)
    quoted: bool = False
    negated: bool = False
    plan: Plan | None = None


class TaskQuery(Boundary):
    task_id: Name


class GoalQuery(Boundary):
    request_id: Name


class TaskStatus(Boundary):
    task_id: Name
    state: Annotated[TaskState, BeforeValidator(lambda value: TaskState(value))]
    reason: str = ""
    steps: dict[str, str] = Field(default_factory=dict)
    active_actions: dict[str, str] = Field(default_factory=dict)
    error: ErrorInfo | None = None


class TaskCancelReceipt(Boundary):
    task_id: Name
    revision: Literal[0] = 0
    state: Annotated[
        Literal[
            TaskState.CANCELLING,
            TaskState.SUCCEEDED,
            TaskState.FAILED,
            TaskState.CANCELLED,
            TaskState.UNKNOWN,
        ],
        BeforeValidator(lambda value: TaskState(value)),
    ]
    accepted: bool
    phase: Literal["STOPPING", "STOPPED", "UNKNOWN"]
    error: ErrorInfo | None = None


class GoalStatus(Boundary):
    request_id: Name
    state: Annotated[GoalState, BeforeValidator(lambda value: GoalState(value))]
    reason: str = ""
    task_id: Name | None = None
    error: ErrorInfo | None = None


class InterruptRequest(Boundary):
    """An explicit operator stop binds the current task/planning once, at the receiver."""

    request_id: Name


class TextInput(Boundary):
    """Recognized text from a trusted local ASR/UI adapter; never raw audio."""

    input_id: Name = Field(default_factory=new_id)
    text: str = Field(min_length=1, max_length=1024)
    is_final: bool = True
    confidence: float = Field(default=1.0, ge=0, le=1)


class TextReceipt(Boundary):
    input_id: Name
    disposition: Literal["ignore", "clarify", "query", "goal", "stop", "cancel_tts"]
    accepted: bool = False
    reason: str = ""
    phase: Literal["STOPPING", "STOPPED", "UNKNOWN"] | None = None
    request_id: Name | None = None
    task_id: Name | None = None
    overview: dict | None = None
    error: ErrorInfo | None = None


class AsrPress(Boundary):
    """One push-to-talk session. The ID keeps begin/end idempotent across retries."""

    press_id: Name = Field(default_factory=new_id)


class AsrResult(Boundary):
    """Capture outcome for the caller that held the button; transcripts are never broadcast."""

    press_id: Name
    capturing: bool = False  # Still recording or recognizing; text is final only when False.
    text: str = Field(default="", max_length=1024)
    reason: str = ""
    receipt: TextReceipt | None = None
