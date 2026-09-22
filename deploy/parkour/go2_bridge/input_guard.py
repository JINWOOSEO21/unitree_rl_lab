"""Pure input validation and freshness tracking for Go2 policy shadow runs.

The thresholds in this module are diagnostic defaults.  They are intentionally
configurable and do not certify the timing limits of the physical robot.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class GuardConfig:
    low_receipt_timeout_ns: int = 20_000_000
    low_source_timeout_ns: int = 20_000_000
    scan_receipt_timeout_ns: int = 500_000_000
    scan_source_timeout_ns: int = 500_000_000
    scan_size: int = 132
    minimum_motor_count: int = 12
    quaternion_norm_tolerance: float = 0.05
    scan_min: float = -1.0
    scan_max: float = 1.0


@dataclass(frozen=True)
class Provenance:
    receipt_ns: int
    source_ns: int
    source_id: int


@dataclass(frozen=True)
class OfferResult:
    accepted: bool
    reason: str
    restart_required: bool = False


@dataclass(frozen=True)
class GuardDecision:
    allowed: bool
    reasons: tuple[str, ...]
    reset_history: bool
    low: dict[str, Any] | None
    scan: np.ndarray | None
    low_provenance: Provenance | None
    scan_provenance: Provenance | None
    low_receipt_age_ns: int | None
    low_source_age_ns: int | None
    scan_receipt_age_ns: int | None
    scan_source_age_ns: int | None


def _timestamp(value: int, name: str) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    value = int(value)
    if value < 0:
        raise ValueError(f"{name} must be non-negative")
    return value


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if array.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def validate_lowstate(low: Mapping[str, Any], config: GuardConfig) -> None:
    """Validate only fields consumed by the parkour observation builder."""
    try:
        imu = low["imu_state"]
        motors = low["motor_state"]
        foot_force = low["foot_force"]
    except (KeyError, TypeError) as exc:
        raise ValueError("lowstate is missing a required field") from exc

    _finite_vector(imu["gyroscope"], 3, "gyroscope")
    quat = _finite_vector(imu["quaternion"], 4, "quaternion")
    norm = float(np.linalg.norm(quat))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("quaternion norm is zero")
    if abs(norm - 1.0) > config.quaternion_norm_tolerance:
        raise ValueError("quaternion norm is outside tolerance")

    if not isinstance(motors, (list, tuple)) or len(motors) < config.minimum_motor_count:
        raise ValueError(f"motor_state must contain at least {config.minimum_motor_count} motors")
    for index, motor in enumerate(motors[:config.minimum_motor_count]):
        try:
            _finite_vector([motor["q"], motor["dq"]], 2, f"motor_state[{index}]")
        except (KeyError, TypeError) as exc:
            raise ValueError(f"motor_state[{index}] is missing q or dq") from exc
    _finite_vector(foot_force, 4, "foot_force")


class PolicyInputGuard:
    """Track validated LowState and scan samples without any DDS dependency.

    Receipt time controls local dropout detection.  Source time controls the
    age of the underlying measurement independently, so repeated delivery of
    an old map cannot make that map fresh.  Source IDs must strictly increase
    within an epoch.  A regression requires an explicit ``reset_stream`` call.
    """

    _STREAMS = ("low", "scan")

    def __init__(self, config: GuardConfig | None = None) -> None:
        self.config = config or GuardConfig()
        self._low: dict[str, Any] | None = None
        self._scan: np.ndarray | None = None
        self._provenance: dict[str, Provenance | None] = {name: None for name in self._STREAMS}
        self._fault: dict[str, str | None] = {name: None for name in self._STREAMS}
        self._restart_required: dict[str, bool] = {name: False for name in self._STREAMS}
        self._reset_history = True
        self._was_allowed = False

    def _offer_provenance(
        self, stream: str, receipt_ns: int, source_ns: int, source_id: int
    ) -> tuple[Provenance | None, OfferResult | None]:
        try:
            provenance = Provenance(
                _timestamp(receipt_ns, "receipt_ns"),
                _timestamp(source_ns, "source_ns"),
                _timestamp(source_id, "source_id"),
            )
        except (TypeError, ValueError) as exc:
            reason = f"{stream}_invalid_provenance: {exc}"
            self._fault[stream] = reason
            self._reset_history = True
            return None, OfferResult(False, reason, self._restart_required[stream])

        if self._restart_required[stream]:
            return None, OfferResult(False, f"{stream}_restart_reset_required", True)

        previous = self._provenance[stream]
        if previous is not None:
            if provenance.receipt_ns <= previous.receipt_ns:
                reason = f"{stream}_nonmonotonic_receipt"
                self._fault[stream] = reason
                self._reset_history = True
                return None, OfferResult(False, reason)
            if provenance.source_id == previous.source_id:
                reason = f"{stream}_duplicate_source_id"
                self._fault[stream] = reason
                self._reset_history = True
                return None, OfferResult(False, reason)
            if provenance.source_id < previous.source_id or provenance.source_ns < previous.source_ns:
                reason = f"{stream}_source_regression"
                self._fault[stream] = reason
                self._restart_required[stream] = True
                self._reset_history = True
                return None, OfferResult(False, reason, True)
        return provenance, None

    def offer_low(
        self,
        low: Mapping[str, Any],
        *,
        receipt_ns: int,
        source_ns: int,
        source_id: int,
    ) -> OfferResult:
        provenance, failure = self._offer_provenance("low", receipt_ns, source_ns, source_id)
        if failure is not None:
            return failure
        try:
            validate_lowstate(low, self.config)
        except (TypeError, ValueError, KeyError) as exc:
            reason = f"low_invalid: {exc}"
            self._fault["low"] = reason
            self._reset_history = True
            return OfferResult(False, reason)
        assert provenance is not None
        self._low = deepcopy(dict(low))
        self._provenance["low"] = provenance
        self._fault["low"] = None
        return OfferResult(True, "accepted")

    def offer_scan(
        self,
        scan: Any,
        *,
        receipt_ns: int,
        source_ns: int,
        source_id: int,
    ) -> OfferResult:
        provenance, failure = self._offer_provenance("scan", receipt_ns, source_ns, source_id)
        if failure is not None:
            return failure
        try:
            values = _finite_vector(scan, self.config.scan_size, "scan").astype(np.float32)
            if np.any(values < self.config.scan_min) or np.any(values > self.config.scan_max):
                raise ValueError("scan values are outside configured range")
        except (TypeError, ValueError) as exc:
            reason = f"scan_invalid: {exc}"
            self._fault["scan"] = reason
            self._reset_history = True
            return OfferResult(False, reason)
        assert provenance is not None
        self._scan = values.copy()
        self._provenance["scan"] = provenance
        self._fault["scan"] = None
        return OfferResult(True, "accepted")

    def reset_stream(self, stream: str | None = None) -> None:
        """Start a new source epoch after a known publisher or bridge restart."""
        streams = self._STREAMS if stream is None else (stream,)
        for name in streams:
            if name not in self._STREAMS:
                raise ValueError(f"unknown stream: {name}")
            if name == "low":
                self._low = None
            else:
                self._scan = None
            self._provenance[name] = None
            self._fault[name] = None
            self._restart_required[name] = False
        self._reset_history = True
        self._was_allowed = False

    def acknowledge_history_reset(self) -> None:
        """Acknowledge that the caller reset/primed all policy temporal state."""
        self._reset_history = False

    @staticmethod
    def _age(now_ns: int, timestamp_ns: int) -> int:
        return now_ns - timestamp_ns

    def evaluate(self, *, receipt_now_ns: int, source_now_ns: int | None = None) -> GuardDecision:
        receipt_now = _timestamp(receipt_now_ns, "receipt_now_ns")
        source_now = receipt_now if source_now_ns is None else _timestamp(source_now_ns, "source_now_ns")
        reasons: list[str] = []
        ages: dict[str, int | None] = {
            "low_receipt": None, "low_source": None, "scan_receipt": None, "scan_source": None,
        }

        limits = {
            "low": (self.config.low_receipt_timeout_ns, self.config.low_source_timeout_ns),
            "scan": (self.config.scan_receipt_timeout_ns, self.config.scan_source_timeout_ns),
        }
        for stream in self._STREAMS:
            provenance = self._provenance[stream]
            if provenance is None:
                reasons.append(f"{stream}_missing")
            else:
                receipt_age = self._age(receipt_now, provenance.receipt_ns)
                source_age = self._age(source_now, provenance.source_ns)
                ages[f"{stream}_receipt"] = receipt_age
                ages[f"{stream}_source"] = source_age
                if receipt_age < 0:
                    reasons.append(f"{stream}_receipt_from_future")
                elif receipt_age > limits[stream][0]:
                    reasons.append(f"{stream}_receipt_stale")
                if source_age < 0:
                    reasons.append(f"{stream}_source_from_future")
                elif source_age > limits[stream][1]:
                    reasons.append(f"{stream}_source_stale")
            if self._fault[stream] is not None:
                reasons.append(self._fault[stream])
            if self._restart_required[stream]:
                reasons.append(f"{stream}_restart_reset_required")

        allowed = not reasons
        if not allowed or (allowed and not self._was_allowed):
            self._reset_history = True
        self._was_allowed = allowed
        return GuardDecision(
            allowed=allowed,
            reasons=tuple(dict.fromkeys(reasons)),
            reset_history=self._reset_history,
            low=deepcopy(self._low),
            scan=None if self._scan is None else self._scan.copy(),
            low_provenance=self._provenance["low"],
            scan_provenance=self._provenance["scan"],
            low_receipt_age_ns=ages["low_receipt"],
            low_source_age_ns=ages["low_source"],
            scan_receipt_age_ns=ages["scan_receipt"],
            scan_source_age_ns=ages["scan_source"],
        )
