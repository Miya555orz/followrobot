"""Safety gate for converting classifier output into control intents."""

from __future__ import annotations

from typing import Any, Mapping


def _top_margin(prediction: Mapping[str, Any]) -> float:
    """Return the gap between the top two predictions."""
    top_k = prediction.get("top_k", [])
    if len(top_k) < 2:
        return 1.0
    return float(top_k[0]["confidence"]) - float(top_k[1]["confidence"])


def evaluate_intent(
    result: Mapping[str, Any],
    control_intents: Mapping[tuple[str, str], str],
    *,
    min_coarse_confidence: float,
    min_coarse_margin: float,
    min_fine_confidence: float,
) -> dict[str, Any]:
    """Validate a classifier result before allowing actuator control.

    The classifier always returns its best prediction, even for an unrelated
    sentence. This gate is the explicit reject layer: a failed check produces
    diagnostics only and must never be converted into ``VoiceCommand``.
    """
    coarse = result["coarse"]
    fine = result["fine"]
    control_intent = control_intents.get((coarse["name"], fine["name"]), "")
    reasons: list[str] = []

    if coarse["name"] == "干扰项":
        reasons.append("distractor_class")
    if not control_intent:
        reasons.append("unmapped_intent")
    if float(coarse["confidence"]) < min_coarse_confidence:
        reasons.append("coarse_confidence_below_threshold")
    if _top_margin(coarse) < min_coarse_margin:
        reasons.append("coarse_margin_below_threshold")
    if float(fine["confidence"]) < min_fine_confidence:
        reasons.append("fine_confidence_below_threshold")
    if not bool(fine.get("passed", False)):
        reasons.append("fine_confusion_above_threshold")

    accepted = not reasons
    return {
        "control_intent": control_intent,
        "accepted": accepted,
        "rejection_reasons": reasons,
        "rejection_reason": "accepted" if accepted else reasons[0],
        "coarse_margin": _top_margin(coarse),
    }
