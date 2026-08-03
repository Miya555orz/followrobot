from voice_intent_pkg.intent_gate import evaluate_intent


CONTROL_INTENTS = {("视野朝向指令", "向右看"): "gimbal_nudge_right"}


def prediction(
    *,
    coarse_name="视野朝向指令",
    coarse_confidence=0.99,
    coarse_top2=0.10,
    fine_name="向右看",
    fine_confidence=0.95,
    fine_passed=True,
):
    return {
        "coarse": {
            "name": coarse_name,
            "confidence": coarse_confidence,
            "top_k": [
                {"name": coarse_name, "confidence": coarse_confidence},
                {"name": "干扰项", "confidence": coarse_top2},
            ],
        },
        "fine": {
            "name": fine_name,
            "confidence": fine_confidence,
            "passed": fine_passed,
        },
    }


def gate(result):
    return evaluate_intent(
        result,
        CONTROL_INTENTS,
        min_coarse_confidence=0.60,
        min_coarse_margin=0.15,
        min_fine_confidence=0.60,
    )


def test_supported_command_is_accepted():
    decision = gate(prediction(coarse_top2=0.01))

    assert decision["accepted"] is True
    assert decision["control_intent"] == "gimbal_nudge_right"
    assert decision["rejection_reason"] == "accepted"


def test_unmapped_prediction_is_rejected():
    decision = gate(prediction(fine_name="未知动作", coarse_top2=0.01))

    assert decision["accepted"] is False
    assert decision["control_intent"] == ""
    assert "unmapped_intent" in decision["rejection_reasons"]


def test_low_confidence_prediction_is_rejected():
    decision = gate(
        prediction(
            coarse_confidence=0.62,
            coarse_top2=0.55,
            fine_confidence=0.95,
        )
    )

    assert decision["accepted"] is False
    assert "coarse_margin_below_threshold" in decision["rejection_reasons"]


def test_distractor_prediction_is_rejected_even_with_high_confidence():
    decision = gate(
        prediction(
            coarse_name="干扰项",
            coarse_confidence=0.99,
            coarse_top2=0.01,
        )
    )

    assert decision["accepted"] is False
    assert "distractor_class" in decision["rejection_reasons"]


def test_fine_confidence_and_confusion_are_rejected():
    decision = gate(
        prediction(
            coarse_top2=0.01,
            fine_confidence=0.40,
            fine_passed=False,
        )
    )

    assert decision["accepted"] is False
    assert "fine_confidence_below_threshold" in decision["rejection_reasons"]
    assert "fine_confusion_above_threshold" in decision["rejection_reasons"]
