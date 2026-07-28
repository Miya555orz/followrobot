#include "servo_control_pkg/target_input.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

using vision_servo_msgs::msg::Target;
using vision_servo_msgs::msg::TargetArray;

namespace {

Target make_target(int id, uint8_t state, bool visible)
{
  Target target;
  target.id = id;
  target.tracking_state = state;
  target.visible = visible;
  return target;
}

TEST(TargetInput, SelectsVisibleConfirmedTrackingId)
{
  TargetArray input;
  input.tracking_id = 7;
  input.targets = {
    make_target(3, Target::TRACKING_STATE_CONFIRMED, true),
    make_target(7, Target::TRACKING_STATE_CONFIRMED, true)};
  const auto selected = servo_control_pkg::select_actionable_target(input);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->id, 7);
}

TEST(TargetInput, RejectsLostOrInvisibleLockedTrack)
{
  TargetArray input;
  input.tracking_id = 7;
  input.targets = {
    make_target(7, Target::TRACKING_STATE_LOST, false),
    make_target(8, Target::TRACKING_STATE_CONFIRMED, true)};
  EXPECT_FALSE(servo_control_pkg::select_actionable_target(input).has_value());
}

TEST(TargetInput, AllowsVisibleUntrackedMeasurements)
{
  TargetArray input;
  input.tracking_id = -1;
  input.targets = {make_target(-1, Target::TRACKING_STATE_UNTRACKED, true)};
  EXPECT_TRUE(servo_control_pkg::select_actionable_target(input).has_value());
}

TEST(TargetInput, PbvsAngularTargetAcceptsFreshPredictionOnly)
{
  auto target = make_target(7, Target::TRACKING_STATE_LOST, false);
  target.position = {0.1F, 0.0F, 2.0F};
  target.depth_confidence = 0.3F;
  target.fusion_state = Target::FUSION_STATE_PREDICTED;
  target.fusion_age = 0.2F;

  EXPECT_TRUE(servo_control_pkg::is_pbvs_angular_target(target, 0.4F, 0.3F));
  target.fusion_age = 0.31F;
  EXPECT_FALSE(servo_control_pkg::is_pbvs_angular_target(target, 0.4F, 0.3F));
}

TEST(TargetInput, PbvsSelectionKeepsLockedPredictedTrack)
{
  TargetArray input;
  input.tracking_id = 7;
  input.targets = {
    make_target(7, Target::TRACKING_STATE_LOST, false),
    make_target(8, Target::TRACKING_STATE_CONFIRMED, true)};

  const auto selected = servo_control_pkg::select_pbvs_target(input);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->id, 7);
}

TEST(TargetInput, PbvsTranslationRequiresFreshValidMetricDepth)
{
  auto target = make_target(7, Target::TRACKING_STATE_CONFIRMED, true);
  target.position = {0.1F, 0.0F, 2.0F};
  target.depth_confidence = 0.8F;
  target.fusion_state = Target::FUSION_STATE_VALID;
  target.fusion_age = 0.0F;

  EXPECT_TRUE(servo_control_pkg::is_pbvs_translation_target(target, 0.65F, 0.2F));
  target.fusion_state = Target::FUSION_STATE_DEGRADED;
  EXPECT_FALSE(servo_control_pkg::is_pbvs_translation_target(target, 0.65F, 0.2F));
}

TEST(TargetInput, PbvsRejectsUnconfirmedOrNonFinitePosition)
{
  auto target = make_target(7, Target::TRACKING_STATE_TENTATIVE, true);
  target.position = {0.1F, 0.0F, 2.0F};
  target.depth_confidence = 0.9F;
  target.fusion_state = Target::FUSION_STATE_VALID;

  EXPECT_FALSE(servo_control_pkg::is_pbvs_angular_target(target, 0.4F, 0.3F));
  target.tracking_state = Target::TRACKING_STATE_CONFIRMED;
  target.position[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(
    servo_control_pkg::is_pbvs_translation_target(target, 0.65F, 0.2F));
}

}  // namespace
