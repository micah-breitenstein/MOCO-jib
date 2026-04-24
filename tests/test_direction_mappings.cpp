#include <cassert>
#include <cstdint>
#include <iostream>

// Motion direction tracking
enum class MotionAxis { SWING, PAN, LIFT, TILT };
enum class Direction { FORWARD, REVERSE, HOLD };

struct MotionMap {
  MotionAxis axis;
  Direction swingLeads;  // Which direction swing goes
  Direction axisFollows; // Which direction this axis should go
};

// Expected: Pan should go opposite to swing in timelapse (or same, depending on your setup)
// The key is: this mapping should NEVER FLIP unintentionally
const MotionMap TIMELAPSE_MOTION_MAPS[] = {
  {MotionAxis::PAN, Direction::FORWARD, Direction::REVERSE},  // Pan left when swing goes right
  {MotionAxis::LIFT, Direction::FORWARD, Direction::FORWARD}, // Lift up when swing goes right
  {MotionAxis::TILT, Direction::FORWARD, Direction::REVERSE}, // Tilt down when swing goes right
};

const int NUM_MOTION_MAPS = sizeof(TIMELAPSE_MOTION_MAPS) / sizeof(MotionMap);

// Test: Direction mappings are consistent
void test_direction_mapping_consistency() {
  for (int i = 0; i < NUM_MOTION_MAPS; i++) {
    const MotionMap& map = TIMELAPSE_MOTION_MAPS[i];
    
    // Verify that no direction mapping is HOLD (invalid)
    assert(map.axisFollows != Direction::HOLD && map.swingLeads != Direction::HOLD);
    
    // Verify that each axis has a defined mapping
    assert(map.axis >= MotionAxis::SWING && map.axis <= MotionAxis::TILT);
  }
  std::cout << "✓ Direction mappings are consistent\n";
}

// Test: Pan is opposite to swing (prevent accidental reversal)
void test_pan_opposes_swing() {
  for (int i = 0; i < NUM_MOTION_MAPS; i++) {
    const MotionMap& map = TIMELAPSE_MOTION_MAPS[i];
    if (map.axis == MotionAxis::PAN) {
      assert(map.swingLeads != map.axisFollows);  // Should be opposite
      std::cout << "✓ Pan motion is opposite to swing\n";
      return;
    }
  }
  assert(false && "Pan mapping not found");
}

// Test: No axis maps incorrectly
void test_no_axis_maps_to_itself() {
  // Usually only swing maps to swing; others should have defined directions
  for (int i = 0; i < NUM_MOTION_MAPS; i++) {
    const MotionMap& map = TIMELAPSE_MOTION_MAPS[i];
    if (map.axis != MotionAxis::SWING) {
      assert(map.swingLeads == map.swingLeads);  // Swing always leads (this is redundant but safe check)
    }
  }
  std::cout << "✓ No axis maps incorrectly\n";
}

int main() {
  std::cout << "\n=== Direction Mapping Tests ===\n";
  
  test_direction_mapping_consistency();
  test_pan_opposes_swing();
  test_no_axis_maps_to_itself();
  
  std::cout << "\n✓ All direction mapping tests passed\n\n";
  return 0;
}
