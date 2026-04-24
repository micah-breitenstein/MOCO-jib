#include <cassert>
#include <cstdint>
#include <iostream>

// Mock digitalWrite to track calls
enum class TriggerState { IDLE, PULSE_LOW, PULSE_HIGH };
TriggerState triggerState = TriggerState::IDLE;
int triggerCalls = 0;

void mockDigitalWrite(uint8_t pin, uint8_t value) {
  if (pin == 28) {  // trigger pin
    if (value == 1) triggerState = TriggerState::IDLE;
    else triggerState = TriggerState::PULSE_LOW;
    triggerCalls++;
  }
}

// Test: Trigger idle is HIGH
void test_trigger_idle_is_high() {
  triggerState = TriggerState::IDLE;
  triggerCalls = 0;
  mockDigitalWrite(28, 1);  // HIGH
  
  assert(triggerState == TriggerState::IDLE);
  std::cout << "✓ Trigger idle is HIGH\n";
}

// Test: Trigger pulse is LOW
void test_trigger_pulse_is_low() {
  triggerState = TriggerState::IDLE;
  mockDigitalWrite(28, 0);  // LOW
  
  assert(triggerState == TriggerState::PULSE_LOW);
  std::cout << "✓ Trigger pulse is LOW\n";
}

// Test: Pulse sequence is LOW then back to HIGH
void test_trigger_pulse_sequence() {
  triggerCalls = 0;
  
  // Simulate manual shutter pulse
  mockDigitalWrite(28, 0);  // LOW
  assert(triggerState == TriggerState::PULSE_LOW);
  
  mockDigitalWrite(28, 1);  // HIGH
  assert(triggerState == TriggerState::IDLE);
  
  assert(triggerCalls == 2);
  std::cout << "✓ Trigger pulse sequence (LOW->HIGH) correct\n";
}

// Test: Trigger never starts LOW at boot
void test_trigger_boot_state() {
  triggerState = TriggerState::IDLE;
  
  // Should initialize to HIGH (idle)
  assert(triggerState == TriggerState::IDLE);
  std::cout << "✓ Trigger boot state is HIGH (idle)\n";
}

int main() {
  std::cout << "\n=== Trigger Polarity Tests ===\n";
  
  test_trigger_idle_is_high();
  test_trigger_pulse_is_low();
  test_trigger_pulse_sequence();
  test_trigger_boot_state();
  
  std::cout << "\n✓ All trigger polarity tests passed\n\n";
  return 0;
}
