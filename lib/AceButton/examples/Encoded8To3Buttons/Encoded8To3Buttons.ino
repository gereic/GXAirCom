/*
 * A demo of Encoded4To2ButtonConfig to detect 3 buttons using 2 pins;
 * and Encoded8To3ButtonConfig to detect 7 buttons using 3 pins.
 */

#include <AceButton.h>
using namespace ace_button;

// Select 3 to use Encoded4To2ButtonConfig or 7 to use Encoded8To3ButtonConfig.
#define NUM_BUTTONS 7

#ifdef ESP32
  // Different ESP32 boards use different pins
  static const int LED_PIN = 2;
#else
  static const int LED_PIN = LED_BUILTIN;
#endif

// LED states. Some microcontrollers wire their built-in LED the reverse.
static const int LED_ON = HIGH;
static const int LED_OFF = LOW;

static const uint8_t BUTTON_PIN0 = 2;
static const uint8_t BUTTON_PIN1 = 3;
static const uint8_t BUTTON_PIN2 = 4;

// Each button is assigned to the virtual pin number (1-7) which comes from the
// binary bit patterns of the 3 actual pins. Button b0 cannot be used
// because it is used to represent "no button pressed".
#if NUM_BUTTONS == 7
  Encoded8To3ButtonConfig buttonConfig(BUTTON_PIN0, BUTTON_PIN1, BUTTON_PIN2);
#else
  Encoded4To2ButtonConfig buttonConfig(BUTTON_PIN0, BUTTON_PIN1);
#endif
  AceButton b1(&buttonConfig, 1);
  AceButton b2(&buttonConfig, 2);
  AceButton b3(&buttonConfig, 3);
#if NUM_BUTTONS == 7
  AceButton b4(&buttonConfig, 4);
  AceButton b5(&buttonConfig, 5);
  AceButton b6(&buttonConfig, 6);
  AceButton b7(&buttonConfig, 7);
#endif

// Forward reference to prevent Arduino compiler becoming confused.
void handleEvent(AceButton*, uint8_t, uint8_t);

void setup() {
  delay(1000); // some microcontrollers reboot twice
  Serial.begin(115200);
  while (! Serial); // Wait until Serial is ready - Leonardo/Micro
  Serial.println(F("setup(): begin"));

  // initialize built-in LED as an output
  pinMode(LED_PIN, OUTPUT);

  // Pins uses the built-in pull up register.
  pinMode(BUTTON_PIN0, INPUT_PULLUP);
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
#if NUM_BUTTONS == 7
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
#endif

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  buttonConfig.setEventHandler(handleEvent);
  buttonConfig.setFeature(ButtonConfig::kFeatureClick);
  buttonConfig.setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig.setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig.setFeature(ButtonConfig::kFeatureRepeatPress);

  Serial.println(F("setup(): ready"));
}

void loop() {
  // Should be called every 4-5ms or faster, for the default debouncing time
  // of ~20ms.
  b1.check();
  b2.check();
  b3.check();
#if NUM_BUTTONS == 7
  b4.check();
  b5.check();
  b6.check();
  b7.check();
#endif
}

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {

  // Print out a message for all events.
  Serial.print(F("handleEvent(): "));
  Serial.print(F("virtualPin: "));
  Serial.print(button->getPin());
  Serial.print(F("; eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);

  // Control the LED only for the Pressed and Released events.
  // Notice that if the MCU is rebooted while the button is pressed down, no
  // event is triggered and the LED remains off.
  switch (eventType) {
    case AceButton::kEventPressed:
      digitalWrite(LED_PIN, LED_ON);
      break;
    case AceButton::kEventReleased:
      digitalWrite(LED_PIN, LED_OFF);
      break;
  }
}
