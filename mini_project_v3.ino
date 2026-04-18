// ============================================================================
//  Assistive Bidirectional Communication Device
//  Authors: Paris,Pranav,Rashid,Richu
//  Hardware: ESP32 + nRF24L01+ + 16x2 I2C LCD + Rotary Encoder
//            + Active Buzzer + 2x Push Buttons
//
//  Three operating modes:
//    1. Letter Select  — scroll A-Z with encoder, build words, send via radio
//    2. Morse Code     — SND=dot, BKS=dash, timeout/encoder press commits char
//    3. Learn Morse    — build words like Letter mode, long-press plays morse
//
//  Reception is IRQ-driven (nRF24 IRQ pin) so the main loop is never blocked
//  waiting for a packet. Morse playback runs on a separate FreeRTOS task
//  (core 0) so the LCD can keep scrolling while the buzzer plays (core 1).
// ============================================================================

#include <Wire.h>               // I2C bus (for LCD)
#include <LiquidCrystal_I2C.h> // 16x2 LCD over I2C
#include <SPI.h>                // SPI bus (for nRF24)
#include <nRF24L01.h>           // nRF24L01+ register definitions
#include <RF24.h>               // nRF24L01+ driver

// ─── Device role ─────────────────────────────────────────────────────────────
// Change to 1 on the second device so they use opposite pipe addresses
#define ROLE 0   // 0 = device A   |   1 = device B

// ─── Pin assignments ─────────────────────────────────────────────────────────
#define CE_PIN   4   // nRF24 Chip Enable
#define CSN_PIN  5   // nRF24 SPI Chip Select
#define IRQ_PIN  2   // nRF24 IRQ (active-low, fires on packet received)
#define ENC_A   12   // Rotary encoder output A (quadrature)
#define ENC_B   13   // Rotary encoder output B (quadrature)
#define ENC_SW  14   // Rotary encoder push button
#define SND     27   // Send / Dot button
#define BKS     26   // Backspace / Dash button
#define BUZZER  25   // Active buzzer (HIGH = on, LOW = off)

// ─── Morse timing (milliseconds) ─────────────────────────────────────────────
#define MORSE_DOT_DUR       120   // Duration of a dot beep
#define MORSE_DASH_DUR      360   // Duration of a dash beep (3x dot)
#define MORSE_ELEM_PAUSE     80   // Silence between elements within a character
#define MORSE_CHAR_PAUSE    300   // Silence between characters
#define MORSE_CHAR_TIMEOUT 1500   // Idle time after which pending morse is auto-committed

// ─── Button timing ───────────────────────────────────────────────────────────
#define LONG_PRESS_MS  500   // Hold duration to trigger a long press

// ─── Display timing ──────────────────────────────────────────────────────────
#define RX_DISPLAY_MS      5000   // How long a received message stays on row 0
#define SCROLL_INTERVAL_MS  350   // Milliseconds between scroll steps for long messages

// ─── Operating modes ─────────────────────────────────────────────────────────
enum Mode { MODE_LETTER = 0, MODE_MORSE = 1, MODE_LEARN = 2 }; 
//An enum is a way to create a custom data type with named values instead of raw numbers.
const char* modeNames[] = {
  "Select Letter ",   // Padded to 14 chars so LCD row is always fully overwritten
  "Morse Code    ",
  "Learn Morse   "
};
Mode currentMode = MODE_LETTER;   // Start in Letter Select mode

// ─── Hardware objects ────────────────────────────────────────────────────────
RF24 radio(CE_PIN, CSN_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);   // I2C address 0x27, 16 columns, 2 rows

// Pipe addresses — device A writes on 00001 and reads on 00002, device B is reversed
const byte addresses[][6] = { "00001", "00002" };

// ─── Morse code table ────────────────────────────────────────────────────────
// Index 0–25 = letters A–Z, index 26 = word space
// '.' = dot, '-' = dash, '\0'(null character) = end of code string
const char* morseTable[27] = {
  ".-",    // A
  "-...",  // B
  "-.-.",  // C
  "-..",   // D
  ".",     // E
  "..-.",  // F
  "--.",   // G
  "....",  // H
  "..",    // I
  ".---",  // J
  "-.-",   // K
  ".-..",  // L
  "--",    // M
  "-.",    // N
  "---",   // O
  ".--.",  // P
  "--.-",  // Q
  ".-.",   // R
  "...",   // S
  "-",     // T
  "..-",   // U
  "...-",  // V
  ".--",   // W
  "-..-",  // X
  "-.--",  // Y
  "--..",  // Z
  " "      // word space (index 26)
};

// Character set available in Letter/Learn mode (A–Z plus space at index 26)
const char Letters[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ ";

// ─── Rotary encoder (quadrature state machine) ───────────────────────────────
//
// A rotary encoder outputs two square waves (A and B) 90° out of phase.
// By reading both pins on every edge change and looking up the transition
// in a table we get reliable +1 / -1 ticks with hardware debounce.
//
// encTable[16]: indexed by (previousState << 2 | currentAB)
//   +1 = clockwise tick
//   -1 = counter-clockwise tick
//    0 = invalid transition (bounce) — ignored
//
volatile int     encDelta = 0;     // Accumulated ticks since last loop() read
volatile uint8_t encState = 0;     // Last known AB state (2 bits)
int              encPosition = 0;  // Current letter index 0–26 (read in loop)

const int8_t encTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ISR fires on any change of ENC_A or ENC_B
// Reads both pins atomically, looks up the transition, accumulates delta
void IRAM_ATTR isr_encoder() {
  uint8_t ab = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  encState   = ((encState << 2) | ab) & 0x0F;
  encDelta  += encTable[encState];
}

// ─── Radio IRQ ───────────────────────────────────────────────────────────────
//
// The nRF24 pulls IRQ_PIN LOW when a packet arrives.
// The ISR just sets a flag — actual SPI work happens in loop() because
// SPI is not safe to call from an ISR on ESP32.
//
volatile bool radioIRQ = false;

void IRAM_ATTR isr_radio() {
  radioIRQ = true;
}

// ─── Button state machine ────────────────────────────────────────────────────
//
// Each button is tracked with a ButtonState struct.
// pollButton() is called every loop() iteration. It:
//   - Detects press (LOW edge on INPUT_PULLUP pin)
//   - Fires onLong() once after LONG_PRESS_MS hold
//   - Fires onShort() on release if long press never fired
//
struct ButtonState {
  bool     lastRaw   = true;   // Previous raw pin read (HIGH = released)
  bool     pressed   = false;  // Currently held down
  uint32_t downAt    = 0;      // millis() when press started
  bool     longFired = false;  // Long press callback already called this press
};

// Forward declaration needed because Arduino IDE won't auto-generate it
// for functions that take a struct reference + function pointers
void pollButton(ButtonState& b, int pin, void (*onShort)(), void (*onLong)());

ButtonState btnSND, btnBKS, btnENC;

// ─── Message buffers ─────────────────────────────────────────────────────────
String   composedMsg    = "";   // Message being built by the local user (row 1)
String   rxDisplay      = "";   // Last message received over radio (row 0)
uint32_t rxDisplayedAt  = 0;    // millis() when rxDisplay was last updated
bool     rxPending      = false;// True while rxDisplay should be shown on row 0
int      rxScrollOffset = 0;    // Current scroll position for long rx messages
uint32_t lastScrollTick = 0;    // millis() of last scroll step

// ─── Morse input state ───────────────────────────────────────────────────────
String   morseBuffer    = "";   // Dots/dashes entered so far for current char e.g. ".-"
uint32_t lastMorseInput = 0;    // millis() of last dot/dash input

// ─── Mode select UI ──────────────────────────────────────────────────────────
bool inModeSelect  = false;  // True while the mode selection screen is active
int  modeSelectIdx = 0;      // Which mode is highlighted in the selector

// ─── Display dirty flag ──────────────────────────────────────────────────────
// updateDisplay() is only called when something changed, reducing LCD flicker
bool displayDirty = true;

// ─── FreeRTOS morse playback task ────────────────────────────────────────────
//
// Morse audio for received messages runs on core 0 as a FreeRTOS task so that
// loop() on core 1 can keep scrolling the LCD during playback.
// morseTaskHandle is NULL when no playback is active.
//
TaskHandle_t morseTaskHandle  = NULL;
char         morsePlayBuffer[64] = "";  // Written before task launch, read by task


// ============================================================================
//  MORSE PLAYBACK — FREERTOS TASK  (core 0)
//
//  Plays the string in morsePlayBuffer character by character on the buzzer.
//  Uses vTaskDelay() instead of delay() so the FreeRTOS scheduler can run
//  other tasks during the pauses.
//  Deletes itself when done and sets morseTaskHandle = NULL.
// ============================================================================
void morsePlayTask(void* param) {
  String word = String(morsePlayBuffer);
  word.toUpperCase();

  for (int i = 0; i < (int)word.length(); i++) {
    char c = word[i];

    // Word space — just wait
    if (c == ' ') {
      vTaskDelay(pdMS_TO_TICKS(MORSE_CHAR_PAUSE * 2));
      continue;
    }

    // Find the letter index in the alphabet
    int idx = -1;
    for (int j = 0; j < 26; j++) {
      if (Letters[j] == c) { idx = j; break; }
    }

    if (idx >= 0) {
      const char* code = morseTable[idx];

      // Play each dot or dash
      for (int k = 0; code[k] != '\0'; k++) {
        digitalWrite(BUZZER, HIGH);                          // Buzzer ON

        if (code[k] == '.')
          vTaskDelay(pdMS_TO_TICKS(MORSE_DOT_DUR));         // Short beep
        else if (code[k] == '-')
          vTaskDelay(pdMS_TO_TICKS(MORSE_DASH_DUR));        // Long beep

        digitalWrite(BUZZER, LOW);                           // Buzzer OFF
        vTaskDelay(pdMS_TO_TICKS(MORSE_ELEM_PAUSE));        // Gap between elements
      }

      vTaskDelay(pdMS_TO_TICKS(MORSE_CHAR_PAUSE));          // Gap between characters
    }
  }

  digitalWrite(BUZZER, LOW);   // Safety: make sure buzzer is off
  morseTaskHandle = NULL;       // Signal that playback has finished
  vTaskDelete(NULL);            // Task deletes itself
}

// Launch a non-blocking morse playback for the given word.
// If playback is already running it is cancelled first.
void playWordMorseAsync(const String& word) {
  // Cancel any currently running playback
  if (morseTaskHandle != NULL) {
    vTaskDelete(morseTaskHandle);
    morseTaskHandle = NULL;
    digitalWrite(BUZZER, LOW);
  }

  // Copy word into shared buffer (task will read this)
  word.toCharArray(morsePlayBuffer, sizeof(morsePlayBuffer));

  // Create task pinned to core 0; loop() runs on core 1
  xTaskCreatePinnedToCore(
    morsePlayTask,      // Task function
    "morse",            // Task name (for debugging)
    2048,               // Stack size in bytes
    NULL,               // No parameter passed
    1,                  // Priority (1 = low, above idle)
    &morseTaskHandle,   // Store handle so we can cancel it
    0                   // Pin to core 0
  );
}

// Blocking morse playback used by Learn mode (user explicitly triggered it,
// so blocking the loop while the word plays is acceptable and expected).
void playWordMorseBlocking(const String& word) {
  // Cancel any async playback first to avoid two tasks fighting the buzzer
  if (morseTaskHandle != NULL) {
    vTaskDelete(morseTaskHandle);
    morseTaskHandle = NULL;
    digitalWrite(BUZZER, LOW);
  }

  String w = word;
  w.toUpperCase();

  for (int i = 0; i < (int)w.length(); i++) {
    char c = w[i];

    if (c == ' ') { delay(MORSE_CHAR_PAUSE * 2); continue; }

    int idx = -1;
    for (int j = 0; j < 26; j++) {
      if (Letters[j] == c) { idx = j; break; }
    }

    if (idx >= 0) {
      const char* code = morseTable[idx];
      for (int k = 0; code[k] != '\0'; k++) {
        digitalWrite(BUZZER, HIGH);

        if (code[k] == '.')      delay(MORSE_DOT_DUR);
        else if (code[k] == '-') delay(MORSE_DASH_DUR);

        digitalWrite(BUZZER, LOW);
        delay(MORSE_ELEM_PAUSE);
      }
      delay(MORSE_CHAR_PAUSE);
    }
  }
  digitalWrite(BUZZER, LOW);
}


// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Configure input pins with internal pull-ups
  // (buttons connect pin to GND when pressed — LOW = pressed)
  pinMode(ENC_A,   INPUT_PULLUP);
  pinMode(ENC_B,   INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);
  pinMode(SND,     INPUT_PULLUP);
  pinMode(BKS,     INPUT_PULLUP);

  // IRQ is driven by the nRF24 module (open-drain, active LOW)
  // No pull-up needed — the module holds the line HIGH when idle
  pinMode(IRQ_PIN, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);   // Make sure buzzer is off at startup

  // Encoder: attach both channels to the same ISR on CHANGE
  // so every edge (rising and falling) of both A and B is captured
  attachInterrupt(digitalPinToInterrupt(ENC_A),   isr_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B),   isr_encoder, CHANGE);

  // Radio IRQ: fires on FALLING edge (nRF24 pulls pin LOW when packet arrives)
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), isr_radio,   FALLING);

  // Seed the encoder state machine with the current pin readings
  encState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);

  // ── LCD init ────────────────────────────────────────────────────────────
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing....");
  delay(1200);
  lcd.clear();

  // ── Radio init ──────────────────────────────────────────────────────────
  if (!radio.begin()) {
    lcd.print("Radio FAIL      ");
    while (1);   // Halt — no point continuing without radio
  }

  radio.setPALevel(RF24_PA_MIN);   // Use minimum power (increase if range is poor)

  // Mask tx-ok (1) and tx-fail (1) interrupts on the nRF24.
  // Only rx-ready (0 = unmasked) will pull IRQ_PIN low.
  // Without this, every transmission on this device would also trigger the ISR.
  radio.maskIRQ(1, 1, 0);

  // Device A: write on pipe 0, read on pipe 1
  // Device B: write on pipe 1, read on pipe 0
  if (ROLE == 0) {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  } else {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  }

  radio.startListening();   // Start in receive mode

  // ── Show startup mode ────────────────────────────────────────────────────
  lcd.setCursor(0, 0); lcd.print("Mode:           ");
  lcd.setCursor(0, 1);
  String mn = String(modeNames[currentMode]);
  while (mn.length() < 16) mn += ' ';
  lcd.print(mn);
  delay(1000);
  lcd.clear();
  displayDirty = true;
}


// ============================================================================
//  MAIN LOOP  (runs on core 1)
// ============================================================================
void loop() {

  // ── 1. Read encoder delta ─────────────────────────────────────────────────
  // Atomically grab accumulated ticks from the ISR and reset to zero
  int delta = 0;
  noInterrupts();
    delta    = encDelta;
    encDelta = 0;
  interrupts();

  if (delta != 0) {
    if (inModeSelect) {
      // Scroll through mode choices (wrap around 0–2)
      modeSelectIdx = (modeSelectIdx + (delta > 0 ? 1 : -1) + 3) % 3;
    } else {
      // Scroll through the alphabet (wrap around 0–26)
      encPosition += (delta > 0 ? 1 : -1);
      if (encPosition < 0)  encPosition = 26;
      if (encPosition > 26) encPosition = 0;
    }
    displayDirty = true;
  }

  // ── 2. Handle radio IRQ ───────────────────────────────────────────────────
  // radioIRQ is set by isr_radio(); we handle it here where SPI is safe
  if (radioIRQ) {
    radioIRQ = false;
    handleRadioRX();
  }

  // ── 3. Poll buttons ───────────────────────────────────────────────────────
  readButtons();

  // ── 4. Morse character timeout ────────────────────────────────────────────
  // If the user stops entering dots/dashes for MORSE_CHAR_TIMEOUT ms,
  // auto-commit the pending morse buffer as a character
  if (currentMode == MODE_MORSE && morseBuffer.length() > 0) {
    if ((millis() - lastMorseInput) >= MORSE_CHAR_TIMEOUT) {
      commitMorseChar();
    }
  }

  // ── 5. RX message scroll tick ─────────────────────────────────────────────
  // Advance the scroll offset every SCROLL_INTERVAL_MS for long messages
  if (rxPending && (int)rxDisplay.length() > 16) {
    if ((millis() - lastScrollTick) >= SCROLL_INTERVAL_MS) {
      lastScrollTick = millis();
      rxScrollOffset++;
      // Reset when we've scrolled through the full message + spacer
      if (rxScrollOffset > (int)rxDisplay.length() + 3)
        rxScrollOffset = 0;
      displayDirty = true;
    }
  }

  // ── 6. RX display timeout ─────────────────────────────────────────────────
  // Clear the received message from row 0 after RX_DISPLAY_MS
  // (only for short messages that don't scroll — scrolling messages
  //  stay until a new message arrives)
  if (rxPending && (int)rxDisplay.length() <= 16) {
    if ((millis() - rxDisplayedAt) >= RX_DISPLAY_MS) {
      rxPending      = false;
      rxDisplay      = "";
      rxScrollOffset = 0;
      displayDirty   = true;
    }
  }

  // ── 7. Refresh LCD only when something changed ────────────────────────────
  if (displayDirty) {
    updateDisplay();
    displayDirty = false;
  }
}


// ============================================================================
//  BUTTON POLLING
// ============================================================================

// Call all three button poll functions each loop iteration
void readButtons() {
  pollButton(btnSND, SND, onSND_short, onSND_long);
  pollButton(btnBKS, BKS, onBKS_short, onBKS_long);
  pollButton(btnENC, ENC_SW, onENC_short, onENC_long);
}

// Generic button state machine.
// Detects short press (released before LONG_PRESS_MS) and
// long press (held for LONG_PRESS_MS — fires once, not repeatedly).
void pollButton(ButtonState& b, int pin,
                void (*onShort)(), void (*onLong)()) {
  bool raw = digitalRead(pin);   // HIGH = released (INPUT_PULLUP)

  // ── Detect press (falling edge) ──────────────────────────────────────────
  if (raw == LOW && b.lastRaw == HIGH) {
    b.pressed   = true;
    b.downAt    = millis();
    b.longFired = false;
  }

  // ── Check for long press while held ──────────────────────────────────────
  if (b.pressed && !b.longFired) {
    if ((millis() - b.downAt) >= LONG_PRESS_MS) {
      b.longFired = true;
      onLong();   // Fire long press callback exactly once
    }
  }

  // ── Detect release (rising edge) ─────────────────────────────────────────
  if (raw == HIGH && b.lastRaw == LOW) {
    if (b.pressed && !b.longFired) {
      onShort();   // Short press: released before long press threshold
    }
    b.pressed = false;
  }

  b.lastRaw = raw;
}


// ============================================================================
//  BUTTON ACTION CALLBACKS
// ============================================================================

// ── SND short press ──────────────────────────────────────────────────────────
//  Mode select : confirm selected mode
//  Letter/Learn: append current letter to composedMsg
//  Morse       : add dot to morseBuffer
void onSND_short() {
  if (inModeSelect) { confirmModeSelect(); return; }

  switch (currentMode) {
    case MODE_LETTER:
    case MODE_LEARN:
      composedMsg += Letters[encPosition];
      displayDirty = true;
      break;

    case MODE_MORSE:
      morseBuffer   += '.';
      lastMorseInput = millis();
      displayDirty   = true;
      break;
  }
}

// ── SND long press ───────────────────────────────────────────────────────────
//  Letter : transmit composedMsg over radio
//  Morse  : commit pending buffer then transmit
//  Learn  : play composedMsg as morse on buzzer (blocking)
void onSND_long() {
  if (inModeSelect) return;

  switch (currentMode) {
    case MODE_LETTER:
      sendMessage();
      break;

    case MODE_MORSE:
      if (morseBuffer.length() > 0) commitMorseChar();
      sendMessage();
      break;

    case MODE_LEARN:
      if (composedMsg.length() > 0)
        playWordMorseBlocking(composedMsg);   // Play entire message as morse
      break;
  }
}

// ── BKS short press ──────────────────────────────────────────────────────────
//  Letter/Learn: delete last character from composedMsg (backspace)
//  Morse       : add dash to morseBuffer
void onBKS_short() {
  if (inModeSelect) return;

  switch (currentMode) {
    case MODE_LETTER:
    case MODE_LEARN:
      if (composedMsg.length() > 0)
        composedMsg.remove(composedMsg.length() - 1);
      displayDirty = true;
      break;

    case MODE_MORSE:
      morseBuffer   += '-';
      lastMorseInput = millis();
      displayDirty   = true;
      break;
  }
}

// ── BKS long press ───────────────────────────────────────────────────────────
//  Letter/Learn: clear entire composedMsg
//  Morse       : clear pending morseBuffer first; if already empty,
//                delete last committed character from composedMsg
void onBKS_long() {
  if (inModeSelect) return;

  switch (currentMode) {
    case MODE_LETTER:
    case MODE_LEARN:
      composedMsg  = "";
      displayDirty = true;
      break;

    case MODE_MORSE:
      if (morseBuffer.length() > 0) {
        // First long-press clears the pending dots/dashes
        morseBuffer  = "";
        displayDirty = true;
      } else if (composedMsg.length() > 0) {
        // Second long-press (buffer already empty) removes last committed char
        composedMsg.remove(composedMsg.length() - 1);
        displayDirty = true;
      }
      break;
  }
}

// ── Encoder short press ──────────────────────────────────────────────────────
//  Morse mode: immediately commit pending morseBuffer without waiting for timeout
void onENC_short() {
  if (inModeSelect) return;
  if (currentMode == MODE_MORSE && morseBuffer.length() > 0)
    commitMorseChar();
}

// ── Encoder long press ───────────────────────────────────────────────────────
//  Opens the mode selection screen
void onENC_long() {
  inModeSelect  = true;
  modeSelectIdx = (int)currentMode;   // Highlight the currently active mode
  displayDirty  = true;
}


// ============================================================================
//  MODE SELECT
// ============================================================================

// Called when SND short is pressed while inModeSelect is true.
// Applies the highlighted mode and exits the mode select screen.
void confirmModeSelect() {
  currentMode  = (Mode)modeSelectIdx;
  inModeSelect = false;
  morseBuffer  = "";   // Clear any partial input from previous mode
  composedMsg  = "";

  // Show confirmation on LCD for 800ms
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Mode:           ");
  lcd.setCursor(0, 1);
  String mn = String(modeNames[currentMode]);
  while (mn.length() < 16) mn += ' ';
  lcd.print(mn);
  delay(800);
  lcd.clear();

  displayDirty = true;
}


// ============================================================================
//  SEND MESSAGE
//  Transmits composedMsg over the nRF24 radio, then clears the buffer.
// ============================================================================
void sendMessage() {
  if (composedMsg.length() == 0) return;   // Nothing to send

  char text[32] = "";
  composedMsg.toCharArray(text, 32);

  // Switch to TX mode, send, switch back to RX mode
  radio.stopListening();
  radio.write(&text, sizeof(text));
  radio.startListening();

  composedMsg  = "";
  displayDirty = true;
  lcd.clear();
}


// ============================================================================
//  MORSE COMMIT
//  Looks up morseBuffer (e.g. ".-") in the morse table, appends the
//  decoded letter to composedMsg, then clears the buffer.
// ============================================================================
void commitMorseChar() {
  if (morseBuffer.length() == 0) return;

  char found = '?';   // '?' shown if the pattern doesn't match any letter

  for (int i = 0; i < 27; i++) {
    if (morseBuffer == String(morseTable[i])) {
      found = Letters[i];
      break;
    }
  }

  composedMsg += found;
  morseBuffer  = "";
  displayDirty = true;
}


// ============================================================================
//  RADIO RX HANDLER
//  Called from loop() when radioIRQ flag is set by isr_radio().
//  Calling radio.whatHappened() clears the IRQ latch on the nRF24 chip,
//  which releases IRQ_PIN so it can fire again for the next packet.
// ============================================================================
void handleRadioRX() {
  bool txOk, txFail, rxReady;
  radio.whatHappened(txOk, txFail, rxReady);   // Must call this to clear IRQ

  if (!rxReady) return;   // Shouldn't happen (we masked the other interrupts),
                          // but guard anyway

  // Read all pending packets (there is usually just one)
  while (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));

    rxDisplay      = String(text);
    rxDisplayedAt  = millis();
    rxScrollOffset = 0;
    lastScrollTick = millis();
    rxPending      = true;

    // Update the display immediately so row 0 shows the new message
    // before the morse task (if launched) takes over the buzzer
    displayDirty = true;
    updateDisplay();
    displayDirty = false;

    // In Morse mode: play the received message as morse on the buzzer.
    // Async so the LCD scroll continues in the background on core 1.
    if (currentMode == MODE_MORSE) {
      playWordMorseAsync(rxDisplay);
    }
  }
}


// ============================================================================
//  DISPLAY
//  Redraws both LCD rows based on current state.
//  Only called when displayDirty is true to minimise flicker.
//
//  Row 0: Mode select UI  OR  received message  OR  mode name
//  Row 1: Compose area (letter cursor + composed text, or morse buffer)
// ============================================================================
void updateDisplay() {

  // ── Mode select screen ────────────────────────────────────────────────────
  if (inModeSelect) {
    lcd.setCursor(0, 0);
    lcd.print("Select mode:    ");
    lcd.setCursor(0, 1);
    String line = ">";
    line += modeNames[modeSelectIdx];
    while (line.length() < 16) line += ' ';
    lcd.print(line.substring(0, 16));
    return;
  }

  // ── Row 0: received message or mode name ─────────────────────────────────
  lcd.setCursor(0, 0);

  if (rxPending) {
    if ((int)rxDisplay.length() <= 16) {
      // Short message: display statically, padded to fill the row
      String padded = rxDisplay;
      while (padded.length() < 16) padded += ' ';
      lcd.print(padded);
    } else {
      // Long message: rotate/scroll continuously
      // Append spaces so the end of the message blends into the beginning
      String src = rxDisplay + "   ";
      int    len = src.length();
      int    off = rxScrollOffset % len;
      // Double the source string so substring() never overruns
      String view = (src + src).substring(off, off + 16);
      lcd.print(view);
    }
  } else {
    // No pending message — show the current mode name
    String row0 = String(modeNames[currentMode]);
    while (row0.length() < 16) row0 += ' ';
    lcd.print(row0.substring(0, 16));
  }

  // ── Row 1: compose area ───────────────────────────────────────────────────
  lcd.setCursor(0, 1);

  if (currentMode == MODE_LETTER || currentMode == MODE_LEARN) {
    // Format:  >X [composed message]
    // ">X " occupies the first 3 characters (cursor indicator + selected letter + space)
    // The remaining 13 characters show the tail of composedMsg
    char cursorChar = Letters[encPosition];
    int  available  = 13;

    String view = composedMsg;
    if ((int)view.length() > available)
      view = view.substring(view.length() - available);   // Show most recent chars
    while ((int)view.length() < available) view += ' ';  // Pad to fill row

    String row1 = ">"; 
    row1 += cursorChar;
    row1 += ' ';
    row1 += view;
    lcd.print(row1);

  } else {
    // Morse mode format:  [pending dots/dashes]|[committed message]
    // The '|' separator visually divides the in-progress character from done chars
    String row1 = morseBuffer + "|" + composedMsg;

    if ((int)row1.length() < 16) {
      while ((int)row1.length() < 16) row1 += ' ';
      lcd.print(row1);
    } else {
      // Right-align: always show the most recently entered content
      lcd.print(row1.substring(row1.length() - 16));
    }
  }
}
