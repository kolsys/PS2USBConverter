/**
 * The PS/2 to USB keyboard converter based on DorianRudolph's work
 * @see https://gist.github.com/DorianRudolph/ca283dfdfd185bc812b7
 *
 */

#include <Keyboard.h>

#define DATA_PIN  4 // Define DATA_PIN (PS/2 Data)
#define CLOCK_PIN 3  // Define CLOCK_PIN (PS/2 Clock). This PIN must supporting interrupts.

#define BUFFER_SIZE 45

static volatile uint8_t buffer[BUFFER_SIZE];
static volatile uint8_t head, tail;
static volatile uint8_t sendBits, msg, bitCount, setBits;

KeyReport report;
uint8_t K[255], KE[255];
uint8_t leds;
bool isSendLeds;

void setupKeymaps() {
  K[0x1C] = 4;
  K[0x32] = 5;
  K[0x21] = 6;
  K[0x23] = 7;
  K[0x24] = 8;
  K[0x2B] = 9;
  K[0x34] = 10;
  K[0x33] = 11;
  K[0x43] = 12;
  K[0x3B] = 13;
  K[0x42] = 14;
  K[0x4B] = 15;
  K[0x3A] = 16;
  K[0x31] = 17;
  K[0x44] = 18;
  K[0x4D] = 19;
  K[0x15] = 20;
  K[0x2D] = 21;
  K[0x1B] = 22;
  K[0x2C] = 23;
  K[0x3C] = 24;
  K[0x2A] = 25;
  K[0x1D] = 26;
  K[0x22] = 27;
  K[0x35] = 28;
  K[0x1A] = 29;
  K[0x45] = 39;
  K[0x16] = 30;
  K[0x1E] = 31;
  K[0x26] = 32;
  K[0x25] = 33;
  K[0x2E] = 34;
  K[0x36] = 35;
  K[0x3D] = 36;
  K[0x3E] = 37;
  K[0x46] = 38;
  K[0x0E] = 53;
  K[0x4E] = 45;
  K[0x55] = 46;
  K[0x5D] = 49;
  K[0x66] = 42;
  K[0x29] = 44;
  K[0x0D] = 43;
  K[0x58] = 227; //CAPS 57
  K[0x12] = 225;
  K[0x14] = 224;
  K[0x11] = 226;
  K[0x59] = 229;
  K[0x5A] = 40;
  K[0x76] = 41;
  K[0x05] = 58;
  K[0x06] = 59;
  K[0x04] = 60;
  K[0x0C] = 61;
  K[0x03] = 62;
  K[0x0B] = 63;
  K[0x83] = 64;
  K[0x0A] = 65;
  K[0x01] = 66;
  K[0x09] = 67;
  K[0x78] = 68;
  K[0x07] = 69;
  K[0x7E] = 57;//Scroll 71
  K[0x54] = 47;
  K[0x77] = 83;
  K[0x7C] = 85;
  K[0x7B] = 86;
  K[0x79] = 87;
  K[0x71] = 99;
  K[0x70] = 98;
  K[0x69] = 89;
  K[0x72] = 90;
  K[0x7A] = 91;
  K[0x6B] = 92;
  K[0x73] = 93;
  K[0x74] = 94;
  K[0x6C] = 95;
  K[0x75] = 96;
  K[0x7D] = 97;
  K[0x5B] = 48;
  K[0x4C] = 51;
  K[0x52] = 52;
  K[0x41] = 54;
  K[0x49] = 55;
  K[0x4A] = 56;
  K[0x61] = 100;

  KE[0x1F] = 227;
  KE[0x14] = 228;
  KE[0x27] = 231;
  KE[0x11] = 230;
  KE[0x2F] = 101;
  KE[0x7c] = 70;
  KE[0x70] = 73;
  KE[0x6C] = 74;
  KE[0x7D] = 75;
  KE[0x71] = 76;
  KE[0x69] = 77;
  KE[0x7A] = 78;
  KE[0x75] = 82;
  KE[0x6B] = 80;
  KE[0x72] = 81;
  KE[0x74] = 79;
  KE[0x4A] = 84;
  KE[0x5A] = 88;
}

void PS2Interrupt(void) {
  static uint8_t bitcount = 0;
  static uint8_t incoming = 0;
  static uint32_t prevMS = 0;
  uint32_t nowMS;
  uint8_t n, val;

  if (!sendBits){
    val = digitalRead(DATA_PIN);
    nowMS = millis();
    if (nowMS - prevMS > 250) {
      bitcount = 0;
      incoming = 0;
    }
    prevMS = nowMS;
    n = bitcount - 1;
    if (n <= 7) {
      incoming |= (val << n);
    }
    bitcount++;
    if (bitcount == 11) {
      uint8_t i = head + 1;
      if (i >= BUFFER_SIZE) i = 0;
      if (i != tail) {
        buffer[i] = incoming;
        head = i;
      }
      bitcount = 0;
      incoming = 0;
    }
  } else {
    --sendBits;
    uint8_t b = bitCount - 1;
    if (b == 8){
      digitalWrite(DATA_PIN, !(setBits & 1));
    } else if (b == 9) {
      pinMode(DATA_PIN, INPUT_PULLUP);
    } else if (b < 8) {
      bool bt = (msg >> b) & 1;
      digitalWrite(DATA_PIN,  bt);
      setBits += bt;
    }
    ++bitCount;
  }
}

static inline uint8_t getScancode(void) {
  uint8_t c, i;

  i = tail;
  if (i == head) return 0;
  i++;
  if (i >= BUFFER_SIZE) i = 0;
  c = buffer[i];
  tail = i;
  return c;
}

void setupPS2() {
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), PS2Interrupt, FALLING);
  pinMode(CLOCK_PIN, INPUT_PULLUP);
  pinMode(DATA_PIN, INPUT_PULLUP);
  head = 0;
  tail = 0;
  sendBits = 0;
}

void setup() {
  setupKeymaps();
  setupPS2();

  Keyboard.begin();
  delay(1000);
}

bool ext, brk;
int skip;

void addToReport(uint8_t k) {
  uint8_t i;
  if (k >= 224) {
    report.modifiers |= 1 << (k - 224);
  } else if (report.keys[0] != k && report.keys[1] != k &&
             report.keys[2] != k && report.keys[3] != k &&
             report.keys[4] != k && report.keys[5] != k) {
    for (i = 0; i < 6; ++i) {
      if (report.keys[i] == 0) {
        report.keys[i] = k;
        break;
      }
    }
  }
}

void removeFromReport(uint8_t k) {
  uint8_t i;
  if (k >= 224) {
    report.modifiers &= ~(1 << (k - 224));
  } else {
    for (i = 0; i < 6; ++i) {
      if (report.keys[i] == k) {
        report.keys[i] = 0;
        break;
      }
    }
  }
}

// Copypaste from Keyboard lib
void sendReport() {
  HID().SendReport(2, &report, sizeof(KeyReport));
}

void sendMessage(uint8_t m) {
  noInterrupts();
  pinMode(CLOCK_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(60);
  pinMode(CLOCK_PIN, INPUT_PULLUP);
  msg = m;
  bitCount = 0;
  sendBits = 12;
  setBits = 0;
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, LOW);
  interrupts();
}

void loop() {
  uint8_t k = getScancode(), k2;
  if (k) {
    if (skip) {
      --skip;
    } else {
      if (k == 0xE0) {
        ext = true;
      } else if (k == 0xF0) {
        brk = true;
      } else if (k == 0xFA) {
        if (isSendLeds) {
          isSendLeds = false;
          sendMessage(leds);
        }
      } else {
        if (k == 0xE1) {
          k2 = 72;
          skip = 7;
          brk = true;
          addToReport(k2);
          sendReport();
        } else {
          k2 = ext ? KE[k] : K[k];
        }

        if (k2){
          if (brk){
            removeFromReport(k2);
            if (k2 == 83 || k2 == 71 || k2 == 57){
              isSendLeds = true;
              if (k2 == 83) {
                leds ^= 2;
              } else if (k2 == 71) {
                leds ^= 1;
              } else if (k2 == 57) {
                leds ^= 4;
              }
              sendMessage(0xED);
            }
          } else {
            addToReport(k2);
          }
          sendReport();
        }

        brk = false;
        ext = false;
      }
    }
  }
}