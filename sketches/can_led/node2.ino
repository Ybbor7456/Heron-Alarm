#include <SPI.h>
#include "mcp_can.h"

constexpr byte CS_PIN   = 17;   // MCP2155 CS
constexpr byte INT_PIN  = 20;   // only necessary if the board lacks a pull-up
constexpr byte LED_PIN  = 8;
constexpr unsigned CAN_ID_MATCH = 0x101;

constexpr unsigned ID_NODE1 = 0x310;   // Pi #1 (this board)
constexpr unsigned ID_NODE2 = 0x311;   // Pi #2
constexpr unsigned ID_NODE3 = 0x312; 

constexpr byte LED_OFF = 0x00;
constexpr byte LED_ON  = 0x01;

MCP_CAN CAN(CS_PIN);


void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}  // allow USB CDC to enumerate (if connected)

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Bring up CAN (must match transmitter: 500 kbps, 8 MHz crystal)
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL");
    while (1) {}
  }

  // Accept only our mailbox ID (standard 11-bit)
  // Mask 0 covers filters 0–1, mask 1 covers filters 2–5.
  CAN.init_Mask(0, 0, 0x7FF);           // match all 11 bits
  CAN.init_Filt(0, 0, ID_NODE2);
  CAN.init_Filt(1, 0, ID_NODE2);

  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(2, 0, ID_NODE2);
  CAN.init_Filt(3, 0, ID_NODE2);
  CAN.init_Filt(4, 0, ID_NODE2);
  CAN.init_Filt(5, 0, ID_NODE2);

  CAN.setMode(MCP_NORMAL);

  // If INT is wired, internal pull-up keeps it defined. Safe even if unconnected.
  pinMode(INT_PIN, INPUT_PULLUP);

  Serial.println("RX ready (node #2)");
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long id = 0;
    byte len = 0;
    byte buf[8] = {0};

    // Use overload that returns ID (supported widely)
    CAN.readMsgBuf(&id, &len, buf);

    // Because filters only allow 0x311 std frames, we can act directly:
    if (id == ID_NODE2 && len >= 1) {
      if (buf[0] == LED_ON)  { digitalWrite(LED_PIN, HIGH); Serial.println("LED ON"); }
      else                   { digitalWrite(LED_PIN, LOW);  Serial.println("LED OFF"); }
    }

    // Optional debug
    Serial.print("RX id=0x"); Serial.print(id, HEX);
    Serial.print(" len=");    Serial.print(len);
    Serial.print(" data:");
    for (byte i = 0; i < len; ++i) { Serial.print(' '); Serial.print(buf[i], HEX); }
    Serial.println();
  }
}
