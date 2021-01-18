#include "ClearCore.h"
#include "EthernetManager.h"
#include "EthernetUdp.h"

// The INPUT_A_B_FILTER must match the Input A, B filter setting in MSP (Advanced >> Input A, B Filtering...)
constexpr auto INPUT_A_B_FILTER = 20;

// Select the baud rate to match the target serial device.
constexpr auto baudRate = 9600;

// Defines the motor's connector as ConnectorM0
auto &motor1 = ConnectorM1;
auto &motor2 = ConnectorM2;
auto &motor3 = ConnectorM3;

// Defines the manual control inputs
constexpr auto inputMotor1Close = A11;
constexpr auto inputMotor1Open = A10;
constexpr auto inputMotor2Close = A9;
constexpr auto inputMotor2Open = DI8;
constexpr auto inputMotor3Close = DI7;
constexpr auto inputMotor3Open = DI6;

// The current state of the input pins
PinStatus inputStatusMotor1Close;
PinStatus inputStatusMotor1Open;
PinStatus inputStatusMotor2Close;
PinStatus inputStatusMotor2Open;
PinStatus inputStatusMotor3Close;
PinStatus inputStatusMotor3Open;

int desiredMotorPosition1 = 1;
int desiredMotorPosition2 = 1;
int desiredMotorPosition3 = 1;

int interMovementDelay = 300000;

// The local port to listen for connections on. Pick a port over 49152
constexpr uint16_t localPort = 49443;

typedef uint8_t u8;
typedef uint16_t u16;

struct {
  u8 MotorOnePos;
  u8 MotorTwoPos;
  u8 MotorThreePos;
} typedef IncomingPacket;

typedef union {
  IncomingPacket packet;
  unsigned char raw[sizeof(IncomingPacket)];
} IncomingPacketSerializer;

// Buffer for holding received packets.
IncomingPacketSerializer incomingPacketBuffer;

struct {
  u8 MotorOneStatus;
  u8 MotorTwoStatus;
  u8 MotorThreeStatus;
  u16 Temperature;
} typedef StatusPacket;

typedef union {
  StatusPacket packet;
  unsigned char raw[sizeof(StatusPacket)];
} StatusPacketSerializer;

// Buffer for holding Status packets.
StatusPacketSerializer statusPacketBuffer;

EthernetUdp Udp;

void setup() {
  // Sets all motor connectors to the correct mode for Absolute Position mode
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_DIRECT);

  // Enforces the state of the motor's A and B inputs before enabling the motor
  motor1.MotorInAState(false);
  motor1.MotorInBState(false);
  motor2.MotorInAState(false);
  motor2.MotorInBState(false);
  motor3.MotorInAState(false);
  motor3.MotorInBState(false);

  // Set up serial communication at a baud rate of 9600 bps then wait up to 3 seconds for a port to open.
  Serial.begin(baudRate);
  uint32_t timeout = 3000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
    continue;
  }

  // Enables the motors; homing will begin automatically
  motor1.EnableRequest(true);
  Serial.println("Motor 1 Enabled");
  motor2.EnableRequest(true);
  Serial.println("Motor 2 Enabled");
  motor3.EnableRequest(true);
  Serial.println("Motor 3 Enabled");

  // Sets the 3 analog/digital inputs to input digital mode
  ConnectorA9.Mode(Connector::INPUT_DIGITAL);
  ConnectorA10.Mode(Connector::INPUT_DIGITAL);
  ConnectorA11.Mode(Connector::INPUT_DIGITAL);

  //    // Waits for HLFB to assert (waits for homing to complete if applicable)
  //   Serial.println("Waiting for motor 2 HLFB...");
  //    while (motor2.HlfbState() != MotorDriver::HLFB_ASSERTED) {
  //        continue;
  //    }
  //   Serial.println("Motor 2 Ready");

  // Run the setup for the ClearCore Ethernet manager.
  EthernetMgr.Setup();
  Serial.println("Ethernet Enabled");

  // Begin listening on the local port for UDP datagrams
  Udp.Begin(localPort);
  Serial.println("Listening for UDP packets");
}

// return text description given an integer motor position
String positionNumToString(int positionNum) {
  switch (positionNum) {
  case 1:
    return "full closed";
    break;
  case 2:
    return "full open";
    break;
  case 3:
    return "half open";
    break;
  case 4:
    return "mostly open";
    break;
  default:
    return "*ERROR*";
    break;
  }
}

void readInputsAndSetDesiredPositions() {
  // Read the state of the input connector.
  inputStatusMotor1Close = digitalRead(inputMotor1Close);
  inputStatusMotor1Open = digitalRead(inputMotor1Open);
  inputStatusMotor2Close = digitalRead(inputMotor2Close);
  inputStatusMotor2Open = digitalRead(inputMotor2Open);
  inputStatusMotor3Close = digitalRead(inputMotor3Close);
  inputStatusMotor3Open = digitalRead(inputMotor3Open);

  // takes close and open manual inputs and sets desired motor positions based on them. Position 1 is closed, position 2
  // is fully open
  if (inputStatusMotor1Close) {
    desiredMotorPosition1 = 1;
  } else if (inputStatusMotor1Open) {
    desiredMotorPosition1 = 2;
  }

  if (inputStatusMotor2Close) {
    desiredMotorPosition2 = 1;
  } else if (inputStatusMotor2Open) {
    desiredMotorPosition2 = 2;
  }

  if (inputStatusMotor3Close) {
    desiredMotorPosition3 = 1;
  } else if (inputStatusMotor3Open) {
    desiredMotorPosition3 = 2;
  }

  Serial.print("Motor 1 desired position: ");
  Serial.println(positionNumToString(desiredMotorPosition1));
  Serial.print("Motor 2 desired position: ");
  Serial.println(positionNumToString(desiredMotorPosition2));
  Serial.print("Motor 3 desired position: ");
  Serial.println(positionNumToString(desiredMotorPosition3));
}

void MoveToPosition(int motorNum, int positionNum) {
  Serial.print("Moving motor ");
  Serial.print(motorNum);
  Serial.print(" to position: ");
  Serial.print(positionNum);
  Serial.print(" (");
  Serial.print(positionNumToString(positionNum));
  Serial.println(")");

  switch (positionNum) {
  case 1:
    // Sets Input A and B for position 1
    switch (motorNum) {
    case 1:
      motor1.MotorInAState(false);
      motor1.MotorInBState(false);
      break;
    case 2:
      motor2.MotorInAState(false);
      motor2.MotorInBState(false);
      break;
    case 3:
      motor3.MotorInAState(false);
      motor3.MotorInBState(false);
      break;
    }
    break;
  case 2:
    // Sets Input A and B for position 2
    switch (motorNum) {
    case 1:
      motor1.MotorInAState(true);
      motor1.MotorInBState(false);
      break;
    case 2:
      motor2.MotorInAState(true);
      motor2.MotorInBState(false);
      break;
    case 3:
      motor3.MotorInAState(true);
      motor3.MotorInBState(false);
      break;
    }
    break;
  case 3:
    // Sets Input A and B for position 3
    switch (motorNum) {
    case 1:
      motor1.MotorInAState(false);
      motor1.MotorInBState(true);
      break;
    case 2:
      motor2.MotorInAState(false);
      motor2.MotorInBState(true);
      break;
    case 3:
      motor3.MotorInAState(false);
      motor3.MotorInBState(true);
      break;
    }
    break;
  case 4:
    // Sets Input A and B for position 4
    switch (motorNum) {
    case 1:
      motor1.MotorInAState(true);
      motor1.MotorInBState(true);
      break;
    case 2:
      motor2.MotorInAState(true);
      motor2.MotorInBState(true);
      break;
    case 3:
      motor3.MotorInAState(true);
      motor3.MotorInBState(true);
      break;
    }
    break;
  }
  // Ensures this delay is at least 2ms longer than the Input A, B filter
  // setting in MSP
  delay(2 + INPUT_A_B_FILTER);
}

constexpr unsigned long updateIntervalMilliseconds = 10e3;
auto lastUpdateTime = Milliseconds() - updateIntervalMilliseconds;

constexpr unsigned long dhcpIntervalMilliseconds = 1000e3;
auto lastDhcpTime = Milliseconds() - dhcpIntervalMilliseconds;

/**
 * Check if a certain amount of time has past since some previously saved time
 *
 * @param last The last time, as returned by Milliseconds()
 * @param time The delta time, in microseconds
 */
bool haveMillisecondsPassed(unsigned long last, unsigned long time) {
  return (Milliseconds() - lastUpdateTime) >= time;
}

/**
 * Have we acquired a DHCP lease?
 */
bool hasLease = false;

/**
 * Function called by UDP datagram handler. Do stuff with incoming packet.
 */
void handleIncomingPacket(IncomingPacket packet) {
  Serial.println("Parsed message:");
  Serial.print("Motor 1 Pos: ");
  Serial.println(packet.MotorOnePos);
  Serial.print("Motor 2 Pos: ");
  Serial.println(packet.MotorTwoPos);
  Serial.print("Motor 3 Pos: ");
  Serial.println(packet.MotorThreePos);

  // TODO: Do something more with these, like give the motors a new command
}

void ethernetLoop() {
  const auto update = haveMillisecondsPassed(lastUpdateTime, updateIntervalMilliseconds);
  if (update) {
    lastUpdateTime += updateIntervalMilliseconds;
  }

  if (!EthernetMgr.PhyLinkActive()) {
    if (update) {
      Serial.println("Ethernet cable not plugged in");
    }
    return;
  }

  // Update DHCP if:
  // - We don't have an address yes and it's been a short while
  // - It's been a longer while
  if ((!hasLease && update) || haveMillisecondsPassed(lastDhcpTime, dhcpIntervalMilliseconds)) {
    lastDhcpTime = Milliseconds();
    // Use DHCP to configure the local IP address. This blocks for up to 1.5sec
    hasLease = EthernetMgr.DhcpBegin();
    if (hasLease) {
      Serial.print("DHCP successfully assigned an IP address: ");
      Serial.println(EthernetMgr.LocalIp().StringValue());
    } else {
      Serial.println("DHCP configuration was unsuccessful!");
      Serial.println("Continuing without DHCP...");
    }
  }

  const auto packetSize = Udp.PacketParse();
  if (packetSize > 0) {
    Serial.print("Received packet of size ");
    Serial.print(packetSize);
    Serial.println(" bytes.");
    Serial.print("Remote IP: ");
    Serial.println(Udp.RemoteIp().StringValue());
    Serial.print("Remote port: ");
    Serial.println(Udp.RemotePort());

    // Read the packet.
    int32_t bytesRead = Udp.PacketRead(incomingPacketBuffer.raw, sizeof(incomingPacketBuffer));
    Serial.print("Number of bytes read from packet: ");
    Serial.println(bytesRead);
    Serial.print("Packet contents:");
    for (int i = 0; i < bytesRead; i++) {
      Serial.print(" ");
      Serial.print(incomingPacketBuffer.raw[i], HEX);
    }
    Serial.println();

    if (bytesRead != sizeof(incomingPacketBuffer)) {
      Serial.print("Received an unexpected number of bytes in UDP datagram. Expected: ");
      Serial.println(sizeof(incomingPacketBuffer));
    } else {
      handleIncomingPacket(incomingPacketBuffer.packet);
    }

    Serial.println("Sending response...");

    // Connect back to whoever sent us something
    Udp.Connect(Udp.RemoteIp(), Udp.RemotePort());

    // TODO: See if we need to make an Atomic copy of statusPacketBuffer (disabled interrupts)
    const auto copy = statusPacketBuffer;

    Serial.print("Response contents: ");
    for (int i = 0; i < bytesRead; i++) {
      Serial.print(" ");
      Serial.print(copy.raw[i], HEX);
    }
    Serial.println();

    Udp.PacketWrite(statusPacketBuffer.raw, sizeof(statusPacketBuffer.raw));
    Udp.PacketSend();
  }
}

/**
 * Main loop to update saved status of motors and temperature
 */
void updateStatusLoop() {
  // TODO: Real numbers for these
  statusPacketBuffer.packet.Temperature = 123;
  statusPacketBuffer.packet.MotorOneStatus = 45;
  statusPacketBuffer.packet.MotorTwoStatus = 67;
  statusPacketBuffer.packet.MotorThreeStatus = 89;
}

void loop() {
  // readInputsAndSetDesiredPositions();

  // Issue position command to all motors
  // MoveToPosition(1,desiredMotorPosition1);
  // MoveToPosition(2,desiredMotorPosition2);
  // MoveToPosition(3,desiredMotorPosition3);

  ethernetLoop();

  updateStatusLoop();

  MoveToPosition(1, 1);
  delay(interMovementDelay);
  MoveToPosition(2, 2);
  delay(interMovementDelay);
  MoveToPosition(3, 3);
  delay(interMovementDelay);
  MoveToPosition(1, 3);
  delay(interMovementDelay);
  MoveToPosition(2, 1);
  delay(interMovementDelay);
  MoveToPosition(3, 2);
  delay(interMovementDelay);
  MoveToPosition(1, 2);
  delay(interMovementDelay);
  MoveToPosition(2, 3);
  delay(interMovementDelay);
  MoveToPosition(3, 1);
  delay(interMovementDelay);

  MoveToPosition(1, 1);
  MoveToPosition(2, 1);
  MoveToPosition(3, 1);
  delay(180000);
  MoveToPosition(1, 2);
  MoveToPosition(2, 2);
  MoveToPosition(3, 2);
  delay(180000);
}
