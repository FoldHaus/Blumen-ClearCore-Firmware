/**
 * THINGS WE NEED:
 * - will try and open serial occasionally not just in setup. So we can watch serial output later and see what's going
 * on. I think right now it only does it on program setup
 * - Are we good on ethernet/address reconnect attemps? IE can we make it so it practically never needs to be restarted?
 * you can plug and unplug ethernet and it tries to reconnect at certain intervals? Any time I hear "oh we just had to
 * restart it" that indicates a problem to me, potentially for a long-term install
 * - If a motor errors out, clearcore needs to disable, wait x minutes, re-enable, and have some behavior around that
 * sort of error condition. It does not.
 * - new code to read a thermometer.
 * - Status needs to be written back in terms of motor positions and possible errors and temps.
 */

#include "ClearCore.h"
#include "EthernetManager.h"
#include "EthernetUdp.h"

bool msgtooMuchTimeFail = false;
bool msgMotorOverride[4] = {false, false, false, false};
bool msgMotorHLFB[4] = {false, false, false, false};

// The INPUT_A_B_FILTER must match the Input A, B filter setting in MSP (Advanced >> Input A, B Filtering...)
constexpr auto INPUT_A_B_FILTER = 20;

// Baud rate for serial output
constexpr auto baudRate = 9600;

// Defines the motor's connector as ConnectorM1, M2, and M3
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
// array only needs 3 spots but initialize as 4 so that we can index positions 1, 2, 3 (arrays start at index 0)
PinStatus inputStatusMotorClose[4];
PinStatus inputStatusMotorOpen[4];

// sets desired motor positions to closes (motorposition 1 = closed, 2 = open)
// array only needs 3 spots but initialize as 4 so that we can index positions 1, 2, 3 (arrays start at index 0)

int previousDesiredMotorPosition[4] = {1, 1, 1, 1};
int desiredMotorPosition[4] = {1, 1, 1, 1};

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

constexpr bool useStaticIP = true;
constexpr bool setStaticIPBeforeDHCP = false;

// Flag that says we should print status
unsigned int printDebug = 0;

/**
 * Run at the start of main loop to possibly enable debugging for this run through
 */
void handleDebug() {
  if (printDebug) {
    printDebug--;
    if (!printDebug) {
      Serial.println("Stopping debugging");
    } else {
      Serial.print("Start of loop. Debugging ");
      Serial.print(printDebug);
      Serial.println(" times.");
    }
  }

  if (!Serial.available())
    return;

  const auto incomingChar = Serial.read();

  // Enable debug this loop if we're received a "D" via serial.
  switch (incomingChar) {
  case 'd':
    printDebug++;
    break;
  case 'D':
    printDebug += 10;
    break;
  default:
    Serial.print("Unhandled character: 0x");
    Serial.println(incomingChar, HEX);
    break;
  case '\r':
  case '\n':
    break;
  }
}

// Print some value to the serial console, iff printing of debug statements has been turned on.
template <class T> void debug(T value) {
  if (printDebug)
    Serial.print(value);
}

// Print some value to the serial console, iff printing of debug statements has been turned on.
template <class T> void debugln(T value) {
  if (printDebug)
    Serial.println(value);
}

/// Allow using human readable units

inline static constexpr const Timestamp operator"" _seconds(unsigned long long const x) { return x * 1000; }
inline static constexpr const Timestamp operator"" _seconds(long double const x) { return x * 1000; }
inline static constexpr const Timestamp operator"" _minutes(unsigned long long const x) { return x * 60_seconds; }
inline static constexpr const Timestamp operator"" _minutes(long double const x) { return x * 60_seconds; }
inline static constexpr const Timestamp operator"" _hours(unsigned long long const x) { return x * 60_minutes; }
inline static constexpr const Timestamp operator"" _hours(long double const x) { return x * 60_minutes; }

IpAddress staticIp(192, 168, 37, 2);

// time in milliseconds that the ethernet command has to be "fresher" than
const Timestamp failsafeCommandTime = 30_minutes;
Timestamp timeOfLastNetworkPositionCommand;

void setup() {
  // Sets all motor connectors to the correct mode for Absolute Position mode
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_DIRECT);

  // Enforces the state of the motor's A and B inputs before enabling the motor.
  // A and B states being false is closed position

  motor1.MotorInAState(false);
  motor1.MotorInBState(false);
  motor2.MotorInAState(false);
  motor2.MotorInBState(false);
  motor3.MotorInAState(false);
  motor3.MotorInBState(false);

  // Set up serial communication at a baud rate of 9600 bps then wait up to 3 seconds for a port to open.
  Serial.begin(baudRate);
  Timestamp timeout = 3_seconds;
  Timestamp startTime = Milliseconds();
  while (!Serial && haveMillisecondsPassed(startTime, timeout)) {
    continue;
  }

  // Enables the motors; homing will begin automatically
  motor1.EnableRequest(true);
  Serial.println("Motor 1 Enabled... homing");
  motor2.EnableRequest(true);
  Serial.println("Motor 2 Enabled... homing");
  motor3.EnableRequest(true);
  Serial.println("Motor 3 Enabled... homing");

  // Sets the 3 analog/digital inputs to input digital mode
  ConnectorA9.Mode(Connector::INPUT_DIGITAL);
  ConnectorA10.Mode(Connector::INPUT_DIGITAL);
  ConnectorA11.Mode(Connector::INPUT_DIGITAL);

  // Run the setup for the ClearCore Ethernet manager.
  EthernetMgr.Setup();
  Serial.println("Ethernet Enabled");

  if (useStaticIP || setStaticIPBeforeDHCP) {
    Serial.print("Setting IP: ");
    Serial.println(staticIp.StringValue());
    EthernetMgr.LocalIp(staticIp);
  }

  // Begin listening on the local port for UDP datagrams
  Udp.Begin(localPort);
  Serial.println("Listening for UDP packets");
} // end of setup

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
  inputStatusMotorClose[1] = digitalRead(inputMotor1Close);
  inputStatusMotorOpen[1] = digitalRead(inputMotor1Open);
  inputStatusMotorClose[2] = digitalRead(inputMotor2Close);
  inputStatusMotorOpen[2] = digitalRead(inputMotor2Open);
  inputStatusMotorClose[3] = digitalRead(inputMotor3Close);
  inputStatusMotorOpen[3] = digitalRead(inputMotor3Open);

  // takes close and open manual inputs and sets desired motor positions based on them. Position 1 is closed, position 2
  // is fully open

  for (int motorNum = 1; motorNum <= 3; motorNum++) {
    if (inputStatusMotorClose[motorNum]) {
      desiredMotorPosition[motorNum] = 1;
      if (!msgMotorOverride[motorNum]) {
        Serial.print("MANUAL OVERRIDE Motor 1 desired position: ");
        Serial.println(positionNumToString(desiredMotorPosition[motorNum]));
      }
      msgMotorOverride[motorNum] = true;
    } else if (inputStatusMotorOpen[motorNum]) {
      desiredMotorPosition[motorNum] = 2;
      if (!msgMotorOverride[motorNum]) {
        Serial.print("MANUAL OVERRIDE Motor 1 desired position: ");
        Serial.println(positionNumToString(desiredMotorPosition[motorNum]));
      }
      msgMotorOverride[motorNum] = true;
    } else {
      msgMotorOverride[motorNum] = false;
    }
  }
}

void MoveToPosition(int motorNum, int positionNum) {

  // if the previous motor position was different from the newly requested one, write to serial
  if (previousDesiredMotorPosition[motorNum] != positionNum) {
    Serial.print("Moving motor ");
    Serial.print(motorNum);
    Serial.print(" to position: ");
    Serial.print(positionNum);
    Serial.print(" (");
    Serial.print(positionNumToString(positionNum));
    Serial.println(")");
    previousDesiredMotorPosition[motorNum] = positionNum;
  }

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
  // Ensures this delay is at least 5ms longer than the Input A, B filter
  // setting in MSP
  delay(5 + INPUT_A_B_FILTER);
}

// Create a time for Time to be used on Time related operations
typedef uint32_t Timestamp;

/**
 * Return number of milliseconds since some Time in the past.
 *
 * Handles overflow because of return type of this function.
 * So, don't do this manually.
 *
 * @param last a value previously returned by Milliseconds()
 */
inline Timestamp timeSince(Timestamp last) { return Milliseconds() - last; }

/**
 * Check if a certain amount of time has past since some previously saved time
 *
 * @param last The last time, as returned by Milliseconds()
 * @param time The delta time, in microseconds
 */
inline bool haveMillisecondsPassed(Timestamp last, Timestamp time) { return timeSince(last) >= time; }

/**
 * Have we acquired a DHCP lease?
 */
bool hasLease = false;

// The timestamp from when we last received a command
Timestamp lastUpdateTime = 0;

/**
 * Function called by UDP datagram handler. Do stuff with incoming packet.
 */
void handleIncomingPacket(IncomingPacket packet) {
  Serial.println("Parsed message from ethernet:");
  Serial.print("ETHERNET Motor 1 desired position: ");
  Serial.println(positionNumToString(packet.MotorOnePos));
  Serial.print("ETHERNET Motor 2 desired position: ");
  Serial.println(positionNumToString(packet.MotorTwoPos));
  Serial.print("ETHERNET Motor 3 desired position: ");
  Serial.println(positionNumToString(packet.MotorThreePos));

  desiredMotorPosition[1] = packet.MotorOnePos;
  desiredMotorPosition[2] = packet.MotorTwoPos;
  desiredMotorPosition[3] = packet.MotorThreePos;

  lastUpdateTime = Milliseconds();
}

constexpr Timestamp updateIntervalMilliseconds = 10_seconds;
constexpr Timestamp dhcpIntervalMilliseconds = 1000_seconds;

/**
 * Manage ethernet status
 */
void ethernetLoop() {
  // Last time that we've checked for updated ethernet status
  static Timestamp lastUpdateTime = timeSince(updateIntervalMilliseconds);
  // Last time that we've checked for DHCP address
  static Timestamp lastDhcpTime = timeSince(dhcpIntervalMilliseconds);

  // Keep the connection alive.
  EthernetMgr.Refresh();

  const float secondsSinceUpdate = ((float)(signed long)(timeSince(lastUpdateTime))) / 1_seconds;
  debug("Seconds since last Ethernet status check");
  debugln(secondsSinceUpdate);

  // Should we update status of ethernet
  const bool update = haveMillisecondsPassed(lastUpdateTime, updateIntervalMilliseconds) || printDebug;

  // Write down last local check time
  if (update) {
    lastUpdateTime = Milliseconds();
  }

  if (!EthernetMgr.PhyLinkActive()) {
    if (update) {
      Serial.println("Ethernet cable not plugged in");
    }
    return;
  }

  const float minutesSinceDHCPUpdate = ((signed long)(timeSince(lastDhcpTime))) / 1_minutes;
  debug("Minutes since last Ethernet status check");
  debugln(minutesSinceDHCPUpdate);

  // Update DHCP if:
  // - We don't have an address yes and it's been a short while
  // - It's been a longer while
  if (!useStaticIP && ((!hasLease && update) || haveMillisecondsPassed(lastDhcpTime, dhcpIntervalMilliseconds))) {
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
    const unsigned int bytesRead = Udp.PacketRead(incomingPacketBuffer.raw, sizeof(incomingPacketBuffer));
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
      timeOfLastNetworkPositionCommand = Milliseconds();
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
  handleDebug();

  // reads packet through ethernet and sets desiredmotorpositions via 'handleIncomingPacket' function.
  ethernetLoop();

  // Print some debug info (when requested)
  if (printDebug) {
    Serial.print("Seconds since successful update from host: ");
    Serial.println(((float)timeSince(lastUpdateTime)) / 1_seconds);
  }

  // if more time has passed than allowed since last ethernet-commanded time then CLOSE
  const auto tooMuchTimeFail = haveMillisecondsPassed(lastUpdateTime, failsafeCommandTime);

  if (tooMuchTimeFail) {
    if (!msgtooMuchTimeFail) {
      Serial.print("Last ethernet command was ");
      Serial.print(timeSince(timeOfLastNetworkPositionCommand) / 1_minutes);
      Serial.println(" minutes ago. REQUESTING CLOSING ALL FLOWERS.");
      msgtooMuchTimeFail = true;
    }
    desiredMotorPosition[1] = 1;
    desiredMotorPosition[2] = 1;
    desiredMotorPosition[3] = 1;
  } else {
    msgtooMuchTimeFail = false;
  }

  // deliberately overwrites desired positions with any manually commanded positions if toggled
  readInputsAndSetDesiredPositions();

  // moves motors to desired positions if HLFB asserts (waits for homing to complete if applicable)
  if (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) {
    if (!msgMotorHLFB[1]) {
      Serial.println("Waiting for motor 1 HLFB...");
      msgMotorHLFB[1] = true;
    }
  } else {
    MoveToPosition(1, desiredMotorPosition[1]);
    msgMotorHLFB[1] = false;
  }
  if (motor2.HlfbState() != MotorDriver::HLFB_ASSERTED) {
    if (!msgMotorHLFB[2]) {
      Serial.println("Waiting for motor 2 HLFB...");
      msgMotorHLFB[2] = true;
    }
  } else {
    MoveToPosition(2, desiredMotorPosition[2]);
    msgMotorHLFB[2] = false;
  }
  if (motor3.HlfbState() != MotorDriver::HLFB_ASSERTED) {
    if (!msgMotorHLFB[3]) {
      Serial.println("Waiting for motor 3 HLFB...");
      msgMotorHLFB[3] = true;
    }
  } else {
    MoveToPosition(3, desiredMotorPosition[3]);
    msgMotorHLFB[3] = false;
  }

  // Prepare status for sending back to host
  updateStatusLoop();
}
