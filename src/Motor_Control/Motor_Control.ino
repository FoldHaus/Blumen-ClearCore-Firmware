#include "ClearCore.h"
#include "EthernetManager.h"
#include "EthernetUdp.h"
#include "IncomingPacket.h"
#include "Timestamp.h"

bool msgtooMuchTimeFail = false;
bool msgMotorOverride[4] = {false, false, false, false};
bool msgMotorHLFB[4] = {false, false, false, false};
Timestamp firstMotorError[4] = {0, 0, 0, 0};
Timestamp TimeofLastClose[4] = {0, 0, 0, 0};
Timestamp TimeofLastWake[4] = {0, 0, 0, 0};
bool MotorSleeping[4] = {false, false, false, false};


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

// The local port to listen for connections on
constexpr uint16_t localPort = 37373;

// IANA specification of registered port range
constexpr bool isRegisteredPort(uint16_t port) {
  return port >= 0x400 && port < 0xC000;
}

// Code checks itself
static_assert(isRegisteredPort(localPort), "Chosen port is in IANA Registered Port range");

typedef union {
  IncomingPacket packet;
  unsigned char raw[sizeof(IncomingPacket)];
} IncomingPacketSerializer;

// Buffer for holding received packets.
IncomingPacketSerializer incomingPacketBuffer;

// motor status legend:  0=ethernetcommanded, 1=NoSignalInTooLong, 2=ButtonCommanded, 3=Error
struct {
  u8 PacketType;
  u8 MotorOnePosition;
  u8 MotorTwoPosition;
  u8 MotorThreePosition;
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

constexpr bool useStaticIP = false;
constexpr bool setStaticIPBeforeDHCP = false;

// Flag that says we should print status
unsigned int printDebug = 0;

/**
   Run at the start of main loop to possibly enable debugging for this run through
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

IpAddress staticIp(192, 168, 37, 2);

// time in milliseconds that the ethernet command has to be "fresher" than
const Timestamp failsafeCommandTime = 30_minutes;
const Timestamp motorResetOnFailureTime = 10_minutes;
const Timestamp timeBeforeClosedFlowerSleeps = 3_minutes;
const Timestamp timeToWake = 30_seconds;
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

//  Serial1.begin(baudRate); 
//  while (!Serial1 ) {
//    Serial.println("no serial 1");
//    continue;
//  }
//  Serial.println("SERIAL 1 SET UP DONE");

  // Enables the motors; homing will begin automatically
  motor1.EnableRequest(true);
  Serial.println("Motor 1 Enabled... homing");
  motor2.EnableRequest(true);
  Serial.println("Motor 2 Enabled... homing");
  motor3.EnableRequest(true);
  Serial.println("Motor 3 Enabled... homing");

  // Initializes new sleep counter
  TimeofLastClose[1] = Milliseconds();
  TimeofLastClose[2] = Milliseconds();
  TimeofLastClose[3] = Milliseconds();
  TimeofLastWake[1] = Milliseconds();
  TimeofLastWake[2] = Milliseconds();
  TimeofLastWake[3] = Milliseconds();

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

void resetMotor(int motorNumToReset) {
  switch (motorNumToReset) {
  case 1:
    motor1.EnableRequest(false);
    Serial.println("Resetting motor 1");
    delay(2000);
    motor1.EnableRequest(true);
    break;
  case 2:
    motor2.EnableRequest(false);
    Serial.println("Resetting motor 2");
    delay(2000);
    motor2.EnableRequest(true);
    break;
  case 3:
    motor3.EnableRequest(false);
    Serial.println("Resetting motor 3");
    delay(2000);
    motor3.EnableRequest(true);
    break;
  }
}

void wakeMotor(int motorNumToWake) {
  switch (motorNumToWake) {
  case 1:
    Serial.println("Wakeing motor 1");
    motor1.EnableRequest(true);
    TimeofLastWake[1] = Milliseconds();
    delay(9000);
    break;
  case 2:
    Serial.println("Wakeing motor 2");
    motor2.EnableRequest(true);
    TimeofLastWake[2] = Milliseconds();
    delay(9000);
    break;
  case 3:
    Serial.println("Wakeing motor 3");
    motor3.EnableRequest(true);
    TimeofLastWake[3] = Milliseconds();
    delay(9000);
    break;
  }
}

void sleepMotor(int motorNumToSleep) {
  switch (motorNumToSleep) {
  case 1:
    Serial.println("Putting to sleep motor 1");
    motor1.EnableRequest(false);
    statusPacketBuffer.packet.MotorOneStatus = 4;
    delay(1000);
    break;
  case 2:
    Serial.println("Putting to sleep motor 2");
    motor2.EnableRequest(false);
    statusPacketBuffer.packet.MotorTwoStatus = 4;
    delay(1000);
    break;
  case 3:
    Serial.println("Putting to sleep motor 3");
    motor3.EnableRequest(false);
    statusPacketBuffer.packet.MotorThreeStatus = 4;
    delay(1000);
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
      // set statuses to button
      switch (motorNum) {
      case 1:
        statusPacketBuffer.packet.MotorOneStatus = 2;
      case 2:
        statusPacketBuffer.packet.MotorTwoStatus = 2;
      case 3:
        statusPacketBuffer.packet.MotorThreeStatus = 2;
      }
      if (!msgMotorOverride[motorNum]) {
        Serial.print("MANUAL OVERRIDE Motor 1 desired position: ");
        Serial.println(positionNumToString(desiredMotorPosition[motorNum]));
      }
      msgMotorOverride[motorNum] = true;
    } else if (inputStatusMotorOpen[motorNum]) {
      desiredMotorPosition[motorNum] = 2;
      // set statuses to button
      switch (motorNum) {
      case 1:
        statusPacketBuffer.packet.MotorOneStatus = 2;
      case 2:
        statusPacketBuffer.packet.MotorTwoStatus = 2;
      case 3:
        statusPacketBuffer.packet.MotorThreeStatus = 2;
      }
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

    // if it's a new position commanded AND that position is CLOSE
    if (positionNum == 1) {
      TimeofLastClose[motorNum] = Milliseconds();
    } else { // if it's a new position and that position is NOT close then you can't sleep
      MotorSleeping[motorNum] = false;
      wakeMotor(motorNum);
    }

    Serial.print("Moving motor ");
    Serial.print(motorNum);
    Serial.print(" to position: ");
    Serial.print(positionNum);
    Serial.print(" (");
    Serial.print(positionNumToString(positionNum));
    Serial.println(")");
    previousDesiredMotorPosition[motorNum] = positionNum;

  } else { // if this is not the first loop with this motor position, IE it was commanded here in the last loop
    if (positionNum == 1) { // and that position is "closed"
      if (haveMillisecondsPassed(TimeofLastClose[motorNum],
                                 timeBeforeClosedFlowerSleeps)) { // and it's been at least timeBeforeClosedFlowerSleeps
                                                                  // time since it was commanded closed
        MotorSleeping[motorNum] = true;
        // disable said motor! (put to sleep)
        sleepMotor(motorNum);
      }
    }
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

/**
   Return number of milliseconds since some Time in the past.

   Handles overflow because of return type of this function.
   So, don't do this manually.

   @param last a value previously returned by Milliseconds()
*/
inline Timestamp timeSince(Timestamp last) {
  return Milliseconds() - last;
}

/**
   Check if a certain amount of time has past since some previously saved time

   @param last The last time, as returned by Milliseconds()
   @param time The delta time, in microseconds
*/
inline bool haveMillisecondsPassed(Timestamp last, Timestamp time) {
  return timeSince(last) >= time;
}

/**
   Have we acquired a DHCP lease?
*/
bool hasLease = false;

const static u8 JANK_HANDSHAKE_FIX = 7;

// The timestamp from when we last received a command
Timestamp lastUpdateTime = 0;

/**
   Function called by UDP datagram handler. Do stuff with incoming packet.
*/
void handleIncomingPacket(IncomingPacket packet) {

  if ((packet.MotorOnePos != 1) && (packet.MotorOnePos != 2)) {
    desiredMotorPosition[1] = 1;
    Serial.println("ETHERNET Motor 1 desired position invalid. Closing.");
  } else {
    desiredMotorPosition[1] = packet.MotorOnePos;
  }

  if ((packet.MotorTwoPos != 1) && (packet.MotorTwoPos != 2)) {
    desiredMotorPosition[2] = 1;
    Serial.println("ETHERNET Motor 2 desired position invalid. Closing.");
  } else {
    desiredMotorPosition[2] = packet.MotorOnePos;
  }

  if ((packet.MotorThreePos != 1) && (packet.MotorThreePos != 2)) {
    desiredMotorPosition[3] = 1;
    Serial.println("ETHERNET Motor 3 desired position invalid. Closing.");
  } else {
    desiredMotorPosition[3] = packet.MotorThreePos;
  }

  lastUpdateTime = Milliseconds();
}

constexpr Timestamp updateIntervalMilliseconds = 10_seconds;
constexpr Timestamp dhcpIntervalMilliseconds = 1000_seconds;

/**
   Manage ethernet status
*/
void sendPacket() {
    Serial.println("Sending response...");

    // Connect back to whoever sent us something
    Udp.Connect(Udp.RemoteIp(), Udp.RemotePort());
    Udp.PacketWrite(statusPacketBuffer.raw, sizeof(statusPacketBuffer.raw));
    Udp.PacketSend();
}

void ethernetLoop() {
  // Last time that we've checked for updated ethernet status
  static Timestamp lastUpdateTime = timeSince(updateIntervalMilliseconds);
  // Last time that we've checked for DHCP address
  static Timestamp lastDhcpTime = timeSince(dhcpIntervalMilliseconds);

  // Keep the connection alive.
  EthernetMgr.Refresh();

  debug("Seconds since last Ethernet status check: ");
  debugln(((signed int)(timeSince(lastUpdateTime))) / 1_seconds);

  // Should we update status of ethernet
  const bool update = haveMillisecondsPassed(lastUpdateTime, updateIntervalMilliseconds) || printDebug;

  // Write down last local check time
  if (update) {
    lastUpdateTime = Milliseconds();
    sendPacket();
  }

  if (!EthernetMgr.PhyLinkActive()) {
    if (update) {
      Serial.println("Ethernet cable not plugged in");
    }
    return;
  }

  const float minutesSinceDHCPUpdate = ((signed long)(timeSince(lastDhcpTime))) / 1_minutes;
  debug("Minutes since last Ethernet status check: ");
  debugln(minutesSinceDHCPUpdate);

  // Update DHCP if:
  // - We don't have an address yes and it's been a short while
  // - It's been a longer while
  if (!useStaticIP && ((!hasLease && update) || haveMillisecondsPassed(lastDhcpTime, dhcpIntervalMilliseconds))) {
    lastDhcpTime = Milliseconds();

    const auto mac = EthernetMgr.MacAddress();

    // Print MAC Address
    Serial.print("MAC Address:");
    for (int i = 0; i < 6; i++) {
      Serial.print(i ? ":" : " ");
      Serial.print(mac[i], HEX);
    }
    Serial.println();

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


    // TODO: See if we need to make an Atomic copy of statusPacketBuffer (disabled interrupts)
    const auto copy = statusPacketBuffer;

    Serial.print("Response contents: ");
    for (int i = 0; i < bytesRead; i++) {
      Serial.print(" ");
      Serial.print(copy.raw[i], HEX);
    }
    Serial.println();
    sendPacket();
  }
}

/**
   Main loop to update saved status of motors and temperature
*/
void updateStatusLoop() {
  // TODO: Real numbers for these
  statusPacketBuffer.packet.PacketType = 2; // what is this?
  statusPacketBuffer.packet.Temperature = 11;
  statusPacketBuffer.packet.MotorOnePosition = desiredMotorPosition[1];
  statusPacketBuffer.packet.MotorTwoPosition = desiredMotorPosition[2];
  statusPacketBuffer.packet.MotorThreePosition = desiredMotorPosition[3];
}

void loop() {

//  // Read the input.
//  int input = Serial1.read();
//  Serial.println(input);

//  // If there was a valid byte read-in, print it.
//  if (input != -1) {
//      // Display the input character received.
//      Serial.print("Received: ");
//      Serial.println((input);
//  }
//  else {
//     Serial.println("No data received...");
//  }
//  delay(1000);

  
  handleDebug();

  // reads packet through ethernet and sets desiredmotorpositions via 'handleIncomingPacket' function.
  ethernetLoop();
  // set statuses to ethernet commanded
  statusPacketBuffer.packet.MotorOneStatus = 0;
  statusPacketBuffer.packet.MotorTwoStatus = 0;
  statusPacketBuffer.packet.MotorThreeStatus = 0;

  // Print some debug info (when requested)
  if (printDebug) {
    Serial.print("Seconds since successful update from host: ");
    Serial.println(((float)timeSince(lastUpdateTime)) / 1_seconds, 1);
  }

  // if more time has passed than allowed since last ethernet-commanded time then CLOSE
  const auto tooMuchTimeFail = haveMillisecondsPassed(lastUpdateTime, failsafeCommandTime);

  if (tooMuchTimeFail) {
    if (!msgtooMuchTimeFail) {
      Serial.print("Last ethernet command was ");
      Serial.print((float)timeSince(timeOfLastNetworkPositionCommand) / 1_minutes, 1);
      Serial.println(" minutes ago. REQUESTING CLOSING ALL FLOWERS.");
      msgtooMuchTimeFail = true;
    }
    desiredMotorPosition[1] = 1;
    desiredMotorPosition[2] = 1;
    desiredMotorPosition[3] = 1;
    // set statuses to no signal in too long
    statusPacketBuffer.packet.MotorOneStatus = 1;
    statusPacketBuffer.packet.MotorTwoStatus = 1;
    statusPacketBuffer.packet.MotorThreeStatus = 1;
  } else {
    msgtooMuchTimeFail = false;
  }

  // deliberately overwrites desired positions with any manually commanded positions if toggled
  readInputsAndSetDesiredPositions();

  // moves motors to desired positions if HLFB asserts (waits for homing to complete if applicable) IF THE MOTOR ISNT
  // SLEEPING
  if ((motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) && (MotorSleeping[1] == false) &&
      haveMillisecondsPassed(TimeofLastWake[1], timeToWake)) {
    // on first time of new error, mark time
    if (firstMotorError[1] == 0) {
      firstMotorError[1] = Milliseconds();
    } else if (haveMillisecondsPassed(firstMotorError[1], motorResetOnFailureTime)) {
      firstMotorError[1] = 0;
      resetMotor(1);
    }
    if (!msgMotorHLFB[1]) {
      Serial.println("Waiting for motor 1 HLFB...");
      msgMotorHLFB[1] = true;
      // set statuses to Error
      statusPacketBuffer.packet.MotorOneStatus = 3;
    }
  } else {
    MoveToPosition(1, desiredMotorPosition[1]);
    firstMotorError[1] = 0;
    msgMotorHLFB[1] = false;
  }
  if ((motor2.HlfbState() != MotorDriver::HLFB_ASSERTED) && (MotorSleeping[2] == false) &&
      haveMillisecondsPassed(TimeofLastWake[2], timeToWake)) {
    // on first time of new error, mark time
    if (firstMotorError[2] == 0) {
      firstMotorError[2] = Milliseconds();
    } else if (haveMillisecondsPassed(firstMotorError[2], motorResetOnFailureTime)) {
      firstMotorError[2] = 0;
      resetMotor(2);
    }
    if (!msgMotorHLFB[2]) {
      Serial.println("Waiting for motor 2 HLFB...");
      msgMotorHLFB[2] = true;
      // set statuses to Error
      statusPacketBuffer.packet.MotorTwoStatus = 3;
    }
  } else {
    MoveToPosition(2, desiredMotorPosition[2]);
    firstMotorError[2] = 0;
    msgMotorHLFB[2] = false;
  }
  if ((motor3.HlfbState() != MotorDriver::HLFB_ASSERTED) && (MotorSleeping[3] == false) &&
      haveMillisecondsPassed(TimeofLastWake[3], timeToWake)) {
    if (firstMotorError[3] == 0) {
      firstMotorError[3] = Milliseconds();
    } else if (haveMillisecondsPassed(firstMotorError[3], motorResetOnFailureTime)) {
      firstMotorError[3] = 0;
      resetMotor(3);
    }
    if (!msgMotorHLFB[3]) {
      Serial.println("Waiting for motor 3 HLFB...");
      msgMotorHLFB[3] = true;
      // set statuses to Error
      statusPacketBuffer.packet.MotorThreeStatus = 3;
    }
  } else {
    MoveToPosition(3, desiredMotorPosition[3]);
    firstMotorError[3] = 0;
    msgMotorHLFB[3] = false;
  }

  // Prepare status for sending back to host
  updateStatusLoop();
}
