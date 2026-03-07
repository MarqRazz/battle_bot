#include <SCServo.h>
#include <math.h>

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

class SCServoManager
{
private:
  SMS_STS st;
  uint8_t * servoIDs;
  uint8_t numServos;
  int8_t * directions;

  // Buffers for SyncRead
  int16_t * positions;
  int16_t * velocities;

  // Buffers for SyncWrite requires arrays for speed/accel
  uint16_t * speedArray;
  uint8_t * accelArray;
  uint8_t * speedBuffer;

public:
  SCServoManager(uint8_t count, uint8_t * ids, int8_t * dirs)
  {
    numServos = count;
    servoIDs = new uint8_t[numServos];
    directions = new int8_t[numServos];
    positions = new int16_t[numServos];
    velocities = new int16_t[numServos];
    speedArray = new uint16_t[numServos];
    accelArray = new uint8_t[numServos];
    speedBuffer = new uint8_t[numServos * 2];

    for (uint8_t i = 0; i < numServos; i++)
    {
      servoIDs[i] = ids[i];
      directions[i] = dirs[i];
      positions[i] = 0;
      velocities[i] = 0;
    }
  }

  ~SCServoManager()
  {
    delete[] servoIDs;
    delete[] directions;
    delete[] positions;
    delete[] velocities;
    delete[] speedArray;
    delete[] accelArray;
    delete[] speedBuffer;
  }

  void initComms()
  {
    // Increase RX buffer size to handle multiple servo responses at once
    Serial1.setRxBufferSize(1024);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    while (!Serial1)
    {
    }
  }

  // --- READ OPERATIONS ---
  bool updateState()
  {
    // Broadcast read request
    st.syncReadPacketTx(servoIDs, numServos, 0x38, 4);

    uint8_t rxBuffer[4];
    bool allRead = true;

    for (uint8_t i = 0; i < numServos; i++)
    {
      // syncReadPacketRx often blocks.
      // If the servo is missing, it can hang here and cause the WDT Reset.
      int result = st.syncReadPacketRx(servoIDs[i], rxBuffer);

      if (result == 4)
      {
        // Combine bytes using bitwise OR
        positions[i] = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
        velocities[i] = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
      }
      else
      {
        allRead = false;
      }
    }

    return allRead;
  }

  // Enables or disables torque for all servos at once
  void setTorque(bool enable)
  {
    uint8_t status = enable ? 1 : 0;
    uint8_t data[numServos];
    for (uint8_t i = 0; i < numServos; i++)
    {
      data[i] = status;
    }

    // Address 0x28: Torque Enable (1 byte)
    st.syncWrite(servoIDs, numServos, 0x28, data, 1);
  }

  // Switches all servos between Position (0) and Wheel/Velocity (1) mode
  void setWorkMode(uint8_t mode)
  {
    // Mode 0: Position, Mode 1: Wheel (Velocity)
    uint8_t modes[numServos];
    for (uint8_t i = 0; i < numServos; i++)
    {
      modes[i] = mode;
    }

    setTorque(false);  // Recommended to disable torque before mode change
    // Address 0x21: Work Mode (1 byte)
    st.syncWrite(servoIDs, numServos, 0x21, modes, 1);
    setTorque(true);
  }

  // --- WRITE OPERATIONS ---
  // Commands all servos to specific positions with a max speed / accel
  void syncWritePosition(int16_t * targetPositions, uint16_t speed = 500, uint8_t accel = 50)
  {
    // Fill the required arrays with the uniform speed/accel provided
    for (uint8_t i = 0; i < numServos; i++)
    {
      speedArray[i] = speed;
      accelArray[i] = accel;
    }
    // Arguments: IDs, Count, TargetPositions, Speeds (Array), Accels (Array)
    st.SyncWritePosEx(servoIDs, numServos, targetPositions, speedArray, accelArray);
  }

  // Commands all servos to specific velocity (Wheel Mode)
  // Direction multiplier is applied internally — caller doesn't need to negate.
  void syncWriteVelocity(int16_t * targetVelocities)
  {
    for (uint8_t i = 0; i < numServos; i++)
    {
      int16_t val = targetVelocities[i] * directions[i];
      uint16_t rawSpeed;

      if (val < 0)
      {
        // If negative: Use magnitude and set the 15th bit (0x8000) to 1
        rawSpeed = (uint16_t)(-val);
        rawSpeed |= 0x8000;
      }
      else
      {
        // If positive: Just use the magnitude (15th bit stays 0)
        rawSpeed = (uint16_t)val;
      }

      speedBuffer[i * 2] = lowByte(rawSpeed);
      speedBuffer[i * 2 + 1] = highByte(rawSpeed);
    }
    // Address 0x2E: Target Speed (2 bytes, Signed)
    st.syncWrite(servoIDs, numServos, 0x2E, speedBuffer, 2);
  }

  // Getters for local buffer — direction multiplier applied to position
  double getPosition(uint8_t index) { return directions[index] * M_PI * positions[index] / 2048; }
  int16_t getVelocity(uint8_t index) { return velocities[index]; }
};
