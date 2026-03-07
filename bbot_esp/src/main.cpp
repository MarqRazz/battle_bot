#include <Arduino.h>
#include <WiFi.h>

#include <math.h>  // For fmod() and M_PI
#include <stdint.h>
#include <stdio.h>
#include "picoros.h"
#include "picoserdes.h"
#include "sc_servo_manager.h"

// WiFi-specific parameters
#define SSID "TODO"
#define PASS "super_secret"

// Zenoh-specific parameters
#define MODE "client"
// change this to match your ROS 2 host router's ip address
#define ROUTER_ADDRESS "tcp/192.168.9.138:7447"

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED 23

// SCServo speed units per rad/s. Derived from the STS3215 datasheet:
// 50 RPM ≈ 5.24 rad/s at speed value 1180, so 1180/5.24 ≈ 225.
static const int16_t VELOCITY_TO_SERVO_SCALE = 225;

// Servo IDs and direction multipliers
// The Phase 1 URDF uses rpy="-pi/2 0 0" + axis="0 0 1" which flips the
// positive-rotation convention vs the original TurtleBot3 URDF. Left motor
// (ID 1) needs negation; right motor (ID 2) does not.
uint8_t ids[] = {1, 2};
int8_t dirs[] = {-1, 1};
SCServoManager motors(2, ids, dirs);
SemaphoreHandle_t servoMutex;

/* ---------- LED Functions ----------- */
unsigned long previousBlinkMillis = 0;
bool ledState = false;
void nonBlockingBlink(int r, int g, int b, unsigned long interval)
{
  if (millis() - previousBlinkMillis >= interval)
  {
    previousBlinkMillis = millis();
    ledState = !ledState;
    if (ledState)
    {
      neopixelWrite(RGB_LED, r, g, b);
    }
    else
    {
      neopixelWrite(RGB_LED, 0, 0, 0);
    }
  }
}

void blockingBlinkRGB(int r, int g, int b, int sleep_ms)
{
  neopixelWrite(RGB_LED, r, g, b);
  delay(sleep_ms / 2);
  neopixelWrite(RGB_LED, 0, 0, 0);
  delay(sleep_ms / 2);
}

// Subscriber callback
void joint_state_callback(uint8_t * rx_data, size_t data_len);

/* -------------------------------------- */
// Example Publisher
picoros_publisher_t pub_js = {
  .topic =
    {
      .name = "picoros/joint_states",
      .type = ROSTYPE_NAME(ros_JointState),
      .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

picoros_subscriber_t sub_js = {
  .topic =
    {
      .name = "picoros/joint_commands",
      .type = ROSTYPE_NAME(ros_JointState),
      .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
  .user_callback = joint_state_callback,
};

// Example node
picoros_node_t node = {
  .name = "bbot_picoros_node",
};

static const int num_joints = 3;
// the latest velocity command from the subscriber
volatile double cmd_vel[num_joints];
// Buffer for publication, used from this thread
uint8_t pub_buf[1024];

void joint_state_callback(uint8_t * rx_data, size_t data_len)
{
  // Define arrays to hold the received data
  char * joint_names[num_joints] = {};
  double joint_positions[num_joints] = {};
  double joint_velocities[num_joints] = {};
  double joint_efforts[num_joints] = {};

  // Create the JointState message and initialize it with the desired array sizes
  // ros2_control HW interface is only sending each joint a velocity command because
  // this example is paired with a diff drive controller.
  ros_JointState js = {
    .name = {.data = joint_names, .n_elements = num_joints},
    .position = {.data = joint_positions, .n_elements = 0},
    .velocity = {.data = joint_velocities, .n_elements = num_joints},
    .effort = {.data = joint_efforts, .n_elements = 0},
  };
  if (ps_deserialize(rx_data, &js, data_len))
  {
    // take the servo semaphore and update each motors commanded velocity
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10)))
    {
      for (auto i = 0; i < num_joints; i++)
      {
        cmd_vel[i] = js.velocity.data[i];
      }
      xSemaphoreGive(servoMutex);
    }
  }
  else
  {
    Serial.printf("JointState message deserialization error\n");
    return;
  }
}

void publish_joint_state()
{
  static const double efforts[] = {0, 0, 0};
  static const char * const names[] = {
    "bbot_wheel_left_joint", "bbot_wheel_right_joint", "bbot_weapon_joint"};

  double positions[num_joints] = {};
  double velocities[num_joints] = {};
  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(5)))
  {
    // Direction multiplier is applied inside getPosition()
    positions[0] = motors.getPosition(0);
    positions[1] = motors.getPosition(1);
    velocities[0] = cmd_vel[0];
    velocities[1] = cmd_vel[1];
    velocities[2] = cmd_vel[2];
    xSemaphoreGive(servoMutex);
  }
  else
  {
    return;
  }

  z_clock_t now = z_clock_now();
  ros_JointState joint_state = {
    .header =
      {
        .stamp =
          {
            .sec = (int32_t)now.tv_sec,
            .nanosec = (uint32_t)now.tv_nsec,
          },
      },
    .name = {.data = (char **)names, .n_elements = 3},
    .position = {.data = positions, .n_elements = 3},
    .velocity = {.data = velocities, .n_elements = 3},
    .effort = {.data = (double *)efforts, .n_elements = 3},
  };
  size_t len = ps_serialize(pub_buf, &joint_state, 1024);
  if (len > 0)
  {
    picoros_publish(&pub_js, pub_buf, len);
  }
  else
  {
    Serial.printf("JointState message serialization error.");
  }
}

// Servo control task pinned to Core 0
void servoTask(void * pvParameters)
{
  int16_t localSpeeds[2];

  while (true)
  {
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10)))
    {
      // Direction multiplier is applied inside syncWriteVelocity()
      localSpeeds[0] = cmd_vel[0] * VELOCITY_TO_SERVO_SCALE;
      localSpeeds[1] = cmd_vel[1] * VELOCITY_TO_SERVO_SCALE;
      xSemaphoreGive(servoMutex);
    }

    motors.syncWriteVelocity(localSpeeds);
    vTaskDelay(pdMS_TO_TICKS(1));
    motors.updateState();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup(void)
{
  pinMode(RGB_LED, OUTPUT);
  neopixelWrite(RGB_LED, 0, 0, 0);
  // Initialize Serial for debug
  Serial.begin(115200);
  do
  {  // blink the LED Orange to signal start of the program
    blockingBlinkRGB(200, 165, 0, 1000);
  } while (!Serial);

  Serial.printf("Connecting to WiFi:[%s]!\n", SSID);
  // Set WiFi in STA mode and trigger attachment
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED)
  {  // blink the LED Blue to signal we are connecting to WiFi
    blockingBlinkRGB(0, 0, 255, 500);
  }
  Serial.printf("Connected to WiFi:[%s] with address:[%s]\n", SSID, WiFi.localIP().toString());
  // Hold Blue solid indicating success
  neopixelWrite(RGB_LED, 0, 0, 255);
  delay(2000);
  neopixelWrite(RGB_LED, 0, 0, 0);

  // Initialize Pico ROS interface
  picoros_interface_t ifx = {
    .mode = MODE,
    .locator = ROUTER_ADDRESS,
  };

  Serial.printf("Starting pico-ros interface:[%s] on router address:[%s]\n", ifx.mode, ifx.locator);
  while (picoros_interface_init(&ifx) == PICOROS_NOT_READY)
  {
    printf("Waiting RMW init...\n");
    // Blink LED Red to signal we are waiting for RMW router
    blockingBlinkRGB(255, 0, 0, 1000);
    z_sleep_ms(1);
  }

  Serial.printf("Starting Pico-ROS node:[%s] domain:[%d]\n", node.name, node.domain_id);
  picoros_node_init(&node);

  Serial.printf("Declaring publisher on [%s]\n", pub_js.topic.name);
  picoros_publisher_declare(&node, &pub_js);
  Serial.printf("Declaring subscriber on [%s]\n", sub_js.topic.name);
  picoros_subscriber_declare(&node, &sub_js);

  motors.initComms();
  motors.setWorkMode(1);
  servoMutex = xSemaphoreCreateMutex();
  Serial.printf("Servos have been initialized\n");
  xTaskCreatePinnedToCore(
    servoTask,    // Task function
    "ServoTask",  // Name
    4096,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    NULL,         // Task handle
    0             // Core ID (0)
  );
}

// Variables to control Core 1's loop rate
unsigned long previousLoopMillis = 0;
const unsigned long loopInterval = 10;  // Run loop logic every 10ms (100 Hz)
void loop()
{
  // Trigger publisher and LED at desired rate
  if (millis() - previousLoopMillis >= loopInterval)
  {
    previousLoopMillis = millis();
    // Publish the current state
    publish_joint_state();

    // Blink the LED without blocking
    nonBlockingBlink(0, 100, 0, 100);  // Green blink every 100ms
  }
}
