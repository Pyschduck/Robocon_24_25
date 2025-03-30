#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <HardwareSerial.h>

Servo esc;

#define ESC_PIN 41

#define I2C_SDA 8
#define I2C_SCL 9
#define SLAVE_ADDR 0x08

#define UART_PORT_1_TX 16
#define UART_PORT_2_TX 40

#define DEBUG 0

#define SERIAL1 Serial1
#define SERIAL2 Serial2

typedef enum {
  UART_SIG_CW = 0,
  UART_SIG_CCW = 1,
  UART_SIG_DIR_MAX,
} uart_ctrl_dir_t;

typedef enum {
  UART_SIG_LCHAN = 0,
  UART_SIG_RCHAN = 1,
  UART_SIG_CHAN_MAX,
} uart_ctrl_chan_t;

typedef enum {
  SERIAL_PORT_1 = 0,
  SERIAL_PORT_2 = 1,
  SERIAL_PORT_MAX,
} uart_custom_port_t;

void __uartWrite(uart_custom_port_t port, uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  uint8_t udata = 0;

  if (port >= SERIAL_PORT_MAX || chan >= UART_SIG_CHAN_MAX || dir >= UART_SIG_DIR_MAX) {
    return;
  }

  speed &= 0x3F;
  udata = (chan << 7) | (dir << 6) | speed;

  if (port == SERIAL_PORT_1) {
    SERIAL1.write(udata);
  } else {
    SERIAL2.write(udata);
  }
}

void uart1Write(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  __uartWrite(SERIAL_PORT_1, chan, dir, speed);
}

void uart2Write(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  __uartWrite(SERIAL_PORT_2, chan, dir, speed);
}

struct PS4Data {
  bool Square;
  bool Cross;
  bool Circle;
  bool Triangle;
  bool touchpad;
  bool Up;
  bool Down;
  bool Left;
  bool Right;
  bool L1;
  bool R1;
  int LStickX;
  int LStickY;
  int RStickX;
  int RStickY;
  int L2;
  int R2;
};

PS4Data ps4Data;

// Global variables for motor control
bool runMotors = false;
int bldcThrottle = 1000;

void TaskMotorControl(void *pvParameters);
void TaskBLDCControl(void *pvParameters);

void receiveEvent(int bytes) {
  if (bytes == sizeof(PS4Data)) {
    Wire.readBytes((uint8_t*)&ps4Data, sizeof(ps4Data));

    if (ps4Data.L1) {
      runMotors = true;
      turnleft();
      Serial.println("L");
    } else if (ps4Data.R1) {
      runMotors = true;
      turnright();
      Serial.println("R");
    } else if (ps4Data.Triangle) {
      runMotors = true;
      forwards();
      Serial.println("F");
    } else {
      runMotors = false;
      stop();
      Serial.println("S");
    }

    if (ps4Data.R2) {
      bldcThrottle = map(ps4Data.R2, 0, 255, 1000, 2000);
    } else {
      bldcThrottle = 1000;
    }
  }
}

void setup() {
  SERIAL1.begin(115200, SERIAL_8N1, -1, UART_PORT_1_TX);
  SERIAL1.flush();
  SERIAL2.begin(115200, SERIAL_8N1, -1, UART_PORT_2_TX);
  SERIAL2.flush();
  Serial.begin(115200);
  
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

  xTaskCreate(TaskMotorControl, "TaskMotorControl", 1000, NULL, 1, NULL);
  xTaskCreate(TaskBLDCControl, "TaskBLDCControl", 1000, NULL, 1, NULL);
}

void loop() {
  // No need for loop since tasks run independently
}

void TaskMotorControl(void *pvParameters) {
  while (1) {
    if (!runMotors) {
      stop();
    }
    // vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskBLDCControl(void *pvParameters) {
  while (1) {
    int currentThrottle = esc.readMicroseconds();

    if (bldcThrottle >= 1000 && bldcThrottle <= 2000) {
      if (bldcThrottle > currentThrottle) {
        for (int i = currentThrottle; i <= bldcThrottle; i++) {
          esc.writeMicroseconds(i);
          vTaskDelay(1 / portTICK_PERIOD_MS);
        }
      } else if (bldcThrottle < currentThrottle) {
        for (int i = currentThrottle; i >= bldcThrottle; i--) {
          esc.writeMicroseconds(i);
          vTaskDelay(1 / portTICK_PERIOD_MS);
        }
      }
    }
    // vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void forwards() {
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, 60);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, 60);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 60);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 60);
}

void backwards() {
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 60);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 60);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, 60);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 60);
}

void turnright() {
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 60);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 60);
}

void turnleft() {
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 60);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, 60);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 0);
}

void stop() {
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
}
