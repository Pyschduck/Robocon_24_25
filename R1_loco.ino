#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


// I2C Pins and Slave Address
#define I2C_SDA 8
#define I2C_SCL 9
#define SLAVE_ADDR 0x05

// UART TX Pins
#define UART_PORT_1_TX 41
#define UART_PORT_2_TX 40

// Serial Ports
#define SERIAL1 Serial1
#define SERIAL2 Serial2

// Enums for motor control
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

// Low-level UART write
void __uartWrite(uart_custom_port_t port, uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  if (port >= SERIAL_PORT_MAX || chan >= UART_SIG_CHAN_MAX || dir >= UART_SIG_DIR_MAX) return;

  speed &= 0x3F;
  uint8_t udata = (chan << 7) | (dir << 6) | speed;

  if (port == SERIAL_PORT_1) {
    SERIAL1.write(udata);
  } else {
    SERIAL2.write(udata);
  }
}

// High-level UART write functions
void uart1Write(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  __uartWrite(SERIAL_PORT_1, chan, dir, speed);
}

void uart2Write(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  __uartWrite(SERIAL_PORT_2, chan, dir, speed);
}

// Movement functions
void backwards(uint8_t speed) {
  // Serial.println("F");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, speed);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, speed);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 0);
}

void forwards(uint8_t speed) {
    // Serial.println("B");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, speed);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, speed);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
}

void left(uint8_t speed) {
    // Serial.println("R");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, speed);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, speed);
}

void right(uint8_t speed) {
    // Serial.println("L");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, speed);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, speed);
}

void anticlock(uint8_t speed) {
    // Serial.println("C");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, speed);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, speed);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, speed);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, speed);
}

void rotate_clock(uint8_t speed) {
    // Serial.println("A");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW,speed);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, speed);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, speed);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, speed);
}

void stop() {
    // Serial.println("S");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
}

void receiveEvent(int bytes) {
  if (bytes == sizeof(PS4Data)) {
    Wire.readBytes((uint8_t*)&ps4Data, sizeof(ps4Data));

    int ly = ps4Data.LStickY;
    int rx = ps4Data.RStickX;
    if (ly > 15) {
      // Serial.println("F");
      forwards(map(ly, 0, 127, 0, 40));
    } else if (ly < -15) {
      // Serial.println("B");
      backwards(map(ly, 0, -127, 0, 40));
    } else if (rx > 15) {
      // Serial.println("R");
      anticlock(map(rx, 0, 127, 0, 25));
    } else if (rx < -15) {
      // Serial.println("L");
      rotate_clock(map(rx, 0, -127, 0, 25));
    } else if (ps4Data.L1) {
      // Serial.println("CCW");
      left(35);
    } else if (ps4Data.R1) {
      // Serial.println("CW");
      right(35);
    } else {
      // Serial.println("S");
      stop();
    }
  }
}

void setup() {
  Serial.begin(115200);
  SERIAL1.begin(115200, SERIAL_8N1, -1, UART_PORT_1_TX);
  SERIAL1.flush();

  SERIAL2.begin(115200, SERIAL_8N1, -1, UART_PORT_2_TX);
  SERIAL2.flush();

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
}

void loop() {

}
