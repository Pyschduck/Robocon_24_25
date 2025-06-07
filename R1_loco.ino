#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

TaskHandle_t locotask = NULL;

TaskHandle_t Taskmot1 = NULL;
TaskHandle_t Taskmot2 = NULL;

#define mot1_in 40
#define mot1_pwm 38

#define mot2_in 41
#define mot2_pwm 39

#define I2C_SDA 8
#define I2C_SCL 9
#define SLAVE_ADDR 0x08

#define UART_PORT_1_TX 16
#define UART_PORT_2_TX 17

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
void forwards(uint8_t speed) {
  Serial.println("F");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, speed);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, speed);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
}

void backwards(uint8_t speed) {
    Serial.println("B");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, speed);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, speed);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 0);
}

void right(uint8_t speed) {
    Serial.println("R");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, speed);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, speed);
}

void left(uint8_t speed) {
    Serial.println("L");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, speed);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, speed);
}

void rotate_clock() {
    Serial.println("C");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, 20);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CW, 20);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CW, 20);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 20);
}

void anticlock() {
    Serial.println("A");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 20);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 20);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 20);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 20);
}

void stop() {
    Serial.println("S");
  uart1Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_LCHAN, UART_SIG_CCW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
}

void loco(void *parameters){
  while(1)
  {
    int ly = ps4Data.LStickY;
    int rx = ps4Data.RStickX;
    if (ly > 15) {
      Serial.println("F");
      forwards(map(ly, 0, 127, 0, 63));
    } else if (ly < -15) {
      Serial.println("B");
      backwards(map(ly, 0, -127, 0, 63));
    } else if (rx > 15) {
      Serial.println("R");
      right(map(rx, 0, 127, 0, 63));
    } else if (rx < -15) {
      Serial.println("L");
      left(map(rx, 0, -127, 0, 63));
    } else if (ps4Data.L1) {
      Serial.println("CCW");
      anticlock();
    } else if (ps4Data.R1) {
      Serial.println("CW");
      rotate_clock();
    } else {
      Serial.println("S");
      stop();
    }
  }
}

void intake(void *parameters) {
  while (1) {
    if (ps4Data.Up) {
      Serial.println("Intake Up"); 
        digitalWrite(mot2_in,LOW);
         analogWrite(mot2_pwm,255);
    } else if (ps4Data.Down) {
      Serial.println("Intake Down");
        digitalWrite(mot2_in,HIGH);
         analogWrite(mot2_pwm,255); 
    } else {
        digitalWrite(mot2_in,LOW);
         analogWrite(mot2_pwm,0); 
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void shooter(void *parameters) {
  while (1) {
    if (ps4Data.L2 >= 10) {
      uint8_t l2 = map(ps4Data.L2, 0, 255, 0, 63);
      Serial.printf("Shooter L2 val %d\n", l2);
      // uartWrite(UART_SIG_RCHAN, UART_SIG_CW, l2);
        digitalWrite(mot1_in,LOW);
         analogWrite(mot1_pwm,l2);
    } else {
      // uartWrite(UART_SIG_RCHAN, UART_SIG_CW, 0);
        digitalWrite(mot1_in,LOW);
         analogWrite(mot1_pwm,0);  
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void receiveEvent(int bytes) {
  if (bytes == sizeof(PS4Data)) {
    uint8_t *ptr = (uint8_t*)&ps4Data;
    for (int i = 0; i < sizeof(PS4Data); i++) {
      if (Wire.available()) {
        ptr[i] = Wire.read();
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  SERIAL1.begin(115200, SERIAL_8N1, -1, UART_PORT_1_TX);
  SERIAL1.flush();

  SERIAL2.begin(115200, SERIAL_8N1, -1, UART_PORT_2_TX);
  SERIAL2.flush();

  pinMode(mot1_in, OUTPUT);
  pinMode(mot2_in, OUTPUT);
  pinMode(mot1_pwm, OUTPUT);
  pinMode(mot2_pwm, OUTPUT);

  digitalWrite(mot1_in,LOW);
  digitalWrite(mot2_in,LOW);
  analogWrite(mot1_pwm,0);
  analogWrite(mot2_pwm,0);

  stop();

  xTaskCreate(loco, "loco", 2048, NULL, 1, &locotask);
  xTaskCreate(shooter, "shooter", 2048, NULL, 1, &Taskmot1);
  xTaskCreate(intake, "intake", 2048, NULL, 1, &Taskmot2);
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
}

void loop() {

}
