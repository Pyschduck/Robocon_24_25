#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

Servo esc;
#define ESC_PIN 40

TaskHandle_t Taskmot1;
TaskHandle_t Taskmot2;
TaskHandle_t bldcTask;
TaskHandle_t Taskmot3;

#define I2C_SDA 8
#define I2C_SCL 9
#define SLAVE_ADDR 0x08

#define UART_PORT_1_TX 41
#define SERIAL1 SERIAL_PORT_1

#define UART_PORT_2_TX 40
#define SERIAL2 SERIAL_PORT_2

#define DEBUG 0


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

void __uartWrite(uart_custom_port_t port, uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  if (port >= SERIAL_PORT_MAX || chan >= UART_SIG_CHAN_MAX || dir >= UART_SIG_DIR_MAX) return;

  speed &= 0x3F;
  uint8_t udata = (chan << 7) | (dir << 6) | speed;

if (port == SERIAL_PORT_1) {
    Serial1.write(udata);
} else if (port == SERIAL_PORT_2) {
    Serial2.write(udata);
}
}


void uart1Write(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  __uartWrite(SERIAL_PORT_1, chan, dir, speed);
}

void uart2Write(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  __uartWrite(SERIAL_PORT_2, chan, dir, speed);
}

void shooter_dc(void *parameters) {
  while (1) {
    if (ps4Data.Right) {
      uart1Write(UART_SIG_RCHAN, UART_SIG_CW, 20);
      // Serial.println("RIght");
    } else if (ps4Data.Left) {
      uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 20);
      // Serial.println("Left");
    } else {
      uart1Write(UART_SIG_RCHAN, UART_SIG_CW, 0);  
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void intake(void *parameters) {

  while (1) {
    if(ps4Data.Triangle){
      // Serial.println("Intake");
      // uint8_t l2 = map(ps4Data.L2,0,255,0,63);
      uart1Write(UART_SIG_LCHAN, UART_SIG_CW,63); 
  }else if(ps4Data.Circle){
     uart1Write(UART_SIG_LCHAN, UART_SIG_CCW,63); 
  }else
  {
    uart1Write(UART_SIG_LCHAN, UART_SIG_CCW,0);
  }
      vTaskDelay(10 / portTICK_PERIOD_MS); 
}
}

void shooter_angle(void *parameters)
{
    while (1) {
    if (ps4Data.Up) {
      uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 20);
      // Serial.println("RIght");
    } else if (ps4Data.Down) {
      uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 20);
      // Serial.println("Left");
    } else {
      uart2Write(UART_SIG_RCHAN, UART_SIG_CW, 0);  
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}
void bldc(void *parameters){
 while (1) {
    // if(ps4Data.R2){
    int throttle = map(ps4Data.R2, 0, 255, 1050, 2000); 

    if (throttle >= 1000 && throttle <= 2000) {
        int currentThrottle = esc.readMicroseconds();

        if (throttle > currentThrottle) {
            for (int i = currentThrottle; i <= throttle; i++) {
                esc.writeMicroseconds(i);
                // Serial.print("Throttle set to: ");
                // Serial.println(i);+
                vTaskDelay(2 / portTICK_PERIOD_MS);
            }
        } else if (throttle < currentThrottle) {
            for (int i = currentThrottle; i >= throttle; i--) {
                esc.writeMicroseconds(i);
                // Serial.print("Throttle set to: ");
                // Serial.println(i);
                vTaskDelay(2 / portTICK_PERIOD_MS);

            }
        }
    }
    // }
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
  Serial1.begin(115200, SERIAL_8N1, -1, UART_PORT_1_TX);
  Serial1.flush();
  Serial2.begin(115200, SERIAL_8N1, -1, UART_PORT_2_TX);
  Serial2.flush();
  Serial.begin(115200);

  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);

  uart1Write(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uart1Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);
  uart2Write(UART_SIG_RCHAN, UART_SIG_CCW, 0);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

  xTaskCreate(shooter_dc, "shooter_dc", 2048, NULL, 1, &Taskmot1);
  xTaskCreate(intake, "intake", 2048, NULL, 2, &Taskmot2);
  xTaskCreate(bldc, "bldc", 2048, NULL, 2, &bldcTask);
  xTaskCreate(shooter_angle, "shooter_angle", 2048, NULL, 1, &Taskmot3);

}

void loop(){}
