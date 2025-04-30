#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <SCServo.h>
SMS_STS st;
Servo esc;

#define S_RXD 17    
#define S_TXD 16

#define ESC_PIN 40

byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];

TaskHandle_t Taskmot1;
TaskHandle_t Taskmot2;
TaskHandle_t bldcTask;
TaskHandle_t servotask;


// I2C configuration
#define I2C_SDA 8
#define I2C_SCL 9
#define SLAVE_ADDR 0x08

// UART configuration
#define UART_PORT_TX 41
#define SERIAL1 Serial1

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

void uartWrite(uart_ctrl_chan_t chan, uart_ctrl_dir_t dir, uint8_t speed) {
  if (chan >= UART_SIG_CHAN_MAX || dir >= UART_SIG_DIR_MAX) {
    return;
  }

  speed &= 0x3F; 
  uint8_t udata = (chan << 7) | (dir << 6) | speed;

  SERIAL1.write(udata);
}

//Task for shooter angle control
void shooter_servo(void *parameters)
{
  while(1){
  if(ps4Data.Up){
    Serial.println("Up");
  Position[0] = 2000;
  Position[1] = -2000;
  st.SyncWritePosEx(ID, 2, Position, Speed, ACC); // servo(ID1/ID2) speed=3400，acc=50，move to position=3000.
  delay(2000);
  }else if(ps4Data.Down){
    Serial.println("Down");
  Position[0] = -2000;
  Position[1] = 2000;
  st.SyncWritePosEx(ID, 2, Position, Speed, ACC); // servo(ID1/ID2) speed=3400，acc=50，move to position=3000.
  delay(2000);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}
// Task to control shooter motor
void shooter_dc(void *parameters) {
  while (1) {
    if (ps4Data.Right) {
      uartWrite(UART_SIG_RCHAN, UART_SIG_CW, 20);
      // Serial.println("RIght");
    } else if (ps4Data.Left) {
      uartWrite(UART_SIG_RCHAN, UART_SIG_CCW, 20);
      // Serial.println("Left");
    } else {
      uartWrite(UART_SIG_RCHAN, UART_SIG_CW, 0);  
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// Task to control intake motor
void intake(void *parameters) {

  while (1) {
    if(ps4Data.Triangle){
      // Serial.println("Intake");
      // uint8_t l2 = map(ps4Data.L2,0,255,0,63);
      uartWrite(UART_SIG_LCHAN, UART_SIG_CW,63); 
  }else {
     uartWrite(UART_SIG_LCHAN, UART_SIG_CW,0); 
  }
      vTaskDelay(10 / portTICK_PERIOD_MS); 
}
}

void servo_config(){
    ID[0] = 1;   
  ID[1] = 2;   
  Speed[0] = 3400;
  Speed[1] = 3400; 
  ACC[0] = 100;   
  ACC[1] = 100;
  Position[0] = 0;
  Position[1] = 0;
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
                delay(2);
            }
        } else if (throttle < currentThrottle) {
            for (int i = currentThrottle; i >= throttle; i--) {
                esc.writeMicroseconds(i);
                // Serial.print("Throttle set to: ");
                // Serial.println(i);
                delay(2);
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
  SERIAL1.begin(115200, SERIAL_8N1, -1, UART_PORT_TX);
  SERIAL1.flush();
  Serial2.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
  servo_config();
  Serial.begin(115200);

  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);

  uartWrite(UART_SIG_LCHAN, UART_SIG_CW, 0);
  uartWrite(UART_SIG_RCHAN, UART_SIG_CCW, 0);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

  xTaskCreate(shooter_dc, "shooter_dc", 2048, NULL, 1, &Taskmot1);
  xTaskCreate(intake, "intake", 2048, NULL, 2, &Taskmot2);
  xTaskCreate(bldc, "bldc", 2048, NULL, 2, &bldcTask);
  xTaskCreate(shooter_servo, "shooter_servo", 2048, NULL, 1, &servotask);

}

void loop(){}