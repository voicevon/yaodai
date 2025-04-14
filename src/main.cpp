#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "motor.h"

#define PIN_LED 19
#define PIN_LED_B 18
#define PIN_MOTOR_FRONT 13 
#define PIN_MOTOR_REAR 14 
#define PIN_MOTOR_LEFT 33 
#define PIN_MOTOR_RIGHT 32 

//   // 如果高电平，会关闭自己（ESP32掉电)。
#define PIN_POWER 25 // 控制总供电。
#define PIN_TOUCH 26 // 软件自定义功能

//   // 蜂鸣器，低电平 响。
#define PIN_BUZZER 21

#define PIN_I2C_SDA 22
#define PIN_I2C_SCL 23

#define PIN_DOCKER 27   
#define PIN_BATTERY_ADC 4

Adafruit_MPU6050 mpu_front;
Adafruit_MPU6050 mpu_rear;
Adafruit_MPU6050 mpu_left;
Adafruit_MPU6050 mpu_right;
Motor motor_front(PIN_MOTOR_FRONT);
Motor motor_rear(PIN_MOTOR_REAR);
Motor motor_left(PIN_MOTOR_LEFT);
Motor motor_right(PIN_MOTOR_RIGHT);

TwoWire my_bus_0(0);
TwoWire my_bus_1(1);

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing system...");

    // 初始化引脚
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    pinMode(PIN_POWER, OUTPUT); // 控制总供电。
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_TOUCH, INPUT);
    pinMode(PIN_DOCKER, INPUT);
    pinMode(PIN_BATTERY_ADC, INPUT);
    digitalWrite(PIN_POWER, LOW); // 高电平 关闭系统。

    // 初始化电机
    motor_front.SetPeriod(1000);
    motor_rear.SetPeriod(1000);
    motor_left.SetPeriod(1000);
    motor_right.SetPeriod(1000);

    // 初始化 I2C 总线
    my_bus_0.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    my_bus_0.setClock(100000); // 100kHz
    my_bus_1.begin(17, 16);
    my_bus_1.setClock(100000); // 100kHz

    // 初始化 MPU6050 传感器
    if (!mpu_front.begin(0x68, &my_bus_0)) { // 0x68 是 front 的地址
        Serial.println("Failed to find front MPU6050");
        while (1) delay(10);
    }
    if (!mpu_right.begin(0x69, &my_bus_0)) { // 0x69 是 right 的地址
        Serial.println("Failed to find right MPU6050");
        while (1) delay(10);
    }
    if (!mpu_rear.begin(0x69, &my_bus_1)) { // 0x69 是 rear 的地址
        Serial.println("Failed to find rear MPU6050");
        while (1) delay(10);
    }
    if (!mpu_left.begin(0x68, &my_bus_1)) { // 0x68 是 left 的地址
        Serial.println("Failed to find left MPU6050");
        while (1) delay(10);
    }

    // 配置 MPU6050
    mpu_front.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_front.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_front.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("System initialization complete.");
}

void loop() {
    // 电源管理
    static int kept_touched_from = 0;
    if(digitalRead(PIN_TOUCH) == HIGH){
        // 如果触摸次数超过 1 秒，关闭系统，包括 ESP32
        if (millis() - kept_touched_from > 1000) {
            digitalWrite(PIN_POWER, HIGH); // 高电平 关闭系统。
        }
    } else {    
        kept_touched_from = millis();
    }

    // LED 控制
    digitalWrite(PIN_LED, digitalRead(PIN_TOUCH) == HIGH ? LOW : HIGH); //低电平 灯亮。
    digitalWrite(PIN_LED_B, digitalRead(PIN_DOCKER) == HIGH ? LOW : HIGH); //低电平 灯亮。

    // 电池电压检测
    int adc = analogRead(PIN_BATTERY_ADC); 
    float voltage = 7.12 * adc / 4095.0;  // 计算电压值
	if(voltage < 3.5){
        digitalWrite(PIN_BUZZER, LOW); //低电平 响。
    }else{
        digitalWrite(PIN_BUZZER, HIGH); //高电平 不响。
    }
    // 打印电池电压
    // Serial.printf("battery adc = %d, voltage = %.2f V\n", adc, voltage);

    // 主工作逻辑
    sensors_event_t a, g, temp;
    mpu_front.getEvent(&a, &g, &temp);
    
    float z = constrain(a.acceleration.z, -20.0f, 20.0f);
    float x = constrain(a.acceleration.x, -20.0f, 20.0f); // 添加 X 分量处理
    int vibrate_ms_z = map(z * 100, -2000, 2000, 100, 500);
    int vibrate_ms_x = map(x * 100, -2000, 2000, 100, 500); // 根据 X 分量计算振动时间

    // 根据 Z 分量控制前后电机
    if (z > 1.0f) {  // 正向加速度
        motor_front.SetVibrate(vibrate_ms_z);
    } else if (z < -1.0f) {  // 负向加速度
        motor_rear.SetVibrate(vibrate_ms_z);
    } else {  // 接近零时停止
        motor_front.SetVibrate(0);
        motor_rear.SetVibrate(0);
    }

    // 根据 X 分量控制左右电机
    if (x > 1.0f) {  // 向右倾斜
        motor_right.SetVibrate(vibrate_ms_x);
    } else if (x < -1.0f) {  // 向左倾斜
        motor_left.SetVibrate(vibrate_ms_x);
    } else {  // 接近零时停止
        motor_left.SetVibrate(0);
        motor_right.SetVibrate(0);
    }

    // 更新所有电机状态
    motor_front.SpinOnce();
    motor_rear.SpinOnce();
    motor_left.SpinOnce();
    motor_right.SpinOnce();

    // delay(100);
}
  