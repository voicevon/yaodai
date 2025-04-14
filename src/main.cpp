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

Adafruit_MPU6050 mpu_front;
Adafruit_MPU6050 mpu_rear;
Adafruit_MPU6050 mpu_left;
Adafruit_MPU6050 mpu_right;

Motor motor_front(PIN_MOTOR_FRONT, 1000);
Motor motor_rear(PIN_MOTOR_REAR, 1000);
Motor motor_left(PIN_MOTOR_LEFT, 1000);
Motor motor_right(PIN_MOTOR_RIGHT, 1000);

void stop_all_motors(){
	motor_front.Stop();
	motor_rear.Stop();
	motor_left.Stop();
	motor_right.Stop();
}

TwoWire my_bus_0(0);
TwoWire my_bus_1(1);

void setup_mpu6050(TwoWire* wire, int addr, Adafruit_MPU6050* mpu) {
    // 检查 I2C 总线是否正常
    wire->beginTransmission(addr);
    if (wire->endTransmission() != 0) {
        while (1) {
            int bus_num = (wire == &my_bus_0) ? 0 : 1;
            Serial.printf("I2C bus %d error, device 0x%02X not found\n", bus_num, addr);
            delay(1000);
        }
    }

    if (!mpu->begin(addr, wire)) {
        while (1){
            Serial.println("Failed to find MPU6050 chip  ");
            delay(1000);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu->setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu->setGyroRange(MPU6050_RANGE_500_DEG);
    mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);
}


void setup() {
	Serial.begin(115200);
	Serial.printf("Hello, Yaodai!\n");
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_LED_B, OUTPUT);

	pinMode(PIN_POWER, OUTPUT); 
	digitalWrite(PIN_POWER, LOW); // 高电平 关闭系统。
	pinMode(PIN_BUZZER, OUTPUT);
	pinMode(PIN_TOUCH, INPUT);
	pinMode(PIN_DOCKER, INPUT);
	stop_all_motors();
	Serial.printf("pinMode is done.\n");

	// 初始化 MPU6050
	my_bus_0.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
	my_bus_0.setClock(100000); // 100kHz

	my_bus_1.begin(17, 16); 
	my_bus_1.setClock(100000); // 100kHz
	
	setup_mpu6050(&my_bus_0, 0x68, &mpu_front);  // 0x68 是 front 的地址    
	setup_mpu6050(&my_bus_0, 0x69, &mpu_right);	// 0x69 是 right 的地址
	// setup_mpu6050(&my_bus_1, 0x69, &mpu_rear);  // 0x69 是 rear 的地址
	// setup_mpu6050(&my_bus_1, 0x68, &mpu_left);  // 0x68 是 left 的地址
				

	// 初始化完成
	Serial.printf("Setup is done.\n");
}

void loop_test_mpu6050() {
	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu_front.getEvent(&a, &g, &temp);
  
  
	/* Print out the values */
	Serial.print("Acceleration X: ");
	Serial.print(a.acceleration.x);
	Serial.print(", Y: ");
	Serial.print(a.acceleration.y);
	Serial.print(", Z: ");
	Serial.print(a.acceleration.z);
	Serial.println(" m/s^2");

  
	Serial.print("Rotation X: ");
	Serial.print(g.gyro.x);
	Serial.print(", Y: ");
	Serial.print(g.gyro.y);
	Serial.print(", Z: ");
	Serial.print(g.gyro.z);
	Serial.println(" rad/s");
  
  
	Serial.print("Temperature: ");
	Serial.print(temp.temperature);
	Serial.println(" degC");
  
  
	Serial.println("");
	delay(500);
}

void loop_test_motors(){
	#define VIB_TIME 50
	#define SIL_TIME 1000
	Serial.printf("front motor    ");
	motor_front.Start();
	delay(VIB_TIME);
	motor_front.Stop();
	delay(SIL_TIME);

	Serial.printf("right motor    ");
	motor_right.Start();
	delay(VIB_TIME);
	motor_right.Stop();
	delay(SIL_TIME);

	Serial.printf("rear motor    ");
	motor_rear.Start();
	delay(VIB_TIME);
	motor_rear.Stop();
	delay(SIL_TIME);

	Serial.printf("left motor     ");
	motor_left.Start();
	delay(VIB_TIME);
	motor_left.Stop();
	delay(SIL_TIME);
}

void loop_test_touch_leds(){
	if(digitalRead(PIN_TOUCH) == HIGH){
		// Serial.printf("touch is HIGH\n");
		digitalWrite(PIN_LED, LOW);		//低电平 灯亮。
	}else{	
		digitalWrite(PIN_LED, HIGH);
	}

	if(digitalRead(PIN_DOCKER) == HIGH){
		// Serial.printf("touch is HIGH\n");
		digitalWrite(PIN_LED_B, LOW);		//低电平 灯亮。
	}else{	
		digitalWrite(PIN_LED_B, HIGH);
	}
}

void touch_power_off(){
	static int kept_touched_from = 0;
	if(digitalRead(PIN_TOUCH) == HIGH){
		if (millis() - kept_touched_from > 1000) {
			// 如果触摸次数超过 1 秒，关闭系统，包括 ESP32
			Serial.printf("touch is HIGH for 1 second, power off now.\n");
			digitalWrite(PIN_POWER, HIGH);	// 高电平 关闭系统。
		}
	}else{	
		kept_touched_from = millis();
	}
}


#define Z_THRESHOLD 10  // Z 分量阈值  

void loop() {

	unsigned long start_time = millis();


    static unsigned long last_read_time = 0;
    const unsigned long read_interval = 100; // 设置读取间隔
	motor_front.SpinOnce();
	motor_rear.SpinOnce();


    if (millis() - last_read_time <= read_interval) {
		return;
	}
	last_read_time = millis();

	sensors_event_t a, g, temp;
	mpu_front.getEvent(&a, &g, &temp);	// 频繁出现：  读取时间 1002 ms
    // unsigned long end_time = millis();  // 记录结束时间
    // unsigned long elapsed_time = end_time - start_time;  // 计算运行时间
	// if (elapsed_time > 200) {
	//     Serial.printf("getEvent()  time: %lu ms\n", elapsed_time);  // 打印运行时间

    int z = a.acceleration.z * 100;
    if (abs(z) > Z_THRESHOLD) {  // 使用 Z 分量
        // int vibrate_ms = map(abs(z), Z_THRESHOLD, 1000, 100, 1000); // 将 Z 分量映射到振动时间
        int vibrate_ms = abs(z); // 将 Z 分量映射到振动时间
		// Serial.printf("z = %d,  vib_ms= %d \n", z, vibrate_ms);
		if (vibrate_ms > 800){
			vibrate_ms = 800;
		}
        if (z < 0) {  // 正向加速度
            motor_front.SetVibrate(vibrate_ms);
            motor_rear.SetVibrate(0);
        } else {  // 负向加速度
            motor_front.SetVibrate(0);
            motor_rear.SetVibrate(vibrate_ms);
        }
    } else {  // Z 分量接近零时停止
        motor_front.SetVibrate(0);
        motor_rear.SetVibrate(0);
    }



   unsigned long end_time2 = millis();  // 记录结束时间
    unsigned long elapsed_time2 = end_time2 - start_time;  // 计算运行时间
	if (elapsed_time2 > 200) {
	    Serial.printf("Loop execution time: %lu ms\n", elapsed_time2);  // 打印运行时间
	}
	

}