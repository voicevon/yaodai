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

Adafruit_MPU6050 mpu;
Motor motor_front(PIN_MOTOR_FRONT);
Motor motor_rear(PIN_MOTOR_REAR);
Motor motor_left(PIN_MOTOR_LEFT);
Motor motor_right(PIN_MOTOR_RIGHT);

void stop_all_motors(){
	motor_front.Stop();
	motor_rear.Stop();
	motor_left.Stop();
	motor_right.Stop();
}

TwoWire my_bus_0(0);
TwoWire my_bus_1(1);

void setup_mpu6050(TwoWire* wire, int addr) {
	// Try to initialize!
	// if (!mpu.begin(0x68, wire)) {
	if (!mpu.begin(addr, wire)) {
	  Serial.println("Failed to find MPU6050 chip");
	  while (1) {
		delay(10);
	  }
	}
	Serial.println("MPU6050 Found!");
  
  
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	Serial.print("Accelerometer range set to: ");
	switch (mpu.getAccelerometerRange()) {
	case MPU6050_RANGE_2_G:
	  Serial.println("+-2G");
	  break;
	case MPU6050_RANGE_4_G:
	  Serial.println("+-4G");
	  break;
	case MPU6050_RANGE_8_G:
	  Serial.println("+-8G");
	  break;
	case MPU6050_RANGE_16_G:
	  Serial.println("+-16G");
	  break;
	}
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
	Serial.print("Gyro range set to: ");
	switch (mpu.getGyroRange()) {
	case MPU6050_RANGE_250_DEG:
	  Serial.println("+- 250 deg/s");
	  break;
	case MPU6050_RANGE_500_DEG:
	  Serial.println("+- 500 deg/s");
	  break;
	case MPU6050_RANGE_1000_DEG:
	  Serial.println("+- 1000 deg/s");
	  break;
	case MPU6050_RANGE_2000_DEG:
	  Serial.println("+- 2000 deg/s");
	  break;
	}
  
  
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
	Serial.print("Filter bandwidth set to: ");
	switch (mpu.getFilterBandwidth()) {
	case MPU6050_BAND_260_HZ:
	  Serial.println("260 Hz");
	  break;
	case MPU6050_BAND_184_HZ:
	  Serial.println("184 Hz");
	  break;
	case MPU6050_BAND_94_HZ:
	  Serial.println("94 Hz");
	  break;
	case MPU6050_BAND_44_HZ:
	  Serial.println("44 Hz");
	  break;
	case MPU6050_BAND_21_HZ:
	  Serial.println("21 Hz");
	  break;
	case MPU6050_BAND_10_HZ:
	  Serial.println("10 Hz");
	  break;
	case MPU6050_BAND_5_HZ:
	  Serial.println("5 Hz");
	  break;
	}
  

  }

void setup_motors(){
	motor_front.SetVibrate(200, 500);	
	motor_rear.SetVibrate(200, 500);
	motor_left.SetVibrate(200, 500);
	motor_right.SetVibrate(200, 500);
	
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
	// 检查 I2C 总线是否正常
	// if (Wire.beginTransmission(0x68) == 0) { // MPU6050 的 I2C 地址是 0x68
	// 	Serial.println("MPU6050 found!");
	// }		
	setup_mpu6050(&my_bus_0, 0x68);  // 0x68 是 front 的地址    
	// setup_mpu6050(&my_bus_0, 0x69);	// 0x69 是 right 的地址
	// setup_mpu6050(&my_bus_1, 0x69);  // 0x69 是 rear 的地址
	// setup_mpu6050(&my_bus_1, 0x68);  // 0x68 是 left 的地址
				

	// 初始化完成
	Serial.printf("Setup is done.\n");
}

void loop_test_mpu6050() {
	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
  
  
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

void loop() {
	touch_power_off();
	// loop_test_mpu6050();
	loop_test_motors();
	loop_test_touch_leds();


	// sensors_event_t a, g, temp;
	// mpu.getEvent(&a, &g, &temp);
	// if (a.acceleration.x > 10) {
	// 	motor_front.Start();
	// }else if(a.acceleration.x < -10){
	// 	motor_front.Stop();	
	// }
}
  