#include <OneButton.h>
#include<Wire.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <String.h>
SoftwareSerial sim808(11, 10); //(RX, TX)

//Định nghĩa một số phần cứng
#define buzzer 9
#define power 13
#define button 2

//Định nghĩa các thanh ghi cho MPU6050
#define sample_rate 25    // 0x19
#define configs 26         // 0x1A
#define gyro_config 27    // 0x1B
#define acc_config 28     // 0x1B
#define interrupt 56      // 0x1B
#define pwr_managment 107 // 0x1B
#define acc_x 59          // 59: bit cao; 60: bit thap.
#define acc_y 61          // 61: bit cao; 62: bit thap.
#define acc_z 63          // 63: bit cao; 64: bit thap.

int mpu6050 = 0x68;
boolean shake = false, sentShakeMode = false, previousshake;
float pitchSetup, rollSetup, yawSetup;
unsigned long getAngleOldTime = 0;
char phonenum[] = "+84378844275";
String data[20];
String state, timegps, latitude, longitude, getPhoneNum;
String response, address, smsData, smsNotify, gpsData;
boolean enable = false;

/*-------------Khai báo một số thanh ghi cần thiết cho MPU6050-------------------*/
void Init_MPU()
{
  Wire.beginTransmission(mpu6050);

  Wire.write(sample_rate);
  Wire.write(15);

  Wire.write(configs);
  Wire.write(0);

  Wire.write(gyro_config);
  Wire.write(0x08);

  Wire.write(acc_config);
  Wire.write(0x10);

  Wire.write(interrupt);
  Wire.write(0x01);

  Wire.write(pwr_managment);
  Wire.write(0x01);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(9600);
  sim808.begin(9600);

  Wire.begin(0x68); //Khởi chạy I2C
  Init_MPU();

  pinMode(power, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  attachInterrupt(0, shakemode, FALLING);

  sendData("AT", 300);
  sendData("AT+CMGF=1", 300);
  sendData("AT+CNMI=2,1", 300);
  sendData("AT+CGPSPWR=1", 1000);
  Serial.println("Power = " + (String)digitalRead(power));
}

void loop()
{
  readSms();
  checkSms();
  getLocation();
  sendLocation();
  if (sim808.available() > 0)
  {
    long int timeout = 3000;
    long int time = millis();
    boolean bg = false;
    while ((time + timeout) > millis())
    {
      while (sim808.available())
      {
        char c = sim808.read();
        if (c == '+') bg = true;
        if (bg)
        {
          response += c;
          delay(5);
        }
      }
    }
    Serial.println(response);

    if (response.startsWith("+CMTI"))
    {
      smsNotify = response;
      response = "";
    }
    else if (response.startsWith("+CMGR"))
    {
      smsData = response;
      response = "";
    }
    else if (response.startsWith("+CGNSINF"))
    {
      gpsData = response;
      response = "";
    }
  }
  /*--------------SHAKE MODE---------------*/
  setupShakeMode();
  if (digitalRead(power) == 1)
  {
    shake = 0;
  }

  if (shake == 1)
  {
    if (millis() - getAngleOldTime > 10000)
    {
      Serial.println("pitch = " + (String)PitchRollYaw("pitch"));
      Serial.println("roll = " + (String)PitchRollYaw("roll"));
      Serial.println("yaw = " + (String)PitchRollYaw("yaw"));
      if ((PitchRollYaw("pitch") - pitchSetup > 10  | PitchRollYaw("pitch") - pitchSetup < -10)
          | (PitchRollYaw("roll") - rollSetup > 10  | PitchRollYaw("roll") - rollSetup < -10)
          | (PitchRollYaw("yaw") - yawSetup > 10    | PitchRollYaw("yaw") - yawSetup < -10))
      {
        Serial.println("Nghieng");
        if ( sentShakeMode == false)
        {
          sendSms("Your car is no longer in the same location");
          sentShakeMode = true;
        }
      }
      getAngleOldTime = millis();
    }
  }
}


/*-------------HÀM CON BẶT TẮT CHẾ ĐỘ CẢNH BÁO RUNG---------------------*/
void shakemode()
{
  noInterrupts();
  if (digitalRead(power) == 0)
  {
    if (digitalRead(button) == 0)
    {
      delay(20);
      while (digitalRead(button) == 0)
      {
        if (shake == 1)
        {
          shake = 0;
        }
        else
        {
          shake = 1;
        }
        Serial.println((String)shake);
        while (digitalRead(button) == 0);
      }
    }
  }
  interrupts();
}

/*--------------HÀM CON SETUP CHẾ ĐỘ CẢNH BÁO RUNG----------------*/
void setupShakeMode()
{
  if (digitalRead(power) == 0)
  {
    if (shake != previousshake)
    {
      if (shake == 1)
      {
        for (int i = 0; i < 3; i++)
        {
          digitalWrite(buzzer, 1);
          delay(100);
          digitalWrite(buzzer, 0);
          delay(50);
        }
        sentShakeMode = false;
        pitchSetup = PitchRollYaw("pitch");
        rollSetup = PitchRollYaw("roll");
        yawSetup = PitchRollYaw("yaw");
        Serial.println("pitchsetup= " + (String)pitchSetup);
        Serial.println("rollsetup= " + (String)rollSetup);
        Serial.println("yawsetup= " + (String)yawSetup);
      }
      else
      {
        digitalWrite(buzzer, 1);
        delay(100);
        digitalWrite(buzzer, 0);
        delay(100);
      }
    }
    previousshake = shake;
  }
}

/*----------HÀM CON ĐỌC GIÁ TRỊ CẢM BIẾN MPU6050-----------------*/
int16_t readMPU(int address)
{
  int16_t high, low, data;
  Wire.beginTransmission(mpu6050);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(mpu6050, 1);
  high = Wire.read();

  Wire.beginTransmission(mpu6050);
  Wire.write(address + 1);
  Wire.endTransmission();
  Wire.requestFrom(mpu6050, 1);
  low = Wire.read();

  data = (high << 8) | low;
  //  Serial.println("data= " + (String)data);
  return data;
}


/*---------HÀM CON TÍNH GIÁ TRỊ GÓC NGHIÊNG Pitch Roll Yaw--------*/
float PitchRollYaw(String angle)
{
  float Ax = (float)readMPU(acc_x) / 4096.0;
  float Ay = (float)readMPU(acc_y) / 4096.0;
  float Az = (float)readMPU(acc_z) / 4096.0;

  float pitch = atan2(Ax, sqrt(pow(Ay, 2) + pow(Az, 2))) * 180 / M_PI;
  float roll = atan2(Ay, sqrt(pow(Ax, 2) + pow(Az, 2))) * 180 / M_PI;
  float yaw = atan2(Az, sqrt(pow(Ay, 2) + pow(Ax, 2))) * 180 / M_PI;

  //    Serial.println("pitch= " + (String)pitch);
  //    Serial.println("roll= " + (String)roll);
  //    Serial.println("yaw= " + (String)yaw);

  if (angle == "pitch")   return pitch;
  else if (angle == "roll")   return roll;
  else if (angle == "yaw")   return yaw;
}




/*--------HÀM CON GỬI TẬP LỆNH AT--------*/
void sendData(String command, const int timeout)
{
  sim808.println(command);
  long int time = millis();
  while ( (time + timeout ) > millis())
  {
    while (sim808.available())
    {
      char c = sim808.read();
      response += c;
    }
  }
  Serial.println("RS=" + response);
  response = "";
}


/*-----------HÀM CON ĐỌC TIN NHẮN ĐẾN-----------*/
void readSms()
{
  if (smsNotify.length() > 0)
  {
    if (smsNotify.length() >= 16)
    {
      address = smsNotify.substring(12, 14);
      //      Serial.println(address);
    }
    else
    {
      address = smsNotify.substring(12, 13);
      Serial.println("address=" + address);
    }

    Serial.println("Please wait while reading the message...");
    sim808.println("AT+CMGR=" + address);
  }
  smsNotify = "";
}

/*--------HÀM CON KIỂM TRA NỘI DUNG TIN NHẮN VÀ SỐ ĐIỆN THOẠI--------*/
void checkSms()
{
  if (smsData.length() > 0)
  {
    if (smsData.indexOf(phonenum) >= 0)
    {
      if (smsData.indexOf("LOCATION") >= 0)
      {
        Serial.println("Please wait while reading GPS information...");
        sim808.println("AT+CGNSINF");
      }
      else if (smsData.indexOf("SOUND") >= 0)
      {
        digitalWrite(buzzer, 1);
        delay(500);
        digitalWrite(buzzer, 0);
        delay(200);
      }
      else if ((smsData.indexOf("LOCATION") == -1) | (smsData.indexOf("SOUND") == -1))
      {
        Serial.println("Syntax error");
        sendSms("Syntax error");
        delay(500);
      }
    }
    else
    {
      Serial.println("Unknown phone Number!!");
      sendData("AT+CMGDA=DEL ALL", 500);
    }

  }

  smsData = "";
}

/*--------HÀM CON LẤY THÔNG TIN GPS--------*/
void getLocation()
{
  if (gpsData.length() > 0)
  {
    int i = 0, j = 0;
    //    Serial.println(gpsData.charAt(20));
    //    Serial.println(gpsData.length());
    while ( j <= gpsData.length())
    {
      char c = gpsData.charAt(j);
      if (c != ',')
      {
        data[i] += c;
        delay(50);
      }
      else
      {
        i++;
      }
      if (i == 5) break;
      j++;
    }
    state = data[1];
    timegps = data[2];
    latitude = data[3];
    longitude = data[4];
    data[0] = "";
    data[1] = "";
    data[2] = "";
    data[3] = "";
    data[4] = "";
    Serial.println("State  :" + state);
    Serial.println("Time  :" + timegps);
    Serial.println("Latitude  :" + latitude);
    Serial.println("Longitude  :" + longitude);
  }
  gpsData = "";
}


/*------------HÀM CON GỬI TIN NHẮN VỀ ĐIỆN THOẠI-----------*/
void sendSms(String message)
{
  sim808.print("AT+CMGS=");
  sim808.print('"');
  sim808.print(phonenum);
  sim808.println('"');
  delay(400);

  //Noi dung tin nhan:
  sim808.print(message);
  delay(200);

  //Ket thuc noi dung tin nhan bang ma Ascii 0x1A(26):
  sim808.println((char)26);
  delay(5000);

  //Cho den khi Serial hoan tat gui du lieu:
  sim808.flush();
  Serial.println("GPS Location sent! Check your mobile phone…");
  sendData("AT", 500);
}

/*------------HÀM CON GỬI THÔNG TIN GPS VỀ ĐIỆN THOẠI-----------*/
void sendLocation()
{
  if (state == "1")
  {
    sendSms("http://maps.google.com/maps?q=loc:" + latitude + "," + longitude);
    delay(100);
    sendData("AT+CMGDA=DEL ALL", 500);
    state = "";
    timegps = "";
    latitude = "";
    longitude = "";
  }
}
