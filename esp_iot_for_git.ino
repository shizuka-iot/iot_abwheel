#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <stdio.h>
#include <stdlib.h>
 
const char* ssid = "";
const char* password =  "";
#define N 256

char s[N] = {'\0'};
int times = 7;




#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float A_angleX, A_angleY, A_angleZ, A_angleXY, fixed_angleZ; //絶対値を入れる変数
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

int p_round = 0;
int n_round = 0;
int cycles = 0;
int prev_p_round = 0;
int prev_n_round = 0;
int t = 0;

int prev_state = 0;

bool onlyOnce = true;

#define SI 25 //シリアルデータを５番ピンに指定
#define RCK 33 //ラッチクロックを７番ピンに指定
#define SCK 32 //シリアルクロックを６番ピンに指定


#define led_k1 14 //一の位のカソードを14番ピンに指定
#define led_k10 27 //十の位のカソードを27番ピンに指定
#define led_k100 26 //百の位のカソードを26番ピンに指定

int fullNumber = 0; //最終的に表示される数（分解される前の数）
int digit_1 = 0; //１の位を入れる変数
int digit_10 = 0; //十の位を入れる変数
int digit_100 = 0; //百の位を入れる変数


int pinOut(int i){
    pinMode(i,OUTPUT);
  }

int pinInput(int p){
    pinMode(p,INPUT_PULLUP);
}

//以下でビットデータと７セグのレイアウトを定義
//カソードコモンタイプのLEDを使用

int num_array[]={
                0b11111100,//0
                0b01100000,//1
                0b11011010,//2
                0b11110010,//3
                0b01100110,//4
                0b10110110,//5
                0b10111110,//6
                0b11100100,//7
                0b11111110,//8
                0b11110110,//9
                0b00000000,//10(off)
                0b00000001,//11 dot point
                0b10000110,//12 c
};

//以下でledを全消灯する。出力をHIGHにするこで同電位となり電流は流れない。
void ledAllOff()
{
            digitalWrite(led_k1,1);
            digitalWrite(led_k10,1);
            digitalWrite(led_k100,1);
            numprint(10);
}

//下記は数字の一桁だけを表示する関数。
//もし1024を表示したい場合、numprint(1),numprint(0),numprnt(2),numprint(4)のように分割しないといけない。
void numprint(int number)
{
  digitalWrite(RCK,LOW);
  shiftOut(SI,SCK,LSBFIRST,num_array[number]);  //shiftOut()はarduinoのライブラリ関数。2進数を送ってくれる。
  digitalWrite(RCK,HIGH);
}

void digit_1print()
{
  digitalWrite(led_k1,0);
  digitalWrite(led_k10,1);
  digitalWrite(led_k100,1);
  numprint(digit_1);
  delay(2);
  ledAllOff();
}

void digit_10print()
{
  digitalWrite(led_k1,1);
  digitalWrite(led_k10,0);
  digitalWrite(led_k100,1);

  numprint(digit_10);
  delay(2);
  ledAllOff();
}

void digit_100print()
{
  digitalWrite(led_k1,1);
  digitalWrite(led_k10,1);
  digitalWrite(led_k100,0);

  numprint(digit_100);
  delay(2);
  ledAllOff();
}

void dpprint()
{
  digitalWrite(led_k1,1);
  digitalWrite(led_k10,1);
  digitalWrite(led_k100,0);

  numprint(11);
  delay(2);
  ledAllOff();
}

int fullNumber_print()
{
  fullNumber = cycles;

  digit_1print();
  if(fullNumber>=10)digit_10print();
  if(fullNumber>=100)digit_100print();
  
  fullNumber = (fullNumber) % 10000;
  digit_1 = (fullNumber) % 10; 
  digit_10 = fullNumber /10 % 10;
  digit_100 = fullNumber /100 % 10;
}


void culcRotation();

//I2c書き込み
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

//i2C読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}

void setup() {
  Serial.begin(115200);
  delay(4000);   //Delay needed before calling the WiFi.begin
   
  while(1){
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  if (readMPU6050(MPU6050_WHO_AM_I) == 0x68){
    break; 
  }
  
  }
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }

  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro

  //キャリブレーション
  Serial.print("Calculate Calibration");
  for(int i = 0; i < 3000; i++){
    
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
  
    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();
    raw_t = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
    dpsX = ((float)raw_gyro_x) / 65.5;
    dpsY = ((float)raw_gyro_y) / 65.5;
    dpsZ = ((float)raw_gyro_z) / 65.5;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    if(i % 1000 == 0){
      Serial.print(".");
    }
  }
  Serial.println();

  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;

  Serial.print("offsetX : ");
  Serial.println(offsetX);
  Serial.print("offsetY : ");
  Serial.println(offsetY);
  Serial.print("offsetZ : ");
  Serial.println(offsetZ);

  pinOut(32); //シリアルデータ(2進数のデータ)用ピンを出力モードに指定
  pinOut(33); //シリアルクロックピンを出力モードに指定
  pinOut(25); //ラッチクロックピンを出力モードに指定
  pinOut(26);
  pinOut(27);
  pinOut(14);  
  pinInput(13); //cycleリセットスイッチ
  
  WiFi.begin(ssid, password); 
 
  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");



}

void loop() {
  t++;
  
  if(digitalRead(13) == LOW)
  {
    delay(2);
    cycles = 0;
    prev_state = 0;
    p_round = 0;
    n_round = 0;
    onlyOnce = true;
  }

  calcRotation();
  fullNumber_print();


  Serial.print("p_round : ");
  Serial.print(p_round);
  Serial.print("n_round : ");
  Serial.print(n_round);
  Serial.print("cycles : ");
  Serial.println(cycles);
}


//加速度、ジャイロから角度を計算
void calcRotation(){

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  //レジスタアドレス0x3Bから、計14バイト分のデータを出力するようMPU6050へ指示
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  //出力されたデータを読み込み、ビットシフト演算
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  //単位Gへ変換
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;
  
  //加速度センサーから角度を算出
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  dpsX = ((float)raw_gyro_x) / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
  dpsY = ((float)raw_gyro_y) / 65.5;
  dpsZ = ((float)raw_gyro_z) / 65.5;
  
  //前回計算した時から今までの経過時間を算出
  interval = millis() - preInterval;
  preInterval = millis();
  
  //数値積分
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);
  
  //相補フィルター
  angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
  angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
  angleZ = gyro_angle_z;

//  if(angleZ > 360 || angleZ < -360 )angleZ = 0;

  // 正転逆転を加算
  if( angleZ > 270 ){ p_round++; angleZ = 0;}
  else if( angleZ < -270 ){ n_round++; angleZ = 0;}

  // 初期状態をセット


  if(prev_state == 0) // 正転逆転のセット
  {
    if(p_round >= 1)
    {
      if(onlyOnce)
      {
        prev_state = 1; // 変数を正転にセット
        onlyOnce = false;
      }
    }
    else if(n_round >=1)
    {
      if(onlyOnce)
      {
        prev_state = 2; // 変数を逆転にセット
        onlyOnce = false;
      }
    }    
  }

  switch(prev_state){
    case 1: // 正転の場合、正転を一回もしていない状態で逆転を何度してもカウントされない。
          if(p_round == 0 && n_round >= 1){ n_round = 0; p_round = 0; angleZ = 0;}
          break;
    case 2: // 逆転の場合、逆転を一回もしていない状態で正転を何度してもカウントされない。
          if(n_round == 0 && p_round >= 1){ p_round = 0; n_round = 0; angleZ = 0;}
          break;
    default:
          break;
          
  }

  if( p_round >= 1 && n_round >= 1 )
  {
    cycles++;
    p_round = 0;
    n_round = 0;
    angleZ = 0;
  }


  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;

  
 if(WiFi.status()== WL_CONNECTED && t>1000){   //Check WiFi connection status
 
   HTTPClient http;   
 
   http.begin("http://192.168.0.15/esp32/post0117/index.php");  //Specify destination for HTTP request. POSTするurl
   http.addHeader("Content-Type", "application/x-www-form-urlencoded");             //Specify content-type header

   // ここでは３つのデータを送っている。
   // esp = esp &
   // username = ゴミムシ & //文字列だけどクオーテーションいらない？
   // times = %d
   sprintf(s, "esp=esp&username=ゴミムシ&times=%d", cycles);
   // sprintf(s, "esp = %s & username = %s & times = %d", 'esp', 'username', times);
   int httpResponseCode = http.POST(s);   //Send the actual POST request
 
   if(httpResponseCode>0){

    /*
    String response = http.getString();                       //Get the response to the request
 
    Serial.println(httpResponseCode);   //Print return code
    Serial.println(response);           //Print request answer
    */

   }else{
 
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
 
   }
 
   http.end();  //Free resources
   t = 0;
 
 }else{
 
    Serial.println("Error in WiFi connection");   
 
 }

}

