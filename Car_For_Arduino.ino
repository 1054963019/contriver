#include <SSD1306.h>
#include <Servo.h>
#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#include <PS2X_lib.h>       //PS2手柄
DATASCOPE data;//实例化一个 上位机 对象，对象名称为 data
Servo myservo;  //创建一个舵机控制对象
PS2X ps2x; // create PS2 Controller Class
//////////PS2引脚//////////////////
#define PS2_DAT        17  //14
#define PS2_CMD        0  //15
#define PS2_SEL        16   //16
#define PS2_CLK        15  //17
////////OLED显示屏引脚///////////
#define OLED_DC 10
#define OLED_CLK 19
#define OLED_MOSI 13
#define OLED_RESET 12
/////////TB6612驱动引脚////
#define AIN1 11
#define AIN2 5
#define BIN1 6
#define BIN2 3
#define SERVO 9
/////////编码器引脚////////
#define ENCODER_L 8  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 4
#define ENCODER_R 7
#define DIRECTION_R 2
/////////按键引脚////////
#define KEY 18
#define T 0.156f
#define L 0.1445f
#define pi 3.1415926
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);
long Sensor_Left, Sensor_Middle, Sensor_Right, Sensor; //电磁巡线相关
volatile long Velocity_L, Velocity_R ;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0, Velocity, Angle;   //左右轮速度
char Flag_Direction, Flag_Way = 1;
float Velocity_KP = 0.3, Velocity_KI =  0.3;
unsigned char Flag_Stop = 0, PID_Send, Flash_Send, Bluetooth_Velocity = 15; //停止标志位和上位机相关变量
float Target_A, Target_B;
int Battery_Voltage, ps2_data; //电池电压采样变量
unsigned char servo,PS2_LY, PS2_RX, CCD_Yuzhi,CCD_Zhongzhi;
unsigned char ADV[128] = {0};
void (* resetFunc) (void) = 0;// Reset func
/************函数功能：线性CCD取中值***************/
void  Find_CCD_Zhongzhi(void) {
      static unsigned int i,j,Left,Right,Last_CCD_Zhongzhi,value1_max,value1_min;
     value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
     value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;   //计算出本次中线提取的阈值
   for(i = 5;i<118; i++)   //寻找左边跳变沿
  {
    if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
    { 
      Left=i;
      break;  
    }
  }
   for(j = 118;j>5; j--)//寻找右边跳变沿
  {
    if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
    { 
      Right=j;
      break;  
    }
  }
  CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
// if((CCD_Zhongzhi-Last_CCD_Zhongzhi)>90||(CCD_Zhongzhi-Last_CCD_Zhongzhi)<-90)   //计算中线的偏差，如果太大
//  CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
//  Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}
void Dly_us(void) {
  for (char ii = 0; ii < 10; ii++);
}
/*****函数功能：CCD数据采集*********/
void RD_TSL(void) {
  unsigned char i = 0, tslp = 0;
  digitalWrite(16, HIGH);  digitalWrite(17, LOW); Dly_us();
  digitalWrite(17, HIGH);  digitalWrite(16, LOW);  Dly_us();
  digitalWrite(16, HIGH);  digitalWrite(17, LOW);  Dly_us();
  for (i = 0; i < 128; i++)  {
    digitalWrite(16, LOW);
    Dly_us();  //调节曝光时间
    Dly_us();  //调节曝光时间
   ADV[tslp++] = analogRead(1) >> 2;
    digitalWrite(16, HIGH);
    Dly_us();
  }
}
/******函数功能：选择运行的模式************************************/
unsigned char  select(void) {
  int count,AngleX = 130;
  static unsigned char flag = 1;
  oled_show_once();  //OLED显示
  count = abs(Velocity_R);
  if (count <= AngleX)                       Flag_Way = 0; //APP遥控模式
  else if (count > AngleX && count <= 2 * AngleX)   Flag_Way = 1; //PS2遥控模式
  else if (count > 2 * AngleX && count <= 3 * AngleX) Flag_Way = 2; //CCD巡线模式
  else if (count > 3 * AngleX && count <= 4 * AngleX) Flag_Way = 3; //电磁巡线模式
  else Velocity_R = 0;
  if (digitalRead(KEY) == 0)oled.clear(), flag = 0; //清除OLED屏幕 程序往下运行
  return flag;
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
unsigned char My_click (void) {
    static byte flag_key = 1; //按键按松开标志
    if (flag_key && (digitalRead(KEY) == 0))   { //如果发生单击事件
    flag_key = 0;
    if (digitalRead(KEY) == 0)  return 1;    //M键
  }
  else if (digitalRead(KEY) == 1)     flag_key = 1;
  return 0;//无按键按下
}
/**************************************************************************
函数功能：求次方的函数
入口参数：m,n
返回  值：m的n次幂
**************************************************************************/
uint32_t oled_pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--)result *= m;
  return result;
}
/**************************************************************************
函数功能：显示变量
入口参数：x:x坐标   y:行     num：显示的变量   len ：变量的长度
**************************************************************************/
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len) {
  uint8_t t, temp;
  uint8_t enshow = 0;
  for (t = 0; t < len; t++)  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    oled.drawchar(x + 6 * t, y, temp + '0');
  }
}
/**************************************************************************
函数功能：赋值给PWM寄存器 作者：平衡小车之家
入口参数：PWM
**************************************************************************/
void Set_Pwm(int motora, int motorb) {
  if (motora > 0)       analogWrite(AIN2, motora), digitalWrite(AIN1, LOW); //赋值给PWM寄存器
  else                 digitalWrite(AIN1, HIGH), analogWrite(AIN2, 255 + motora); //赋值给PWM寄存器

  if (motorb > 0)        digitalWrite(BIN2, LOW), analogWrite(BIN1, motorb); //赋值给PWM寄存器
  else                  analogWrite(BIN1,255 +motorb), digitalWrite(BIN2, HIGH); //赋值给PWM寄存器
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
/**************************************************************************/
unsigned char  Turn_Off() {
  byte temp;
  if (Flag_Stop == 1 || Battery_Voltage < 700) { //Flag_Stop置1或者电压太低关闭电机
    temp = 1;
    digitalWrite(AIN1, LOW);  //电机驱动的电平控制
    digitalWrite(AIN2, LOW);  //电机驱动的电平控制
    digitalWrite(BIN1, LOW);  //电机驱动的电平控制
    digitalWrite(BIN2, LOW);  //电机驱动的电平控制
  }
  else      temp = 0;
  return temp;
}
/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
//**************************************************************************/
void Kinematic_Analysis(float velocity, float angle) {
  char K = 1;
  Target_A = velocity * (1 + T * tan(angle * pi / 180) / 2 / L);
  Target_B = velocity * (1 - T * tan(angle * pi / 180) / 2 / L); //后轮差速
  servo = 95 + angle * K;                      //舵机转向
//  if(servo>95)servo=servo*1.15;
  myservo.write(servo);                        // 指定舵机转向的角度
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{   
   static float Bias,Pwm,Last_bias;
   Bias=Encoder-Target;                                  //计算偏差
   Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
   if(Pwm>255)Pwm=255;                                 //限幅
   if(Pwm<-255)Pwm=-255;                                 //限幅  
   Last_bias=Bias;                                       //保存上一次偏差 
   return Pwm;                                           //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{   
   static float Bias,Pwm,Last_bias;
   Bias=Encoder-Target;                                  //计算偏差
   Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
   if(Pwm>255)Pwm=255;                                 //限幅
    if(Pwm<-255)Pwm=-255;                                 //限幅  
   Last_bias=Bias;                                       //保存上一次偏差 
   return Pwm;                                           //增量输出
}
/*********函数功能：5ms控制函数 核心代码 作者：平衡小车之家*******/
void control() {
 int Temp, Temp2, Motora, Motorb; //临时变量
  static float Voltage_All; //电压采样相关变量
  static unsigned char Position_Count,Voltage_Count;  //位置控制分频用的变量
  sei();//全局中断开启
  Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
  Get_RC();
  Kinematic_Analysis(Velocity, Angle);
  Motora = Incremental_PI_A(Target_A, Velocity_Left); //===速度PI控制器
  Motorb = Incremental_PI_B(Target_B, Velocity_Right); //===速度PI控制器
 if (Turn_Off() == 0) Set_Pwm(Motora, Motorb); //如果不存在异常，使能电机
  Temp2 = analogRead(0);  //采集一下电池电压
  Voltage_Count++;       //平均值计数器
  Voltage_All += Temp2;   //多次采样累积
  if (Voltage_Count == 200) Battery_Voltage = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0; //求平均值
 Temp = My_click();   //按键检查
 if (Temp == 1)Flag_Stop = !Flag_Stop;
}
/***************函数功能：遥控**********/
void Get_RC(void){
  char Yuzhi = 2;
  static float Last_Bias;
  float Bias,LY, RX;
  if (Flag_Way == 0)   {//蓝牙控制
    if (Flag_Direction == 0) Velocity = 0, Angle = 0; //停止
    else if (Flag_Direction == 1) Velocity = Bluetooth_Velocity, Angle = 0; //前进
    else if (Flag_Direction == 2) Velocity = Bluetooth_Velocity, Angle = 36; //右前
    else if (Flag_Direction == 3) Velocity = 0, Angle = 0; //舵机向右
    else if (Flag_Direction == 4) Velocity = -Bluetooth_Velocity, Angle = 36; // 右后
    else if (Flag_Direction == 5) Velocity = -Bluetooth_Velocity, Angle = 0; //后退
    else if (Flag_Direction == 6) Velocity = -Bluetooth_Velocity, Angle = -36; //左后
    else if (Flag_Direction == 7) Velocity = 0, Angle = 0;               //舵机向左
    else if (Flag_Direction == 8) Velocity = Bluetooth_Velocity, Angle = -36; //左前
  }
  else  if (Flag_Way == 1)   {//PS2控制
    LY = PS2_LY - 128; //计算偏差
    RX = PS2_RX - 128;
    if (LY > -Yuzhi && LY < Yuzhi)LY = 0; //小角度设为死区 防止抖动出现异常
    if (RX > -Yuzhi && RX < Yuzhi)RX = 0;
    Velocity = -LY / 5; //速度和摇杆的力度相关。
    Angle = RX / 4;
  }
  else  if (Flag_Way == 2)  { //CCD巡线
    RD_TSL();
    Find_CCD_Zhongzhi();
    Velocity = 28;  //CCD巡线模式的速度
    Bias = CCD_Zhongzhi - 64; //提取偏差
    Angle = Bias * 0.6 + (Bias - Last_Bias) * 3; //PD控制
    Last_Bias = Bias; //保存上一次的偏差
  }
    else  if (Flag_Way == 3)   {        //电磁巡线
    Sensor_Left =   analogRead(1);    // delay(2);
    Sensor_Middle = analogRead(2);    // delay(1);
    Sensor_Right =  analogRead(3);
    if ((Sensor_Left + Sensor_Middle + Sensor_Right )> 100)    {
      Sensor=( Sensor_Left * 1 + Sensor_Middle * 50 + Sensor_Right * 99)/ (Sensor_Left + Sensor_Middle + Sensor_Right); //归一化处理
    }
    Velocity = 18; //电磁巡线模式下的速度
    Bias = Sensor - 50; //提取偏差
    Angle = abs(Bias)*Bias * 0.04 + Bias * 0.1 + (Bias - Last_Bias) * 1; //
    Last_Bias = Bias; //上一次的偏差
  }
  if (Angle < -45)Angle = -45;//////舵机角度限制
  if (Angle > 45)Angle = 45;//舵机角度限制
}
/********函数功能：OLED显示*********/
void OLED()
{
  char  i, t;
  if(Flag_Way==0)   {
      oled.drawstring(00, 00, "SPEED");
      OLED_ShowNumber(45,00, Bluetooth_Velocity,3);  //PS2的数据 
      oled.drawstring(00,01,"RX");
      OLED_ShowNumber(30,01, Flag_Direction,3);  //PS2的数据      
    }
    else if(Flag_Way==1)
    {
      oled.drawstring(00,0,"LY");
      OLED_ShowNumber(15,0, PS2_LY,3);  //PS2的数据
      oled.drawstring(40,0,"RX");
      OLED_ShowNumber(55,0, PS2_RX,3);
    }
    else if(Flag_Way==2)
    {
      OLED_Show_CCD(); 
      oled.drawstring(00,01,"Z");
      OLED_ShowNumber(35,01, CCD_Zhongzhi,3);
      oled.drawstring(70,01,"Y");
      OLED_ShowNumber(95,01, CCD_Yuzhi,3);
    }
    else if(Flag_Way==3)
    {
      oled.drawstring(00,0,"L");
      OLED_ShowNumber(10,0,Sensor_Left,4); 
      oled.drawstring(40,0,"M");
      OLED_ShowNumber(50,0,Sensor_Middle,4);
      oled.drawstring(80,0,"R");
      OLED_ShowNumber(90,0,Sensor_Right,4);
      oled.drawstring(0,01,"Z");
      OLED_ShowNumber(20,01,Sensor,3);     
    }                 
                          oled.drawstring(00,02,"EncoLEFT");    //编码器数据
    if( Velocity_Left<0)  oled.drawstring(80,02,"-"),
                          OLED_ShowNumber(95,02,-Velocity_Left,3);
    else                  oled.drawstring(80,02,"+"),
                          OLED_ShowNumber(95,02, Velocity_Left,3);
                          
                          oled.drawstring(00,03,"EncoRIGHT");
    if(Velocity_Right<0)  oled.drawstring(80,03,"-"),
                          OLED_ShowNumber(95,03,-Velocity_Right,3);
     else                 oled.drawstring(80,03,"+"),
                          OLED_ShowNumber(95,03,Velocity_Right,3);  
                          oled.drawstring(00, 4, "VOLTAGE:");
                          oled.drawstring(71, 4, ".");
                          oled.drawstring(93, 4, "V");
                          OLED_ShowNumber(58, 4, Battery_Voltage / 100, 2);
                          OLED_ShowNumber(81, 4, Battery_Voltage % 100, 2);      
                          if(Flag_Stop==0)
                          oled.drawstring(103,04,"O-N");
                          if(Flag_Stop==1)
                          oled.drawstring(103,04,"OFF");
                          oled.drawstring(00,05,"MODE-");
                          if(Flag_Way==0)               oled.drawstring(40,05,"APP");
                          else if(Flag_Way==1)          oled.drawstring(40,05,"PS2");
                          else if(Flag_Way==2)          oled.drawstring(40,05,"CCD");
                          else if(Flag_Way==3)          oled.drawstring(40,05,"ELE");

                          oled.drawstring(80,05,"S");
                          OLED_ShowNumber(95,05,servo,3);
                          oled.display();
}
//开机显示一次的内容
void OLED_Show_CCD(void)  {
  char t;
  for ( unsigned char j = 0; j < 128; j++)   {
    if (ADV[j] > CCD_Yuzhi) t = 1; else t = 0;
    oled_show_shu(j, t);
  }
}
void oled_show_shu(unsigned char x, unsigned char t)  {
  for (unsigned char i = 0; i < 1; i++)    {
    if (t == 1)  oled.setpixel(x, i, WHITE);
    else oled.setpixel(x, i, 0);
  }
}
//开机显示一次的内容
void oled_show_once(void) {
  oled.drawstring(0, 0, "Turn Right Wheel");
  oled.drawstring(0, 1, "TO Select Mode");
  oled.drawstring(0, 2, "Current Mode Is");
  if (Flag_Way == 0)         oled.drawstring(50, 3, "APP");
  if (Flag_Way == 1)         oled.drawstring(50, 3, "PS2");
  if (Flag_Way == 2)         oled.drawstring(50, 3, "CCD");
  if (Flag_Way == 3)         oled.drawstring(50, 3, "ELE");
  oled.drawstring(0, 4, "Press User Key");
  oled.drawstring(0, 5, "TO End Selection");
  OLED_ShowNumber(0, 6, abs(Velocity_R), 4);
  oled.display();
}
/***********函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家************/
void setup()   {
  char error;
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear();   // clears the screen and buffer
  pinMode(AIN1, OUTPUT);          //电机控制引脚
  pinMode(AIN2, OUTPUT);          //电机控制引脚，
  pinMode(BIN1, OUTPUT);          //电机速度控制引脚
  pinMode(BIN2, OUTPUT);          //电机速度控制引脚
  myservo.attach(SERVO);           // 选择控制的舵机

  pinMode(ENCODER_L, INPUT);       //编码器引脚
  pinMode(DIRECTION_L, INPUT);       //编码器引脚
  pinMode(ENCODER_R, INPUT);        //编码器引脚
  pinMode(DIRECTION_R, INPUT);       //编码器引脚
  pinMode(KEY, INPUT);       //按键引脚
  delay(200);                      //延时等待初始化完成
  attachInterrupt(0, READ_ENCODER_R, CHANGE);           //开启外部中断 编码器接口1
  attachPinChangeInterrupt(4, READ_ENCODER_L, CHANGE);  //开启外部中断 编码器接口2
  while (select())  { }            //=====选择运行模式
  MsTimer2::set(10, control);       //使用Timer2设置5ms定时中断
  MsTimer2::start();               //中断使能
  if (Flag_Way == 0)    Serial.begin(9600);            //蓝牙控制    //开启串口
 else  if (Flag_Way == 1)  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);//PS2控制
  else  if (Flag_Way == 2)   pinMode(16, OUTPUT), pinMode(17, OUTPUT);          //CCD引脚初始化
}
/******函数功能：主循环程序体*******/
void loop(){
     static char flag;
      int Voltage_Temp;
     OLED();
     if(Flag_Way==0)     {
       flag=!flag;
       Voltage_Temp = (Battery_Voltage - 740) ;  //根据APP的协议对电池电压变量进行处理
      if (Voltage_Temp > 100)Voltage_Temp = 100;
      if (Voltage_Temp < 0)Voltage_Temp = 0;
      if(PID_Send==1) {//发送参数          
        Serial.print("{C");
        Serial.print(Bluetooth_Velocity);   //速度
        Serial.print(":");
        Serial.print((int)(Velocity_KP*100));  //电池电压
        Serial.print(":");
        Serial.print((int)(Velocity_KI*100));  //平衡倾角
        Serial.print("}$");
        PID_Send=0; 
      } 
     else if(flag==0) {
      Serial.print("{A");
      Serial.print(abs(Velocity_Left));   //左轮编码器
      Serial.print(":");
      Serial.print(abs(Velocity_Right ));  //右轮编码器
      Serial.print(":");
      Serial.print(Voltage_Temp);  //电池电压
      Serial.print(":");
      Serial.print(servo-90);  //舵机倾角
      Serial.print("}$");
      }
      else {
      Serial.print("{B");
      Serial.print(servo);    //舵机倾角
      Serial.print(":");
      Serial.print(Voltage_Temp);  
      Serial.print(":");
      Serial.print(Velocity_Left); 
      Serial.print(":");
      Serial.print(Velocity_Right); 
      Serial.print("}$");
          }
     }
     else  if (Flag_Way == 1)    {
    ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed
    PS2_LY=ps2x.Analog(PSS_LY);
    PS2_RX=ps2x.Analog(PSS_RX);
   }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++;//根据另外一相电平判定方向
    else      Velocity_R--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--; //根据另外一相电平判定方向
    else     Velocity_R++;
  }
}
/****函数功能：串口接收中断******/
void serialEvent()  {
  static unsigned char Flag_PID, Receive[10], Receive_Data, i, j;
  static float Data;
  while (Serial.available()) {
    Receive_Data = Serial.read();
    if (Receive_Data >= 0x41 && Receive_Data <= 0x48)        Flag_Direction = Receive_Data - 0x40;
    else  if (Receive_Data < 10)        Flag_Direction = Receive_Data;
    else  if (Receive_Data == 0X5A)        Flag_Direction = 0;
    if (Receive_Data == 0x7B) Flag_PID = 1; //参数指令起始位
    if (Receive_Data == 0x7D) Flag_PID = 2; //参数指令停止位
    if (Flag_PID == 1)      Receive[i] = Receive_Data,        i++;
    else  if (Flag_PID == 2)  { //执行指令
      if (Receive[3] == 0x50)          PID_Send = 1; //获取PID参数
   //   else  if (Receive[3] == 0x57)    Flash_Send = 1; //掉电保存参数
      else  if (Receive[1] != 0x23)     { //更新PID参数
        for (j = i; j >= 4; j--)          {
          Data += (Receive[j - 1] - 48) * pow(10, i - j); //通讯协议
        }
        switch (Receive[1])
        {
          case 0x30:  Bluetooth_Velocity=Data;break;
          case 0x31:  Velocity_KP=Data/100;break;
          case 0x32:  Velocity_KI=Data/100;break;
          case 0x33:  break;
          case 0x34:  break; //9个通道
          case 0x35:  break;
          case 0x36:  break;
          case 0x37:  break;
          case 0x38:  break;
        }
      }
      Flag_PID = 0;       i = 0;       j = 0;      Data = 0; //相关标志位清零
    }
  }
}

