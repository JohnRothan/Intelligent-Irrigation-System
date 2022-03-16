//头文件引用
#include <Gizwits.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <dht11.h> 
dht11 DHT11;  

//变量定义
bool varR_switch_water = 0;
bool varR_light = 0;
unsigned long varR_switch_RGB = 18;
long varW_temperature = 0;//Add Sensor Data Collection
unsigned long varW_humidity_air = 0;//Add Sensor Data Collection
unsigned long varW_humidity_soil =11;//Add Sensor Data Collection
unsigned long varW_illumination = 0;//Add Sensor Data Collection
unsigned long varR_switch_mode = 0;
int val;
unsigned long Last_KeyTime = 0;

//接口宏定义
Gizwits myGizwits;
#define   ledPin            13
#define   KEY1              6
#define   KEY2              7

#define   KEY1_SHORT_PRESS  1
#define   KEY1_LONG_PRESS   2
#define   KEY2_SHORT_PRESS  4
#define   KEY2_LONG_PRESS   8
#define   NO_KEY            0
#define   KEY_LONG_TIMER    3

#define   RPIN              11
#define   GPIN              10
#define   BPIN              9
 
#define DHT11PIN            2  
#define Moisture            A0 
#define LIGHT               A1

#define WATPOW1             3
#define WATPOW2             4
#define WATPOW3             5

#define ILLULEVEL           300
#define SOILLEVEL           10

#define I2C_ADDR                    0x0f   //语音识别模块地址
#define ASR_ADD_WORD_ADDR           0x01   //词条添加地址
#define ASR_MODE_ADDR               0x02   //识别模式设置地址，值为0-2，0:循环识别模式 1:口令模式 ,2:按键模式，默认为循环检测
#define ASR_RGB_ADDR                0x03   //RGB灯设置地址,需要发两位，第一个直接为灯号1：蓝 2:红 3：绿 ,                                           
#define ASR_REC_GAIN                0x04   //识别灵敏度设置地址，灵敏度可设置为0x00-0x55，值越高越容易检测但是越容易误判，                                       
#define ASR_CLEAR_ADDR              0x05   //清除掉电缓存操作地址，录入信息前均要清除下缓存区信息
#define ASR_KEY_FLAG                0x06  //用于按键模式下，设置启动识别模式
#define ASR_VOICE_FLAG              0x07   //用于设置是否开启识别结果提示音
#define ASR_RESULT                  0x08  //识别结果存放地址
#define ASR_BUZZER                  0x09 //蜂鸣器控制写1开启，写0关闭
#define ASR_NUM_CLECK               0x0a //录入词条数目校验

/****************************
      寄存器设置函数  
*****************************/
bool I2CWrite(unsigned char reg_addr,unsigned char date)
{
    Wire.beginTransmission(I2C_ADDR);  //发送Device地址
    Wire.write(reg_addr);              //发送要操作的寄存器地址 
    Wire.write(date);                  //发送要设置的值
    if(Wire.endTransmission()!=0)      //发送结束信号
      {
          delay(10);
          return false;
      }
      delay(10);
      return true;  
}

/****************************
      检测值读取函数  
*****************************/

bool WireReadData(unsigned char reg_addr,unsigned char *value,int num)
{   
    Wire.beginTransmission(I2C_ADDR);  //发送Device地址
    Wire.write(reg_addr);              //发送要操作的寄存器地址  
    delay(10);
    if(Wire.endTransmission()!=0)            //发送结束信号
     {
          delay(10);
          return false;
     }
      delay(10);


    Wire.requestFrom(I2C_ADDR, num);

    while(Wire.available())
    {
        char ff = Wire.read();    // receive a byte as character
        *value = ff;
        value++;
        delay(10);
    } 
        
     return true; 
 }

/*****************************
       RGB设置函数
******************************/
bool RGB_Set(unsigned char R,unsigned char G,unsigned char B)
{
      Wire.beginTransmission(I2C_ADDR);  //发送Device地址
      Wire.write(ASR_RGB_ADDR); 
      Wire.write(R);
      Wire.write(G);      
      Wire.write(B);
      if(Wire.endTransmission()!=0)            //发送结束信号
      {
          delay(10);
          return false;
      }
      delay(10);
      return true;
}


/*****************************
       添加词条函数
******************************/
bool AsrAddWords(unsigned char idNum,unsigned char * words)
{
      Wire.beginTransmission(I2C_ADDR);  //发送Device地址
      Wire.write(ASR_ADD_WORD_ADDR);     //发送存放词条寄存器地址
      Wire.write(idNum);                 //发送词条对应的识别号
      Wire.write(words,strlen(words));    
      if(Wire.endTransmission()!=0)            //发送结束信号
      {
          delay(10);
          return false;
      }
      delay(10);
      return true;
}

//配网模式
unsigned long gokit_time_s(void)
{
  return millis() / 1000;
}

char gokit_key1down(void)
{
  unsigned long keep_time = 0;
  if (digitalRead(KEY1) == LOW)
  {
    delay(100);
    if (digitalRead(KEY1) == LOW)

    {
      keep_time = gokit_time_s();
      while (digitalRead(KEY1) == LOW)
      {
        if ((gokit_time_s() - keep_time) > KEY_LONG_TIMER)
        {
          Last_KeyTime = gokit_time_s();
          return KEY1_LONG_PRESS;
        }
      } //until open the key

      if ((gokit_time_s() - Last_KeyTime) > KEY_LONG_TIMER)
      {
        return KEY1_SHORT_PRESS;
      }
      return 0;
    }
    return 0;
  }
  return 0;
}

char gokit_key2down(void)
{
  unsigned long keep_time = 0;
  if (digitalRead(KEY2) == LOW)
  {
    delay(100);
    if (digitalRead(KEY2) == LOW)
    {
      keep_time = gokit_time_s();
      while (digitalRead(KEY2) == LOW) //until open the key
      {

        if ((gokit_time_s() - keep_time) > KEY_LONG_TIMER)
        {
          Last_KeyTime = gokit_time_s();
          return KEY2_LONG_PRESS;
        }
      }

      if ((gokit_time_s() - Last_KeyTime) > KEY_LONG_TIMER)
      {
        return KEY2_SHORT_PRESS;
      }
      return 0;
    }
    return 0;
  }
  return 0;
}

char gokit_keydown(void)
{
  char ret = 0;
  ret |= gokit_key2down();
  ret |= gokit_key1down();
  return ret;

}

/**
* KEY_Handle 
* @param none
* @return none
*/
void KEY_Handle(void)
{
  /*  Press for over than 3 second is Long Press  */
  switch (gokit_keydown())
  {
    case KEY1_SHORT_PRESS:
      //mySerial.println(F("KEY1_SHORT_PRESS , Production Test Mode "));
      myGizwits.setBindMode(WIFI_PRODUCTION_TEST);
      break;
    case KEY1_LONG_PRESS:
      //mySerial.println(F("KEY1_LONG_PRESS ,Wifi Reset"));
      myGizwits.setBindMode(WIFI_RESET_MODE);
      break;
    case KEY2_SHORT_PRESS:
      //mySerial.println(F("KEY2_SHORT_PRESS Soft AP mode"));
      myGizwits.setBindMode(WIFI_SOFTAP_MODE);
      //Soft AP mode
      break;
    case KEY2_LONG_PRESS:
      //mySerial.println(F("KEY2_LONG_PRESS ,AirLink mode"));
      myGizwits.setBindMode(WIFI_AIRLINK_MODE);

      //AirLink mode
      break;
    default:
      break;
  }
}

/**
* Serial Init , Gizwits Init  
* @param none
* @return none
*/
void setup() {
  Serial.begin(9600);
//水泵接口
  pinMode(ledPin, OUTPUT);      
  pinMode(KEY1, INPUT_PULLUP);
  pinMode(KEY2, INPUT_PULLUP);

//RGB模组
  pinMode(RPIN,OUTPUT);
  pinMode(GPIN,OUTPUT);
  pinMode(BPIN,OUTPUT);

//dht
 pinMode(DHT11PIN,OUTPUT);  
//humi
 pinMode(Moisture,INPUT);  
//light
 pinMode(LIGHT,INPUT);

 pinMode(WATPOW1,OUTPUT);
 pinMode(WATPOW2,OUTPUT);
 pinMode(WATPOW3,OUTPUT);
 digitalWrite(WATPOW3,HIGH);
 unsigned char cleck = 0xff;
 Wire.begin();
 Wire.setClock(100000);

 myGizwits.begin();
/***************************************************************************************
录入词条和模式可以掉电保存，录入一次后下次如果无需修改可以将1设置为0，为0时这段程序不折行
****************************************************************************************/

#if 0
    I2CWrite(ASR_CLEAR_ADDR,0x40);//清除掉电保存区,录入前需要清除掉电保存区
    delay(5000);//清楚掉电保存区需要延时一段时间
    I2CWrite(ASR_MODE_ADDR,0);//设置检测模式               
    AsrAddWords(5,"kai shui");
    AsrAddWords(6,"guan shui");
    while(cleck != 6)
    {
      WireReadData(ASR_NUM_CLECK,&cleck,1);
      
      delay(100);
      }     
#endif
 
    I2CWrite(ASR_REC_GAIN,0x40);  //识别的灵敏度，建议0x40-0x55
    I2CWrite(ASR_VOICE_FLAG,1);  //识别结果提示音开关设置
    RGB_Set(255,255,255);        //设置模块的RGB灯为白色
}

/**
* Wifi status printf  
* @param none
* @return none
*/
void wifiStatusHandle()
{
  /*
  if(myGizwits.wifiHasBeenSet(WIFI_SOFTAP))
  {
    mySerial.println(F("WIFI_SOFTAP!"));
  }
  
  if(myGizwits.wifiHasBeenSet(WIFI_AIRLINK))
  {
    mySerial.println(F("WIFI_AIRLINK!"));
  }
  
  if(myGizwits.wifiHasBeenSet(WIFI_STATION))
  {
    mySerial.println(F("WIFI_STATION!"));
  }
  */
  if(myGizwits.wifiHasBeenSet(WIFI_CON_ROUTER))
  {
    digitalWrite(ledPin, HIGH); 
  }
}

//control RGB
  void sColor(int red,int green,int blue)
  {
  analogWrite(RPIN,red);
  analogWrite(GPIN,green);
  analogWrite(BPIN,blue);
  }
  
/**
* Arduino loop 
* @param none
* @return none
*/
void loop() { 
  //Configure network
  //if(XXX) //Trigger Condition
  //myGizwits.setBindMode(0x01);  //0x01:Enter AP Mode;0x02:Enter Airlink Mode
   
  
  myGizwits.write(VALUE_temperature, varW_temperature);

  myGizwits.write(VALUE_humidity_air, varW_humidity_air);

  myGizwits.write(VALUE_humidity_soil, varW_humidity_soil);

  myGizwits.write(VALUE_illumination, varW_illumination);


//control water

  if(myGizwits.hasBeenSet(EVENT_switch_mode))
  {
    myGizwits.read(EVENT_switch_mode,&varR_switch_mode);
  switch(varR_switch_mode)
  {
    case 0:
    if(myGizwits.hasBeenSet(EVENT_switch_water))
   {
     myGizwits.read(EVENT_switch_water,&varR_switch_water);
   }
   if(varR_switch_water==1)
  {
    digitalWrite(WATPOW3,LOW);  
    delay(100);
    analogWrite(WATPOW1,255);
    digitalWrite(WATPOW2,LOW);
  }
  else
  {
     digitalWrite(WATPOW3, HIGH); // sets the user LED off
      digitalWrite(WATPOW1, LOW);
  }
    break;
    case 1:
    unsigned char result;
    WireReadData(ASR_RESULT,&result,1);
    if(result == 5)
  {
    digitalWrite(WATPOW3,LOW);  
    delay(100);
    analogWrite(WATPOW1,255);
    digitalWrite(WATPOW2,LOW);
  }
  else if(result == 6)
        {
         digitalWrite(WATPOW3, HIGH);
        digitalWrite(WATPOW1, LOW);
        } 
    break;
    default:
   {
     digitalWrite(WATPOW3, HIGH); 
      digitalWrite(WATPOW1, LOW);
  }
    break;
  }

       
//control Light
 
  if(myGizwits.hasBeenSet(EVENT_light))
  {
    myGizwits.read(EVENT_light,&varR_light);
  }
  
  if(myGizwits.hasBeenSet(EVENT_switch_RGB))
  {
    myGizwits.read(EVENT_switch_RGB,&varR_switch_RGB);//Address for storing data,read rgb data from gizwits to arduino
    //mySerial.println(F("EVENT_switch_RGB"));
    //mySerial.println(varR_switch_RGB,DEC);

  }

  if(varR_light==1)
  {
    switch(varR_switch_RGB)
    {
     case 0:
     sColor(255,255,240);
     break;
     case 1:
     sColor(255,0,0);
     break;
     case 2:
     sColor(0,255,0);
     break;
     case 3:
     sColor(0,0,255);
     break;
     default:
     sColor(0,0,0);
     break;
    }
  }
  else
  {
      if(varW_illumination>ILLULEVEL)
      {
        sColor(0,0,0);
       
        
      }
      else
      {
        sColor(255-varW_illumination/500.0*255,255-varW_illumination/500.0*255,255-varW_illumination/500.0*255);
        
      }
      
    
  }

 //dht
  DHT11.read(DHT11PIN);                 //将读取到的值赋给chk
  varW_temperature = (float)DHT11.temperature;               //将温度值赋值给tem
  varW_humidity_air = (float)DHT11.humidity-245;                   //将湿度值赋给hum

  //humi
  varW_humidity_soil = (1023-analogRead(Moisture))/1023.0*100;

  //light
  varW_illumination =(1024-analogRead(LIGHT))/1023.0*500;
  
  //binary datapoint handle
  
  KEY_Handle();//key handle , network configure
  wifiStatusHandle();//WIFI Status Handle
  myGizwits.process();
}
