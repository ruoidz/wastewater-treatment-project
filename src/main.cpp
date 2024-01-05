//#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>
#include <EEPROM.h>
#include <SimpleKalmanFilter.h>
#include <FirebaseESP32.h>

// Provide the token generation process info.
#include <addons/TokenHelper.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>
#define WIFI_SSID "Yae Miko"
#define WIFI_PASSWORD "1234567890"
#define API_KEY "AIzaSyCWzZTdZLs7APp1ZehynD1gvEgOG3lh55Y"
/* 3. Define the RTDB URL */
#define DATABASE_URL "https://giam-sat-nuoc-bk-2023-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "USER_EMAIL"
#define USER_PASSWORD "USER_PASSWORD"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 10000;


SimpleKalmanFilter locnhieuph(1,1,0.01);
SimpleKalmanFilter locnhieuntu(1,1,0.01);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define i2c_Address 0x3c ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define TdsSensorPin 35
#define PH_PIN 36 // cảm biến PH
#define sensor_turbidity 39 // cảm biến độ đục
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
#define LedRed1 5
#define LedYellow1 18
#define LedGreen1 23
#define LedRed2 4
#define LedYellow2 16
#define LedGreen2 17
#define LedRed3 19
#define LedYellow3 15
#define LedGreen3 2
#define buzz 13// còi
#define button1 34
#define button2 25
#define button3 26
#define button4 27
#define button5 14
#define pump1 12// may bom 1
#define pump2 33// may bom2

int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue,temperature = 20;
float ntuValue,analog_ntu = 0,analog_ntu_filter=0;
int sensorValue;
int setpump=10000,timeLoop=30000,Millis=0; // thời gian bật máy bơm, thời gian lặp,millis bắt đầu bằng 0
float analogPH,analogPH_filter,pHValue,voltagePH;
// Hiệu chỉnh cảm biến pH bằng cách thay đổi biến acidVoltage và neutraVoltage
float acidVoltage = 1950;    // buffer solution 4.0 at 25C
float neutralVoltage = 1425; // buffer solution 7.0 at 25C
unsigned long int TimeSetData=0;
int status=0;
void display_oled();
void display_led();
void SensorTurbidity();
int getMedianNum(int bArray[], int iFilterLen);
void Sensor_TDS();
void Sensor_PH();
void setData();//day du lieu len app
void setDataDefault(); // day du lieu default len app
void run_pump();
void button();
void setled();

void setup()
{
  Serial.begin(9600);
  display.begin(i2c_Address, true);
  display.display();
  delay(2000);
  display.clearDisplay();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  display.println("connecting to wifi");
  display.display();
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  display.println(WiFi.localIP());
  display.display();
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
  config.api_key = API_KEY;
  /* Assign the user sign in credentials */
  // auth.user.email = USER_EMAIL;
  // auth.user.password = USER_PASSWORD;
  if(Firebase.signUp(&config,&auth,"",""))
  {
    Serial.println("ok");
    display.println("fire base: ok");
  }
  else{
    Serial.println("fail");
    display.println("fire base: fail");
  }
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);
  Firebase.begin(&config, &auth);
  Firebase.setDoubleDigits(5);

  display.setTextSize(1); // Draw 2X-scale text
  display.setCursor(0,10);
  display.setTextColor(SH110X_WHITE);
  pinMode(TdsSensorPin, INPUT);
  pinMode(sensor_turbidity,INPUT);
  pinMode(PH_PIN,INPUT);
  pinMode(LedGreen1,OUTPUT);
  pinMode(LedGreen2,OUTPUT);
  pinMode(LedGreen3,OUTPUT);
  pinMode(LedRed1,OUTPUT);
  pinMode(LedRed2,OUTPUT);
  pinMode(LedRed3,OUTPUT);
  pinMode(LedYellow1,OUTPUT);
  pinMode(LedYellow2,OUTPUT);
  pinMode(LedYellow3,OUTPUT);
  pinMode(pump1,OUTPUT);
  pinMode(pump2,OUTPUT);
  pinMode(button1,INPUT);
  pinMode(button2,INPUT);
  pinMode(button3,INPUT);
  pinMode(button4,INPUT);
  pinMode(button5,INPUT);
  pinMode(buzz,OUTPUT);
}

void loop()
{
  display.clearDisplay();
  // đọc giá trị cảm biến
  SensorTurbidity();
  Sensor_TDS();
  Sensor_PH();
  display_oled();// hiển thị chỉ số lên oled
  button();
  setData();
  setled();
  if(millis()-Millis>=timeLoop){ // vòng lặp sau timeLoop s
    run_pump();
  }
  display_oled();// hiển thị chỉ số lên oled
  delay(200);
  display_led(); // hiển thị đèn 
}
// function caculate value Turbidity
void SensorTurbidity()
{
  analog_ntu = analogRead(sensor_turbidity);// read the input on analog pin 0:
    //Serial.println(analog_ntu);
  int analog_ntu_map=map(analog_ntu,0,4095,0,1023);
  Serial.println(analog_ntu_map);
  int analog_adjust = map(analog_ntu_map,365,455,655,860);
  analog_ntu_filter=locnhieuntu.updateEstimate(analog_adjust);
  //Serial.println(analog_ntu_filter);
  float vol = analog_ntu_filter * (5 / 1024.0); // Convert the analog reading (which goes from 0 - 4095) to a voltage (0 - 3v3V):
  ntuValue= -1120.4*vol*vol + 5742.3*vol - 4352.9;
  if(ntuValue<0){
    ntuValue=0;
  }
}
// function support to caculate TDS
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
//funtion caculate for tds sensor
void Sensor_TDS()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096;                                                                                                    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                               // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                            // temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value
    if(tdsValue<=5.0)
    {tdsValue=5.0;
    }
  }
}
//function caculate pH
void Sensor_PH()
{
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        analogPH=analogRead(PH_PIN);
        analogPH_filter=locnhieuph.updateEstimate(analogPH);
        float slope = (7.0-4.0)/((neutralVoltage-1500.0)/3.0 - (acidVoltage-1500.0)/3.0);  // two point: (_neutralVoltage,7.0),(_acidVoltage,4.0)
        float intercept =  7.0 - slope*(neutralVoltage-1500.0)/3.0;
        voltagePH = analogPH_filter/4096.0*3300;  // read the voltage
        pHValue = slope*(voltagePH-1500.0)/3.0+intercept;
    }
}
//function display oled
void display_oled()
{
  display.setCursor(0, 10);
  display.print("TDS Value ");
  display.print(tdsValue);
  display.println("ppm");
  display.setCursor(0, 20);
  display.print("NTU: ");
  display.print(ntuValue);
  display.setCursor(0, 30);
  display.print("pH: ");
  display.print(pHValue);
  display.setCursor(0, 40);
  display.print("time pump: ");
  display.print(setpump/1000);
  display.print(" s");
  display.setCursor(0, 50);
  display.print("time loop: ");
  display.print(timeLoop/1000);
  display.print(" s");
  display.display();
  delay(200);
}
void display_led()
{
  if (tdsValue < 100)
  {
    digitalWrite(LedGreen1, HIGH);
    digitalWrite(LedYellow1, LOW);
    digitalWrite(LedRed1, LOW);
  }
  else if(tdsValue > 1000)
  {
    digitalWrite(LedGreen1, LOW);
    digitalWrite(LedYellow1, LOW);
    digitalWrite(LedRed1, HIGH);
  }
  else
  {
    digitalWrite(LedGreen1, LOW);
    digitalWrite(LedYellow1, HIGH);
    digitalWrite(LedRed1, LOW);
  }
  if (ntuValue < 100)
  {
    digitalWrite(LedGreen2, HIGH);
    digitalWrite(LedYellow2, LOW);
    digitalWrite(LedRed2, LOW);
  }
  else if (ntuValue > 1000)
  {
    digitalWrite(LedGreen2, LOW);
    digitalWrite(LedYellow2, LOW);
    digitalWrite(LedRed2, HIGH);
  }
  else
  {
    digitalWrite(LedGreen2, LOW);
    digitalWrite(LedYellow2, HIGH);
    digitalWrite(LedRed2, LOW);
  }
  if ((pHValue >= 3 && pHValue <=5 )||(pHValue>=9 && pHValue<=12))
  {
    digitalWrite(LedGreen3, LOW);
    digitalWrite(LedYellow3, HIGH);
    digitalWrite(LedRed3, LOW);
  }
  else if (pHValue > 12||pHValue<3)
  {
    digitalWrite(LedGreen3, LOW);
    digitalWrite(LedYellow3, LOW);
    digitalWrite(LedRed3, HIGH);
  }
  else
  {
    digitalWrite(LedGreen3, HIGH);
    digitalWrite(LedYellow3, LOW);
    digitalWrite(LedRed3, LOW);
  }
  // if(tds_filter>1000 || ntd_filter>1000 || (ph_filter > 10||ph_filter <4)) // 1 trong ba chỉ số xấu thì bật còi
  // {
  //   digitalWrite(buzz,HIGH);
  // }
  // else
  // digitalWrite(buzz,LOW);
}
void run_pump(){
  if (status==0){
    Serial.println("bom1 on");
    digitalWrite(pump1,HIGH);
    status=1;
  }
  if (millis() - Millis-timeLoop >= setpump && status==1) {  //sau a giay thi tat may bom, dong thoi đẩy dữ liệu lên app
    //digitalWrite(pump1, LOW);
    status=2;
    Serial.println("bom1 off");
    digitalWrite(pump1,LOW);
  }

  if (millis() - Millis-timeLoop >= setpump + 10000 && status==2)  // sau 10s thì gửi lên giá trị default
  {
    // setDataDefault();
    // Serial.println("set data default");
    status=3;
  }

  if (millis() - Millis-timeLoop >= setpump + 12000 && status==3)  // sau 12s thì bật máy bơm
  {
    //digitalWrite(pump2, HIGH);
    digitalWrite(pump2,HIGH);
    Serial.println("bom2 on");
    status=4;
  }

  if (millis() - Millis-timeLoop >= (setpump + 14000 + setpump) && status==4)  //máy bơm 2 hoạt động a giây rồi tắt
  {
    //digitalWrite(pump2, LOW);
    digitalWrite(pump2,LOW);
    Serial.println("bom2 off");
    status=0;
    Millis = millis();
  }
}
void setData()
{
  if (Firebase.ready())
  {
    Serial.println("ready");
    Serial.printf("Set TDS... %s\n", Firebase.setFloat(fbdo, F("/iot/tds"), tdsValue) ? "ok" : fbdo.errorReason().c_str());
    //Serial.printf("Get bool... %s\n", Firebase.getBool(fbdo, FPSTR("/test/bool")) ? fbdo.to<bool>() ? "true" : "false" : fbdo.errorReason().c_str());
    Serial.printf("Set NTU... %s\n", Firebase.setFloat(fbdo, F("/iot/ntu"), ntuValue) ? "ok" : fbdo.errorReason().c_str());
    //Serial.printf("Get int... %s\n", Firebase.getInt(fbdo, F("/test/int")) ? String(fbdo.to<int>()).c_str() : fbdo.errorReason().c_str());
    Serial.printf("Set pH... %s\n", Firebase.setFloat(fbdo, F("/iot/ph"), pHValue) ? "ok" : fbdo.errorReason().c_str());
    Serial.printf("Set temp... %s\n", Firebase.setFloat(fbdo, F("/iot/temp"), 20) ? "ok" : fbdo.errorReason().c_str());
    Serial.println();
  }
  else{
    Serial.println("firebase no ready");
  }
}
// void setDataDefault()
// {
//   if (Firebase.ready())
//   {
//     Serial.println("ready");
//     Serial.printf("Set TDS... %s\n", Firebase.setFloat(fbdo, F("/iot/tds"), 0) ? "ok" : fbdo.errorReason().c_str());
//     //Serial.printf("Get bool... %s\n", Firebase.getBool(fbdo, FPSTR("/test/bool")) ? fbdo.to<bool>() ? "true" : "false" : fbdo.errorReason().c_str());
//     Serial.printf("Set NTU... %s\n", Firebase.setFloat(fbdo, F("/iot/ntu"), 0) ? "ok" : fbdo.errorReason().c_str());
//     //Serial.printf("Get int... %s\n", Firebase.getInt(fbdo, F("/test/int")) ? String(fbdo.to<int>()).c_str() : fbdo.errorReason().c_str());
//     Serial.printf("Set pH... %s\n", Firebase.setFloat(fbdo, F("/iot/ph"), 7) ? "ok" : fbdo.errorReason().c_str());
//     Serial.printf("Set temp... %s\n", Firebase.setFloat(fbdo, F("/iot/temp"), 25) ? "ok" : fbdo.errorReason().c_str());
//     Serial.println();
//   }
//   else{
//     Serial.println("firebase no ready");
//   }
// }
void button()
{
  if (digitalRead(button1) == LOW)
  { // mỗi lần bấm tăng lên 1000ms
    setpump = setpump + 1000;
    if (setpump == 60)
    {
      setpump = 0;
    }
  }
  if (digitalRead(button2) == LOW)
  { // mỗi lần bấm tăng lên 1000ms

    timeLoop = timeLoop + 1000;
    if (timeLoop == 60)
    {
      timeLoop = 0;
    }
  }
  if (digitalRead(button4) == LOW){
    setpump=0;
    timeLoop=0;
  }
  if(digitalRead(button5)==LOW){
    //
  }
}
void setled(){
    if(ntuValue >= 1700 && ntuValue <= 2000){
    Firebase.setDouble(fbdo, "/iot/ntu1", 3); //// độ đục
  }
  else if(ntuValue >= 600 && ntuValue <= 1600){
   Firebase.setDouble(fbdo, "/iot/ntu1", 2); //// độ đục
  }
  else if(ntuValue < 600 ){
    Firebase.setDouble(fbdo, "/iot/ntu1", 1); //// độ đục
    
  }
/////////led cho ph
  if(pHValue >= 7 && pHValue <= 9){
    Firebase.setDouble(fbdo, "/iot/ph1", 1); //// độ ph
  }
  else if(pHValue > 9 && pHValue <= 12){
Firebase.setDouble(fbdo, "/iot/ph1", 2); //// độ ph
  }
  else if(pHValue >12 || pHValue <7 ){
Firebase.setDouble(fbdo, "/iot/ph1", 3); //// độ ph
  }
/////////led cho tds
  if(tdsValue >= 300 && tdsValue <= 450){
    Firebase.setDouble(fbdo, "/iot/tds1", 3); //// độ tds
  }
  else if(tdsValue >= 100 && tdsValue <= 250){
    Firebase.setDouble(fbdo, "/iot/tds1", 2); //// độ tds
  }
  else if(tdsValue >0 && tdsValue <100 ){
    Firebase.setDouble(fbdo, "/iot/tds1", 1); //// độ tds
  }
/////////led cho temp
  if(temperature >= 27.5 && temperature <= 29.5){
    Firebase.setDouble(fbdo, "/iot/temp1", 1); //// độ temp
  }
  else if(temperature >= 29.5 && temperature <= 35){
    Firebase.setDouble(fbdo, "/iot/temp1", 2); //// độ temp
  }
  else if(temperature >35 || temperature <27 ){
    Firebase.setDouble(fbdo, "/iot/temp1", 3); //// độ temp
  }
}
