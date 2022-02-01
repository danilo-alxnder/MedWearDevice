#include <Wire.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
using namespace BLA;
//ble code
BLEServer *ESP_Server = NULL; //Create the server
BLECharacteristic * ESP_TxCharacteristic; //Create an characterist
bool phone_Connected = false; //Check iif device is connected in the callback function
bool oldPhone_Connected = false;
std::string ESP_txValue ="microcontroller team is definetly the most proactive"; //Transmited value

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // define UUID of the service
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // define characterist UUID of receiver
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // define characterist UUID of transmitter

//
float volte[22] = {0.75, 0.79, 0.93, 1.125, 1.25, 1.36, 1.435, 1.49, 1.565, 1.625, 1.66, 1.68, 1.705, 1.735, 1.755, 1.755, 1.825, 1.86, 1.88, 1.89, 1.9, 1.94};
float voltl[22] = {0.75, 0.8, 0.9, 1.05, 1.16, 1.21, 1.27, 1.41, 1.46, 1.48, 1.52, 1.57, 1.6, 1.63, 1.69, 1.71, 1.74, 1.75, 1.78, 1.83, 1.85, 1.89};
float cm[22] = {0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5, 3.75, 4, 4.25, 4.5, 4.75, 5, 5.25};
unsigned long begin_time;
unsigned long now_time;
bool posture = true;// true means good posture, flase means bad posture
float minimalangle = 65 ;//75 gives the minimal threshold angle 
float maximalangle = 100;//100 gives the maximal threshold angle 
float anglecombination;
//
float dt = 0.001; //1/Fs

const int MPU_addr1=0x68;
const int MPU_addr2=0x69;
int16_t Ac1X, Ac1Y, Ac1Z,Ac2X, Ac2Y, Ac2Z;
int16_t Rot1X,Rot1Y,Rot1Z,Rot2X,Rot2Y,Rot2Z;
float Acc1_X,Acc1_Y,Acc1_Z,Acc2_X,Acc2_Y,Acc2_Z;
float Rot1_X,Rot1_Y,Rot1_Z,Rot2_X,Rot2_Y,Rot2_Z;

Matrix<4,4> data_state = {1, -dt, 0, 0,0, 1, 0, 0,0, 0, 1, -dt,0, 0, 0, 1};
Matrix<4,2> error_state = {dt, 0,0, 0,0, dt,0, 0};
Matrix<2,4> est_unc_state = {1, 0, 0, 0, 0, 0, 1, 0};
Matrix<4,4> Q = {0.0001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001};
Matrix<2,2> R = {0.001, 0, 0, 0.0001};
Matrix<4,4> I = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

float angle_data1[4];
float estimates1[4];
float P1[4][4] = {{pow(0.2, 2), 0, 0, 0}, 
                  {0, pow(0.2, 2), 0, 0}, 
                  {0, 0, pow(0.2, 2), 0}, 
                  {0, 0, 0, pow(0.2, 2)}}; //initial estimate uncertainty
float angle_data2[4];
float estimates2[4];
float P2[4][4] = {{pow(0.2, 2), 0, 0, 0}, 
                  {0, pow(0.2, 2), 0, 0}, 
                  {0, 0, pow(0.2, 2), 0}, 
                  {0, 0, 0, pow(0.2, 2)}}; //initial estimate uncertainty
class MyServerCallbacks: public BLEServerCallbacks {  
    void onConnect(BLEServer* ESP_Server) {
      phone_Connected = true;
    };

    void onDisconnect(BLEServer* ESP_Server) {
      phone_Connected = false;
    }
};

//get data from the phone
class My_Receiving_Callbacks: public BLECharacteristicCallbacks {
   void onWrite(BLECharacteristic *ESP_RxCharacteristic) {
      std::string received_Value = ESP_RxCharacteristic->getValue();
      Serial.println("Received value is :");
      //Print the received data
      for (int i = 0; i < received_Value.length(); i++)
      Serial.print(received_Value[i]);
      Serial.println(); 
    }
};
void setup() 
{
  
  // put your setup code here, to run once:
  delay(3000);
  pinMode(12, INPUT);//stretchsensorpin
  pinMode(14, OUTPUT);
  Serial.print("print stuff");
  digitalWrite(14, LOW);
  Serial.print("get's here");
 
  Serial.begin(9600);
  Serial.println("Ready");
  Wire.begin();
  setupMPU_transmission(MPU_addr1);
  setupMPU_transmission(MPU_addr2);
  delay(1000);
  //ble setup
  // Create the BLE Device and name it
  BLEDevice::init("Seiza");

  // Create the BLE Server
  ESP_Server = BLEDevice::createServer();
  //Set the callback class
  ESP_Server->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *ESP_Service = ESP_Server->createService(SERVICE_UUID);

  // Create a transmisssion BLE Characteristic
  ESP_TxCharacteristic = ESP_Service->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  //Add a descriptor to the characterist            
  ESP_TxCharacteristic->addDescriptor(new BLE2902());
  //Create the receiving characteristcs
  BLECharacteristic * ESP_RxCharacteristic = ESP_Service->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  ESP_RxCharacteristic->setCallbacks(new My_Receiving_Callbacks);

  // Start the service
   ESP_Service->start();

  // Start advertising
  ESP_Server->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() 
{
  //stetchsensorcode
    // put your main code here, to run repeatedly:
  int stretchpin = 0;
  float stretch = 0;
  float voltage = 0;
  now_time = millis();
  stretchpin = analogRead(12); //don't know the pin

  voltage = stretchpin*3.3/4096;

  if(now_time - begin_time < 30000)
  {
    for(int i=0;i<22;i++)
    {
      if(volte[i] >= voltage && volte[i-1] <= voltage)
      {
        stretch = cm[i];
        break;
      }
    }
  }
  else if(now_time - begin_time > 30000)
  {
    for(int i=0;i<22;i++)
    {
      if(voltl[i] >= voltage && voltl[i-1] <= voltage)
      {
        stretch = cm[i];
        break;
      }
    }
  }
  Serial.print("Stretch = ");
  Serial.println(voltage);
  // put your main code here, to run repeatedly:
  get_accelometer_data(MPU_addr1,Ac1X,Ac1Y,Ac1Z);
  get_acceleration_g(Ac1X,Ac1Y,Ac1Z,Acc1_X,Acc1_Y,Acc1_Z);
  //Serial.print("(IMU1) acceleration on x axis=");Serial.print(Acc1_X); //prints acceleration in g
  //Serial.print("   acceleration on y axis=");Serial.print(Acc1_Y);
  //Serial.print("   acceleration on Z axis=");Serial.print(Acc1_Z);
  //Serial.println();
  
  get_gyro_data(MPU_addr1,Rot1X,Rot1Y,Rot1Z);
  get_rotation_angle(Rot1X,Rot1Y,Rot1Z,Rot1_X,Rot1_Y,Rot1_Z);
  //Serial.print("(IMU1) rotation on x axis=");Serial.print(Rot1_X); //prints angle/s
  //Serial.print("  rotation on y axis=");Serial.print(Rot1_Y);
  //Serial.print("  rotation on Z axis=");Serial.print(Rot1_Z);
  //Serial.println();
  
  get_accelometer_data(MPU_addr2,Ac2X,Ac2Y,Ac2Z);
  get_acceleration_g(Ac2X,Ac2Y,Ac2Z,Acc2_X,Acc2_Y,Acc2_Z);
  //Serial.print("(IMU2) acceleration on x axis=");Serial.print(Acc2_X); //prints acceleration in g
  //Serial.print("   acceleration on y axis=");Serial.print(Acc2_Y);
  //Serial.print("   acceleration on Z axis=");Serial.print(Acc2_Z);
  //Serial.println();
  
  get_gyro_data(MPU_addr2,Rot2X,Rot2Y,Rot2Z);
  get_rotation_angle(Rot2X,Rot2Y,Rot2Z,Rot2_X,Rot2_Y,Rot2_Z);
  //Serial.print("(IMU2) rotation on x axis=");Serial.print(Rot2_X); //prints angle/s
  //Serial.print("  rotation on y axis=");Serial.print(Rot2_Y);
  //Serial.print("  rotation on Z axis=");Serial.print(Rot2_Z);
  //Serial.println();

  //convert data
  to_angle(Acc1_X,Acc1_Y,Acc1_Z,Rot1_X,Rot1_Y,Rot1_Z,angle_data1);
  to_angle(Acc2_X,Acc2_Y,Acc2_Z,Rot2_X,Rot2_Y,Rot2_Z,angle_data2);
  //Serial.print("(IMU1) raw y angle=");Serial.print(angle_data1[0]);
  //Serial.print("  (IMU1) raw x angle=");Serial.print(angle_data1[2]);
  //Serial.println();
  //Serial.print("(IMU2) raw y angle=");Serial.print(angle_data2[0]);
  //Serial.print("  (IMU2) raw x angle=");Serial.print(angle_data2[2]);
  //Serial.println();

  //filter
  kalman(P1, estimates1, estimates1, angle_data1);
  kalman(P2, estimates2, estimates2, angle_data2);
  Serial.print("(IMU1) est. y angle=");Serial.print(57.3*estimates1[0]);
  Serial.print("  (IMU1) est. x angle=");Serial.print(57.3*estimates1[2]);
  Serial.println();
  Serial.print("(IMU2) est. y angle=");Serial.print(57.3*estimates2[0]);
  Serial.print("  (IMU2) est. x angle=");Serial.print(57.3*estimates2[2]);
  Serial.println();
  anglecombination = ((57.3*estimates1[0]+57.3*estimates2[0])/2);
  if (anglecombination > minimalangle && anglecombination < maximalangle){
    posture = true;
    }
  else{
    posture = false;}
  Serial.println(anglecombination); 
  if (!posture){
    digitalWrite(14, HIGH);
    Serial.println("bad posture");
    }
  else{
    digitalWrite(14, LOW);
    Serial.println("Good posture");
    }

    //ble code
    Serial.println("cool");
    if (phone_Connected) {
         Serial.println("it works");
        char finalvalue[20];
        char imuangle[4]; // Buffer big enough for 7-character float
        dtostrf(estimates1[0], 3, 0, imuangle);
        strcat (finalvalue,imuangle);
        
        dtostrf(estimates1[2], 3, 0, imuangle);
        strcat (finalvalue,imuangle);
       
        dtostrf(estimates2[0], 3, 0, imuangle);
        strcat (finalvalue,imuangle);
       
        dtostrf(estimates2[2], 3, 0, imuangle);
        strcat (finalvalue,imuangle);
     
        dtostrf(stretch, 2, 1, imuangle);
        strcat (finalvalue,imuangle);
        ESP_TxCharacteristic->setValue(finalvalue); 
        ESP_TxCharacteristic->notify();
        delay(100);
    
    
    delay(100); // wait a second befor another transmission ,u can make it smaller but dont make it smaller than 10ms as the BLE stuck will crash
  }

    // handle phone disconnection ,so it does not start sending data although nothing is conencted
    if (!phone_Connected && oldPhone_Connected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        ESP_Server->startAdvertising(); // restart advertising
       
        oldPhone_Connected = phone_Connected;
    }
    // handle connection 
    if (phone_Connected && !oldPhone_Connected) {
    //this is when the firts time u connects so if u want to put something here just put it
        oldPhone_Connected = phone_Connected;
    }
  
}

void kalman(float Pin[4][4], float input[4], float output[4], float raw[4])
{
  //declare
  Matrix<4> guess;
  Matrix<4> inputm = {input[0], input[1], input[2], input[3]};
  Matrix<4> outputm;
  Matrix<2> rawg = {raw[1], raw[3]};
  Matrix<2> rawa = {raw[0], raw[2]};
  Matrix<4,4> P = {Pin[0][0], Pin[0][1], Pin[0][2], Pin[0][3],
                  Pin[1][0], Pin[1][1], Pin[1][2], Pin[1][3],
                  Pin[2][0], Pin[2][1], Pin[2][2], Pin[2][3],
                  Pin[3][0], Pin[3][1], Pin[3][2], Pin[3][3]};
  Matrix<4,2> K;
  
  //predict
  guess = data_state*inputm+error_state*rawg;
  P = data_state*P*(~data_state) + Q;

  //update
  K = P*(~est_unc_state)*((est_unc_state*P*(~est_unc_state) + R).Inverse());
  outputm = guess + K*(rawa-est_unc_state*guess);
  P = (I - K*est_unc_state)*P;

  //convert for output
  for(int i=0;i<4;i++)
  {
    output[i] = outputm(i);
    for(int j=0;j<4;j++)
    {
      Pin[i][j] = P(i,j);
    }
  }
}

void to_angle(float ax, float ay, float az, float gx, float gy, float gz, float angle_data[4])
{
  angle_data[0] = atan(ay/sqrt(pow(ax, 2)+pow(az, 2)))* 180/ M_PI; //roll
  angle_data[2] = atan(ay/sqrt(pow(ay, 2)+pow(az, 2)))*180/ M_PI; //pitch
  angle_data[1] = gx + tan(angle_data[1])*(sin(angle_data[0])*gy + cos(angle_data[0])*gz); //x gyro bias
  angle_data[3] = cos(angle_data[1])*gy - sin(angle_data[1])*gz; //y gyro bias
}

void get_acceleration_g(int16_t a,int16_t b,int16_t c,float &AccX,float &AccY,float &AccZ){
  AccX=a/16384.0;
  AccY=b/16384.0;
  AccZ=c/16384.0;
  
}
void get_rotation_angle(int16_t a,int16_t b,int16_t c,float &RotX,float &RotY,float &RotZ){
   RotX=a/131.0;
   RotY=b/131.0;
   RotZ=c/131.0;
  
}
void setupMPU_transmission(int adress){
  Wire.beginTransmission(adress); //This is the I2C address of the IMU
  Wire.write(0x6B); //Accessing the register for power managment
  Wire.write(0b00000000); //Wake up the IMU
  Wire.endTransmission();  
  Wire.beginTransmission(adress); //I2C address of the IMU
  Wire.write(0x1B); //Accessing the register for gyro
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(adress); //I2C address of the IMU
  Wire.write(0x1C); //Accessing the register 1C for accelometer 
  Wire.write(0b00000000); //Setting the accelometer scale to +/- 2g
  Wire.endTransmission(); 
}
void get_accelometer_data(int adress,int16_t &AcX,int16_t &AcY,int16_t &AcZ) {
  Wire.beginTransmission(adress); //I2C address of the IMU
  Wire.write(0x3B); //Start acquiring acceleration dataa
  Wire.endTransmission();
  Wire.requestFrom(adress,6); //Request acceleration register
  while(Wire.available() < 6);
  AcX = (Wire.read()<<8|Wire.read()); //Get acceleration on X
  AcY = (Wire.read()<<8|Wire.read()); //Get acceleration on Y
  AcZ = (Wire.read()<<8|Wire.read()); //Get acceleration on Z
  delay(200);
}

void get_gyro_data(int adress,int16_t &RotX,int16_t &RotY,int16_t &RotZ) {
  Wire.beginTransmission(adress); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(adress,6); //Request Gyro data
  while(Wire.available() < 6);
  RotX = (Wire.read()<<8|Wire.read()); //Get rotation on X
  RotY = (Wire.read()<<8|Wire.read()); //Get rotation on Y
  RotZ = (Wire.read()<<8|Wire.read()); //Get rotation on Z
  delay(200);
}
