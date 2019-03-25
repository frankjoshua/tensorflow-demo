#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = "tesseract";
const char* password = "Tessmo67";

IPAddress server(192, 168, 2, 95); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11412);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};


std_msgs::Float32MultiArray readings_msg;
ros::Publisher camPub("thermal_cam", &readings_msg);

ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup() {
  Serial.begin(115200);

  //Initilize the thermal camera
  bool status = amg.begin();
  if (!status) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
      while (1);
  }
    
  setupWiFi();
  delay(2000);

  //Start ROS
  nh.initNode();
  nh.advertise(camPub);
  //Point ROS message to camera ouput
  readings_msg.data = pixels;
  readings_msg.data_length = AMG88xx_PIXEL_ARRAY_SIZE;
}

void loop() {
  //read the camera data into pixels
  amg.readPixels(pixels);
  camPub.publish(&readings_msg);
  nh.spinOnce();
  delay(100);
}
