
#include <ESP8266WiFi.h>
#include "FS.h"
#include <WebSocketsServer.h>
//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "manchester";
#define MOTOR1_A 16
#define MOTOR1_B 14

#define MOTOR2_A 12
#define MOTOR2_B 13

#define MOTOR3_A 1
#define MOTOR3_B 3

#define MOTOR4_A 4
#define MOTOR4_B 5

/////////////////////
// Pin Definitions //
/////////////////////

WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
uint8_t last_socket_num;
int Wheel_FL,Wheel_FR,Wheel_BL,Wheel_BR; //Front Left, Front Right, Back Left, Back Right
void setup() 
{
  initHardware();
  setupWiFi();
  server.begin();
  //bool s_result=SPIFFS.begin();
  ////Serial.println("SPIFFS opened: " + s_result);
  //Dir dir = SPIFFS.openDir("/");
//Serial.println("Hello world");
/*while (dir.next()) {
    //Serial.print(dir.fileName());
    File f = dir.openFile("r");
    //Serial.println(f.size());
}*/
webSocket.begin();
//Serial.println("Websocket created ");
webSocket.onEvent(webSocketEvent);

//pinMode(MOTOR1_A,OUTPUT);
//pinMode(MOTOR1_B,OUTPUT);

//pinMode(MOTOR2_A,OUTPUT);
//pinMode(MOTOR2_B,OUTPUT);

pinMode(MOTOR3_A,OUTPUT); //motor 3 is OK
pinMode(MOTOR3_B,OUTPUT);

pinMode(MOTOR4_A,OUTPUT); //motor 4 is OK
pinMode(MOTOR4_B,OUTPUT);

digitalWrite(MOTOR1_A,LOW);
digitalWrite(MOTOR2_A,LOW);
digitalWrite(MOTOR3_A,LOW);
digitalWrite(MOTOR4_A,LOW);

digitalWrite(MOTOR1_B,LOW);
digitalWrite(MOTOR2_B,LOW);
digitalWrite(MOTOR3_B,LOW);
digitalWrite(MOTOR4_B,LOW);


}



#define USE_//Serial //Serial

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
int c_index;
String command;
int speed1;
last_socket_num=num;

 ESP.wdtFeed();
    switch(type) {
        case WStype_DISCONNECTED:
            USE_//Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                USE_//Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        
        // send message to client
        webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            USE_//Serial.printf("[%u] get Text: %s\n", num, payload);
            command=String((char*)payload);

            //First, get speed
            c_index=command.indexOf(",");
            
            if (c_index>0)
            {
              speed1=command.substring(c_index+1).toInt();
              //Serial.print("Got speed ");
              //Serial.println(speed1);
            }
              Wheel_FL=0;
              Wheel_FR=0;
              Wheel_BL=0;
              Wheel_BR=0;
            if (command.indexOf("up")>0)
            {
              
              Wheel_FL=1000;
              Wheel_FR=1000;
              Wheel_BL=1000;
              Wheel_BR=1000;
            }
            else if (command.indexOf("down")>0)
            {
              Wheel_FL=-1000;
              Wheel_FR=-1000;
              Wheel_BL=-1000;
              Wheel_BR=-1000;            
            }
            else if (command.indexOf("right")>0)
            {
              Wheel_FL=500;
              Wheel_FR=-500;
              Wheel_BL=500;
              Wheel_BR=-500;
            }
            else if (command.indexOf("left")>0)
            {
              Wheel_FL=-500;
              Wheel_FR=500;
              Wheel_BL=-500;
              Wheel_BR=500;  
            }

 if (command.indexOf("0000")>=0)
 {
  Wheel_FL=0;
  Wheel_FR=0;
  Wheel_BL=0;
  Wheel_BR=0;
  }
/*//Serial.print("Wheel_FL ");
//Serial.println(Wheel_FL);
//Serial.print("Wheel_FR ");
//Serial.println(Wheel_FR);
//Serial.print("Wheel_BL ");
//Serial.println(Wheel_BL);
//Serial.print("Wheel_BR ");
//Serial.println(Wheel_BR);*/
            
SetMotors(Wheel_FL, Wheel_FR, Wheel_BL,  Wheel_BR);

              

            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            USE_//Serial.printf("[%u] get binary lenght: %u\n", num, lenght);
            hexdump(payload, lenght);

            // send message to client
            // webSocket.sendBIN(num, payload, lenght);
            break;
    }

}




void SetMotors(int FL, int FR, int BL, int BR)
{
  ESP.wdtFeed();
/*//Serial.print("FL ");
//Serial.println(FL);
//Serial.print("FR ");
//Serial.println(FR);
//Serial.print("BL ");
//Serial.println(BL);
//Serial.print("BR ");
//Serial.println(BR);*/
//webSocket.sendTXT(last_socket_num, "\nFL ");
  if (FL==0)
  {
digitalWrite(MOTOR1_A,LOW);
digitalWrite(MOTOR1_B,LOW);
 }
 else if (FL>0)
 {
digitalWrite(MOTOR1_B,LOW);
digitalWrite(MOTOR1_A,HIGH);
  }
  else if (FL<0)
  {
digitalWrite(MOTOR1_A,LOW);
digitalWrite(MOTOR1_B,HIGH);
  }

//======FRONT RIGHT===========
 //webSocket.sendTXT(last_socket_num, "\nFR ");
  if (FR==0)
  {
digitalWrite(MOTOR2_A,LOW);
digitalWrite(MOTOR2_B,LOW);
 }
 else if (FR>0)
 {
  digitalWrite(MOTOR2_B,LOW);
digitalWrite(MOTOR2_A,HIGH);
  }
  else if (FR<0)
  {
digitalWrite(MOTOR2_A,LOW);
digitalWrite(MOTOR2_B,HIGH);
  }
//======BACK LEFT===========
 // webSocket.sendTXT(last_socket_num, "\nBL ");
    if (BL==0)
  {
   //  webSocket.sendTXT(last_socket_num, "STOP\n");
//Serial.println("BL STOP");

digitalWrite(MOTOR4_A,LOW);
digitalWrite(MOTOR4_B,LOW);
 }
 else if (BL>0)
 {
 // webSocket.sendTXT(last_socket_num, "FWD\n");
 //Serial.println("BL FWD");
digitalWrite(MOTOR4_B,LOW);
digitalWrite(MOTOR4_A,HIGH);

  }
  else if (BL<0)
  {
     // webSocket.sendTXT(last_socket_num, "BACK\n");
//Serial.println("BL BACK");
digitalWrite(MOTOR4_A,LOW);
digitalWrite(MOTOR4_B,HIGH);
  }

//======BACK RIGHT===========
//webSocket.sendTXT(last_socket_num, "\nBR ");
    if (BR==0)
  {
   // webSocket.sendTXT(last_socket_num, "STOP\n");
   //Serial.println("BR STOP");
digitalWrite(MOTOR3_A,LOW);
digitalWrite(MOTOR3_B,LOW);
 }
 else if (BR>0)
 {
 // webSocket.sendTXT(last_socket_num, "FWD\n");
 //Serial.println("BR FWD");
digitalWrite(MOTOR3_B,LOW);
digitalWrite(MOTOR3_A,HIGH);
  }
  else if (BR<0)
  {
         // webSocket.sendTXT(last_socket_num, "BACK\n");
//Serial.println("BR BACK");
digitalWrite(MOTOR3_A,LOW);
digitalWrite(MOTOR3_B,HIGH);
  }
  
  
  
}

void loop() 
{
  webSocket.loop();
   // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  
  // Wait until the client sends some data
  //Serial.println("new client");
  while(!client.available()){
    ESP.wdtFeed();
    delay(1);
  }
  
  // Read the first line of the request
  String req = client.readStringUntil('\r');
  //Serial.println(req);
  client.flush();
  
  // Match the request
  int val;
  if (req.indexOf("/gpio/0") != -1)
    val = 0;
  else if (req.indexOf("/gpio/1") != -1)
    val = 1;
  else {
    //Serial.println("invalid request");
    client.stop();
    return;
  }

  // Set GPIO2 according to the request
  digitalWrite(2, val);
  
  client.flush();

  // Prepare the response
  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\nGPIO is now ";
  s += (val)?"high":"low";
  s += "</html>\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  //Serial.println("Client disonnected");
}

void setupWiFi()
{
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "EvoAllTerrain " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  IPAddress Ip(192, 168, 1, 1);
  IPAddress NMask(255, 255, 255, 0);


  WiFi.softAPConfig(Ip, Ip, NMask);
 if (!WiFi.softAP( AP_NameChar, WiFiAPPSK))
  {
   ////Serial.println("WiFi.softAP failed.(Password too short?)");
   return;
  }
  IPAddress myIP = WiFi.softAPIP();
  ////Serial.println();
  ////Serial.print("AP IP address: ");
  ////Serial.println(myIP);
  
 
}

void initHardware()
{
  //Serial.begin(115200);
// ESP.wdtDisable();
}
