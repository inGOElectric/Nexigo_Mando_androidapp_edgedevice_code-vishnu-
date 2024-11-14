#include <SPI.h>          //Library for using SPI Communication 
#include <mcp2515.h>      //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)
struct can_frame canMsg;
MCP2515 mcp2515(5);
int id = 0x800001FA;
int AA = 0xAA;
void setup()
{
  while (!Serial);
  Serial.begin(115200);
  SPI.begin();               //Begins SPI communication
  //dht.begin();               //Begins to read temperature & humidity sesnor value
 
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();
}
 
 
void loop()
{
  canMsg.can_id  = id;        
  Serial.println("");   
  Serial.print(canMsg.can_id, HEX);
  canMsg.can_dlc = 8;
  Serial.print("  ");
  Serial.print(canMsg.can_dlc, HEX);               
  canMsg.data[0] = AA;
  Serial.print("  ");
  Serial.print(canMsg.data[0], HEX);               
  canMsg.data[1] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[1], HEX);                
  canMsg.data[2] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[2], HEX);              
  canMsg.data[3] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[3], HEX);  
  canMsg.data[4] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[4], HEX);  
  canMsg.data[5] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[5], HEX);  
  canMsg.data[6] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[6], HEX);  
  canMsg.data[7] = 0x00;
  Serial.print("  ");
  Serial.print(canMsg.data[7], HEX);  
 
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message
  delay(500);
}
