#include <mcp_can.h>
#include <mcp_can_dfs.h>

// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
//#include "mcp_can.h"
#include <mcp_can.h>


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(10000);
    }
    Serial.println("CAN BUS Shield init ok!");
}

int rpm = 0;
float tps = 0;
float fuel = 0;
float batVolt = 0;
float Coolant = 0;

void loop()
{
    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned int canId = CAN.getCanId();
        
        //Serial.println("-----------------------------");
        //Serial.print("Get data from ID: ");
        //Serial.println(canId, HEX);

        for(int i = 0; i<len; i++)    // print the data
        { 
            //Serial.print(i);
            //Serial.print(buf[i], HEX);
            //Serial.print("\t");
        }
        //Serial.println();
        if (canId == 0xF048){
          rpm = buf[1]*256 + buf[0];
          tps = buf[3]*256 + buf[2];
          if (tps > 32767)
            tps -= 65536;
          tps = tps*0.1;
          //Serial.print("RPM: ");
          //Serial.println(rpm);
//          Serial.print("TPS: ");
//          Serial.println(tps);
        }
        else if (canId == 0xF548){
          batVolt = buf[1]*256 + buf[0];
          if (batVolt > 32767)
            batVolt -= 65536;
          batVolt=batVolt*0.01;
          Coolant = buf[5]*256 + buf[4];
          if (Coolant > 32767)
            Coolant -= 65536;
          Coolant=Coolant*0.1;
          Serial.print("Battery Voltage: ");
          Serial.println(batVolt);
        }
        else if (canId == 0xF448){
          fuel = buf[1]*256 + buf[0];
          if (fuel > 32767)
            fuel -= 65536;
          fuel=fuel*0.2;
          //Serial.print("Fuel Flow: ");
          //Serial.println(batVolt);
        }
        //Serial.println("Fuel\t Volt\t tps\t rpm\t Coolant");
        Serial.print(fuel);
        Serial.print("\t ");
        Serial.print(batVolt);
        Serial.print("\t ");
        Serial.print(tps);
        Serial.print("\t ");
        Serial.print(rpm);
        Serial.print("\t ");
        Serial.println(Coolant);
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
