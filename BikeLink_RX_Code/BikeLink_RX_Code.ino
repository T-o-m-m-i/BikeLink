#include <SPI.h>
#include <LoRa.h>

char Received_TXT;
bool Receiver_responses_OK = false;
bool connection_OK = false;
bool ALARM = false;
//char akun_varaus = "A9";
uint16_t jannite_raja = 700;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(866E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //First, after powering on, try to make handling with transmitter. Send message untill transmitter responses
    while(!connection_OK)
    {
      //Serial.println("Connecting...");
          // send packet
          //LoRa.beginPacket();
          //LoRa.print("Establish connection?");
          //LoRa.endPacket();
          //delay(1000);
       // Received response from transmitter??
          int packetSize = LoRa.parsePacket();
          if (packetSize) {
            char paketti [packetSize];
            char paketti2 [packetSize];
            memset(paketti2, 0, sizeof(paketti2));
            int i=0;
              // received a packet
              Serial.print("Received packet '");

              // read packet
                  while (LoRa.available()) {
                  paketti[i] = (char)LoRa.read(); //TRANSMITTER SHOULD SEND OK OR SOMETHING?
                  i++;

                  //if (LoRa.read() == "YES,establish")
                  //  Receiver_responses_OK = 1;
                  }
                  memcpy(paketti2, paketti, 5);
                  Serial.println(paketti2);

                  if ((String)paketti2 == "TESTI")
                  {
                    Serial.print ("HELLO KUNINGAS");
                    Receiver_responses_OK = 1;
                  }
          }

          if (Receiver_responses_OK == 1)
          {
            connection_OK = 1;
            // Connection is now established.
             delay(1000);
            //Set receiver mode
             LoRa.receive();
             ALARM = 0;
             //Arduino to power saving mode. Can be waken up with WDT, how about RFM95???
             //sleep();

            break;
          }
    }
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    char paketti [packetSize];
    char paketti2 [packetSize];
    memset(paketti2, 0, sizeof(paketti2));
    int i=0;

    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Received_TXT = (char)LoRa.read();
      paketti[i] = Received_TXT;
      i++;
      Serial.print(Received_TXT);
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.print(LoRa.packetRssi());

    Serial.println();
    Serial.println("paketti");
    Serial.println(paketti);
    memcpy(paketti2, paketti, 5);

    Serial.println("paketti2");
    Serial.println(paketti2);
    if ((String)paketti2 == "hello")
      {
       Serial.print ("HELLO KUNINGAS");
      }

  Serial.println();
  Serial.println();
  }
}
