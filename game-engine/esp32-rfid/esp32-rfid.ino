#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
//#include <MFRC522DriverI2C.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include "EspMQTTClient.h"

MFRC522DriverPinSimple ss_pin(5); // Configurable, see typical pin layout above.

MFRC522DriverSPI driver{ss_pin}; // Create SPI driver.
//MFRC522DriverI2C driver{}; // Create I2C driver.
MFRC522 mfrc522{driver};  // Create MFRC522 instance.

uint32_t chipId = 0;
EspMQTTClient client(
  "DigitalAGclub",  // WiFi SSID at the maker space
  "username",       // WiFi password
  "192.168.0.100"  // MQTT Broker server IP
);

void onConnectionEstablished()
// This function activate when connection is established with the broker
{
  Serial.printf("Connection Established\n");
}

void setup() {
  Serial.begin(115200);  // Initialize serial communications with the PC for debugging.
  while (!Serial);     // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4).
  mfrc522.PCD_Init();  // Init MFRC522 board.
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc522, Serial);	// Show details of PCD - MFRC522 Card Reader details.
  mfrc522.PCD_SetAntennaGain(0x70);
	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  Serial.print(mfrc522.PCD_GetAntennaGain()); 
  client.setWifiReconnectionAttemptDelay(5000);
}

void loop() {
  client.loop();
  if (!client.isConnected()) {
    Serial.printf("WiFi Connection: %d\n", client.isWifiConnected());
    Serial.printf("MQTT Connection: %d\n", client.isMqttConnected());
    delay(1000);
  }


	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	if ( !mfrc522.PICC_IsNewCardPresent()) {
		return;
	}

	// Select one of the cards.
	if ( !mfrc522.PICC_ReadCardSerial()) {
		return;
	}

	// Dump debug info about the card; PICC_HaltA() is automatically called.
  // MFRC522Debug::PICC_DumpToSerial(mfrc522, Serial, &(mfrc522.uid));

  Serial.print(F("Card UID:"));
  dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
  Serial.println();
  delay(1000);
  
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }

    client.publish("purdue-dac/carrot", "Hello", false);
}