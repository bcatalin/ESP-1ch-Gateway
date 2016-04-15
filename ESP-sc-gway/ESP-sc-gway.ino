/*******************************************************************************
 * Copyright (c) 2016 Maarten Westenberg version for ESP8266
 * Copyright (c) 2015 Thomas Telkamp for initial Raspberry Version
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Notes: 
 * - Once call gethostbyname() to get IP for services, after that only use IP
 *	 addresses (too many gethost name makes ESP unstable)
 * - Only call yield() in main stream (not for background NTP sync). 
 *
 *******************************************************************************/

//
#define VERSION " ! V. 1.1.2, 160415"

#include <Esp.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <SPI.h>
#include <Time.h>								// http://playground.arduino.cc/code/time
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
extern "C" {
#include "user_interface.h"
#include "lwip/err.h"
#include "lwip/dns.h"
}
#include <pins_arduino.h>
#include <gBase64.h>							// https://github.com/adamvr/arduino-base64 (I changed the name)
#include "ESP-sc-gway.h"						// This file contains configuration of GWay

int debug=1;									// Debug level! 0 is no msgs, 1 normal, 2 is extensive

using namespace std;

byte currentMode = 0x81;
char message[256];
char b64[256];
bool sx1272 = true;								// Actually we use sx1276/RFM95
byte receivedbytes;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };

uint8_t MAC_array[6];
char MAC_char[18];

/*******************************************************************************
 *
 * Configure these values if necessary!
 *
 *******************************************************************************/

// SX1276 - ESP8266 connections
int ssPin = 15;									// GPIO15, D8
int dio0  = 5;									// GPIO5,  D1
int RST   = 0;									// GPIO16, D0, not connected

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency. If in doubt, choose the first one, comment all others
// Each "real" gateway should support the first 3 frequencies according to LoRa spec.
uint32_t  freq = 868100000; 					// Channel 0, 868.1 MHz
//uint32_t  freq = 868300000; 					// Channel 1, 868.3 MHz
//uint32_t  freq = 868500000; 					// in Mhz! (868.5)
//uint32_t  freq = 867100000; 					// in Mhz! (867.1)
//uint32_t  freq = 867300000; 					// in Mhz! (867.3)
//uint32_t  freq = 867500000; 					// in Mhz! (867.5)
//uint32_t  freq = 867700000; 					// in Mhz! (867.7)
//uint32_t  freq = 867900000; 					// in Mhz! (867.9)
//uint32_t  freq = 868800000; 					// in Mhz! (868.8)
//uint32_t  freq = 869525000; 					// in Mhz! (869.525)
// TTN defines an additional channel at 869.525Mhz using SF9 for class B. Not used

// Set location, description and other configuration parameters
// Defined in ESP-sc_gway.h
//
float lat			= _LAT;						// Configuration specific info...
float lon			= _LON;
int   alt			= _ALT;
char platform[24]	= _PLATFORM; 				// platform definition
char email[40]		= _EMAIL;    				// used for contact email
char description[64]= _DESCRIPTION;				// used for free form description 

// define servers

IPAddress ntpServer;							// IP address of NTP_TIMESERVER
IPAddress ttnServer;							// IP Address of thethingsnetwork server

#define SERVER1 _TTNSERVER  					// The Things Network: croft.thethings.girovito.nl "54.72.145.119"
#define PORT1 1700								// The port on which to send data

//#define SERVER2 _MQTTSERVER      				// 2nd server to send to, e.g. private server
//#define PORT2 "1700"

WiFiUDP Udp;
uint32_t lasttime;

// You can switch webserver off if not necessary
// Probably better to leave it in though.
#if A_SERVER==1
#include <Streaming.h>          				// http://arduiniana.org/libraries/streaming/
String webPage;
ESP8266WebServer server(SERVERPORT);
#endif


// ============================================================================
// Set all definitions for Gateway
// ============================================================================	

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 	// mandated for SF11 and SF12

// FRF
#define REG_FRF_MSB					0x06
#define REG_FRF_MID					0x07
#define REG_FRF_LSB					0x08

#define FRF_MSB						0xD9		// 868.1 Mhz
#define FRF_MID						0x06
#define FRF_LSB						0x66

#define BUFLEN 2048  							//Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  512						// This should(!) be enough based on the static text part.. was 1024


// ----------------------------------------------------------------------------
// DIE is not use actively in the source code anymore.
// It is replaced by a Serial.print command so we know that we have a problem
// somewhere.
// There are at least 3 other ways to restart the ESP. Pick one if you want.
// ----------------------------------------------------------------------------
void die(const char *s)
{
    Serial.println(s);
	delay(50);
	// system_restart();						// SDK function
	// ESP.reset();				
	abort();									// Within a second
}

// ----------------------------------------------------------------------------
// Print leading '0' digits for hours(0) and second(0) when
// printing values less than 10
// ----------------------------------------------------------------------------
void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    if(digits < 10)
        Serial.print(F("0"));
    Serial.print(digits);
}


// ----------------------------------------------------------------------------
// Print the current time
// ----------------------------------------------------------------------------
void printTime() {
	char *Days [] ={"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
	Serial.print(Days[weekday()-1]);
	Serial.print(F(" "));
	printDigits(hour());
	Serial.print(F(":"));
	printDigits(minute());
	Serial.print(F(":"));
	printDigits(second());
	return;
}


// ----------------------------------------------------------------------------
// Convert a float to string for printing
// f is value to convert
// p is precision in decimal digits
// val is character array for results
// ----------------------------------------------------------------------------
void ftoa(float f, char *val, int p) {
	int j=1;
	int ival, fval;
	char b[6];
	
	for (int i=0; i< p; i++) { j= j*10; }

	ival = (int) f;								// Make integer part
	fval = (int) ((f- ival)*j);					// Make fraction. Has same sign as integer part
	if (fval<0) fval = -fval;					// So if it is negative make fraction positive again.
												// sprintf does NOT fit in memory
	strcat(val,itoa(ival,b,10));
	strcat(val,".");							// decimal point
	
	itoa(fval,b,10);
	for (int i=0; i<(p-strlen(b)); i++) strcat(val,"0");
	// Fraction can be anything from 0 to 10^p , so can have less digits
	strcat(val,b);
}

// =============================================================================
// NTP TIME functions

const int NTP_PACKET_SIZE = 48;					// Fixed size of NTP record
byte packetBuffer[NTP_PACKET_SIZE];

// ----------------------------------------------------------------------------
// Send the request packet to the NTP server.
//
// ----------------------------------------------------------------------------
void sendNTPpacket(IPAddress& timeServerIP) {
  // Zeroise the buffer.
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	packetBuffer[0] = 0b11100011;   			// LI, Version, Mode
	packetBuffer[1] = 0;						// Stratum, or type of clock
	packetBuffer[2] = 6;						// Polling Interval
	packetBuffer[3] = 0xEC;						// Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12]  = 49;
	packetBuffer[13]  = 0x4E;
	packetBuffer[14]  = 49;
	packetBuffer[15]  = 52;	

	Udp.beginPacket(timeServerIP, (int) 123);	// NTP Server and Port

	if ((Udp.write((char *)packetBuffer, NTP_PACKET_SIZE)) != NTP_PACKET_SIZE) {
		die("sendNtpPacket:: Error write");
	}
	else {
		// Success
	}
	Udp.endPacket();
}


// ----------------------------------------------------------------------------
// Get the NTP time from one of the time servers
// Note: As this function is called from SyncINterval in the background
//	make sure we have no blocking calls in this function
// ----------------------------------------------------------------------------
time_t getNtpTime()
{
  WiFi.hostByName(NTP_TIMESERVER, ntpServer);
  //while (Udp.parsePacket() > 0) ; 			// discard any previously received packets
  for (int i = 0 ; i < 4 ; i++) { 				// 5 retries.
    sendNTPpacket(ntpServer);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 5000) 
	{
      if (Udp.parsePacket()) {
        Udp.read(packetBuffer, NTP_PACKET_SIZE);
        // Extract seconds portion.
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secSince1900 = highWord << 16 | lowWord;
        Udp.flush();
        return secSince1900 - 2208988800UL + NTP_TIMEZONES * SECS_PER_HOUR;				
		// UTC is 1 TimeZone correction when no daylight saving time
      }
      //delay(10);
    }
  }
  return 0; 									// return 0 if unable to get the time
}

// ----------------------------------------------------------------------------
// Set up regular synchronization of NTP server and the local time.
// ----------------------------------------------------------------------------
void setupTime() {
  setSyncProvider(getNtpTime);
  setSyncInterval(NTP_INTERVAL);
}



// ============================================================================
// UDP AND WLAN FUNCTIONS

// ----------------------------------------------------------------------------
// GET THE DNS SERVER IP address
// ----------------------------------------------------------------------------
IPAddress getDnsIP() {
	ip_addr_t dns_ip = dns_getserver(0);
	IPAddress dns = IPAddress(dns_ip.addr);
	return((IPAddress) dns);
}


// ----------------------------------------------------------------------------
// Read a package
//
// ----------------------------------------------------------------------------
int readUdp(int packetSize)
{
  char receiveBuffer[64]; //buffer to hold incoming packet
  Udp.read(receiveBuffer, packetSize);
  receiveBuffer[packetSize] = 0;
  IPAddress remoteIpNo = Udp.remoteIP();
  unsigned int remotePortNo = Udp.remotePort();
  if (debug>=1) {
	Serial.print(F(" Received packet of size "));
	Serial.print(packetSize);
	Serial.print(F(" From "));
	Serial.print(remoteIpNo);
	Serial.print(F(", port "));
	Serial.print(remotePortNo);
	Serial.print(F(", Contents: "));
	for (int i=0; i<packetSize; i++) {
		Serial.print(receiveBuffer[i],HEX);
		Serial.print(':');
	}
	Serial.println();
  }
  return packetSize;
}

// ----------------------------------------------------------------------------
// Function to join the Wifi Network
// XXX Maybe we should make the reconnect shorter in order to avoid watchdog resets.
//	It is a matter of returning to the main loop() asap and make sure in next loop
//	the reconnect is done first thing.
// ----------------------------------------------------------------------------
int WlanConnect(char * ssid, char * password) {
	// We start by connecting to a WiFi network 
	Serial.print(F("WiFi connect to: ")); Serial.println(ssid);
	int agains = 0;
	int ledStatus = LOW;
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		agains++;
		delay(agains*500);
		digitalWrite(BUILTIN_LED, ledStatus); 	// Write LED high/low
		ledStatus = (ledStatus == HIGH) ? LOW : HIGH;
		if (debug>=2) Serial.print(".");
		yield();
		// If after 10 times there is still no connection, we probably wait forever
		// So restart the WiFI.begin process!!
		if (agains == 10) {
			agains = 0;
			Serial.println();
			//WiFi.disconnect();
			delay(500);
			return(-1);
		}
	}
	Serial.print(F("WiFi connected. local IP address: ")); 
	Serial.println(WiFi.localIP());
	yield();
	return(0);
}

// ----------------------------------------------------------------------------
// Send an UDP/DGRAM message to the MQTT server
// If we send to more than one host (not sure why) then we need to set sockaddr 
// before sending.
// ----------------------------------------------------------------------------
void sendUdp(char *msg, int length) {
	int l;
	if (WiFi.status() != WL_CONNECTED) {
		Serial.println(F("sendUdp: ERROR not connected to WLAN"));
		Udp.flush();

		if (WlanConnect( (char *) _SSID, (char *)_PASS) < 0) {
			Serial.print(F("sendUdp: ERROR connecting to "));
			Serial.print(_SSID);
			return;
		}
		if (debug>=1) Serial.println(F("WiFi reconnected"));	
		delay(10);
	}

	//send the update
	Udp.beginPacket(ttnServer, (int) PORT1);

#ifdef SERVER2
	delay(1);
	Udp.beginPacket((char *)SERVER2, (int) PORT2);
#endif

	if ((l = Udp.write((char *)msg, length)) != length) {
		Serial.println("sendUdp:: Error write");
	}
	else {
		if (debug>=2) {
			Serial.print(F("sendUdp: sent "));
			Serial.print(l);
			Serial.println(F(" bytes"));
		}
	}
	yield();
	Udp.endPacket();
}


// ----------------------------------------------------------------------------
// connect to UDP – returns true if successful or false if not
// ----------------------------------------------------------------------------
bool UDPconnect() {

	bool ret = false;
	if (debug>=1) Serial.println(F("Connecting to UDP"));
	unsigned int localPort = 1701;			// XXX Do not listen to return messages from WiFi
	if (Udp.begin(localPort) == 1) {
		if (debug>=1) Serial.println(F("Connection successful"));
		ret = true;
	}
	else{
		//Serial.println("Connection failed");
	}
	return(ret);
}

// =================================================================================
// LORA GATEWAY FUNCTIONS
// The LoRa supporting functions are in the section below

// ----------------------------------------------------------------------------
// The SS (Chip select) pin is used to make sure the RFM95 is selected
// ----------------------------------------------------------------------------
void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

// ----------------------------------------------------------------------------
// ... or unselected
// ----------------------------------------------------------------------------
void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

// ----------------------------------------------------------------------------
// Read one byte value, par addr is address
// Returns the value of register(addr)
// ----------------------------------------------------------------------------
byte readRegister(byte addr)
{
    selectreceiver();
	SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
	SPI.transfer(addr & 0x7F);
	uint8_t res = SPI.transfer(0x00);
	SPI.endTransaction();
    unselectreceiver();
    return res;
}

// ----------------------------------------------------------------------------
// Write value to a register with address addr. 
// Function writes one byte at a time.
// ----------------------------------------------------------------------------
void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
	SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
	SPI.transfer(spibuf[0]);
	SPI.transfer(spibuf[1]);
	SPI.endTransaction();
    unselectreceiver();
}

// ----------------------------------------------------------------------------
// This LoRa function reads a message from the LoRa transceiver
// returns true when message correctly received or fails on error 
// (CRC error for example)
// ----------------------------------------------------------------------------
bool receivePkt(char *payload)
{
    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);

    cp_nb_rx_rcv++;											// Receive statistics counter

    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        Serial.println(F("CRC error"));
        writeRegister(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {

        cp_nb_rx_ok++;										// Receive OK statistics counter

        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);
        }
		//yield();
    }
    return true;
}

// ----------------------------------------------------------------------------
// Setup the LoRa environment on the connected transceiver.
// - Determine the correct transceiver type (sx1272/RFM92 or sx1276/RFM95)
// - Set the frequency to listen to (1-channel remember)
// - Set Spreading Factor (standard SF7)
// The reset RST pin might not be necessary for at least the RGM95 transceiver
// ----------------------------------------------------------------------------
void SetupLoRa()
{
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readRegister(REG_VERSION);					// Read the LoRa chip version id
    if (version == 0x22) {
        // sx1272
        Serial.println(F("SX1272 detected, starting."));
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            Serial.println(F("SX1276 detected, starting."));
            sx1272 = false;
        } else {
            Serial.print(F("Unrecognized transceiver, version: "));
            Serial.println(version,HEX);
            die("");
        }
    }

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

	// Set spreading Factor
    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG,0x0B);
        } else {
            writeRegister(REG_MODEM_CONFIG,0x0A);
        }
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeRegister(REG_MODEM_CONFIG3,0x04);
        }
        writeRegister(REG_MODEM_CONFIG,0x72);
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeRegister(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeRegister(REG_HOP_PERIOD,0xFF);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
    writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);
}


// ----------------------------------------------------------------------------
// Send periodic status message to server even when we do not receive any
// data.
// Parameter is socketr to TX to
// ----------------------------------------------------------------------------
void sendstat() {
// XXX removed static
    char status_report[STATUS_SIZE]; 						// status report as a JSON object
    char stat_timestamp[32];								// XXX was 24
    time_t t;
	char clat[8]={0};
	char clon[8]={0};

    int stat_index=0;
	
    // pre-fill the data buffer with fixed fields
    status_report[0]  = PROTOCOL_VERSION;
    status_report[3]  = PKT_PUSH_DATA;
	
	// READ MAC ADDRESS OF ESP8266
    status_report[4]  = MAC_array[0];
    status_report[5]  = MAC_array[1];
    status_report[6]  = MAC_array[2];
    status_report[7]  = 0xFF;
    status_report[8]  = 0xFF;
    status_report[9]  = MAC_array[3];
    status_report[10] = MAC_array[4];
    status_report[11] = MAC_array[5];

    uint8_t token_h   = (uint8_t)rand(); 					// random token
    uint8_t token_l   = (uint8_t)rand();					// random token
    status_report[1]  = token_h;
    status_report[2]  = token_l;
    stat_index = 12;										// 12-byte header
	
    t = now();												// get timestamp for statistics
	// %F Short YYYY-MM-DD date, %T  %H:%M:%S,  %Z CET
    //strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));
		
	sprintf(stat_timestamp, "%d-%d-%d %d:%d:%d CET", year(),month(),day(),hour(),minute(),second());
	yield();
	
	ftoa(lat,clat,4);										// Convert lat to char array with 4 decimals
	ftoa(lon,clon,4);										// As Arduino CANNOT prints floats
	
	// Build the Status message in JSON format
	// XXX Split this one up...
    int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE-stat_index, 
		"{\"stat\":{\"time\":\"%s\",\"lati\":%s,\"long\":%s,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%u.0,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}", 
		stat_timestamp, clat, clon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 0, 0, 0,platform,email,description);
	yield();												// Give way to the internal housekeeping of the ESP8266
    stat_index += j;
    status_report[stat_index] = 0; 							// add string terminator, for safety

    if (debug>=1) {
		Serial.print(F("stat update: <"));
		Serial.print(stat_index);
		Serial.print(F("> "));
		Serial.println((char *)(status_report+12));			// DEBUG: display JSON stat
	}
    //send the update
    sendUdp(status_report, stat_index);

}

// ----------------------------------------------------------------------------
// Receive a LoRa package
//
// Receive a LoRa message and fill the buff_up char buffer.
// returns values:
// - returns the length of string returned in buff_up
// - returns -1 when no message arrived.
// ----------------------------------------------------------------------------
int receivepacket(char *buff_up) {

    long int SNR;
    int rssicorr;
	char cfreq[12] = {0};										// Character array to hold freq in MHz

    if(digitalRead(dio0) == 1)									// READY?
    {
        if(receivePkt(message)) {
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {											// Probably SX1276 or RFM95
                rssicorr = 157;
            }
			
			if (debug>=1) {
			    Serial.print(F("Packet RSSI: "));
				Serial.print(readRegister(0x1A)-rssicorr);
				Serial.print(F(" RSSI: "));
				Serial.print(readRegister(0x1B)-rssicorr);
				Serial.print(F(" SNR: "));
				Serial.print(SNR);
				Serial.print(F(" Length: "));
				Serial.print((int)receivedbytes);
				Serial.println();
				yield();
			}
			
            int j;
			// XXX Base64 library is nopad. So we may have to add padding characters until
			// 	length is multiple of 4!
			int encodedLen = base64_enc_len(receivedbytes);		// max 341
			base64_encode(b64, message, receivedbytes);			// max 341
			
            //j = bin_to_b64((uint8_t *)message, receivedbytes, (char *)(b64), 341);
            //fwrite(b64, sizeof(char), j, stdout);

            int buff_index=0;

            // pre-fill the data buffer with fixed fields
            buff_up[0] = PROTOCOL_VERSION;
            buff_up[3] = PKT_PUSH_DATA;

			// XXX READ MAC ADDRESS OF ESP8266
            buff_up[4]  = MAC_array[0];
            buff_up[5]  = MAC_array[1];
            buff_up[6]  = MAC_array[2];
            buff_up[7]  = 0xFF;
            buff_up[8]  = 0xFF;
            buff_up[9]  = MAC_array[3];
            buff_up[10] = MAC_array[4];
            buff_up[11] = MAC_array[5];

            // start composing datagram with the header 
            uint8_t token_h = (uint8_t)rand(); 					// random token
            uint8_t token_l = (uint8_t)rand(); 					// random token
            buff_up[1] = token_h;
            buff_up[2] = token_l;
            buff_index = 12; /* 12-byte header */

            // TODO: tmst can jump if time is (re)set, not good.
            struct timeval now;
            gettimeofday(&now, NULL);
            uint32_t tmst = (uint32_t)(now.tv_sec*1000000 + now.tv_usec);

            // start of JSON structure that will make payload
            memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
            buff_index += 9;
            buff_up[buff_index] = '{';
            ++buff_index;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%u", tmst);
            buff_index += j;
			ftoa((double)freq/1000000,cfreq,6);						// XXX This can be done better
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%s", 0, 0, cfreq);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
            buff_index += 9;
            memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
            buff_index += 14;
            /* Lora datarate & bandwidth, 16-19 useful chars */
            switch (sf) {
            case SF7:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                buff_index += 12;
                break;
            case SF8:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                buff_index += 12;
                break;
            case SF9:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                buff_index += 12;
                break;
            case SF10:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                buff_index += 13;
                break;
            case SF11:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                buff_index += 13;
                break;
            case SF12:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                buff_index += 13;
                break;
            default:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                buff_index += 12;
            }
            memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
            buff_index += 6;
            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
            buff_index += 13;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%li", SNR);
            buff_index += j;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%d,\"size\":%u", readRegister(0x1A)-rssicorr, receivedbytes);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
            buff_index += 9;

			// Use gBase64 library
			encodedLen = base64_enc_len(receivedbytes);		// max 341
			j = base64_encode((char *)(buff_up + buff_index), message, receivedbytes);

            buff_index += j;
            buff_up[buff_index] = '"';
            ++buff_index;

            // End of packet serialization
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = ']';
            ++buff_index;
            // end of JSON datagram payload */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = 0; 						// add string terminator, for safety

			if (debug>=1) {
				Serial.print(F("rxpk update: "));
				Serial.println((char *)(buff_up + 12));		// DEBUG: display JSON payload
			}
            
			return(buff_index);
			
        } // received a message
    } // dio0=1
	return(-1);
}


// ================================================================================
// WEBSERVER FUNCTIONS (PORT 8080)

#if A_SERVER==1


// ----------------------------------------------------------------------------
// Output the 4-byte IP address for easy printing
// ----------------------------------------------------------------------------
String printIP(IPAddress ipa) {
	String response;
	response+=(IPAddress)ipa[0]; response+=".";
	response+=(IPAddress)ipa[1]; response+=".";
	response+=(IPAddress)ipa[2]; response+=".";
	response+=(IPAddress)ipa[3];
	return (response);
}

// ----------------------------------------------------------------------------
// stringTime
// Only when RTC is present we print real time values
// t contains number of milli seconds since system started that the event happened.
// So a value of 100 wold mean that the event took place 1 minute and 40 seconds ago
// ----------------------------------------------------------------------------
String stringTime(unsigned long t) {
	String response;
	String Days[7]={"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};

	if (t==0) { response = " -none- "; return(response); }
	
	// now() gives seconds since 1970
	time_t eventTime = now() - ((millis()-t)/1000);
	byte _hour   = hour(eventTime);
	byte _minute = minute(eventTime);
	byte _second = second(eventTime);
	
	response += Days[weekday(eventTime)-1]; response += " ";
	response += day(eventTime); response += "-";
	response += month(eventTime); response += "-";
	response += year(eventTime); response += " ";
	if (_hour < 10) response += "0";
	response += _hour; response +=":";
	if (_minute < 10) response += "0";
	response += _minute; response +=":";
	if (_second < 10) response += "0";
	response += _second;
	return (response);
}


// ----------------------------------------------------------------------------
// WIFI SERVER
//
// This funtion implements the WiFI Webserver (very simple one). The purpose
// of this server is to receive simple admin commands, and execute these
// results are sent back to the web client.
// Commands: DEBUG, ADDRESS, IP, CONFIG, GETTIME, SETTIME
// The webpage is completely built response and then printed on screen.
// ----------------------------------------------------------------------------
String WifiServer(char *cmd, char *arg) {

	String response;
	char *dup, *pch;

	yield();	
	if (debug >=2) { Serial.print(F("WifiServer new client")); }

	// These can be used as a single argument
	if (strcmp(cmd, "DEBUG")==0) {									// Set debug level 0-2
		debug=atoi(arg); response+=" debug="; response+=arg;
	}
	if (strcmp(cmd, "IP")==0) {										// List local IP address
		response+=" local IP="; 
		response+=(IPAddress) WiFi.localIP()[0]; response += ".";
		response+=(IPAddress) WiFi.localIP()[1]; response += ".";
		response+=(IPAddress) WiFi.localIP()[2]; response += ".";
		response+=(IPAddress) WiFi.localIP()[3];
	}

	if (strcmp(cmd, "GETTIME")==0) { response += "gettime tbd"; }	// Get the local time
	if (strcmp(cmd, "SETTIME")==0) { response += "settime tbd"; }	// Set the local time
	if (strcmp(cmd, "HELP")==0)    { response += "Display Help Topics"; }
	if (strcmp(cmd, "RESET")==0)   { response += "Resetting Statistics"; 
		cp_nb_rx_rcv = 0;
		cp_nb_rx_ok = 0;
		cp_up_pkt_fwd = 0;
	}

	// Do work, fill the webpage
	delay(15);	
	response +="<!DOCTYPE HTML>";
	response +="<HTML><HEAD>";
	response +="<TITLE>ESP8266 1ch Gateway</TITLE>";
	response +="</HEAD>";
	response +="<BODY>";
		
	response +="<h1>ESP Gateway Config:</h1>";
	response +="Version: "; response+=VERSION;
	response +="<br>ESP is alive since "; response+=stringTime(1); 
	response +="<br>Current time is    "; response+=stringTime(millis()); 
	response +="<br>";
		
	response +="<h2>WiFi Config</h2>";
	response +="<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
	response +="<tr>";
	response +="<th style=\"background-color: green; color: white;\">Parameter</th>";
	response +="<th style=\"background-color: green; color: white;\">Value</th>";
	response +="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">IP Address</td><td style=\"border: 1px solid black;\">"; response+=printIP((IPAddress)WiFi.localIP()); response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">IP Gateway</td><td style=\"border: 1px solid black;\">"; response+=printIP((IPAddress)WiFi.gatewayIP()); response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">NTP Server</td><td style=\"border: 1px solid black;\">"; response+=NTP_TIMESERVER; response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">LoRa Router</td><td style=\"border: 1px solid black;\">"; response+=_TTNSERVER; response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">LoRa Router IP</td><td style=\"border: 1px solid black;\">"; response+=printIP((IPAddress)ttnServer); response+="</tr>";
	response +="</table>";
	
	response +="<h2>System Status</h2>";
	response +="<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
	response +="<tr>";
	response +="<th style=\"background-color: green; color: white;\">Parameter</th>";
	response +="<th style=\"background-color: green; color: white;\">Value</th>";
	response +="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Free heap</td><td style=\"border: 1px solid black;\">"; response+=ESP.getFreeHeap(); response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">ESP Chip ID</td><td style=\"border: 1px solid black;\">"; response+=ESP.getChipId(); response+="</tr>";
	response +="</table>";
		
	response +="<h2>LoRa Status</h2>";
	response +="<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
	response +="<tr>";
	response +="<th style=\"background-color: green; color: white;\">Parameter</th>";
	response +="<th style=\"background-color: green; color: white;\">Value</th>";
	response +="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Frequency</td><td style=\"border: 1px solid black;\">"; response+=freq; response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Spreading Factor</td><td style=\"border: 1px solid black;\">"; response+=sf; response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Gateway ID</td><td style=\"border: 1px solid black;\">";	
	response +=String(MAC_array[0],HEX);									// The MAC array is always returned in lowercase
	response +=String(MAC_array[1],HEX); 
	response +=String(MAC_array[2],HEX); 
	response +="ffff"; 
	response +=String(MAC_array[3],HEX); 
	response +=String(MAC_array[4],HEX); 
	response +=String(MAC_array[5],HEX);
	response+="</tr>";
	response +="</table>";
		
	response +="<h2>Statistics</h2>";
	delay(1);
	response +="<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
	response +="<tr>";
	response +="<th style=\"background-color: green; color: white;\">Counter</th>";
	response +="<th style=\"background-color: green; color: white;\">Value</th>";
	response +="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Packages Received</td><td style=\"border: 1px solid black;\">"; response +=cp_nb_rx_rcv; response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Packages OK </td><td style=\"border: 1px solid black;\">"; response +=cp_nb_rx_ok; response+="</tr>";
	response +="<tr><td style=\"border: 1px solid black;\">Packages Forwarded</td><td style=\"border: 1px solid black;\">"; response +=cp_up_pkt_fwd; response+="</tr>";
	response +="<tr><td>&nbsp</td><td> </tr>";
			
	response +="</table>";

	response +="<br>";
	response +="<h2>Settings</h2>";
	response +="Click <a href=\"/RESET\">here</a> to reset statistics<br>";

	response +="Debug level is: "; 
	response += debug; 
	response +=" set to: ";
	response +=" <a href=\"DEBUG=0\">0</a>";
	response +=" <a href=\"DEBUG=1\">1</a>";
	response +=" <a href=\"DEBUG=2\">2</a><br>";
		
	response +="Click <a href=\"/HELP\">here</a> to explain Help and REST options<br>";
	response +="</BODY></HTML>";

	delay(5);
	free(dup);									// free the memory used, before jumping to other page
	return (response);
}



#endif



// ========================================================================
// MAIN PROGRAM (SETUP AND LOOP)

// ----------------------------------------------------------------------------
// Setup code (one time)
// ----------------------------------------------------------------------------
void setup () {
	Serial.begin(_BAUDRATE);						// As fast as possible for bus

	delay(1500);
	yield();
		
	if (debug>=1) {
		Serial.print(F("! debug: ")); 
		Serial.println(debug);
		yield();
	}
	
	// Setup WiFi UDP connection. Give it some time ..
	if (WlanConnect( (char *) _SSID, (char *)_PASS) < 0) {
		Serial.print(F("Error Wifi network SSID: "));
		Serial.print(_SSID);
		Serial.print(F(":"));
		Serial.print(_PASS);
		Serial.print(F(", Local IP: "));
		//Serial.print(WiFi.localIP());
		Serial.println();
		yield();
	}
	else {
		Serial.println("Wlan Connected");
		delay(200);
		// If we are here we are connected to WLAN
		// So now test the UDP function
		if (!UDPconnect()) {
			Serial.println("Error UDPconnect");
		}
		delay(500);
	}
	 
	WiFi.macAddress(MAC_array);
    for (int i = 0; i < sizeof(MAC_array); ++i){
      sprintf(MAC_char,"%s%02x:",MAC_char,MAC_array[i]);
    }
	Serial.print("MAC: ");
    Serial.println(MAC_char);
	
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);
	
	SPI.begin();
	delay(1000);
    SetupLoRa();
	delay(500);
	
	// We choose the Gateway ID to be the Ethernet Address of our Gateway card
    // display results of getting hardware address
	// 
    Serial.print("Gateway ID: ");
    Serial.print(MAC_array[0],HEX);
    Serial.print(MAC_array[1],HEX);
    Serial.print(MAC_array[2],HEX);
	Serial.print(0xFF, HEX);
	Serial.print(0xFF, HEX);
    Serial.print(MAC_array[3],HEX);
    Serial.print(MAC_array[4],HEX);
    Serial.print(MAC_array[5],HEX);

    Serial.print(", Listening at SF");
	Serial.print(sf);
	Serial.print(" on ");
	Serial.print((double)freq/1000000);
	Serial.println(" Mhz.");

	WiFi.hostByName(_TTNSERVER, ttnServer);					// Use DNS to get server IP once
	delay(500);
	
	setupTime();											// Set NTP time host and interval
	setTime((time_t)getNtpTime());
	Serial.print("time "); printTime();
	Serial.println();

#if A_SERVER==1	
	server.on("/", []() {
		webPage = WifiServer("","");
		server.send(200, "text/html", webPage);
	});
	server.on("/HELP", []() {
		webPage = WifiServer("HELP","");
		server.send(200, "text/html", webPage);
	});
	server.on("/RESET", []() {
		webPage = WifiServer("RESET","");
		server.send(200, "text/html", webPage);
	});
	server.on("/DEBUG=0", []() {
		webPage = WifiServer("DEBUG","0");
		server.send(200, "text/html", webPage);
	});
	server.on("/DEBUG=1", []() {
		webPage = WifiServer("DEBUG","1");
		server.send(200, "text/html", webPage);
	});
	server.on("/DEBUG=2", []() {
		webPage = WifiServer("DEBUG","2");
		server.send(200, "text/html", webPage);
	});
	server.begin();											// Start the webserver
	Serial.print(F("Admin Server started on port "));
	Serial.println(SERVERPORT);
#endif	
	delay(1000);											// Wait after setup
	Serial.println("-----------------------------");
}

// ----------------------------------------------------------------------------
// LOOP
// This is the main program that is executed time and time again.
// We need to geive way to the bacjend WiFi processing that 
// takes place somewhere in the ESP8266 firmware and therefore
// we include yield() statements at important points.
//
// Note: If we spend too much time in user processing functions
//	and the backend system cannot do its housekeeping, the watchdog
// function will be executed which means effectively that the 
// program crashes.
// ----------------------------------------------------------------------------
void loop ()
{
	int buff_index;
	char buff_up[TX_BUFF_SIZE]; 						// buffer to compose the upstream packet
	
	// Receive Lora messages
    if ((buff_index = receivepacket(buff_up)) >= 0) {	// read is successful
		yield();
		sendUdp(buff_up, buff_index);					// We can send to multiple sockets if necessary
	}
	else {
		// No message received
	}
	yield();
	
	// Receive WiFi messages. This is important since the TTN broker will return confirmation
	// messages on UDP for every message sent by the gateway.
	int packetSize = Udp.parsePacket();
	if (packetSize >0) {
		yield();
		readUdp(packetSize);
	}
	yield();
	
	uint32_t nowseconds = (uint32_t) millis() /1000;
    if (nowseconds - lasttime >= 30) {					// Send status every 30 seconds
        sendstat();
		lasttime = nowseconds;
    }

	// Handle the WiFi server part of this sketch. Mainly used for administration of the node
#if A_SERVER==1
	server.handleClient();
#endif	
	
	yield();
}
