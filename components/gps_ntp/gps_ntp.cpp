#include "esphome.h"

#include <TimeLib.h> // Time functions  https://github.com/PaulStoffregen/Time

#define SYNC_INTERVAL 10 // time, in seconds, between GPS sync attempts
#define SYNC_TIMEOUT 30  // time(sec) without GPS input before error
time_t syncTime = 0;     // time of last GPS or RTC synchronization
bool gpsLocked = false;

WiFiUDP Udp;

#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

const unsigned long seventyYears = 2208988800UL; // to convert unix time to epoch

void startNTP() {
  Udp.begin(NTP_PORT);
}

void processNTP(bool gpsLocked) {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    Serial.print("NTP request from ");
    Serial.println(Remote.toString());

#ifdef DEBUG
    Serial.println();
    Serial.print("Received UDP packet size ");
    Serial.println(packetSize);
    Serial.print("From ");

    for (int i = 0; i < 4; i++) {
      Serial.print(Remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.print(PortNum);

    byte LIVNMODE = packetBuffer[0];
    Serial.print("  LI, Vers, Mode :");
    Serial.print(LIVNMODE, HEX);

    byte STRATUM = packetBuffer[1];
    Serial.print("  Stratum :");
    Serial.print(STRATUM, HEX);

    byte POLLING = packetBuffer[2];
    Serial.print("  Polling :");
    Serial.print(POLLING, HEX);

    byte PRECISION = packetBuffer[3];
    Serial.print("  Precision :");
    Serial.println(PRECISION, HEX);

    for (int z = 0; z < NTP_PACKET_SIZE; z++) {
      Serial.print(packetBuffer[z], HEX);
      if (((z + 1) % 4) == 0) {
        Serial.println();
      }
    }
    Serial.println();
#endif

    packetBuffer[0] = 0b00100100; // LI, Version, Mode
    if (gpsLocked) {
      packetBuffer[1] = 1; // stratum 1 if synced with GPS
    } else {
      packetBuffer[1] = 16; // stratum 16 if not synced
    }
    // think that should be at least 4 or so as you do not use fractional seconds
    // packetBuffer[1] = 4;    // stratum
    packetBuffer[2] = 6;    // polling minimum
    packetBuffer[3] = 0xFA; // precision

    packetBuffer[4] = 0; // root delay
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;

    packetBuffer[8] = 0; // root dispersion
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;

    // int year;
    // byte month, day, hour, minute, second, hundredths;
    // unsigned long date, time, age;
    uint32_t timestamp, tempval;
    time_t t = now();

    // gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    // timestamp = numberOfSecondsSince1900Epoch(year,month,day,hour,minute,second);

    // timestamp = numberOfSecondsSince1900Epoch(year(t), month(t), day(t), hour(t), minute(t), second(t));
    //  Better, less verbose, more elegant timestamp calculation, suggested by cheise @ Github - 20230206
    timestamp = t + seventyYears;

#ifdef DEBUG
    Serial.println(timestamp);
    // print_date(gps);
#endif

    tempval = timestamp;

    if (gpsLocked) {
      packetBuffer[12] = 71; //"G";
      packetBuffer[13] = 80; //"P";
      packetBuffer[14] = 83; //"S";
      packetBuffer[15] = 0;  //"0";
    } else {
      // Set refid to IP address if not locked
      IPAddress myIP = WiFi.localIP();
      packetBuffer[12] = myIP[0];
      packetBuffer[13] = myIP[1];
      packetBuffer[14] = myIP[2];
      packetBuffer[15] = myIP[3];
    }

    // reference timestamp
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval)&0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    // copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval)&0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // transmitt timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval)&0xFF;

    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;

    // Reply to the IP address and port that sent the NTP request

    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

void SyncWithGPS(TinyGPSPlus &gps) {
  int y;
  // byte h, m, s, mon, d, hundredths;
  byte h, m, s, mon, d;
  unsigned long age;
  y = gps.date.year();
  mon = gps.date.month();
  d = gps.date.day();
  h = gps.time.hour();
  m = gps.time.minute();
  s = gps.time.second();
  age = gps.location.age();
  // gps.crack_datetime(&y, &mon, &d, &h, &m, &s, NULL, &age); // get time from GPS

  if (age < 1000) // dont use data older than 1 second
  {
    setTime(h, m, s, d, mon, y); // copy GPS time to system time
    Serial.print("Time from GPS: ");
    Serial.print(h);
    Serial.print(":");
    Serial.print(m);
    Serial.print(":");
    Serial.println(s);

    syncTime = now(); // remember time of this sync
    if (!gpsLocked) {
      Serial.print("GPS sychronized - ");
      Serial.print(gps.satellites.value());
      Serial.println(" satellites");
    }

    gpsLocked = true; // set flag that time is reflects GPS time

  } else {
    // Serial.println("Old data with age: ");
    // Serial.println(age);
  }
}

void SyncCheck(TinyGPSPlus &tiny_gps) {
  unsigned long timeSinceSync = now() - syncTime; // how long has it been since last sync?
  if (timeSinceSync >= SYNC_INTERVAL) {           // is it time to sync with GPS yet?
    SyncWithGPS(tiny_gps);                        // yes, so attempt it.
  }
  if (timeSinceSync >= SYNC_TIMEOUT) // GPS sync has failed
  {
    if (gpsLocked) {
      Serial.println("GPS sync lost!");
    }
    gpsLocked = false; // flag that clock is no longer in GPS sync
  }
}

namespace esphome {
namespace gps {

bool first_loop_flag = true;

void GPS_NTP_Server::from_tiny_gps_(TinyGPSPlus &tiny_gps) {
  SyncCheck(tiny_gps);
}

void GPS_NTP_Server::loop() {
  if (first_loop_flag) { // wifi must init first...
    first_loop_flag = false;
    startNTP();
  }
  processNTP(gpsLocked);
}

} // namespace gps
} // namespace esphome