#include <ArduinoJson.h>
#include <QNEthernet.h>
#include <VL53L0X.h>
#include <Wire.h>

using namespace qindesign::network;

/*
 * Local port to send data on.
 * NOTE: this is not in the FRC range to be sent over the radio to the
 * driverstation. This is on purpose, we do not want this data sent to the
 * driverstation!
 */
constexpr uint16_t UDP_PORT = 25000;

/*
 * IP Configuration
 */
const IPAddress STATIC_IP(10, 42, 01, 12);
const IPAddress NETMASK(255, 255, 255, 0);
const IPAddress GATEWAY_IP(10, 42, 01, 1);

const IPAddress MULTICAST_IP(10, 42, 01, 2);

EthernetUDP udp;

uint8_t ReplyBuffer[] = "acknowledged";  // a string to send back

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

StaticJsonDocument<1000> doc;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000) {
        delay(10);
    }
    stdPrint = &Serial;  // Make printf work, a QNEthernet feature

    // Print the MAC address
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);

    // Start Ethernet
    printf("Starting Ethernet with static IP...\n");
    if (!Ethernet.begin(STATIC_IP, NETMASK, GATEWAY_IP)) {
        printf("Failed to start Ethernet!\n");
        return;
    }
    Ethernet.setDNSServerIP(GATEWAY_IP);

    // Print IP addresses
    IPAddress ip = Ethernet.localIP();
    printf("    Local IP    = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.subnetMask();
    printf("    Subnet mask = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.gatewayIP();
    printf("    Gateway     = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.dnsServerIP();
    printf("    DNS         = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);

    udp.beginMulticast(MULTICAST_IP, UDP_PORT);

    Wire.begin();
    Wire1.begin();
    Wire2.begin();

    sensor1.setTimeout(500);
    if (!sensor1.init()) {
        Serial.println("Failed to detect and initialize sensor1!");
        doc["sensor1.status"] = "disconnected";
    }
    sensor1.startContinuous();

    sensor2.setBus(&Wire1);
    sensor2.setTimeout(500);
    if (!sensor2.init()) {
        Serial.println("Failed to detect and initialize sensor2!");
        doc["sensor2.status"] = "disconnected";
    }
    sensor2.startContinuous();

    sensor3.setBus(&Wire2);
    sensor3.setTimeout(500);
    if (!sensor3.init()) {
        Serial.println("Failed to detect and initialize sensor3!");
        doc["sensor3.status"] = "disconnected";
    }
    sensor3.startContinuous();
}

uint8_t i = 0;
void loop() {
    int distance = sensor1.readRangeContinuousMillimeters();
    if (sensor1.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
        doc["sensor1.status"] = "timeout";
    } else if (distance == -1 || distance == 65535) {
        doc["sensor1.status"] = "failed";
    } else {
        doc["sensor1.status"] = "connected";
    }
    doc["sensor1.mm"] = distance;

    int distance2 = sensor2.readRangeContinuousMillimeters();
    if (sensor2.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
        doc["sensor2.status"] = "timeout";
    } else if (distance2 == -1 || distance2 == 65535) {
        doc["sensor2.status"] = "failed";
    } else {
        doc["sensor2.status"] = "connected";
    }
    doc["sensor2.mm"] = distance2;

    int distance3 = sensor3.readRangeContinuousMillimeters();
    if (sensor3.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
        doc["sensor3.status"] = "timeout";
    } else if (distance3 == -1 || distance3 == 65535) {
        doc["sensor3.status"] = "failed";
    } else {
        doc["sensor3.status"] = "connected";
    }
    doc["sensor3.mm"] = distance3;


    /*
     * Read our sensor values (fake value for now)
     */
    i = (i + 1) % 100;
    doc["test"] = i;

    /*
     * Send our JSON packet over UDP to the multicast address
     */
    udp.beginPacket(MULTICAST_IP, UDP_PORT);
    serializeJson(doc, udp);
    udp.endPacket();

    /*
     * Also print our JSON packet
     */
    serializeJson(doc, Serial);
    Serial.println("");

    delay(100);
}