#include <QNEthernet.h>
#include <ArduinoJson.h>

using namespace qindesign::network;

/*
 * Local port to send data on.
 * NOTE: this is not in the FRC range to be sent over the radio to the driverstation. 
 * This is on purpose, we do not want this data sent to the driverstation!
 */
constexpr uint16_t UDP_PORT = 25000;
                        
/*
 * IP Configuration
 */
const IPAddress STATIC_IP(10, 42, 01, 12);
const IPAddress NETMASK(255, 255, 255, 0);
const IPAddress GATEWAY_IP(10, 42, 01, 1);

const IPAddress MULTICAST_IP(239, 42, 01, 1);

EthernetUDP udp;

uint8_t ReplyBuffer[] = "acknowledged";        // a string to send back


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
}

StaticJsonDocument<1000> doc;

uint8_t i = 0;
void loop() {
    /*
     * Read our sensor values (fake value for now)
     */
    i = (i + 1) % 100;
    doc["sensor1"] = i;

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