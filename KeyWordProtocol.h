#include <SoftwareSerial.h>

#define ADR_Engine 0x01 // Engine
#define ADR_Dashboard 0x17 // Instruments

#define MAX_Pocket_Size 0x0F
#define SENSORS_PER_GROUP 4

struct Sensor{
  uint8_t type;
  uint8_t a;
  uint8_t b;
};

class KeyWordProtocol{
  public:
    KeyWordProtocol(uint8_t receivePin, uint8_t transmitPin);
    bool connect(uint8_t addr, int baudrate);
    void disconnect();
    bool isConnected();
    bool sendBlock(uint8_t *data, uint8_t arraySize);
    bool readBlock(uint8_t data[], uint8_t arraySize);
    Sensor getSensorData(uint8_t group, uint8_t id);
  private:
    uint8_t readByte();
    bool writeByte(uint8_t data);
    bool sendHeader(uint8_t size);
    void counterChanged();
    void sendInitPocket(uint8_t addr);
    uint8_t read();
    void write(uint8_t data);    
    SoftwareSerial *serial;
    bool connected = false;
    uint8_t txpin;
    uint8_t counter = 0;
};