#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

static constexpr size_t BUFFER_SIZE = 4;   // ต้องตรงกับ STM32 และหาร 4 ลงตัว
uint8_t* tx_buf;
uint8_t* rx_buf;

void setup()
{
    Serial.begin(115200);
    delay(500);

    // จอง DMA buffer (aligned + DMA-capable)
    tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    rx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    memset(tx_buf, 0, BUFFER_SIZE);
    memset(rx_buf, 0, BUFFER_SIZE);

    slave.setDataMode(SPI_MODE0);        // ตรงกับ CPOL=0 CPHA=0 ของ STM32
    slave.setMaxTransferSize(BUFFER_SIZE);
    slave.setQueueSize(1);

    slave.begin(VSPI, 18, 19, 23, 5);    // SCK, MISO, MOSI, CS

    tx_buf[0] = 0xAA;
    Serial.println("Slave ready");
}

void loop()
{
    // block รอจนกว่า master จะตีคล็อกครบและ CS ขึ้น
    const size_t received = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    Serial.print("Received ");
    Serial.print(received);
    Serial.print(" bytes: ");
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        Serial.printf("%02X ", rx_buf[i]);
    }
    Serial.println();
}