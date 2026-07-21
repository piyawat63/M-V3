//#include "soc/soc.h"
//#include "soc/rtc_cntl_reg.h"
#include <EthernetESP32.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "myconfig.h"

//Time curr_time;


typedef struct {
  //int id;
  uint8_t t_sec;
  uint8_t t_min;
  uint8_t t_hour;
  uint8_t t_base_sec;
  
  int16_t ns_max_adc = 0;
  int16_t ns_min_adc = 0;
  uint32_t ns_max_c = 0;
  uint32_t ns_min_c = 0;

  // ADC value info
  int16_t ew_max_adc = 0;
  int16_t ew_min_adc = 0;
  uint32_t ew_max_c = 0;
  uint32_t ew_min_c = 0;

  float ns_max_v = 0.0;
  float ns_min_v = 0.0;
  float ew_max_v = 0.0;
  float ew_min_v = 0.0;

  double ns_max_t_sec;
  double ns_min_t_sec;
  double ew_max_t_sec;
  double ew_min_t_sec;

  // GNSS Time & Position info
  String gnss_lat;
  String gnss_long;
  String gnss_time;
  String raw_gps;
  String raw_stm;
} DataRecord_t;

DataRecord_t m_field_data;





/* 
  * Function prototype
*/
int gps_read();
int stm_read();
void calculate_data();

String build_mqtt_payload();
void publish_data(String mqtt_payload);
void send_data(DataRecord_t rec);

void gps_uart_send_cmd(char *cmd);
int get_substr(const String str, const char delim, int start_pos, String *ret_str);
void callback(char *topic, byte *payload, unsigned int length);

#ifdef DEBUG_FN_PRINT_DATA
void print_data();
#endif


/* 
  * Global Variables
*/
HardwareSerial *gps_serial;
HardwareSerial *stm_serial;
HardwareSerial *debug_serial;

const char *mqtt_server = MQTT_SERVER;  // Broker address
IPAddress ip(192, 168, 1, 110);
EMACDriver driver(ETH_PHY_LAN8720, 23, 18, 16);
EthernetClient eth_client;
PubSubClient mqtt_client(mqtt_server, MQTT_PORT, callback, eth_client);

// End Global Variables


// ====================================================================================================================
//  Core Function
// ====================================================================================================================
/*
  * Setup Function
*/
void setup() {
  serial_setup();
  debug_serial->println("ESP32 started");
  gps_setup();
  ethernet_setup();
  mqtt_setup();
  debug_serial->println("ESP32 Application started\n");
}

/*
  * Loop Function
*/

void loop() {

  // put your main code here, to run repeatedly:
  //DataRecord_t data_rec;
  //int byte_read;

  //String mqtt_pub_str = "This is a test from K**ell";


  // Read GPS first, more important as it is affect timing
  if (gps_serial->available() != 0) {
    gps_read();
  }


  if (stm_serial->available() != 0) {
    // Mark base time
   // m_field_data.t_base_sec = m_field_data.t_sec;
    if (stm_read()) {
      // Interpret and calculate and send data
      String mqtt_payload = build_mqtt_payload();
      //debug_serial->write(mqtt_payload.c_str());
      //debug_serial->println("");
      if (!mqtt_client.connected()) {
      reconnect();
      }
      publish_data(mqtt_payload);
  
    }
  }

  mqtt_client.loop();
  //delay(5000);
  //debug_serial->println("Testing");
  //mqtt_client.publish(MQTT_PUB_TOPIC, mqtt_pub_str.c_str());
  //send_data(data_rec);
  //mqtt_client.loop();
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    debug_serial->print("Attempting MQTT connection to ");
    debug_serial->println(MQTT_SERVER);
    
 if (mqtt_client.connect(mqtt_id, mqtt_user, mqtt_pass)) {

      debug_serial->println("...connected");
      debug_serial->println("-> MQTT client connected");

      mqtt_client.subscribe(MQTT_SUB_DATA_TOPIC);
      mqtt_client.subscribe(MQTT_SUB_CMD_TOPIC);
    } else {
      Serial.print("...failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
// ====================================================================================================================



// ====================================================================================================================
// ====================================================================================================================

/* Send command to GNSS module function
 * the cmd is exclude start symbol ($) and end symbol (*)
 * this function will put '$' and '*XX<CR><LF>' and send over the uart interface
 */
void gps_uart_send_cmd(char *cmd) {
  char chksum = 0;
  char *pcmd = cmd;
  char gps_cmd[100];
  do {
    chksum ^= (*pcmd);
    pcmd++;
  } while (*pcmd != '\0');
  sprintf(gps_cmd, "$%s*%02X\r\n", cmd, chksum);
  debug_serial->print(gps_cmd);
  gps_serial->print(gps_cmd);
  delay(500);
}

/*
  * gps_read Function
  * Read information from GNSS module
*/
int gps_read() {

 String gps_sentence;
  String header;
  String pos_lat;
  String pos_ns;
  String pos_long;
  String pos_ew;
  String time_str;
  String status;
  String mode;


  int index = 0;

  gps_sentence = gps_serial->readStringUntil('\n');
  gps_sentence.trim();

  if(gps_sentence.length() < 10) return 0;

  if (!gps_sentence.startsWith("$GPGLL") && !gps_sentence.startsWith("$GNGLL")) {
    return 0;
  }

#ifdef DEBUG_PRINT
  debug_serial->println(gps_sentence);
#endif

  index = get_substr(gps_sentence, ',', index, &header);
  index = get_substr(gps_sentence, ',', index, &pos_lat);
  index = get_substr(gps_sentence, ',', index, &pos_ns);
  index = get_substr(gps_sentence, ',', index, &pos_long);
  index = get_substr(gps_sentence, ',', index, &pos_ew);
  index = get_substr(gps_sentence, ',', index, &time_str);
  index = get_substr(gps_sentence, ',', index, &status);
  index = get_substr(gps_sentence, '*', index, &mode);

  m_field_data.raw_gps = gps_sentence;

  return 1;
}



/*
  // Format : $STMFIELD,AAAA,BBBB,CCCC,DDDD*XX<CR><LF>
*/
int stm_read() {
   String stm_sentence = stm_serial->readStringUntil('\n');

  if (!stm_sentence.startsWith("$STMFIELD")) return 0;

  int index = 0;
  String header;
  String ns_one_adc, ew_one_adc, one_c;
  String ns_max_adc, ns_max_c, ns_min_adc, ns_min_c;
  String ew_max_adc, ew_max_c, ew_min_adc, ew_min_c;

  index = get_substr(stm_sentence, ',', index, &header);
  index = get_substr(stm_sentence, ',', index, &ns_one_adc);
  index = get_substr(stm_sentence, ',', index, &ew_one_adc);
  index = get_substr(stm_sentence, ',', index, &one_c);
  index = get_substr(stm_sentence, ',', index, &ns_max_adc);
  index = get_substr(stm_sentence, ',', index, &ns_max_c);
  index = get_substr(stm_sentence, ',', index, &ns_min_adc);
  index = get_substr(stm_sentence, ',', index, &ns_min_c);
  index = get_substr(stm_sentence, ',', index, &ew_max_adc);
  index = get_substr(stm_sentence, ',', index, &ew_max_c);
  index = get_substr(stm_sentence, ',', index, &ew_min_adc);
  index = get_substr(stm_sentence, '*', index, &ew_min_c);
  debug_serial->println(stm_sentence);
  m_field_data.raw_stm = stm_sentence;
  // m_field_data.ns_max_adc = ns_max_adc.toInt();
  // m_field_data.ns_min_adc = ns_min_adc.toInt();
  // m_field_data.ew_max_adc = ew_max_adc.toInt();
  // m_field_data.ew_min_adc = ew_min_adc.toInt();

  // m_field_data.ns_max_c = ns_max_c.toInt();
  // m_field_data.ns_min_c = ns_min_c.toInt();
  // m_field_data.ew_max_c = ew_max_c.toInt();
  // m_field_data.ew_min_c = ew_min_c.toInt();

  return 1;
}

/*
  * Prepare the MQTT publish payload
  * Voltage data - 4 decimal points
  * Time data - 7 decimal points
*/

String build_mqtt_payload() {     // BUG!! - when single digit (hour/min) should display 01:02 instaed of 1:2
 JsonDocument json;
  String str;

json["GPS_RAW"] = m_field_data.raw_gps;
json["STM_RAW"] = m_field_data.raw_stm;
//json["offset"] = offset;
//json["offset"] = k;

  serializeJson(json, str);
  return str;
}

/*
  * Publish MQTT Payload to server
*/

void publish_data(String mqtt_payload) {
  //if() {
  char mqtt_topic[100];

  sprintf(mqtt_topic, "%s/%s/%s%03d", MQTT_TOPIC_PUB_NAME, MQTT_TOPIC_PUB_VER, MQTT_TOPIC_PUB_DEV_NAME, MQTT_TOPIC_PUB_DEV_ID);

  debug_serial->printf("Publish %d byte to Topic %s, with data = %s\n", strlen(mqtt_payload.c_str()), mqtt_topic, mqtt_payload.c_str());
  mqtt_client.publish(mqtt_topic, mqtt_payload.c_str());
  //}
}


/*
  Function to extract the substring from the 'start_pos' to the position of the 'delim; minus 1
  return substring to the pointer 'ret_str' and function with the next position of the 'delim'
*/
int get_substr(const String str, const char delim, int start_pos, String *ret_str) {
  int end_pos = str.indexOf(delim, start_pos);
  // TODO : Check if the 'delim' cannot be found in 'str'
    if(end_pos == -1){
    *ret_str = "";
    return str.length();
  }

  *ret_str = str.substring(start_pos, end_pos);

  return end_pos + 1;
}

/*
  * This function calculate the voltage from ADC value string ("-512" ... "511"), and round to the defined decimal place (4)
*/

void serial_setup() {
  // Serial Port mapping
  gps_serial = &Serial1;
  stm_serial = &Serial2;
  debug_serial = &Serial;

  // Serial Port configuration
  gps_serial->begin(38400, SERIAL_8N1, 5, 17);
  gps_serial->setTimeout(50);
  stm_serial->begin(115200 , SERIAL_8N1, 12, 2);
  debug_serial->begin(115200 , SERIAL_8N1, 3, 1);
  delay(2000);
  debug_serial->println("- Serial setup : Done");
}

void gps_setup() {
  debug_serial->print("- GPS Setup : ");
  gps_uart_send_cmd("PERDCFG,NMEAOUT,ALL,0");      // turn off all NMEA output sentences
  gps_uart_send_cmd("PERDAPI,CROUT,DGJPQWXYZ,0");  // turn off all CR output sentences
  gps_uart_send_cmd("PERDCFG,UART1,115200");       // Change GPS uart buadrate
  gps_serial->flush();                             // Ensure all GPS data to be sent
  gps_serial->end();
  delay(1000);

  gps_serial->begin(115200, SERIAL_8N1, 5, 17);  // Chenge ESP32 GPS uart buadrate
  delay(1000);
  gps_uart_send_cmd("PERDSYS,VERSION");  // Check communication

    //เปิด 10MHz clock (OCXO discipline)
  //gps_uart_send_cmd("PERDAPI,GCLK,1,10000000,50,0");

  // ตั้ง PPS เป็น timing mode
  gps_uart_send_cmd("PERDAPI,PPS,VCLK,1,0,1,0,0");

  // บังคับ sync ครั้งแรก
  gps_uart_send_cmd("PERDAPI,PHASESKIP,1");

  //ตั้ง lock behaviour (สำคัญที่สุด)
  gps_uart_send_cmd("PERDAPI,MODESET,2,1500,200,20,15");
  /*
    TODO : To have code here to read serial port 
    if the version returned corecttly
  */
  gps_uart_send_cmd("PERDCFG,NMEAOUT,GLL,1");  // Enable GLL sentence every second
  debug_serial->println("Done");
}

void ethernet_setup() {
  debug_serial->println("- Setup Ethernet & MQTT");

  Ethernet.init(driver);
  debug_serial->println("-- Initialize Ethernet with DHCP:");
  if (Ethernet.begin() == 0) {
    debug_serial->println("--- Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      debug_serial->println("--- Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      while (true) {
        delay(10000);  // do nothing, no point running without Ethernet hardware
        ESP.restart();
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      debug_serial->println("--- Ethernet cable is not connected.");
    }
    // try to configure using IP address instead of DHCP:
    Ethernet.begin(ip);
  } else {
    debug_serial->print("--- DHCP assigned IP ");
    debug_serial->println(Ethernet.localIP());
  }
}

void mqtt_setup() {
  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt_client.setClient(eth_client);
  mqtt_client.setCallback(callback);
  mqtt_client.setBufferSize(512);

  while (!mqtt_client.connected()) {
    debug_serial->print("Attempting MQTT connection to ");
    debug_serial->println(MQTT_SERVER);

    if (mqtt_client.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      debug_serial->println("...connected");

      // Once connected, publish an announcement...
      //String data = "Hello from MQTTClient_SSL on " + String(BOARD_NAME);

      //client.publish(TOPIC, data.c_str());

      //Serial.println("Published connection message successfully!");
      //Serial.print("Subcribed to: ");
      //Serial.println(subTopic);

      // This is a workaround to address https://github.com/OPEnSLab-OSU/SSLClient/issues/9
      //ethClientSSL.flush();
      // ... and resubscribe
      mqtt_client.subscribe(MQTT_SUB_DATA_TOPIC);
      // for loopback testing
      mqtt_client.subscribe(MQTT_SUB_CMD_TOPIC);
      // This is a workaround to address https://github.com/OPEnSLab-OSU/SSLClient/issues/9
      //ethClientSSL.flush();
    } else {
      debug_serial->print("...failed, rc=");
      debug_serial->print(mqtt_client.state());
      debug_serial->println(" try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

























/*






// Send data over MQTT
void send_data(DataRecord_t rec){
  JsonDocument json_payload;
  String mqtt_payload;

  //json_payload["ID"] = rec.id;
  //json_payload["SERIAL"] = rec.serial;
  //json_payload["BASE_TIME"] = "";
  json_payload["NS_MAX_V"] = rec.max_ns_v;
  json_payload["max_ns_t"] = rec.max_ns_t;
  json_payload["MAX_EW_V"] = rec.max_ew_v;
  json_payload["max_ew_t"] = rec.max_ew_t;
  json_payload["LAT"] = rec.pos_lat;
  json_payload["LONG"] = rec.pos_long;
  json_payload["GPSTIME"] = rec.gps_time;
  
  serializeJson(json_payload, mqtt_payload);
  publish_data(mqtt_payload);
}*/












/*
// ================================================================================== //
Key functions and Loop function below
// ================================================================================== //
*/





void callback(char *topic, byte *payload, unsigned int length) {
  debug_serial->print("Message arrived [");
  debug_serial->print(topic);
  debug_serial->print("] ");

  for (unsigned int i = 0; i < length; i++) {
    debug_serial->print((char)payload[i]);
  }

  debug_serial->println();
}


#ifdef DEBUG_FN_PRINT_DATA

#endif
