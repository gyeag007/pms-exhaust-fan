// Sketch for a handheld particulate matter sensor, designed for Arduino
// Nano. Plantower PMS1003 is used as a sensor, but the code should work for
// PMS5003 as well, since it has the same interface.

// Measurements for atmospheric PM0.1, PM2.5 and PM10 are displayed on an
// Adafruit 1.8" TFT color display. The color changes to indicate the
// category of PM2.5 air quality (good, acceptable, unhealthy for sensitive
// groups, unhealthy, very unhealthy, hazardous).

// Data is also exported via serial connection if one is available. Full
// reported data set is exported, including CF=1 measurements and particle
// counts per 0.1L.

// Note that, while the sensor reports data up to PM10, it only measures
// data up to PM3 - the rest is estimated.

// By Nenad Rijavec
// Feel free to use, modify, share, etc. as you see fit, but note that
// Adafruit libraries are used for display handling and they have different
// restrictions.

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <SPI.h>
#include <SoftwareSerial.h>


#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8

int relay = 7;  // Tells Arduino the relay is connected to pin 7

/*
 * PIR sensor tester
 */

//int ledPin = 13;                // choose the pin for the LED
int inputPin = 4;    // choose the input pin (for PIR sensor)
int pirState = LOW;  // we start, assuming no motion detected
int val = 0;         // variable for reading the pin status


/*
   ST7735 supports 18 bit color, but the interface used uint16_t and thus
   only provides 16 bits. R and B are specified using 5 bits, while G uses 6
   bits. The bits are thus packed as RRRR RGGG GGGB BBBB. Note that this is
   logical packing. Since Arduino Nano is little endian, the bytes in the
   actual storage are reversed (GGGB BBBB RRRR RGGG). Treating RGB as an
   uint16_t removes the byte ordering from consideration.
*/
// color definitions
const uint16_t color_black = 0x0000;
const uint16_t color_blue = 0x001F;
const uint16_t color_red = 0xF800;
const uint16_t color_green = 0x07E0;
const uint16_t color_magenta = 0xF81F;
const uint16_t color_yellow = 0xFFE0;
const uint16_t color_orange = 0xFB40;
const uint16_t color_white = 0xFFFF;

uint16_t text_color = color_blue;
uint16_t background_color = color_white;
uint16_t font_size = 3;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
SoftwareSerial pms_serial(2, 3);

// we know the record is always 32 bytes and frame length is 28
unsigned char serial_signature[4] = { 0x42, 0x4d, 0x00, 0x1c };
unsigned char serial_data[32];
uint16_t *pm_data = (unsigned short *)&serial_data[0];
unsigned long time_sent = 0;
unsigned long send_interval = 3000;
unsigned long time_funcs_called = 0;
unsigned long funcs_call_interval = 3000;


unsigned int n_read = 0;
unsigned long previousOnMillis = 0;
unsigned long previousOffMillis = 0;
unsigned long recently_on_interval = 120000;   //600000 is 10 min, stays on for 10 min after acheiving clean air
unsigned long recently_off_interval = 180000;  //300000 is 5 min, stays off for 5 min after fan turns off
unsigned long previousMotionMillis = 0;
unsigned long previousMotionEndedMillis = 0;
unsigned long recent_motion_interval = 2700000;  //3600000 is 60 min 300000 is 5 min
//unsigned long recent_motion_ended_interval = 30000;  //3600000 is 60 min
unsigned long air_dirty_time = 0;
unsigned long air_dirty_interval = 300000;  // 3000000 is 5 min

struct pm_data {
  uint16_t signature;
  uint16_t frame_len;
  uint16_t pm_1_0_std;
  uint16_t pm_2_5_std;
  uint16_t pm_10_0_std;
  uint16_t pm_1_0_env;
  uint16_t pm_2_5_env;
  uint16_t pm_10_0_env;
  uint16_t cnt_0_3;
  uint16_t cnt_0_5;
  uint16_t cnt_1_0;
  uint16_t cnt_2_5;
  uint16_t cnt_5_0;
  uint16_t cnt_10_0;
  uint16_t reserved;
  uint16_t checksum;
} parsed;

uint8_t wake[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
uint8_t sleep[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };



struct fan {
  bool on = false;
  bool recently_on = false;
  bool recently_off = false;
};

struct airq {
  bool air_dirty = false;
};

struct pir {
  bool recent_motion = false;
};

struct mypms {
  bool awake = false;
};
struct fan f;
struct airq a;
struct pir p;
struct mypms m;

unsigned int x_orig = 10, y_orig = 10, y_skip = 40, num_offset = 55;
char buff1[100], buff2[100], buff3[100], buff4[100];
char label1[] = " 1:";
char label2[] = " 2:";
char label3[] = "On ";
char label4[] = "Off";


long onTime, offTime;

void display_vals(struct pm_data *parsed, struct fan &f) {

  uint16_t new_bg, new_fg;


  if (f.on) {
    onTime = (millis() - previousOnMillis) / 60000;
  } else {
    offTime = (millis() - previousOffMillis) / 60000;
  }

  if (parsed->pm_2_5_env <= 12)
    new_bg = color_white;
  else if (parsed->pm_2_5_env < 36)
    new_bg = color_yellow;
  else if (parsed->pm_2_5_env < 56)
    new_bg = color_orange;
  else if (parsed->pm_2_5_env < 141)
    new_bg = color_red;
  else if (parsed->pm_2_5_env < 211)
    new_bg = color_magenta;
  else
    new_bg = color_black;

  // if new colors, just erase the whole display
  if (background_color != new_bg) {
    background_color = new_bg;
    if (parsed->pm_2_5_env < 56)
      text_color = color_blue;
    else
      text_color = color_white;
    tft.fillScreen(background_color);
  } else {
    // unchanged colors, erase only the variable text
    // change the text color to the background color
    tft.setTextColor(background_color);
    tft.setCursor(x_orig + num_offset, y_orig);
    tft.print(buff1);
    tft.setCursor(x_orig + num_offset, y_orig + y_skip);
    tft.print(buff2);
    tft.setCursor(x_orig + num_offset, y_orig + 2 * y_skip);
    tft.print(buff3);
    tft.setCursor(x_orig + num_offset, y_orig + 3 * y_skip);  //my stuff
    tft.print(buff4);
  }

  // ready to draw new values
  tft.setTextColor(text_color);

  sprintf(buff1, "%03hu", parsed->pm_1_0_env);
  sprintf(buff2, "%03hu", parsed->pm_2_5_env);
  snprintf(buff3, sizeof(buff3), "%03hu", onTime);
  snprintf(buff4, sizeof(buff4), "%03hu", offTime);

  //Serial.println(myLong);
  //Serial.println(buff4);

  tft.setCursor(x_orig, y_orig);
  // labels are always printed
  tft.print(label1);
  tft.setCursor(x_orig, y_orig + y_skip);
  tft.print(label2);
  tft.setCursor(x_orig, y_orig + 2 * y_skip);
  tft.print(label3);
  tft.setCursor(x_orig, y_orig + 3 * y_skip);
  tft.print(label4);


  // now print the values
  tft.setCursor(x_orig + num_offset, y_orig);
  tft.print(buff1);
  tft.setCursor(x_orig + num_offset, y_orig + y_skip);
  tft.print(buff2);
  tft.setCursor(x_orig + num_offset, y_orig + 2 * y_skip);
  tft.print(buff3);
  tft.setCursor(x_orig + num_offset, y_orig + 3 * y_skip);
  tft.print(buff4);

}

// Gets the next record. Returns true if the record has been read.
// Extra logic, because the sensor sometimes returns corrupted data.
bool get_data() {
  int i;
  uint16_t sum;
  uint16_t *flip_buff = (unsigned short *)&parsed;

  if (pms_serial.available() < 4)
    return false;

  for (i = 0; i < 4; i++)
    if (pms_serial.read() != serial_signature[i])
      return false;
    else
      serial_data[i] = serial_signature[i];

  // signature OK, read the data

  while (pms_serial.available() < 28) {
    // wait for the rest of the record
  }
  for (i = 4; i < 32; i++)
    serial_data[i] = pms_serial.read();

  // load the parsed by going from big endian to local

  for (i = 0; i < 16; i++)
    flip_buff[i] = ((uint16_t)serial_data[2 * i]) << 8 | (uint16_t)serial_data[2 * i + 1];

  // compute and check checksum
  sum = 0;
  for (i = 0; i < 30; i++)
    sum += serial_data[i];
  if (sum != parsed.checksum)
    return false;

  return true;
}


void setup() {
  Serial.begin(115200);
  pms_serial.begin(9600);    // connect to the PMS1003
  pinMode(inputPin, INPUT);  // declare sensor as input
  pinMode(relay, OUTPUT);    // Initialize the Atmel GPIO pin as an output
  buff1[0] = '\0';
  buff2[0] = '\0';
  buff3[0] = '\0';
  buff4[0] = '\0';
  delay(1000);

  // connect to the display
  tft.initR(INITR_BLACKTAB);  // Init ST7735R chip

  // initialise the display
  tft.setFont();
  tft.fillScreen(background_color);
  tft.setTextColor(text_color);
  tft.setTextSize(font_size);

  Serial.println(sizeof(int));

  Serial.println("done setup");
}


void pir_func(struct pir &p) {
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
                                //digitalWrite(ledPin, HIGH);  // turn LED ON
    p.recent_motion = true;
    previousMotionMillis = millis();
    if (pirState == LOW) {
      // we have just turned on
      //Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    //digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState == HIGH) {
      // we have just turned of
      previousMotionEndedMillis = millis();
      //Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
}

unsigned long sleep_test_time = millis();

void sleep_test_func() {
  if (millis() - sleep_test_time > 600000) {
    Serial.print("Trying Sleep  ");
    pms_serial.write(sleep, sizeof(sleep));
    delay(30000);
    Serial.print("Trying Wake  ");
    delay(10000);
    pms_serial.write(wake, sizeof(wake));
    delay(30000);
    sleep_test_time = millis();
  }
}

void recent_motion_func(struct fan &f, struct pir &p, struct mypms &m) {
  Serial.print("previousMotion Min:  ");
  Serial.println((millis() - previousMotionMillis) / 60000.0);
  if ((millis() - previousMotionMillis > recent_motion_interval) && m.awake) {
    p.recent_motion = false;
    Serial.println("putting PMS to sleep  ");
    pms_serial.write(sleep, sizeof(sleep));
    m.awake = false;
  } else if (p.recent_motion && !m.awake) {
    Serial.print("previousMotion Min:  ");
    Serial.println((millis() - previousMotionMillis) / 60000.0);
    Serial.println("waking up pms ");
    pms_serial.write(wake, sizeof(wake));
    m.awake = true;
    //delay(30000); //delay after wake
    Serial.print("p.recent_motion:   ");
    Serial.print(p.recent_motion);
  }
}

void recently_on_func(struct fan &f) {
  Serial.print("previousOn Min:  ");
  Serial.println((millis() - previousOnMillis) / 60000.0);


  if (millis() - previousOnMillis > recently_on_interval) {
    f.recently_on = false;
  }
}

void recently_off_func(struct fan &f) {
  //Serial.println("inside recently off");
  //Serial.print("previousOffMillis:  ");
  //Serial.println(millis() - previousOffMillis);
  Serial.print("previousOff Min:  ");
  Serial.println((millis() - previousOffMillis) / 60000.0);


  if (millis() - previousOffMillis > recently_off_interval) {
    f.recently_off = false;
  }
}

void fan_control(struct airq &a, struct fan &f, struct pir &p) {
  Serial.print("air_dirty_time Min:  ");
  Serial.println((millis() - air_dirty_time) / 60000.0);

  if (parsed.pm_2_5_env >= 56) {
    a.air_dirty = true;
    air_dirty_time = millis();
  } else if (millis() - air_dirty_time > air_dirty_interval) {
    a.air_dirty = false;
  }

  if (a.air_dirty && !f.on) {
    f.on = true;
    digitalWrite(relay, HIGH);
    previousOnMillis = millis();
  } else if (!a.air_dirty && f.on) {
    f.on = false;
    digitalWrite(relay, LOW);
    previousOffMillis = millis();
  }
}

void loop() {

  int i;

  if (!get_data()) {
    return;
  }

  if (millis() - time_funcs_called >= funcs_call_interval) {
    //Serial.println("");
    pir_func(p);
    //Serial.println("");
    recent_motion_func(f, p, m);
    fan_control(a, f, p);
    recently_on_func(f);
    recently_off_func(f);
    Serial.println("");
    time_funcs_called = millis();
  }

  if (millis() - time_sent >= send_interval) {
    display_vals(&parsed, f);
    time_sent = millis();
    delay(1000);
  }
}
