/*
 * @FileName: start.ino
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-04-10 11:24:27
 * @LastEditTime: 2021-04-28 19:48:31
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: This demo is based on the Arduino UNO, and the LEDs on the UNO board are controlled by the Tuya Smart App. 
 *               Enter network connection mode when Pin7 to GND.
 * @Github:https://github.com/tuya/tuya-wifi-mcu-sdk-arduino-library
 */

#include <TuyaWifi.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

SoftwareSerial DebugSerial(13, 14);
TuyaWifi my_device;

/* Current LED status */
unsigned char led_state ;

#define NUMPIXELS 24 //灯珠数量
#define PIN 4        //灯珠连接的IO
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/* Connect network button pin */
#define key_pin 12
#define LED_BUILTIN 2
/* Data point define */
#define DPID_SWITCH 20
#define DPID_Mode 21
#define DPID_Colours 24
/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] = {
    {DPID_SWITCH, DP_TYPE_BOOL}, //开关
    {DPID_Mode, DP_TYPE_ENUM},   //模式
                                 //彩光
};

unsigned char pid[] = {"sfr1npxmtxq6dhmb"};
unsigned char mcu_ver[] = {"1.0.0"};

uint8_t R = 0;
uint8_t G = 50;
uint8_t B = 0;
uint8_t liangdu = 100;
uint8_t brightnes_s = 0;
uint8_t fadeAmount = 3; //亮度变化增量
uint8_t Mode_Switch = 0;
uint16_t j;
static uint8_t lampStep = 0;
/* last time */
unsigned long last_time = 0;
unsigned char dp_mode_value ; //模式

uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

////////////呼吸灯
void RGB_breath()
{
    strip.setBrightness(brightnes_s);
    for (uint8_t i = 0; i < NUMPIXELS; i++)
    {
        strip.setPixelColor(i, R, G, B);
        strip.show();
    }

    brightnes_s = brightnes_s + fadeAmount;

    if (brightnes_s <= 0 || brightnes_s >= 180)

        fadeAmount = -fadeAmount; //亮度翻转
}

void rainbowCycle(uint8_t wait)
{ //七彩旋转
    if (j < 256 * 5)
    {
        j++;
        for (uint16_t i = 0; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        strip.setBrightness(liangdu);
        strip.show();
        delay(wait);
    }
    else
        j = 0;
}
void rainbowDisplay(uint8_t _delay)
{ //七彩循环
    strip.setBrightness(liangdu);
    for (uint8_t i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, Wheel((i + lampStep) & 255));
    }
    lampStep = (lampStep + 1) & 0xFF;

    strip.show();
    delay(_delay);
}
//音乐模式
void music()
{
    uint8_t mic_val[8];
    uint8_t data;
    for (int i = 0; i <= 8; i++)
    {
        mic_val[i] = analogRead(A0);
    }
    data = mic_val[1] + mic_val[2] + mic_val[3] + mic_val[4] + mic_val[5] + mic_val[6] + mic_val[7] + mic_val[8] / 8;
    
    if (data>0) //大于0才输出防止串口刷屏
    {
        DebugSerial.println("mode3");
        DebugSerial.println(data);
    }

    data = map(data, 0, 200, 0, 25);
    R = random(0, 100);
    G = random(0, 100);
    B = random(0, 100);

    for (int i = 0; i < data; i++)
    {
        strip.setPixelColor(i, R, G, B);
        delay(10);
        strip.show();
    }
    
    for (int i = 0; i < data; i++)
    {
        strip.setPixelColor(i, 0, 0, 0);
    }
    strip.show();
}
void setup()
{
    // Serial.begin(9600);
    Serial.begin(9600);
    DebugSerial.begin(9600);
    DebugSerial.println("debug print begin");
    //Initialize led port, turn off led.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    //Initialize networking keys.
    pinMode(key_pin, INPUT_PULLUP);

    strip.begin();
    strip.setBrightness(liangdu);
    strip.show();

    //Enter the PID and MCU software version
    my_device.init(pid, mcu_ver);
    //incoming all DPs and their types array, DP numbers
    my_device.set_dp_cmd_total(dp_array, 2);
    //register DP download processing callback function
    my_device.dp_process_func_register(dp_process);
    //register upload all DP callback function
    my_device.dp_update_all_func_register(dp_update_all);

    last_time = millis();
}

void loop()
{
    my_device.uart_service();

    //Enter the connection network mode when Pin7 is pressed.
    if (digitalRead(key_pin) == LOW)
    {
        delay(100);
        if (digitalRead(key_pin) == LOW)
        {
            DebugSerial.println("wifi config begin");
            my_device.mcu_set_wifi_mode(SMART_CONFIG);
        }
    }
    /* LED blinks when network is being connected */
    if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW))
    {
        if (millis() - last_time >= 500)
        {
            last_time = millis();

            if (led_state == LOW)
            {
                led_state = HIGH;
            }
            else
            {
                led_state = LOW;
            }
            Serial.println("wifi config beging.....");
            digitalWrite(LED_BUILTIN, led_state);
        }
    }
    if (led_state)
    {
        switch (dp_mode_value)
        {
        case 0: //呼吸
            rainbowDisplay(300);
            break;
        case 1: //时钟
            /* code */
            break;
        case 2: //幻彩
            rainbowCycle(10);
            break;
        case 3: //音乐
            music();
            break;

        default:
            break;
        }
    }
    else
    {
        for (int i = 0; i < NUMPIXELS; i++)
        {
            strip.setPixelColor(i, 0, 0, 0);
        }
        strip.show();
    }
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
    DebugSerial.print("dpid is: ");
    DebugSerial.println(dpid);
    switch (dpid)
    {
    //开关
    case DPID_SWITCH:
        led_state = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
        DebugSerial.print("led_state is: ");
        DebugSerial.println(led_state);

        //Status changes should be reported.
        my_device.mcu_dp_update(dpid, value, length);
        break;
    //模式
    case DPID_Mode:
        DebugSerial.println("Enum type:");
        dp_mode_value = my_device.mcu_get_dp_download_data(dpid, value, length);
        DebugSerial.println(dp_mode_value);
        /* After processing the download DP command, the current status should be reported. */
        my_device.mcu_dp_update(dpid, value, length);
        break;

    default:
        break;
    }
    return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
    my_device.mcu_dp_update(DPID_SWITCH, led_state, 1);
    my_device.mcu_dp_update(DPID_Mode, dp_mode_value, 1);
}
