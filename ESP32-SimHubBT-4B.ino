#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "soc/ledc_reg.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled! Please run `make menuconfig` to and enable it"
#endif

#define BT_NAME                 "BT-DASH-4B"

#define LEDC_TIMER_RES          LEDC_TIMER_10_BIT
#define DUTY_MAX                ((1 << LEDC_TIMER_10_BIT) -1 )
#define FREQ_MIN_Hz             1 /* Do not decrease it! */

#define FUEL_DUTY_CYCLE_MIN     100     /* FULL     */
#define FUEL_DUTY_CYCLE_MAX     540     /* EMPTY    */
#define TEMP_DUTY_CYCLE_MIN     26      /* 60°C     */
#define TEMP_DUTY_CYCLE_MAX     512     /* >120°C   */
#define OILTEMP_DUTY_CYCLE_MIN  12      /* 60°C     */
#define OILTEMP_DUTY_CYCLE_MAX  600     /* >180°C   */
#define OIL_WARN_TRESH_RPM      300

#define TURN_R_PIN              5
#define FUEL_PIN                2
#define TEMP_PIN                32
#define OIL_PIN                 25
#define RPM_PIN                 21
#define BATTERY_PIN             26
#define H_BEAM_PIN              22
#define TURN_L_PIN              18
#define WATER_MIN_PIN           23
#define SPEED_PIN               27
#define BRAKE_PIN               13
#define POS_LAMP_PIN            19
#define OIL_TEMP_PIN            4
#define DIMM_PIN                33
#define PWR_PIN                 17

#define BUF_SIZE                64

char simhub_message_buf[BUF_SIZE];
BluetoothSerial bt_serial;


void ledc_init(uint8_t pin, float freq_Hz, ledc_channel_t channel, ledc_timer_t timer) {
    const char * ME = __func__;

    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);

    uint32_t precision = DUTY_MAX + 1;
    uint32_t div_param = ((uint64_t) LEDC_REF_CLK_HZ << 8) / freq_Hz / precision;
    if (div_param < 256 || div_param > LEDC_DIV_NUM_HSTIMER0_V)
    {
        ESP_LOGE(ME, "requested frequency and duty resolution can not be achieved, try increasing freq_hz or duty_resolution. div_param=%d", (uint32_t ) div_param);
    }

    ledc_channel_config_t ledc_channel = {
      .gpio_num   = pin,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel    = channel,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = timer,
      .duty       = DUTY_MAX,
      .hpoint     = 0         // TODO: AD 10.11.2018: new, does 0 work (0xfffff does not work)
    };
    err = ledc_channel_config(&ledc_channel);
    ESP_LOGD(ME,"ledc_channel_config returned %d",err);
    
    err = ledc_timer_set(LEDC_HIGH_SPEED_MODE, timer, div_param, LEDC_TIMER_RES, LEDC_REF_TICK);
    if (err)
    {
        ESP_LOGE(ME, "ledc_timer_set returned %d",err);
    }
    
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, timer);
    ESP_LOGD(ME, "ledc_timer_set: divider: 0x%05x duty_resolution: %d\n", (uint32_t) div_param, LEDC_TIMER_RES);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    pinMode(TURN_R_PIN, OUTPUT);
    pinMode(OIL_PIN, OUTPUT);
    pinMode(BATTERY_PIN, OUTPUT);
    pinMode(H_BEAM_PIN, OUTPUT);
    pinMode(TURN_L_PIN, OUTPUT);
    pinMode(WATER_MIN_PIN, OUTPUT);
    pinMode(BRAKE_PIN, OUTPUT);
    pinMode(POS_LAMP_PIN, OUTPUT);
    pinMode(DIMM_PIN, OUTPUT);
    pinMode(PWR_PIN, OUTPUT);

    digitalWrite(TURN_R_PIN, LOW);
    digitalWrite(OIL_PIN, LOW);
    digitalWrite(BATTERY_PIN, LOW);
    digitalWrite(H_BEAM_PIN, LOW);
    digitalWrite(TURN_L_PIN, LOW);
    digitalWrite(WATER_MIN_PIN, HIGH);
    digitalWrite(BRAKE_PIN, LOW);
    digitalWrite(POS_LAMP_PIN, LOW);
    digitalWrite(DIMM_PIN, HIGH);

    ledc_init(TEMP_PIN, 490, LEDC_CHANNEL_0, LEDC_TIMER_0);
    ledc_init(RPM_PIN, FREQ_MIN_Hz, LEDC_CHANNEL_1, LEDC_TIMER_1);
    ledc_init(SPEED_PIN, FREQ_MIN_Hz, LEDC_CHANNEL_2, LEDC_TIMER_2);
    ledc_init(FUEL_PIN, 490, LEDC_CHANNEL_3, LEDC_TIMER_3);
    ledc_init(OIL_TEMP_PIN, 490, LEDC_CHANNEL_4, LEDC_TIMER_3);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 84);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 512);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 512);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 190);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, 78);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
    
    delay(1000);

    digitalWrite(PWR_PIN, HIGH);

    memset(simhub_message_buf, 0x0, BUF_SIZE);
    bt_serial.begin(BT_NAME);
}

void loop() {
    if (bt_serial.available() > 0)
    {
        bt_serial.readBytesUntil('{', simhub_message_buf, BUF_SIZE);
        int readCount = bt_serial.readBytesUntil('}', simhub_message_buf, BUF_SIZE);
        simhub_message_buf[min(readCount, BUF_SIZE - 1)] = 0x0;
        process_message();
        memset(simhub_message_buf, 0x0, BUF_SIZE);
    }
}

void process_message() {
    unsigned int revs;
    unsigned int speed_kmh;
    unsigned int fuel_percent;
    float water_temperature_degC;
    int turn_left;
    int turn_right;
    int brake;
    float oil_temperature_degC;
    
    sscanf(simhub_message_buf, "%u&%u&%u&%f&%d&%d&%d&%f",
        &revs,
        &speed_kmh,
        &fuel_percent,
        &water_temperature_degC,
        &turn_left,
        &turn_right,
        &brake,
        &oil_temperature_degC
    );

    float rpm_Hz = rpm_get_Hz(revs);
    float speed_Hz = speed_get_Hz(speed_kmh);
    unsigned int fuel_duty_cycle = fuel_duty_cycle_get(fuel_percent);
    unsigned int temp_duty_cycle = water_temperature_duty_cycle_get(water_temperature_degC);
    unsigned int oil_temp_duty_cycle = oil_temperature_duty_cycle_get(oil_temperature_degC);

    handle_lights(turn_left, turn_right, brake, revs);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, temp_duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, rpm_Hz);
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_2, speed_Hz);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, fuel_duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, oil_temp_duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
}

float rpm_get_Hz(float revs) {
    float freq_Hz = 0.05 * revs;

    if (freq_Hz < FREQ_MIN_Hz)
    {
        return FREQ_MIN_Hz;
    }
    
    return freq_Hz;
}

float speed_get_Hz(float speed_kmh) {
    float freq_Hz = 1.087 * speed_kmh - 0.6;

    if (freq_Hz < FREQ_MIN_Hz)
    {
        return FREQ_MIN_Hz;
    }
    
    return freq_Hz;
}

unsigned int fuel_duty_cycle_get(float fuel_percent) {
    unsigned int fuel_duty_cycle = FUEL_DUTY_CYCLE_MIN;
    
    if (fuel_percent > 0.0 && fuel_percent <= 25.0)
    {
        fuel_duty_cycle = -9.6 * fuel_percent + 540;
    }
    else if (fuel_percent > 25.0 && fuel_percent <= 50.0)
    {
        fuel_duty_cycle = -4.4 * fuel_percent + 410;
    }
    else if (fuel_percent > 50.0 && fuel_percent <= 75.0)
    {
        fuel_duty_cycle = -2.08 * fuel_percent + 294;
    }
    else if (fuel_percent > 75.0 && fuel_percent <= 100.0)
    {
        fuel_duty_cycle = -1.52 * fuel_percent + 252;
    }
    
    return fuel_duty_cycle;
}

unsigned int water_temperature_duty_cycle_get(float water_temperature_degC) {
    unsigned int temp_duty_cycle = TEMP_DUTY_CYCLE_MIN;
    
    if (water_temperature_degC > 60.0 && water_temperature_degC <= 80.0)
    {
        temp_duty_cycle = water_temperature_degC - 34;
    }
    else if (water_temperature_degC > 80.0 && water_temperature_degC <= 100.0)
    {
        temp_duty_cycle = 4.2 * water_temperature_degC - 290;
    }
    else if (water_temperature_degC > 100.0 && water_temperature_degC <= 110.0)
    {
        temp_duty_cycle = 8.0 * water_temperature_degC - 670;
    }
    else if (water_temperature_degC > 110.0)
    {
        temp_duty_cycle = 21.0 * water_temperature_degC - 2100;
    }

    if (temp_duty_cycle >= TEMP_DUTY_CYCLE_MAX)
    {
        return TEMP_DUTY_CYCLE_MAX;
    }

    return temp_duty_cycle;
}

unsigned int oil_temperature_duty_cycle_get(float oil_temperature_degC) {
    unsigned int oil_temp_duty_cycle = OILTEMP_DUTY_CYCLE_MIN;
    
    if (oil_temperature_degC > 60.0 && oil_temperature_degC <= 100.0)
    {
        oil_temp_duty_cycle = 0.9 * oil_temperature_degC - 42;
    }
    else if (oil_temperature_degC > 100.0 && oil_temperature_degC <= 120.0)
    {
        oil_temp_duty_cycle = 1.5 * oil_temperature_degC - 102;
    }
    else if (oil_temperature_degC > 120.0 && oil_temperature_degC <= 140.0)
    {
        oil_temp_duty_cycle = 2.6 * oil_temperature_degC - 234;
    }
    else if (oil_temperature_degC > 140.0 && oil_temperature_degC <= 160.0)
    {
        oil_temp_duty_cycle = 5.0 * oil_temperature_degC - 570;
    }
    else if (oil_temperature_degC > 160.0)
    {
        oil_temp_duty_cycle = 17.5 * oil_temperature_degC - 2570;
    }

    if (oil_temp_duty_cycle >= OILTEMP_DUTY_CYCLE_MAX)
    {
        return OILTEMP_DUTY_CYCLE_MAX;
    }

    return oil_temp_duty_cycle;
}

void handle_lights(
    int turn_left,
    int turn_right,
    int brake,
    unsigned int revs
)
{
    if (revs >= OIL_WARN_TRESH_RPM)
    {
        digitalWrite(OIL_PIN, HIGH);
    }
    if (revs < OIL_WARN_TRESH_RPM)
    {
        digitalWrite(OIL_PIN, LOW);
    }
    
    if (turn_right != 0)
    {
        digitalWrite(TURN_R_PIN, HIGH);
    }
    else
    {
        digitalWrite(TURN_R_PIN, LOW);
    }
    if (turn_left != 0)
    {
        digitalWrite(TURN_L_PIN, HIGH);
    }
    else
    {
        digitalWrite(TURN_L_PIN, LOW);
    }
    if (brake != 0)
    {
        digitalWrite(BRAKE_PIN, HIGH);
    }
    else
    {
        digitalWrite(BRAKE_PIN, LOW);
    }
}
