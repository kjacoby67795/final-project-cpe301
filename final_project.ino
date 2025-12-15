// Code: Kyle Jacoby
#include <Stepper.h>
#include <RTClib.h>
#include <DHT.h>
#include <LiquidCrystal.h>
#include <avr/interrupt.h>

/* ===== Masks ===== */
#define FAN   0x10
#define GREEN_LED  0x80
#define YELLOW_LED  0x20
#define RED_LED  0x08
#define BLUE_LED  0x02

#define ST_BTN     0x08
#define RES_BTN    0x04
#define CTRL_BTN   0x02

/* ===== Registers ===== */
volatile unsigned char *PORT_B = (unsigned char *)0x25;
volatile unsigned char *DDR_B  = (unsigned char *)0x24;
volatile unsigned char *PIN_B  = (unsigned char *)0x23;

volatile unsigned char *PORT_C = (unsigned char *)0x28;
volatile unsigned char *DDR_C  = (unsigned char *)0x27;

volatile unsigned char *myPCICR  = (unsigned char *)0x68;
volatile unsigned char *myPCMSK0 = (unsigned char *)0x6B;
volatile unsigned char *myPCIFR = (unsigned char *) 0x3B;

volatile unsigned char *UCSR_0A = (unsigned char *) 0x00C0;
volatile unsigned char *UCSR_0B = (unsigned char *) 0x00C1;
volatile unsigned char *UCSR_0C = (unsigned char *) 0x00C2;
volatile unsigned char *UBRR_0 = (unsigned char *) 0x00C4;
volatile unsigned char *UDR_0 = (unsigned char *) 0x00C6;

volatile unsigned char* my_ADMUX  = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

/* ===== State Machine ===== */
typedef enum {DISABLED, IDLE, ERROR, RUNNING} STATE;
STATE dev_state = DISABLED;
STATE prev_state;

/* ===== LCD ===== */
#define LCD_RS    12
#define LCD_EN    11
#define LCD_D4    6
#define LCD_D5    5
#define LCD_D6    4
#define LCD_D7    3
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#define LCD_LEN 16
char lcd_buf[LCD_LEN], err_msg[LCD_LEN];
char state_map[4][16] = {"DISABLED","IDLE","ERROR","RUNNING"};
unsigned char led_mask_map[4] = {YELLOW_LED,GREEN_LED,RED_LED,BLUE_LED};

/* ===== Sensors ===== */
#define DHT_PIN   7
#define DHTTYPE   DHT11
DHT dht(DHT_PIN,DHTTYPE);
RTC_DS3231 rtc;

/* ===== Fan / Stepper ===== */
#define REV_STEPS 2038
Stepper step(REV_STEPS, 28, 26, 24, 22);

/* ===== Project Constants ===== */
const unsigned int temp_threshold = 80;
const unsigned int wtr_threshold  = 80;

unsigned int wtr_level = 0;
unsigned int update_timer = 0;

float last_temp = 0;
float last_hum  = 0;

volatile unsigned char prev_pinb;

/* ===== Prototypes ===== */
void IO_INIT();
void ADC_INIT();
void UART0_INIT(unsigned long baud);
unsigned int ADC_READ(unsigned char ch);
void UART0_PUTCHAR(unsigned char c);
void UART0_PUTSTR(unsigned char *s, int len);
void LED_UPDATE(STATE state);
void load_ht(char *buf);
void update_sensors();

/* ===== Setup ===== */
void setup() {
    lcd.begin(16,2);
    dht.begin();
    rtc.begin();
    rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
    lcd.clear();
    step.setSpeed(2);

    UART0_INIT(9600);
    IO_INIT();
    ADC_INIT();

    prev_pinb = *PIN_B;
    sei();
}

/* ===== Loop ===== */
void loop(){
    DateTime now = rtc.now();

    // Update water level (except DISABLED)
    if(dev_state != DISABLED) wtr_level = ADC_READ(0);

    // Update sensors every minute
    update_sensors();

    // Display state on LCD
    lcd.setCursor(0,1);
    lcd.print(state_map[dev_state]);
    LED_UPDATE(dev_state);

    // State machine
    switch(dev_state){
        case DISABLED:
            *PORT_B &= ~FAN;
            break;

        case IDLE:
            *PORT_B &= ~FAN;
            lcd.setCursor(0,0);
            lcd.print(lcd_buf);
            if((int)last_temp >= temp_threshold) dev_state = RUNNING;
            if(wtr_level < wtr_threshold){
                snprintf(err_msg,LCD_LEN,"Low water!");
                dev_state = ERROR;
            }
            break;

        case ERROR:
            *PORT_B &= ~FAN;
            lcd.setCursor(0,0);
            lcd.print(err_msg);
            break;

        case RUNNING:
            *PORT_B |= FAN;
            lcd.setCursor(0,0);
            lcd.print(lcd_buf);
            if((int)last_temp < temp_threshold) dev_state = IDLE;
            if(wtr_level < wtr_threshold){
                snprintf(err_msg,LCD_LEN,"Low water!");
                dev_state = ERROR;
            }
            break;
    }

    // Vent control (stepper) allowed in all states except DISABLED
    if(dev_state != DISABLED){
        bool ctrl_now = (*PIN_B & CTRL_BTN) == 0;
        static bool ctrl_prev = false;
        if(ctrl_now && !ctrl_prev){
            step.step(1);
            UART0_PUTSTR((unsigned char*)"VENT POSITION UPDATED\n",23);
        }
        ctrl_prev = ctrl_now;
    }

    // State transition report
    if(prev_state != dev_state){
        char buf[128];
        snprintf(buf,128,"STATE: %s -> %s\n",state_map[prev_state],state_map[dev_state]);
        UART0_PUTSTR((unsigned char*)buf,strlen(buf));
        snprintf(buf,128,"TIME: %02d:%02d:%02d DATE: %02d/%02d/%04d\n", \
            now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
        UART0_PUTSTR((unsigned char*)buf,strlen(buf));
        lcd.clear();
    }

    prev_state = dev_state;
}

/* ===== Sensor update ===== */
void update_sensors(){
    update_timer++;
    if(update_timer < 60) return;
    update_timer=0;
    load_ht(lcd_buf);

    char buf[64];
    snprintf(buf,64,"TEMP: %dF HUM: %d%%\n",(int)last_temp,(int)last_hum);
    UART0_PUTSTR((unsigned char*)buf,strlen(buf));
}

/* ===== LED ===== */
void LED_UPDATE(STATE state){
    *PORT_C = led_mask_map[state];
}

/* ===== DHT11 readings ===== */
void load_ht(char *buf){
    float t = dht.readTemperature(true);
    float h = dht.readHumidity();

    if(isnan(t) || isnan(h)){
        if(last_temp !=0 && last_hum !=0) snprintf(buf,LCD_LEN,"H:%d T:%dF",(int)last_hum,(int)last_temp);
        else snprintf(buf,LCD_LEN,"H:-- T:--");
    } else {
        last_temp=t;
        last_hum=h;
        snprintf(buf,LCD_LEN,"H:%d T:%dF",(int)h,(int)t);
    }
}

ISR(PCINT0_vect){
    unsigned char curr = *PIN_B;
    unsigned char changed = curr ^ prev_pinb;

    // START button (PB3) pressed
    if(changed & ST_BTN){
        if(!(curr & ST_BTN)){ // active-low press
            if(dev_state == DISABLED) dev_state = IDLE;
            UART0_PUTSTR((unsigned char*)"START pressed\n",14);
        }
    }

    // RESET button (PB2) pressed
    if(changed & RES_BTN){
        if(!(curr & RES_BTN)){ // active-low press
            if(dev_state == ERROR && wtr_level >= wtr_threshold){
                dev_state = IDLE;
                UART0_PUTSTR((unsigned char*)"RESET -> IDLE\n",15);
            } else {
                UART0_PUTSTR((unsigned char*)"RESET pressed\n",14);
            }
        }
    }

    // CTRL button (PB1) pressed
    if(changed & CTRL_BTN){
        if(!(curr & CTRL_BTN)){ // active-low press
            *PORT_B &= ~FAN;  // turn fan off
            dev_state = DISABLED;
            UART0_PUTSTR((unsigned char*)"CTRL pressed -> DISABLED\n",27);
        }
    }

    prev_pinb = curr;
}


/* ===== Initialization ===== */
void IO_INIT(){
    // LEDs
    *DDR_C |= GREEN_LED|YELLOW_LED|RED_LED|BLUE_LED;
    *PORT_C &= ~(GREEN_LED|YELLOW_LED|RED_LED|BLUE_LED);

    // Fan
    *DDR_B |= FAN;

    // Buttons (input pull-up)
    *DDR_B &= ~(ST_BTN|RES_BTN|CTRL_BTN);
    *PORT_B |= ST_BTN|RES_BTN|CTRL_BTN;

    // PCINT
    *myPCICR |= 0x01;
    *myPCMSK0 |= ST_BTN|RES_BTN|CTRL_BTN;
    *myPCIFR |= 0x01;
}

void ADC_INIT(){
    *my_ADCSRA |= 0b10000000;
    *my_ADCSRA &= 0b11011111;
    *my_ADCSRB &= 0b11110111;
    *my_ADCSRB &= 0b11111000;
    *my_ADMUX &= 0b01111111;
    *my_ADMUX |= 0b01000000;
    *my_ADMUX &= 0b11011111;
    *my_ADMUX &= 0b11100000;
}

/* ===== UART ===== */
void UART0_INIT(unsigned long baud){
    unsigned long FCPU = 16000000;
    unsigned int tbaud = (FCPU/16/baud - 1);
    *UCSR_0A = 0x20;
    *UCSR_0B = 0x18;
    *UCSR_0C = 0x06;
    *UBRR_0 = tbaud;
}

unsigned int ADC_READ(unsigned char adc_channel_num){
    *my_ADMUX &= 0b11100000;
    *my_ADCSRB &= 0b11110111;
    if(adc_channel_num>7){
        adc_channel_num-=8;
        *my_ADCSRB |= 0b00001000;
    }
    *my_ADMUX += adc_channel_num;
    *my_ADCSRA |= 0x40;
    while((*my_ADCSRA & 0x40) != 0);
    return *my_ADC_DATA;
}

void UART0_PUTCHAR(unsigned char c){
    while((*UCSR_0A & 0x20) == 0);
    *UDR_0 = c;
}

void UART0_PUTSTR(unsigned char *s,int len){
    for(int i=0;i<len && s[i]!='\0';i++){
        while((*UCSR_0A & 0x20) == 0);
        *UDR_0 = s[i];
    }
}

