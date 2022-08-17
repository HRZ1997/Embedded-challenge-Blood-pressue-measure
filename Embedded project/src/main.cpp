/*This is the project code of Embedded Challenge,
 this Work is contributed by Haoran Zhang(hz2470) and Pengfei Song(ps4516)
 In the project we use STM32F4-discovery Board and Honeywell MPRLS0300YG00001BB
 The board transfer data with the sensor through spi
 The pins are assigned as
SPI spi(PB_15, PB_14, PB_10); // mosi, miso, sclk
DigitalOut cs(PB_12);  //ss
Instructions:
1.Pump up until the pressure reach 150mmHg
2.slowly realease the valve
3.wait for the data to be collected and read data.
*/
#include <mbed.h>
#include <string.h>
#include <stdio.h>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"
#include "drivers/stm32f429i_discovery_lcd.h"
#include "drivers/stm32f429i_discovery.h"
#include <stdlib.h>

//basic setting from sensor's manual, for converting raw data to pressure data in mmHg
#define OUT_RAW_MIN 0x66666
#define OUT_RAW_MAX 0x3995B2
#define P_MAX 300
#define P_MIN 0

//Converting raw data to pressure data in mmHg, the resolution of sensor is 8.9e-5, 
//so calculating with float is enough
float raw2p (uint32_t raw_data){
	float pressure;
	pressure =((float) (raw_data - OUT_RAW_MIN)*(P_MAX - P_MIN)/(float) (OUT_RAW_MAX-OUT_RAW_MIN));
	return pressure;
}

//the microcontroller read from the sensor every SAMPLE_SPEED ms
//assume the heart rate of a normal is 80 bpm, according to nyquist theory
//the SAMPLE_SPEED should be at least 6
//The pressure decreasing rate should not be fast than 4 mmHg/s 
#define SAMPLE_SPEED 4
#define DECREASING_RATE_BOUND 6
//float array to store pressure data, assume the pressure decreasing rate is 4 mmHg/s
//and the sampling range is (150mmHg, 50mmHg), so there is at least 100 mmHg/(4 mmHg/s)/SAMPLE_SPEED = 6750 samples
//the data array should be 2 times of the least sample size for margin
float sdata[15000];
float speed[15000];
float acc[15000];
//ready flag for spi transformation
#define READY_FLAG 1

//set up of LCD
#define BACKGROUND 1
#define FOREGROUND 0
#define GRAPH_PADDING 5
LCD_DISCO_F429ZI lcd;

  //buffer for holding displayed text strings
char display_buf[2][60];
uint32_t graph_width=lcd.GetXSize()-2*GRAPH_PADDING;
uint32_t graph_height=graph_width;

  //sets the background layer 
  //to be visible, transparent, and
  //resets its colors to all black
void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}

  //resets the foreground layer to
  //all black
void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN);
}

TS_StateTypeDef  TS_State = {0};


//buffer clear for the first 4 bytes
void buf_clear(uint8_t * buf){
  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
}

//define spi pins
SPI spi(PB_15, PB_14, PB_10); // mosi, miso, sclk
DigitalOut cs(PB_12);  //ss
//define read&write buffer for spi
static uint8_t read_buffer[32];
static uint8_t write_buffer[32];
//event flag and cd function for spi
EventFlags flags;

void cb(int event){
  flags.set(READY_FLAG);
  //deselect the sensor
  cs=1;
}
/*
float pulse(uint32_t ii, float a){
    if (ii%376 <188){
      
      return a*(ii%376)/188;
    }
    else {
      return a*(188-(ii%376))/188;
    }
    return -1;
}

void data_generate(uint32_t ii, float * data){
    float a;
    data[ii] = 100 + (7500 - (float)ii) /(float)150;
    if (ii >= 1500 &&ii < 13500){
      if (ii <7500){
        a = (float)(ii-1500)/2000;
      }
      else {
        a = (float)(13500-ii)/2000;
      }
    data[ii] += pulse(ii, a);
    }
}
*/
//read data from sensor through spi, which is an 24-bit integer, and convert it to float,
//storing it into the data array
//this function needs one parameter i for indexing and one pointer for the data array
void spi_read(int i, float * data){

    buf_clear(read_buffer);
    buf_clear(write_buffer);

    write_buffer[0] = 0xAA;
    write_buffer[1] = 0x00;
    write_buffer[2] = 0x00;

    // Select the device by seting chip select low 
    cs=0;
    spi.transfer(write_buffer,3,read_buffer,3,cb,SPI_EVENT_COMPLETE );
    flags.wait_all(READY_FLAG);

    buf_clear(read_buffer);
    buf_clear(write_buffer); 

    write_buffer[0] = 0xF0;
    cs=0;
    spi.transfer(write_buffer,4,read_buffer,4,cb,SPI_EVENT_COMPLETE );
    flags.wait_all(READY_FLAG);

    //reconstruct the data into uint32_t type which indicate raw data from the sensor
    //the count in the range of (2.5%, 22.5%) of 2^24 
    uint32_t raw_value = 0;
    for (uint8_t i = 1; i < 4; i++){
      raw_value |= read_buffer[i];
      if (i != 3) raw_value = raw_value << 8; //bitwise shift
    }

    data[i] = raw2p(raw_value);
}

//data process function, the algorithm reference from
//https://www.researchgate.net/profile/Mohamad-Forouzanfar/publication/277083801_Oscillometric_Blood_Pressure_Estimation_Past_Present_and_Future/links/567f964508ae1e63f1e866e7/Oscillometric-Blood-Pressure-Estimation-Past-Present-and-Future.pdf
float meanbp;
float maxvalue;
float dbpvalue;
float sbpvalue;
float real_dbpvalue;
uint32_t dbp_location;
float real_sbpvalue;
uint32_t sbp_location;
float sbp;
float dbp;
uint32_t location;
void data_process(uint32_t bound, float * DataSet){
    uint32_t i= 0;
    for(i=0;i<bound;i++){
      if(i==0 || i == (bound-1)){
        speed[i] = DataSet[i];
      }
      else{
        speed[i] = (DataSet[i+1] - DataSet[i-1])/2;
      }
    }

    for(i=0;i<bound;i++){
      if(i==0 || i == (bound-1)){
          acc[i] = speed[i];
      }
      else{
        acc[i] = (speed[i+1] - speed[i-1])/2;
      }
    }

    location = 10;
    maxvalue = acc[1];
    for(i=1;i<bound-10;i++){
        if (acc[i]> maxvalue){
            maxvalue = acc[i];
            location = i;
        }
    }

    meanbp = DataSet[location];
    dbpvalue = maxvalue*0.82;
    sbpvalue = maxvalue*0.55;


    real_dbpvalue = dbpvalue;
    for(i=location+10;i<bound-10;i++){
        if (abs(dbpvalue-acc[i])>abs(dbpvalue-acc[i+1])){
            real_dbpvalue = acc[i];
            dbp_location = i;
        }
    }

    real_sbpvalue = sbpvalue;
    for(i=10;i<location-10;i++){
        if (abs(sbpvalue-acc[i])>abs(sbpvalue-acc[i+1])){
            real_sbpvalue = acc[i];
            sbp_location = i;
        }
    }

    sbp = DataSet[sbp_location];
    dbp = DataSet[dbp_location];
    
    int beat = 0;
    for(i=sbp_location; i<dbp_location;i++){
        if (DataSet[i]>DataSet[i-1]&&DataSet[i]>DataSet[i-2]&&DataSet[i]>DataSet[i-3]
        &&DataSet[i]>DataSet[i+1]&&DataSet[i]>DataSet[i+2]&&DataSet[i]>DataSet[i+3])
        {
            beat++;
        }

    }


    float timebetween = (dbp_location - sbp_location)*((float)SAMPLE_SPEED/1000);
    int hrate = (int)(60*beat/timebetween);


    setup_foreground_layer();
    lcd.SelectLayer(FOREGROUND); 
    snprintf(display_buf[0],60,"heart rate: %d", hrate);
    snprintf(display_buf[1],60,"sbp:%d,dbp:%d",sbp, dbp);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *) display_buf[0], CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *) display_buf[1], CENTER_MODE);
}

int main(){
  //set up background
  setup_background_layer();
  setup_foreground_layer();
  
  //set up spi
  cs = 1;
  spi.format(8,0);
  spi.frequency(400'000);

  buf_clear(read_buffer);
  buf_clear(write_buffer);

//the parameters that need to be defined before entering the loop
//int ii for indexing the data array
//int speed_monitor_counter to define if the pressure is decreasing too fast
  uint32_t ii = 0;
  uint16_t speed_monitor_counter = 0;
  while(1){
  
    setup_foreground_layer();
    lcd.SelectLayer(FOREGROUND); 
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *) "Charge up to begin", CENTER_MODE);
    thread_sleep_for(5000);
//begin the process while the pump was charge over 150 mmHg
    while(1){
      spi_read(0, sdata);
      if (sdata[0] >=  145){
        //reset the screen
        setup_foreground_layer();
        lcd.SelectLayer(FOREGROUND); 
        //display instruction on the screen
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *) "Charge ready", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *) "Touch the screen", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(7), (uint8_t *) "to continue", CENTER_MODE); 
        break;
      }
    }

//after touching the screen, the read from sensor starts, 
//while the data drop below 145 mmHg, the data would be officially stored into data array
/*add touching screen function*/
    while(1){
      thread_sleep_for(3000);
      break;
      BSP_TS_GetState(&TS_State);
      if(TS_State.TouchDetected) {
        break;
      }
    }
    setup_foreground_layer();
    lcd.SelectLayer(FOREGROUND); 
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *) "Release the valve", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *) "smoothly", CENTER_MODE);
    thread_sleep_for(2000);

    ii = 0;
    speed_monitor_counter = 0;
    while(ii<15000){
      spi_read(ii, sdata);
      if(ii != 0){
        //notification if the data drop too fast for ten time continuesly
        if((sdata[ii-1]-sdata[ii])>(0.004*SAMPLE_SPEED)){
          speed_monitor_counter++;
          if(speed_monitor_counter>300){
            setup_foreground_layer();
            lcd.SelectLayer(FOREGROUND); 
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *) "Please be slower", CENTER_MODE); 
          }
        }
        else{
          speed_monitor_counter = 0;
          setup_foreground_layer();
        }
      }
      if (sdata[ii] < 50){
        break;
      }
      /*display loading page on lcd*/
      if(ii%700 == 300){
        lcd.SelectLayer(BACKGROUND); 
        lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "Loading", CENTER_MODE);  
      }
      ii++;
      thread_sleep_for(SAMPLE_SPEED);
    }

    setup_foreground_layer();
    setup_background_layer();
    lcd.SelectLayer(FOREGROUND); 
    lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "Reading Completed", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(11), (uint8_t *) "Processing", CENTER_MODE);
    thread_sleep_for(2000);
    /*data process function*/
    data_process(ii, sdata);

    thread_sleep_for(5000);
    lcd.DisplayStringAt(0, LINE(15), (uint8_t *) "Thanks for using", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(16), (uint8_t *) "Wait to continue", CENTER_MODE);
    thread_sleep_for(10000); 
  }
}

