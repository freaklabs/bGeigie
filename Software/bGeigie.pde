#include <SD.h>
#include <chibi.h>
#include <limits.h>
#include "gps_geiger.h"

#define TIME_INTERVAL 5000
#define NX 12
#define DEST_ADDR 0x1234
#define PRINT_BUFSZ 80

static const int chipSelect = 10;
static const int radioSelect = A3;
static const int sdPwr = 4;

static char line[LINE_SZ];
static byte i = 0;
static gps_t gps;
static unsigned long start_time;
 
unsigned int shift_reg[NX] = {0};
unsigned int reg_index = 0;

// This is the data file object that we'll use to access the data file
File dataFile; 

static char msg1[] PROGMEM = "SD init...\n";
static char msg2[] PROGMEM = "Card failure...\n";
static char msg3[] PROGMEM = "Card initialized\n";
static char msg4[] PROGMEM = "Error: Log file cannot be opened.\n";

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup()
{
  char tmp[25];
  gps_init();
    
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  pinMode(radioSelect, OUTPUT);
  pinMode(sdPwr, OUTPUT);
  
  digitalWrite(sdPwr, LOW);           // turn on SD card power
  digitalWrite(radioSelect, HIGH);    // disable radio chip select
  digitalWrite(chipSelect, HIGH);     // disable SD card chip select 
  
  Serial.begin(9600);
  chibiInit();
  
  // put radio to sleep to save power
  chibiSleepRadio(1);
  
  strcpy_P(tmp, msg1);
  Serial.print(tmp);

  // attach interrupt to INT1
  attachInterrupt(1, count_pulse, RISING);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    while(1) 
    {
      strcpy_P(tmp, msg2);
      Serial.print(tmp);
    }
  }
  strcpy_P(tmp, msg3);
  Serial.print(tmp);
  
  // print free RAM. we should try to maintain about 300 bytes for stack space
  Serial.println(FreeRam());

  start_time = millis();
  digitalWrite(sdPwr, HIGH);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void loop()
{
  char tmp[25];
  char *tok[SYM_SZ] = {0};
  int j = 0;
  
  while (Serial.available())
  {
    if (i < LINE_SZ)
    {
      line[i] = Serial.read();
      if (line[i] == '\n')
      {
        line[i+1] = '\0';
        i=0;
        
          // dump the raw GPS data
//        Serial.print(line);
        
        // tokenize line
        tok[j] = strtok(line, ",");
        do
        {
           tok[++j] = strtok(NULL, ",");
        } while ((j < SYM_SZ) && (tok[j] != NULL));

        // reset the index variable for next run
        j = 0;

        // parse line and date/time and update gps struct
        if (strcmp(tok[0], "$GPRMC") == 0)
        {
          parse_line_rmc(tok);
          parse_datetime();
        }
        else if (strcmp(tok[0], "$GPGGA") == 0)
        {
           parse_line_gga(tok);
        }
        
        // clear the flag so that we know its okay to read the data
        gps.updating = 0;
      }
      else
      {
        // still taking in data. increment index and make sure the updating flag is set
        gps.updating = 1;
        i++;
      }
    }
    else
    {
      i = 0;
    }
  }
  
  // generate CPM every TIME_INTERVAL seconds
  if (elapsed_time(start_time) > TIME_INTERVAL)
  {
    if (!gps.updating)
    {
      unsigned int cpm;
      
      // generate timestamp. only update the start time if 
      // we printed the timestamp. otherwise, the GPS is still 
      // updating so wait until its finished and generate timestamp
      cpm = cpm_gen();
      memset(line, 0, LINE_SZ);
      gps_gen_timestamp(line, shift_reg[reg_index], cpm);
      
      // turn on SD card power and delay a bit to initialize
      digitalWrite(sdPwr, LOW);
      delay(10);
      
      // init the SD card see if the card is present and can be initialized:
      while (!SD.begin(chipSelect));
      
      // dump data to SD card
      dataFile = SD.open("SAFECAST.LOG", FILE_WRITE);
      if (dataFile)
      {
        Serial.println(line);
        dataFile.println(line);
        dataFile.close();
        
        // send out wirelessly. first wake up the radio, do the transmit, then go back to sleep
        chibiSleepRadio(0);
        delay(10);
        chibiTx(DEST_ADDR, (byte *)line, LINE_SZ);
        chibiSleepRadio(1);
      }
      else
      {
        char tmp[40];
        strcpy_P(tmp, msg4);
        Serial.print(tmp);
      }   
      
      // write to backup file as well
      dataFile = SD.open("SAFECAST.BAK", FILE_WRITE);
      if (dataFile)
      {
        dataFile.println(line);
        dataFile.close();
      }
      else
      {
        char tmp[40];
        strcpy_P(tmp, msg4);
        Serial.print(tmp);
      }
      
      //turn off sd power        
      digitalWrite(sdPwr, HIGH); 
      
      // update the start time for the next interval
      start_time = millis();
      
      // change the shift register and clear it out
      reg_index = (reg_index+1) % NX;
      shift_reg[reg_index] = 0;
    }
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void gps_init()
{
  memset(&gps, 0, sizeof(gps_t));
  
  // set static values first
  strcpy(gps.dev_name, "GT-703");
  strcpy(gps.meas_type, "AIR");
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void parse_line_rmc(char **token)
{
  memcpy(&gps.utc,        token[1],     UTC_SZ-1);
  memcpy(&gps.lat,        token[3],     LAT_SZ-1);
  memcpy(&gps.lat_hem,    token[4],     DEFAULT_SZ-1);
  memcpy(&gps.lon,        token[5],     LON_SZ-1);
  memcpy(&gps.lon_hem,    token[6],     DEFAULT_SZ-1);
  memcpy(&gps.speed,      token[7],     SPD_SZ-1);
  memcpy(&gps.course,     token[8],     CRS_SZ-1);
  memcpy(&gps.date,       token[9],     DATE_SZ-1);
  memcpy(&gps.checksum,   token[10],    CKSUM_SZ-1);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void parse_line_gga(char **token)
{
    memcpy(&gps.status,    token[6], DEFAULT_SZ-1);
    memcpy(&gps.num_sat,   token[7], NUM_SAT_SZ-1);
    memcpy(&gps.precision, token[8], PRECISION_SZ-1);
    memcpy(&gps.altitude,  token[9], ALTITUDE_SZ-1);
}


/**************************************************************************/
/*!

*/
/**************************************************************************/
void parse_datetime()
{
    memset(&gps.datetime, 0, sizeof(date_time_t));

    // parse UTC time
    memcpy(gps.datetime.hour, &gps.utc[0], 2);
    memcpy(gps.datetime.minute, &gps.utc[2], 2);
    memcpy(gps.datetime.second, &gps.utc[4], 2);

    // parse UTC calendar
    memcpy(gps.datetime.day, &gps.date[0], 2);
    memcpy(gps.datetime.month, &gps.date[2], 2);
    memcpy(gps.datetime.year, &gps.date[4], 2);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void gps_gen_timestamp(char *buf, unsigned int counts, unsigned int cpm)
{
  byte len;
  
  memset(buf, 0, LINE_SZ);
  sprintf(buf, "20%s-%s-%s,%s:%s:%s,%d,%s%s,%s%s,%s,%s,%s,%s,%s,%s",  \
              gps.datetime.year, gps.datetime.month, gps.datetime.day,  \
              gps.datetime.hour, gps.datetime.minute, gps.datetime.second, \
              cpm, \
              gps.lat, gps.lat_hem, \
              gps.lon, gps.lon_hem, \
              gps.status, \
              gps.num_sat, \
              gps.precision, \
              gps.altitude, \
              gps.dev_name, \
              gps.meas_type);
   len = strlen(buf) + 1;
   buf[len] = '\0';
}

/**************************************************************************/
// calculate elapsed time
/**************************************************************************/
unsigned long elapsed_time(unsigned long start_time)
{
  unsigned long stop_time = millis();
  
  if (start_time >= stop_time)
  {
    return start_time - stop_time;
  }
  else
  {
    return (ULONG_MAX - (start_time - stop_time));
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void count_pulse()
{
  shift_reg[reg_index]++;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
int cpm_gen()
{
   unsigned int i;
   unsigned int c_p_m = 0;
   
   // sum up
   for (i=0 ; i < NX ; i++)
     c_p_m += shift_reg[i];
   
   return c_p_m;
}



