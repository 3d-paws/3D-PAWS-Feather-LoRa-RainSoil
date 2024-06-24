/*
 * ======================================================================================================================
 *  Soil.h - Soil Moaisture, Soil Temp (OneWire), Rain Gauge
 * ======================================================================================================================
 */

 /*
 * =======================================================================================================================
 *  Number of Soil Moisture Sensors Connected
 * =======================================================================================================================
 */
#define NPROBES           2        // Max Number of Soil Probes


 /*
 * =======================================================================================================================
 *  Number of Soil Moisture Sensors Connected
 * =======================================================================================================================
 */
#define PROBES_POWER_PIN  11       // Pin for enambling power to Dallas and Soil Moisture Sensors

/*
 * =======================================================================================================================
 *  Soil Moisture Sensor
 * =======================================================================================================================
 */
// Define Soil Moisture Sensor Pin Names
int sm_pn[NPROBES] = { A0, A2};
int sm_reading[NPROBES] = { -1, -1 };



/*
 * ======================================================================================================================
 *  rainguage_interrupt_handler() - This function is called whenever a magnet/interrupt is detected by the arduino
 * ======================================================================================================================
 */
void rainguage_interrupt_handler()
{
  if ((millis() - rainguage_interrupt_ltime) > 500) { // Count tip if a half second has gone by since last interupt
                                                      // Not worring about system clock roll over
    rainguage_interrupt_ltime = millis();
    rainguage_interrupt_count++;
  }   
}


/*
 * =======================================================================================================================
 *  One Wire
 * =======================================================================================================================
 */
#define DS0_PIN A1
#define DS1_PIN A3
// Allociation for each pin with Dallas Sensors attached
OneWire  ds[NPROBES] = { OneWire(DS0_PIN), OneWire(DS1_PIN)};


// Dallas Sensor Address
byte ds_addr[NPROBES][8] = { 
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
};

bool  ds_found[NPROBES]    = { false, false };
float ds_reading[NPROBES]  = { 0.0, 0.0 };
bool  ds_valid[NPROBES]    = { false, false };

/*
 * =============================================================
 * getDSTempByAddr() - 
 * =============================================================
 */
bool getDSTempByAddr(int probe, int delayms) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  
  ds[probe].reset();
  ds[probe].select(ds_addr[probe]);

  // start conversion, with parasite power on at the end
  ds[probe].write(0x44,0); // set to 1 for parasite otherwise 0
  
  delay(delayms);     // maybe 750ms is enough, maybe not
  
  present = ds[probe].reset();
  ds[probe].select(ds_addr[probe]);    
  ds[probe].write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds[probe].read();
  }
    
  if (OneWire::crc8(data, 8) != data[8]) {
    // CRC on the Data Read
    
    // Return false no temperture because of the CRC error
    ds_reading[probe]=0.0;
    ds_valid[probe] = false;
  }
  else {
    // convert the data to actual temperature
    unsigned int raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9bit res, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time

    float t = raw / 16.0;  // Max 85.0C, for fahrenheit = (raw / 16.0) * 1.8 + 32.0   or Max 185.00F
    ds_reading[probe] = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    
    if (ds_reading[probe] != QC_ERR_T) { 
      ds_valid[probe] = true;
    }
    else {
      ds_valid[probe] = false;
    }
    
    // Have a temp,  but it might have value 85.00C / 185.00F which means it was just plugged in
  }
  return(ds_valid[probe]);
}

/*
 * =============================================================
 * getDSTemp()
 * =============================================================
 */
void getDSTemp(int probe) {
  int status;
  byte i;

  status = getDSTempByAddr(probe, 250);
  if (status == false) {
    // reread temp - it might of just been plugged in
    status = getDSTempByAddr(probe, 750);
  }

  // temperture returned in ds_reading[probe]
  // state returned in ds_valid[probe]
  // if false and temp was equal to 0.0 then a CRC happened

  /*
  if (status) { // Good Value Read
    sprintf (msgbuf, "%d TS %d.%02d OK", 
      probe, (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100);
    Output (msgbuf);
  }
  else { // We read a temp but it was a bad value
    sprintf (msgbuf, "%d TS %d.%02d BAD", 
      probe, (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100);
    Output (msgbuf);
  }
  */
}

/*
 * =============================================================
 * Scan1WireBus() - Get Sensor Address and Read Temperature
 * =============================================================
 */
bool Scan1WireBus(int probe) {
  byte i;
  byte present = 0;
  byte type_s;
  byte addr[8];
  bool found = false;
  
  // Reset and Start search
  ds[probe].reset_search();
  delay(250);

  // Expecting one and only one probe on the pin
  
  if (!ds[probe].search(ds_addr[probe])) {
    // Sensor Not Found
    sprintf (msgbuf, "DS %d NF", probe+1);
    Output (msgbuf);
  }
  else if (OneWire::crc8(ds_addr[probe], 7) != ds_addr[probe][7]) {
    // Bad CRC
    sprintf (msgbuf, "DS %d CRC", probe+1); // Bad CRC
    Output (msgbuf);
  }
  else if (ds_addr[probe][0] != 0x28) { // DS18B20
    // Unknown Device Type
    sprintf (msgbuf, "DS %d UKN %d", probe+1, ds_addr[probe][0]); // Unknown Device type
    Output (msgbuf);
  } 
  else {
    ds_found[probe] = true;
    sprintf (msgbuf, "DS %d %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", 
       probe+1,
       ds_addr[probe][0], ds_addr[probe][1], ds_addr[probe][2], ds_addr[probe][3],
       ds_addr[probe][4], ds_addr[probe][5], ds_addr[probe][7], ds_addr[probe][7]); 
    Output (msgbuf);
    found = true;
  }
  return (found);
}

/*
 * =============================================================
 * ds_initialize() - 
 * =============================================================
 */
void ds_initialize() {
  digitalWrite(PROBES_POWER_PIN, HIGH);      // turn on power to sensors
  delay(250);
  pinMode(DS0_PIN, INPUT);
  pinMode(DS1_PIN, INPUT);
  for (int probe=0; probe<NPROBES; probe++) {
    
    bool ds_found = Scan1WireBus(probe);  // Look for Dallas Sensor on pin get it's address
    
    if (!ds_found) {
      // Retry 
      delay (250);
      ds_found = Scan1WireBus(probe);
    }
    
    if (ds_found) {
      getDSTemp(probe);
      if (ds_valid[probe]) { // Good Value Read
        sprintf (msgbuf, "DS %d %d.%02d OK", 
          probe+1, (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100); 
      }
      else { // We read a temp but it was a bad value
        sprintf (msgbuf, "DS %d %d.%02d BAD", 
          probe+1, (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100);
      }
      Output (msgbuf);
      
      pinMode(sm_pn[probe], INPUT);
      sm_reading[probe] = (int) analogRead(sm_pn[probe]);
      sprintf (msgbuf, "SM %d %d", probe+1, sm_reading[probe]);
      Output (msgbuf);
      
    }
  }
  digitalWrite(PROBES_POWER_PIN, LOW);      // turn off power to sensors
}

/*
 * ======================================================================================================================
 * DoSoilReadings() - Get Soil Moisture and temperature readings if we have found any DS probe in setup()
 * ======================================================================================================================
 */
void DoSoilReadings () {
  if (ds_found[0] || ds_found[1] || ds_found[2]) {
    digitalWrite(PROBES_POWER_PIN, HIGH);      // turn on power to 2n2222 / sensor
    delay(10);//wait 10 milliseconds
    
    for (int probe=0; probe<NPROBES; probe++) {
      // If DS was found in setup() get Temperature and soil moisture readings
      if (ds_found[probe]) {
        getDSTemp(probe); // Get soil temperature           
        sm_reading[probe] = (int) analogRead(sm_pn[probe]);  // Get Soil Moisture
      }
    }
    
    digitalWrite(PROBES_POWER_PIN, LOW);      // turn off power to 2n2222 / sensor
  } 
}
