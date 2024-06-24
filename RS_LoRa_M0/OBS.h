/*
 * ======================================================================================================================
 *  OBS.h - Observation Handeling
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 * OBS_Do() - Collect Observations, Build message, Send to logging site
 * ======================================================================================================================
 */
void OBS_Do (bool log_obs) {
  float bmx1_pressure = 0.0;
  float bmx1_temp = 0.0;
  float bmx1_humid = 0.0;
  float bmx2_pressure = 0.0;
  float bmx2_temp = 0.0;
  float bmx2_humid = 0.0;
  float sht1_temp = 0.0;
  float sht1_humid = 0.0;
  float sht2_temp = 0.0;
  float sht2_humid = 0.0;
  float mcp1_temp = 0.0;
  float mcp2_temp = 0.0;
  float batt = 0.0;
  float rain = 0.0;
  unsigned long rgds;    // rain gauge delta seconds, seconds since last rain gauge observation logged
  int msgLength;
  unsigned short checksum;
  bool SoilProbesExist = false;

  // Safty Check for Vaild Time
  if (!RTC_valid) {
    Output ("OBS_Do: Time NV");
    return;
  }

  Output ("OBS_Do()");

  // Rain Gauge
  rgds = (millis()-rainguage_interrupt_stime)/1000;
  rain = rainguage_interrupt_count * 0.2;
  rain = (isnan(rain) || (rain < QC_MIN_RG) || (rain > ((rgds / 60) * QC_MAX_RG)) ) ? QC_ERR_RG : rain;
  rainguage_interrupt_count = 0;
  rainguage_interrupt_stime = millis();
  rainguage_interrupt_ltime = 0; // used to debounce the tip

  if (ds_found[0] || ds_found[1] || ds_found[2]) {
    SoilProbesExist = true;
  }

  // Read Dallas Temp and Soil Moisture Probes
  if (SoilProbesExist) {
    DoSoilReadings();
    for (int probe=0; probe<NPROBES; probe++) {
      if (ds_found[probe]) {       
        sprintf (msgbuf, "S %d M%d T%d.%02d", 
          probe+1,  sm_reading[probe],
          (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100);
        Output (msgbuf);
      }
    }
  }

  //
  // Add I2C Sensors
  //
  if (BMX_1_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_1_chip_id == BMP280_CHIP_ID) {
      p = bmp1.readPressure()/100.0F;       // bp1 hPa
      t = bmp1.readTemperature();           // bt1
    }
    else if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_1_type == BMX_TYPE_BME280) {
        p = bme1.readPressure()/100.0F;     // bp1 hPa
        t = bme1.readTemperature();         // bt1
        h = bme1.readHumidity();            // bh1
      }
      if (BMX_1_type == BMX_TYPE_BMP390) {
        p = bm31.readPressure()/100.0F;     // bp1 hPa
        t = bm31.readTemperature();         // bt1       
      }
    }
    else { // BMP388
      p = bm31.readPressure()/100.0F;       // bp1 hPa
      t = bm31.readTemperature();           // bt1
    }
    bmx1_pressure = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
    bmx1_temp     = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    bmx1_humid    = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (BMX_2_exists) {
    float p = 0.0;
    float t = 0.0;
    float h = 0.0;

    if (BMX_2_chip_id == BMP280_CHIP_ID) {
      p = bmp2.readPressure()/100.0F;       // bp2 hPa
      t = bmp2.readTemperature();           // bt2
    }
    else if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
      if (BMX_2_type == BMX_TYPE_BME280) {
        p = bme2.readPressure()/100.0F;     // bp2 hPa
        t = bme2.readTemperature();         // bt2
        h = bme2.readHumidity();            // bh2 
      }
      if (BMX_2_type == BMX_TYPE_BMP390) {
        p = bm32.readPressure()/100.0F;       // bp2 hPa
        t = bm32.readTemperature();           // bt2
      }      
    }
    else { // BMP388
      p = bm32.readPressure()/100.0F;       // bp2 hPa
      t = bm32.readTemperature();           // bt2
    }
    bmx2_pressure = (isnan(p) || (p < QC_MIN_P)  || (p > QC_MAX_P))  ? QC_ERR_P  : p;
    bmx2_temp     = (isnan(t) || (t < QC_MIN_T)  || (t > QC_MAX_T))  ? QC_ERR_T  : t;
    bmx2_humid    = (isnan(h) || (h < QC_MIN_RH) || (h > QC_MAX_RH)) ? QC_ERR_RH : h;
  }

  if (SHT_1_exists) {                                                                               
    // SHT1 Temperature
    sht1_temp = sht1.readTemperature();
    sht1_temp = (isnan(sht1_temp) || (sht1_temp < QC_MIN_T)  || (sht1_temp > QC_MAX_T))  ? QC_ERR_T  : sht1_temp;
    
    // SHT1 Humidity   
    sht1_humid = sht1.readHumidity();
    sht1_humid = (isnan(sht1_humid) || (sht1_humid < QC_MIN_RH) || (sht1_humid > QC_MAX_RH)) ? QC_ERR_RH : sht1_humid;
  }

  if (SHT_2_exists) {
    // SHT2 Temperature
    sht2_temp = sht2.readTemperature();
    sht2_temp = (isnan(sht2_temp) || (sht2_temp < QC_MIN_T)  || (sht2_temp > QC_MAX_T))  ? QC_ERR_T  : sht2_temp;
    
    // SHT2 Humidity   
    sht2_humid = sht2.readHumidity();
    sht2_humid = (isnan(sht2_humid) || (sht2_humid < QC_MIN_RH) || (sht2_humid > QC_MAX_RH)) ? QC_ERR_RH : sht2_humid;
  }

  if (MCP_1_exists) {
    // MCP1 Temperature
    mcp1_temp = mcp1.readTempC();
    mcp1_temp = (isnan(mcp1_temp) || (mcp1_temp < QC_MIN_T)  || (mcp1_temp > QC_MAX_T))  ? QC_ERR_T  : mcp1_temp;
  }

  if (MCP_2_exists) {
    // MCP2 Temperature
    mcp2_temp = mcp2.readTempC();
    mcp2_temp = (isnan(mcp2_temp) || (mcp2_temp < QC_MIN_T)  || (mcp2_temp > QC_MAX_T))  ? QC_ERR_T  : mcp2_temp;
  }
  
  batt = vbat_get();

  // Set the time for this observation
  rtc_timestamp();
  if (log_obs) {
    Output(timestamp);
  }

  // Build JSON log entry by hand  
  // {"at":"2021-03-05T11:43:59","rg":49,"rgs":333,"sm1":234,"st1":22.33,"bp1":3,"bt1":97.875,"bh1":40.20,"bv":3.5,"hth":9}

  sprintf (msgbuf, "{\"at\":\"%s\",\"rg\":%d.%02d,\"rgs\":%d,", 
    timestamp, (int)rain, (int)(rain*100)%100, rgds);
  
  if (SoilProbesExist) {
    for (int probe=0; probe<NPROBES; probe++) {
      if (ds_found[probe]) {
        sprintf (msgbuf+strlen(msgbuf), "\"sm%d\":%d,\"st%d\":%d.%02d,",  
          probe+1, sm_reading[probe],
          probe+1, (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100);   
      }
    }
  }

  if (BMX_1_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"bp1\":%u.%04d,\"bt1\":%d.%02d,\"bh1\":%d.%02d,",
      (int)bmx1_pressure, (int)(bmx1_pressure*100)%100,
      (int)bmx1_temp, (int)(bmx1_temp*100)%100,
      (int)bmx1_humid, (int)(bmx1_humid*100)%100);
  }
  if (BMX_2_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"bp2\":%u.%04d,\"bt2\":%d.%02d,\"bh2\":%d.%02d,",
      (int)bmx2_pressure, (int)(bmx2_pressure*100)%100,
      (int)bmx2_temp, (int)(bmx2_temp*100)%100,
      (int)bmx2_humid, (int)(bmx2_humid*100)%100);
  }
  
  if (SHT_1_exists) { 
     sprintf (msgbuf+strlen(msgbuf), "\"st1\":%d.%02d,\"sh1\":%d.%02d,",
      (int)sht1_temp, (int)(sht1_temp*100)%100,
      (int)sht1_humid, (int)(sht1_humid*100)%100);   
  }
  if (SHT_2_exists) { 
    sprintf (msgbuf+strlen(msgbuf), "\"st2\":%d.%02d,\"sh2\":%d.%02d,",
      (int)sht2_temp, (int)(sht2_temp*100)%100,
      (int)sht2_humid, (int)(sht2_humid*100)%100);   
  }
  
  if (MCP_1_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"mt1\":%d.%02d,",
      (int)mcp1_temp, (int)(mcp1_temp*100)%100);    
  }
  if (MCP_2_exists) {
    sprintf (msgbuf+strlen(msgbuf), "\"mt2\":%d.%02d,",
      (int)mcp1_temp, (int)(mcp2_temp*100)%100);        
  }
   
  sprintf (msgbuf+strlen(msgbuf), "\"bv\":%d.%02d,\"hth\":%d}", 
    (int)batt, (int)(batt*100)%100, SystemStatusBits);

  // Log Observation to SD Card
  if (log_obs) {
    SD_LogObservation(msgbuf);
  }
  Serial_write (msgbuf);

  // ====================
  // Send LoRa Message
  // ====================
  // Message Formats
  //    NCS    Length and Checksum
  //    RS2,   Rain Soil type 2
  //    1,     Unit ID
  //    13,    Transmit Counter
  //    nn.nn,  Rain
  //    nnnn,   Rain Gauge Delta Seconds 
  //    363,   Soil 1
  //    74.07, Temp 1
  //    242,   Soil 2
  //    73.96, Temp 2
  //    180,   Soil 3
  //    74.19, Temp 3
  //    bmxt,bm1p,bm1h,
  //    bm2t,bm2p,bm2h, 
  //    6.35   Volts
  //    status bits

  // Build LoRa message
  strcpy (msgbuf, "NCS");                 // N will be replaced with binary value (byte) representing (string length - 1)
                                          // This is sow we can send variable length AES encrypted strings
                                          // The receiving side need to know characters folling this first byte
                                          // CS is the place holder for the Checksum
  // Message type
  if (!BMX_1_exists && !BMX_2_exists && !SoilProbesExist) {
    sprintf (msgbuf+strlen(msgbuf), "RS2,"); // report rain only
  }
  else if (!BMX_1_exists && !BMX_2_exists && SoilProbesExist) {
    sprintf (msgbuf+strlen(msgbuf), "RS3,"); // report rain and soil
  }
  else if ((BMX_1_exists || BMX_2_exists) && !SoilProbesExist) {
    sprintf (msgbuf+strlen(msgbuf), "RS4,"); // report rain and bmx
  }
  else {
    sprintf (msgbuf+strlen(msgbuf), "RS5,"); // report all
  }
  
  // Rain Soil Station ID
  sprintf (msgbuf+strlen(msgbuf), "%d,", cf_lora_unitid);    // Must be unique if multiple are transmitting

  // Transmit Counter
  sprintf (msgbuf+strlen(msgbuf), "%d,", SendSensorMsgCount);

  // Rain Gauge
  sprintf (msgbuf+strlen(msgbuf), "%d.%02d,%d,", (int)rain, (int)(rain*100)%100, rgds);

  if (SoilProbesExist) {
    for (int probe=0; probe<NPROBES; probe++) {
      if (ds_found[probe]) {
        sprintf (msgbuf+strlen(msgbuf), "%d,%d.%02d,",  
          sm_reading[probe],
          (int)ds_reading[probe], (int)(ds_reading[probe]*100)%100);   
      }
      else {
        sprintf (msgbuf+strlen(msgbuf), "0,85.00,"); // No Sensor 
      }
    }    
  }     

  if (BMX_1_exists || BMX_2_exists) {
    sprintf (msgbuf+strlen(msgbuf), "%u.%02d,%d.%02d,%d.%02d,", 
      (int)bmx1_pressure, (int)(bmx1_pressure*100)%100,
      (int)bmx1_temp, (int)(bmx1_temp*100)%100,
      (int)bmx1_humid, (int)(bmx1_humid*100)%100);
    sprintf (msgbuf+strlen(msgbuf), "%u.%02d,%d.%02d,%d.%02d,", 
      (int)bmx2_pressure, (int)(bmx2_pressure*100)%100,
      (int)bmx2_temp, (int)(bmx2_temp*100)%100,
      (int)bmx2_humid, (int)(bmx2_humid*100)%100);
  }
 
  sprintf (msgbuf+strlen(msgbuf), "%d.%02d,%d", 
    (int)batt, (int)(batt*100)%100, SystemStatusBits);

  msgLength = strlen(msgbuf);
  // Compute checksum
  checksum=0;
  for(int i=3;i<msgLength;i++) {
     checksum += msgbuf[i];
  }

  // Let serial console see this LoRa message
  Serial_write (msgbuf);
  
  msgbuf[0] = msgLength;
  msgbuf[1] = checksum >> 8;
  msgbuf[2] = checksum % 256;
   
  SendAESLoraWanMsg (128, msgbuf, msgLength);

  SendSensorMsgCount++;

  if (log_obs) {
    Output(timestamp);
    sprintf (msgbuf, "%d %d.%02d %04X", 
      0,                                                 // FOOBAR
      (int)batt, (int)(batt*100)%100,
      SystemStatusBits);
    
    Output(msgbuf);
    
    sprintf (msgbuf, "%d.%02d %d.%02d",
      (int)bmx1_pressure, (int)(bmx1_pressure*100)%100,
      (int)bmx2_pressure, (int)(bmx2_pressure*100)%100);
    Output(msgbuf);
  }
}
