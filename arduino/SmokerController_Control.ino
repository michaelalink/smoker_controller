void sampleTemperatures(void *pvParameters) {
  //setup the chip select pins for the thermocouple chips
  Adafruit_MAX31856 maxthermo[numSensors] = {Adafruit_MAX31856(5),Adafruit_MAX31856(27),Adafruit_MAX31856(26),Adafruit_MAX31856(25)};

  // the max31856 take around 100 ms to do a reading we'll give them a little extra time
  // we sample so frequently to filter and hopefully get a less noisey value (so far seems to be working!)
  int sampleTime = 150;
  
  //unsigned long previousMillis = millis();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // setup variables to temporarly store readings so we don't have to hold other processes from reading as long
  float tempProbe[numSensors];
  float tempProbeCJ[numSensors];
  int probefault[numSensors];
  float inputVoltsRaw;
  float adcMaxInput = 3.3*((inputVoltR1+inputVoltR2)/inputVoltR2);
  
  // setup a variable to store the filer value for each thermocouple
  float filterStregnth[numSensors];

  // set intial values for the sensors
  for(int tcSensor = 0; tcSensor < numSensors; tcSensor++){
    maxthermo[tcSensor].begin();
    maxthermo[tcSensor].setThermocoupleType(MAX31856_TCTYPE_K);
    maxthermo[tcSensor].setConversionMode(MAX31856_CONTINUOUS);
    maxthermo[tcSensor].setNoiseFilter(MAX31856_NOISE_FILTER_60HZ);
    maxthermo[tcSensor].setTempFaultThreshholds(-40.0, 1300.0);
    maxthermo[tcSensor].setColdJunctionFaultThreshholds(-40, 125);
    //initial read wait for chip to have its first read after power up
    vTaskDelay(sampleTime / portTICK_PERIOD_MS);
    filteredTemps[tcSensor] =(maxthermo[tcSensor].readThermocoupleTemperature()*1.8) + 32.0;
    filteredCJs[tcSensor] = (maxthermo[tcSensor].readCJTemperature()*1.8) + 32.0;
    filteredFaults[tcSensor] = maxthermo[tcSensor].readFault();
    filterStregnth[tcSensor] = 0.15;
  }
  for (;;) {
    for(int tcSensor = 0; tcSensor < numSensors; tcSensor++){
      filterTime[tcSensor] = max(filterTime[tcSensor],float(sampleTime/1000.0));
      filterStregnth[tcSensor] = (sampleTime/1000.0) * (1/max(filterTime[tcSensor],float(sampleTime/1000.0)));
      tempProbeCJ[tcSensor] = (maxthermo[tcSensor].readCJTemperature()*1.8) + 32.0;
      tempProbe[tcSensor] = (maxthermo[tcSensor].readThermocoupleTemperature()*1.8) + 32.0;
      probefault[tcSensor] = maxthermo[tcSensor].readFault();
      probefault[tcSensor] = maxthermo[tcSensor].readFault();
      // Read the thermocouple type, we only use K (0b0011) so if we read a "0" we probably aren't talking to the chip
      // set the fault, and try restarting the comms
      // the way this is setup we can never use type B therocouples, we could probably read a different register to verify comms
      int tcType = maxthermo[tcSensor].getThermocoupleType();
      if(!tcType or filteredFaults[tcSensor] == 0xff){
        if(!tcType) probefault[tcSensor] = 0xff; // set all the fault bits I don't currently know of a way a combination of other faults could set all of them
        if (filteredFaults[tcSensor] != probefault[tcSensor]){
          Serial.print("[SMPL] Error While Reading TC ");
          Serial.println(tcSensor);
        }
        // if we are able to restart comms we should see a reading next scan
        maxthermo[tcSensor].begin();
        maxthermo[tcSensor].setThermocoupleType(MAX31856_TCTYPE_K);
        maxthermo[tcSensor].setConversionMode(MAX31856_CONTINUOUS);
        tempProbe[tcSensor] = 0;
      }


      // take exclusive access to the filtered temperatues
      xSemaphoreTake( xFiltTempMutex, portMAX_DELAY );
      tempDeviation[tcSensor] = ((tempProbe[tcSensor] - filteredTemps[tcSensor]) / tempProbe[tcSensor])*100;
      // Filter the value so long as it is good
      if(!probefault[tcSensor] || !filteredFaults[tcSensor]) filteredTemps[tcSensor] = (filterStregnth[tcSensor] * tempProbe[tcSensor]) + ((1 - filterStregnth[tcSensor]) * filteredTemps[tcSensor]);
      // When there is a reported fault don't use / reset the filter so we don't have a bunch of garbage in it
      else filteredTemps[tcSensor] = tempProbe[tcSensor];
      filteredCJs[tcSensor] = (filterStregnth[tcSensor] * tempProbeCJ[tcSensor]) + ((1 - filterStregnth[tcSensor]) * filteredCJs[tcSensor]);
      filteredFaults[tcSensor] = probefault[tcSensor];
      xSemaphoreGive( xFiltTempMutex );
      //vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    //Read input voltage
    inputVoltsRaw = analogRead(inputVoltPin);
    // The ADC is not great, this will need to be adjusted or a calibrate function created
    inputVoltsRaw = inputVoltsRaw * 1.143;
    inputVoltsRaw = inputVoltsRaw / 4095.0;
    inputVoltsRaw = inputVoltsRaw * adcMaxInput;
    InputVolts = (0.075 * inputVoltsRaw) + ((1-0.075) * InputVolts);
    
    vTaskDelayUntil(&xLastWakeTime,sampleTime / portTICK_PERIOD_MS);
  }
}

void pidControl(void *pvParameters){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const int taskScanTime = 100; // milliseconds
  servo.attach(4);
  ledcSetup(fanChannel,fanPWMFreq,10);
  ledcAttachPin(fanPin, fanChannel);
  unsigned int fanDuty = 0;
  vTaskDelay(100 / portTICK_PERIOD_MS);
  myPID.SetSampleTime(500);
  myPID.SetOutputLimits(0,100);
  double servoInMin, servoInMax, servoOutMin, servoOutMax;
  double fanInMin, fanInMax, fanOutMin, fanOutMax;
  for (;;) {
    if((pidTuneP!= myPID.GetKp()) || (pidTuneI!= myPID.GetKi()) || (pidTuneD!= myPID.GetKd())){
      myPID.SetTunings(pidTuneP, pidTuneI, pidTuneD);
      pidTuneP = myPID.GetKp();
      pidTuneI = myPID.GetKi();
      pidTuneD = myPID.GetKd();
    }
    xSemaphoreTake( xFiltTempMutex, portMAX_DELAY );
    Input = filteredTemps[0];
    xSemaphoreGive( xFiltTempMutex );

    // Lid Open Detection
    if ((lidOpenTimeLeft > 0)){
      lidOpenTimeLeft = max((lidOpenTimeLeft - taskScanTime),0);
    }
    else if((lidOpenTimeLeft == 0) and lidOpen) lidOpen = false;
    else if((lidOpnPctDeviation != 0)){
      if((tempDeviation[0] * -1) > lidOpnPctDeviation){
        lidOpen = true;
        lidOpenTimeLeft = max(int(config["lidOpenTime"]) * 1000,taskScanTime);
      }
    }
    else lidOpenTimeLeft = -1; // negative value indicates an issue with lid open detection
    
    // Calculate Actual Mode
    if (filteredFaults[0] or (targetMode == MANUAL) or lidOpen){
      myPID.SetMode(MANUAL);
    }
    else if(targetMode == AUTOMATIC) myPID.SetMode(AUTOMATIC);
    myPID.Compute();

    double servoInMin = min(servoIn0,servoIn100);
    double servoInMax = max(servoIn0,servoIn100);
    double servoOut = constrain(Output,servoInMin,servoInMax);
    servoOut = map(servoOut,servoIn0,servoIn100,servoOut0,servoOut100);
    double servoOutMin = min(servoOut0,servoOut100);
    double servoOutMax = max(servoOut0,servoOut100);
    servoOut = constrain(servoOut,servoOutMin,servoOutMax);
    servo.write(servoOut);

    double fanInMin = min(fanIn0,fanIn100);
    double fanInMax = max(fanIn0,fanIn100);
    double fanOut = constrain(Output,fanInMin,fanInMax);
    fanOut = map(fanOut,fanIn0,fanIn100,fanOut0,fanOut100);
    double fanOutMin = min(fanOut0,fanOut100);
    double fanOutMax = max(fanOut0,fanOut100);
    fanOut = constrain(fanOut,fanOutMin,fanOutMax);
    fanDuty = map(fanOut,0,100,0,1023);
    if(Output < fanInMin)fanDuty = 0;
    fanDuty = constrain(fanDuty,0,1023);
    ledcWrite(fanChannel,fanDuty);
    
    vTaskDelayUntil(&xLastWakeTime,taskScanTime / portTICK_PERIOD_MS);
  }
}

void tone(uint8_t pin, unsigned int frequency, unsigned long duration, uint8_t channel)
{
    if (ledcRead(channel)) {
        log_e("Tone channel %d is already in use", ledcRead(channel));
        return;
    }
    ledcAttachPin(pin, channel);
    ledcWriteTone(channel, frequency);
    if (duration) {
        vTaskDelay(duration / portTICK_PERIOD_MS);
        noTone(pin, channel);
    }    
}

void noTone(uint8_t pin, uint8_t channel)
{
    ledcDetachPin(pin);
    ledcWrite(channel, 0);
}

void playMelody(int melody[8], int noteDurations[8]){
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration,buzzerChannel);
    vTaskDelay((noteDuration * .50) / portTICK_PERIOD_MS);
  }
  noTone(buzzerPin, buzzerChannel);
}

void soundBuzzer(void *pvParameters){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  bool alarmActive;
  const unsigned int alarmFreq = 10; // only sound the buzzer every 10 seconds (10 loops)
  unsigned int alarmWait; // incriments every loop until it reaches alarmFreq and the alarm melody plays
  // Startup Melody
    int startupMelody[]        = { NOTE_D4, NOTE_D4, NOTE_C4, NOTE_A3, NOTE_D4, NOTE_C4, NOTE_B3, NOTE_E4 };
    int startupNoteDurations[] = {       8,       8,       8,       8,       8,       8,       8,       4 };
  // Alarm
    int alarmMelody[]        = { NOTE_D4, NOTE_C4, NOTE_A3, NOTE_E4, NOTE_A3, NOTE_E4, NOTE_A3, NOTE_E4 };
    int alarmNoteDurations[] = {       8,       8,       8,       8,       8,       8,       8,       8 };
  Serial.println("[BUZZ] Play Startup Melody");
  playMelody(startupMelody, startupNoteDurations);
  for(;;){
    vTaskDelayUntil(&xLastWakeTime,1000 / portTICK_PERIOD_MS);
    alarmActive = false;
    for(int i=0; i<= numSensors; i++){
      if(probeTgtAlm[i] && (filteredTemps[i] > probeTarget[i])){
        if(!probeTgtAct[i]){
          Serial.print("[ALM] Alarm Detected for Probe ");
          Serial.println(i);
        }
        probeTgtAct[i] = true;
        if(!probeTgtAck[i]){
          alarmActive = true;
        }
      }
      else{
        probeTgtAct[i] = false;
        probeTgtAck[i] = false;
      }
    }
    if(alarmActive && (alarmWait >= alarmFreq)){
      Serial.println("[BUZZ] Play Alarm Melody");
      playMelody(alarmMelody, alarmNoteDurations);
      alarmWait = 0;
    }
    else if(alarmActive) alarmWait++;
    else alarmWait = 0;
    //if lidOpen
  }
}


void collectHistory(void *pvParameters){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  File history = SPIFFS.open("/history.json");
  xSemaphoreTake(xJsonDoc, portMAX_DELAY );
  doc.clear();
  int err;
  deserializeJson(doc, history);
    Serial.println("[HIST] Loading History From File");
    JsonArray timestamp = doc["timestamp"];
    JsonArray probe0 = doc["probe0"];
    JsonArray probe1 = doc["probe1"];
    JsonArray probe2 = doc["probe2"];
    JsonArray probe3 = doc["probe3"];
    JsonArray jOutput = doc["output"];
    JsonArray jSetpoint = doc["setpoint"];
    JsonArray millisArray = doc["millis"];
    for (JsonVariant value : timestamp) {
      historyTime[historyIndex] = timestamp[historyIndex];
      historyMillis[historyIndex] = millisArray[historyIndex];
      TemperatureHistory[0][historyIndex] = probe0[historyIndex];
      TemperatureHistory[1][historyIndex] = probe1[historyIndex];
      TemperatureHistory[2][historyIndex] = probe2[historyIndex];
      TemperatureHistory[3][historyIndex] = probe3[historyIndex];
      outputHistory[historyIndex] = jOutput[historyIndex];
      setpointHistory[historyIndex] = jSetpoint[historyIndex];
      historyIndex++;
      if(historyIndex >= historyValues) historyIndex = 0;
    }
  history.close();
  xSemaphoreGive( xJsonDoc );
  for(;;){
    vTaskDelayUntil(&xLastWakeTime,(historyCollectInterval * 1000) / portTICK_PERIOD_MS);
    xSemaphoreTake( xHistoryMutex, portMAX_DELAY );
    if(timeSet) historyTime[historyIndex] = timeClient.getEpochTime();
    else historyTime[historyIndex] = 0;
    historyMillis[historyIndex] = millis();
    //get history access first so we don't block data collection if another history task runs long
    xSemaphoreTake( xFiltTempMutex, portMAX_DELAY );
    for(int i = 0; i<4;i++){
      TemperatureHistory[i][historyIndex] = filteredTemps[i];
      faultHistory[i][historyIndex] = filteredFaults[i];
    }
    xSemaphoreGive( xFiltTempMutex );
    outputHistory[historyIndex] = Output;
    setpointHistory[historyIndex] = Setpoint;
    historyIndex++;
    if(historyIndex >= historyValues) historyIndex = 0;
    xSemaphoreGive( xHistoryMutex );
  }
}

String historyToJson(){
  xSemaphoreTake(xJsonDoc, portMAX_DELAY );
  doc.clear();
  String jsonString;
  JsonArray timestamp = doc.createNestedArray("timestamp");
  JsonArray probe0 = doc.createNestedArray("probe0");
  JsonArray probe1 = doc.createNestedArray("probe1");
  JsonArray probe2 = doc.createNestedArray("probe2");
  JsonArray probe3 = doc.createNestedArray("probe3");
  JsonArray jOutput = doc.createNestedArray("output");
  JsonArray jSetpoint = doc.createNestedArray("setpoint");
  JsonArray millisArray = doc.createNestedArray("millis");
  doc["history"] = true;
  xSemaphoreTake( xHistoryMutex, portMAX_DELAY );
  int curHistValue = historyIndex;
  if(curHistValue >= historyValues or historyMillis[curHistValue] == 0) curHistValue = 0;
  for(int i=0;i<historyValues;i++){
    if(!historyMillis[curHistValue]) break;
    millisArray.add(historyMillis[curHistValue]);
    probe0.add(TemperatureHistory[0][curHistValue]);
    probe1.add(TemperatureHistory[1][curHistValue]);
    probe2.add(TemperatureHistory[2][curHistValue]);
    probe3.add(TemperatureHistory[3][curHistValue]);
    timestamp.add(historyTime[curHistValue]);
    jOutput.add(outputHistory[curHistValue]);
    jSetpoint.add(setpointHistory[curHistValue]);
    curHistValue++;
    if(curHistValue >= historyValues) curHistValue = 0;
  }
  xSemaphoreGive( xHistoryMutex );
  serializeJson(doc, jsonString);
  xSemaphoreGive( xJsonDoc );
  return jsonString;
}

String configToJson(){
  String jsonString;
  xSemaphoreTake( xConfigDoc, portMAX_DELAY );
  config["setpoint"] = Setpoint;
  config["output"] = Output;
  config["targetMode"] = targetMode;
  config["probeTarget1"] = probeTarget[1];
  config["probeTarget2"] = probeTarget[2];
  config["probeTarget3"] = probeTarget[3];
  config["probeTgtAlm1"] = probeTgtAlm[1];
  config["probeTgtAlm2"] = probeTgtAlm[2];
  config["probeTgtAlm3"] = probeTgtAlm[3];
  config["filterTime0"] = filterTime[0];
  config["filterTime1"] = filterTime[1];
  config["filterTime2"] = filterTime[2];
  config["filterTime3"] = filterTime[3];
  config["pidTuneP"] = pidTuneP;
  config["pidTuneI"] = pidTuneI;
  config["pidTuneD"] = pidTuneD;
  config["servoIn0"] = servoIn0;
  config["servoIn100"] = servoIn100;
  config["servoOut0"] = servoOut0;
  config["servoOut100"] = servoOut100;
  config["fanIn0"] = fanIn0;
  config["fanIn100"] = fanIn100;
  config["fanOut0"] = fanOut0;
  config["fanOut100"] = fanOut100;
  config["lidOpenDetect"] = lidOpenDetect;
  config["lidOpnPctDeviation"] = lidOpnPctDeviation;
  serializeJson(config, jsonString);
  xSemaphoreGive( xConfigDoc );
  return jsonString;
}

void writeConfigToFlash(void *pvParameters){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    vTaskDelayUntil(&xLastWakeTime,20000 / portTICK_PERIOD_MS);
    if(configChange == true){
      Serial.println("[FILE] Write Config to file");
      File configFile = SPIFFS.open("/config.json", FILE_WRITE);
      configFile.print(configToJson());
      configFile.close();
      configChange = false;
    }
  }
}

void writeHistToFlash(void *pvParameters){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    vTaskDelayUntil(&xLastWakeTime,600000 / portTICK_PERIOD_MS);
    Serial.println("[FILE] Write History to file");
    File history = SPIFFS.open("/history.json", FILE_WRITE);
    history.print(historyToJson());
    history.close();
  }
}
