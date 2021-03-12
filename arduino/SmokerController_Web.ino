void setupServerPages(){
  // setup websockets on /ws
  initWebSocket();
  // redirect / to /index.html
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->redirect("/index.html");
  });
  // serve the index.html file from spiffs
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  
  // serve static Javascript files from the /js folder on SPIFFS
  server.serveStatic("/js/", SPIFFS, "/js/").setCacheControl("max-age=3600");
  
  // the rest of these are for testing
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain",historyToJson().c_str());
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain",configToJson().c_str());
  });
  server.on("/historyFile", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/history.json");
  });
  server.on("/configFile", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/config.json");
  });
}

void sendWebEvents(void *pvParameters){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
    vTaskDelayUntil(&xLastWakeTime,1000 / portTICK_PERIOD_MS);

    notifyClients();
    ws.cleanupClients();
  }
}

void notifyClients() {
  String clientUpdate;
  curData.clear();
  curData["update"] = true;
  curData["timestamp"] = timeClient.getEpochTime();
  curData["millis"] = millis();
  curData["setpoint"] = Setpoint;
  curData["output"] = Output;
  curData["mode"] = myPID.GetMode();
  curData["targetMode"] = targetMode;
  curData["lidOpen"] = lidOpen;
  curData["lidOpenTimeLeft"] = lidOpenTimeLeft;
  for(int i = 0; i<numSensors; i++){
    curData["probe"+String(i)] = filteredTemps[i];
    curData["probe"+String(i)+"Deviation"] = tempDeviation[i];
    curData["probe"+String(i)+"Fault"] = (filteredFaults[i] or (filteredCJs[i] == filteredTemps[i]));
    curData["probe"+String(i)+"FaultCode"] = filteredFaults[i];
    curData["probe"+String(i)+"Desc"] = config["probe"+String(i)+"Desc"];
    curData["probe"+String(i)+"Target"] = probeTarget[i];
    curData["probeTgtAlm"+String(i)] = probeTgtAlm[i];
    curData["probeTgtAct"+String(i)] = probeTgtAct[i];
    curData["probeTgtAck"+String(i)] = probeTgtAck[i];
  }
  curData["pitTemp"] = filteredTemps[0];
  curData["pitTempFault"] = (filteredFaults[0] or (filteredCJs[0] == filteredTemps[0]));
  curData["pitTempFaultCode"] = filteredFaults[0];
  curData["filterTime0"] = filterTime[0];
  curData["inputVolts"] = InputVolts;
  serializeJson(curData, clientUpdate);
  ws.textAll(clientUpdate);
}

void sendConfig(){
  Serial.print("[WSRECV] Sending Smoker Settings ");
  ws.textAll(configToJson());
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  Serial.print("[WSRECV] Message Recieved: ");
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    Serial.println((char*)data);
    data[len] = 0;
    StaticJsonDocument<500> recievedJson;                     //Memory pool
    deserializeJson(recievedJson, (char*)data);
    if(recievedJson.containsKey("wifi")){
      if(recievedJson.containsKey("wifiClient")){
        configChange = true;
        wifiConfigChange = true;
        xSemaphoreTake( xConfigDoc, portMAX_DELAY );
        config["config"] = true;
        config["wifiClient"] = recievedJson["wifiClient"];
        config["clientSSID"] = recievedJson["clientSSID"];
        config["clientPWD"] = recievedJson["clientPWD"];
        config["wifiAP"] = recievedJson["wifiAP"];
        config["apSSID"] = recievedJson["apSSID"];
        config["apPWD"] = recievedJson["apPWD"];
        xSemaphoreGive( xConfigDoc );
        vTaskDelay(50 / portTICK_PERIOD_MS);
      }
      sendConfig();
    }
    else if(recievedJson.containsKey("servoIn0")){
      servoIn0 = recievedJson["servoIn0"];
      servoIn100 = recievedJson["servoIn100"];
      servoOut0 = recievedJson["servoOut0"];
      servoOut100 = recievedJson["servoOut100"];
      xSemaphoreTake( xConfigDoc, portMAX_DELAY );
      config["servoIn0"] = servoIn0;
      config["servoIn100"] = servoIn100;
      config["servoOut0"] = servoOut0;
      config["servoOut100"] = servoOut100;
      xSemaphoreGive( xConfigDoc );
      configChange = true;
      vTaskDelay(50 / portTICK_PERIOD_MS);
      sendConfig();
    }
    else if(recievedJson.containsKey("fanIn0")){
      fanIn0 = recievedJson["fanIn0"];
      fanIn100 = recievedJson["fanIn100"];
      fanOut0 = recievedJson["fanOut0"];
      fanOut100 = recievedJson["fanOut100"];
      xSemaphoreTake( xConfigDoc, portMAX_DELAY );
      config["fanIn0"] = fanIn0;
      config["fanIn100"] = fanIn100;
      config["fanOut0"] = fanOut0;
      config["fanOut100"] = fanOut100;
      xSemaphoreGive( xConfigDoc );
      configChange = true;
      vTaskDelay(50 / portTICK_PERIOD_MS);
      sendConfig();
    }
    else if(recievedJson.containsKey("history")){
      if(recievedJson["clearHistory"] == true){
        xSemaphoreTake(xJsonDoc, portMAX_DELAY );
        xSemaphoreTake( xHistoryMutex, portMAX_DELAY );
        SPIFFS.remove("/history.json");
        // millis array is used to determine if a history value has been recorded in that location
        memset(historyMillis, 0, sizeof(historyMillis));
        historyIndex = 0;
        xSemaphoreGive( xHistoryMutex );
        xSemaphoreGive( xJsonDoc );
      }
    }
    else{
      configChange = true;
      if(recievedJson["targetMode"] == "AUTOMATIC") targetMode = AUTOMATIC;
      else if(recievedJson["targetMode"] == "MANUAL") targetMode = MANUAL;
      if(recievedJson.containsKey("setpoint")) Setpoint = recievedJson["setpoint"];
      if(recievedJson.containsKey("output")) Output = recievedJson["output"];
      xSemaphoreTake( xConfigDoc, portMAX_DELAY );
      if(recievedJson.containsKey("probe1Desc")) config["probe1Desc"] = recievedJson["probe1Desc"].as<String>();
      if(recievedJson.containsKey("probe2Desc")) config["probe2Desc"] = recievedJson["probe2Desc"].as<String>();
      if(recievedJson.containsKey("probe3Desc")) config["probe3Desc"] = recievedJson["probe3Desc"].as<String>();
      xSemaphoreGive( xConfigDoc );
      if(recievedJson.containsKey("probe1Target")) probeTarget[1] = recievedJson["probe1Target"];
      if(recievedJson.containsKey("probe2Target")) probeTarget[2] = recievedJson["probe2Target"];
      if(recievedJson.containsKey("probe3Target")) probeTarget[3] = recievedJson["probe3Target"];
      if(recievedJson.containsKey("probeTgtAlm1")) probeTgtAlm[1] = recievedJson["probeTgtAlm1"];
      if(recievedJson.containsKey("probeTgtAlm2")) probeTgtAlm[2] = recievedJson["probeTgtAlm2"];
      if(recievedJson.containsKey("probeTgtAlm3")) probeTgtAlm[3] = recievedJson["probeTgtAlm3"];
      if(recievedJson.containsKey("probeTgtAck1")) probeTgtAck[1] = recievedJson["probeTgtAck1"];
      if(recievedJson.containsKey("probeTgtAck2")) probeTgtAck[2] = recievedJson["probeTgtAck2"];
      if(recievedJson.containsKey("probeTgtAck3")) probeTgtAck[3] = recievedJson["probeTgtAck3"];
      if(recievedJson.containsKey("filterTime0")) filterTime[0] = recievedJson["filterTime0"];
      if(recievedJson.containsKey("filterTime1")) filterTime[1] = recievedJson["filterTime1"];
      if(recievedJson.containsKey("filterTime2")) filterTime[2] = recievedJson["filterTime2"];
      if(recievedJson.containsKey("filterTime3")) filterTime[3] = recievedJson["filterTime3"];
      if(recievedJson.containsKey("pidTuneP")) pidTuneP = recievedJson["pidTuneP"];
      if(recievedJson.containsKey("pidTuneI")) pidTuneI = recievedJson["pidTuneI"];
      if(recievedJson.containsKey("pidTuneD")) pidTuneD = recievedJson["pidTuneD"];
      if(recievedJson.containsKey("lidOpenDetect")) lidOpenDetect = recievedJson["lidOpenDetect"];
      vTaskDelay(50 / portTICK_PERIOD_MS);
      sendConfig();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      ws.textAll(historyToJson());
      sendConfig();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
