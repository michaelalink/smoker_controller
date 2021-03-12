//////////////////////////////////////////////////////////////////////////////////////
// WiFi Keep Alive Task
//////////////////////////////////////////////////////////////////////////////////////
void keepWiFiAlive(void * parameter) {
  IPAddress myIP;
  WiFi.begin();
  for (;;) {
    while (((WiFi.status() == WL_CONNECTED) || wifiApMode) && !wifiConfigChange) {
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    wifiConfigChange = false;
    Serial.println("[WIFI] Connecting");
    WiFi.mode(WIFI_AP_STA);
    xSemaphoreTake( xConfigDoc, portMAX_DELAY );
    String clientSSID = config["clientSSID"];
    String clientPWD = config["clientPWD"];
    if(config["wifiClient"] && (clientSSID.length() >1) && (clientPWD.length()>=8)){
      WiFi.begin(clientSSID.c_str(), clientPWD.c_str());
      Serial.println("[WIFI] Starting Client from Config");
      Serial.print(  "  SSID: ");
      Serial.println(clientSSID);
      Serial.print(  "  Password: ");
      Serial.println(clientPWD);
    }
    String apSSID = config["apSSID"];
    String apPWD = config["apPWD"];
    if(config["wifiAP"] && (apSSID.length() >1) && (apPWD.length()>=8)){
      WiFi.softAP(apSSID.c_str(), apPWD.c_str());
      myIP = WiFi.softAPIP();
      wifiApMode = true;
      Serial.println("[WIFI] Starting AP from Config");
      Serial.print(  "  SSID: ");
      Serial.println(apSSID);
      Serial.print(  "  Password: ");
      Serial.println(apPWD);
      Serial.print(  "  IP address: ");
      Serial.println(myIP);
    }
    xSemaphoreGive( xConfigDoc );

    unsigned long startAttemptTime = millis();

    // Keep looping while we're not connected and haven't reached the timeout
    while ((!wifiApMode || WiFi.status() != WL_CONNECTED) && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if ((!wifiApMode && WiFi.status() != WL_CONNECTED)) {
      Serial.print("[WIFI] Connection FAILED ");
      Serial.println(WiFi.status());
      Serial.println("[WIFI] Starting AP for Default Setup");
      WiFi.softAP(defaultAP, defaultApPassword);
      myIP = WiFi.softAPIP();
      wifiApMode = true;
      Serial.println("[WIFI] Starting AP from Config");
      Serial.print(  "  SSID: ");
      Serial.println(defaultAP);
      Serial.print(  "  Password: ");
      Serial.println(defaultApPassword);
      Serial.print(  "  IP address: ");
      Serial.println(myIP);
    }
    else{
      Serial.print("[WIFI] Connected: ");
      Serial.println(WiFi.localIP());
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
// Wait for WiFi Task
//////////////////////////////////////////////////////////////////////////////////////
void waitForWifi(void * parameter) {
  //Wait for Wifi to Come Up
  while ((WiFi.status() != WL_CONNECTED) && !wifiApMode) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  Serial.println("[WTWIFI] Network is up, Starting tasks that require network");

  //Setup MDNS responder
  if (!MDNS.begin(hostName)) Serial.println("[MNDS] Error Starting MDNS");
  else Serial.println("[MDNS] Started");
  MDNS.addService("http", "tcp", 80);

  //Setup NTP
  Serial.println("[NTP] Starting");
  timeClient.begin();
  //force and NTP Update
  if (!timeClient.forceUpdate()) Serial.println("[NTP] Update Failed");
  else {
    Serial.print("[NTP] Time Set: ");
    Serial.println(timeClient.getFormattedTime());
    timeSet = true;
  }
  /////////////////////////////////////////////////////////
  //Create a task to update NTP
  xTaskCreate(
    ntpUpdate,
    "ntpUpdate",  // Task name
    2048,             // Stack size (bytes)
    NULL,             // Parameter
    2,                // Task priority
    &xNtpUpdate             // Task handle
  );

  setupServerPages();

  //Start the Webserver
  Serial.println("[WEB] Starting");
  server.begin();

  //This task is deleted now that post WiFi Connection tasks are done
  vTaskDelete(NULL);
}


//////////////////////////////////////////////////////////////////////////////////////
// Update NTP Task
//////////////////////////////////////////////////////////////////////////////////////
void ntpUpdate(void * parameter) {
  while (!timeSet) {
    timeSet = timeClient.update(); //returns true on successful update
    if (timeSet) {
      Serial.print("[NTP] Time Set: ");
      Serial.println(timeClient.getFormattedTime());
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  for (;;) {
    timeClient.update();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
