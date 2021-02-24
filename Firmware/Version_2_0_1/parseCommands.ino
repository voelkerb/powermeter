/****************************************************
 * A request happended, handle it
 ****************************************************/
void handleEvent(Stream * getter) {
  if (!getter->available()) return;
  getter->readStringUntil('\n').toCharArray(command,COMMAND_MAX_SIZE);
  // Be backwards compatible to "?" command all the time
  if (command[0] == '?') {
    getter->println(F("Info:Setup done"));
    return;
  }
  // This is just a keepalive msg
  if (command[0] == '!') {
    return;
  }
  #ifdef DEBUG_DEEP
  logger.log(INFO, command);
  #endif

  newGetter = getter;

  response = "";

  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();

  if (parseCommand()) {
    handleJSON();

    JsonObject object = docSend.as<JsonObject>();
    if (object.size()) {
      // NOTE: This flush causes socket broke pipe... WTF
      // getter->flush();
      response = "";
      serializeJson(docSend, response);
      response = LOG_PREFIX + response;
      getter->println(response.c_str());
    }
  }
  response = "";
  command[0] = '\0';
}

/****************************************************
 * Decode JSON command from string to json dictionary
 ****************************************************/
bool parseCommand() {
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docRcv, command);
  
  // Test if parsing succeeds.
  if (error) {
    // Remove all unallowed json characters to prevent error 
    uint32_t len = strlen(command);
    if (len > 30) len = 30;
    for (size_t i = 0; i < len; i++) {
      if (command[i] == '\r' || command[i] == '\n' || command[i] == '"' || command[i] == '}' || command[i] == '{') command[i] = '_';
    }
    logger.log(ERROR, "deserializeJson() failed: %.30s", &command[0]);
    return false;
  }
  //docSend.clear();
  return true;
}

/****************************************************
 * Handle a command in form of a JSON dict
 ****************************************************/
void handleJSON() {
  // All commands look like the following:
  // {"cmd":"commandName", "payload":{<possible data>}}
  // e.g. mdns

  const char* cmd = docRcv["cmd"];
  JsonObject root = docRcv.as<JsonObject>();
  if (cmd == nullptr) {
    docSend["msg"] = "Syntax error: \"{\"cmd\":<commandName>,\"[payload\":{<possible data>}]}";
    return;
  }

  /*********************** SAMPLING COMMAND ****************************/
  // e.g. {"cmd":"sample", "payload":{"type":"Serial", "rate":4000}}
  if(strcmp(cmd, CMD_SAMPLE) == 0) {
    if (state == SampleState::IDLE) {
      // For sampling we need type payload and rate payload
      const char* typeC = root["payload"]["type"];
      const char* measuresC = root["payload"]["measures"];
      int rate = docRcv["payload"]["rate"].as<int>();
      unsigned long ts = docRcv["payload"]["time"].as<unsigned long>();
      bool prefix = docRcv["payload"]["prefix"].as<bool>();
      JsonVariant prefixVariant = root["payload"]["prefix"];
      bool flowControl = docRcv["payload"]["flowCtr"].as<bool>();
      JsonVariant flowControlVariant = root["payload"]["flowCtr"];
      JsonVariant slotVariant = root["payload"]["slot"];

      docSend["error"] = true;
      if (typeC == nullptr or rate == 0) {
        response = "Not a valid \"sample\" command";
        if (typeC == nullptr) response += ", \"type\" missing";
        if (rate == 0) response += ", \"rate\" missing";
        docSend["msg"] = response;
        return;
      }
      // Its a valid command, begin from standard config.
      standardConfig();
      // Check rate
      if (rate > 8000 || rate <= 0) {
        response = "SamplingRate could not be set to ";
        response += rate;
        docSend["msg"] = response;
        return;
      }

      streamConfig.measurementBytes = 8;

      if (measuresC == nullptr or strcmp(measuresC, MEASURE_VI) == 0) {
        streamConfig.measures = Measures::VI;
      } else if (strcmp(measuresC, MEASURE_PQ) == 0) {
        streamConfig.measures = Measures::PQ;
      } else if (strcmp(measuresC, MEASURE_VI_RMS) == 0) {
        streamConfig.measures = Measures::VI_RMS;
      } else if (strcmp(measuresC, MEASURE_VIPQ) == 0) {
        streamConfig.measures = Measures::VIPQ;
        streamConfig.measurementBytes = 16;
      } else {
        response = "Unsupported measures";
        response += measuresC;
        docSend["msg"] = response;
        return;
      }

      streamConfig.numValues = streamConfig.measurementBytes/sizeof(float);

     
      outputWriter = &writeChunks;
      // e.g. {"cmd":"sample", "payload":{"type":"Serial", "rate":4000}}
      if (strcmp(typeC, "Serial") == 0) {
        streamConfig.stream = StreamType::USB;
        sendStream = (Stream*)&Serial; 
      // e.g. {"cmd":"sample", "payload":{"type":"MQTT", "rate":4000}}
      } else if (strcmp(typeC, "TCP") == 0) {
        sendClient = (WiFiClient*)newGetter;
        sendStream = sendClient; 
        streamConfig.stream = StreamType::TCP;
        streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
        streamConfig.ip = sendClient->remoteIP();
      // e.g. {"cmd":"sample", "payload":{"type":"UDP", "rate":4000}}
      } else if (strcmp(typeC, "UDP") == 0) {
        streamConfig.stream = StreamType::UDP;
        int port = docRcv["payload"]["port"].as<int>();
        if (port > 80000 || port <= 0) {
          streamConfig.port = STANDARD_UDP_PORT;
          response = "Unsupported UDP port";
          response += port;
          docSend["msg"] = response;
          return;
        } else {
          streamConfig.port = port;
        }
        outputWriter = &writeChunksUDP;
        sendClient = (WiFiClient*)newGetter;
        sendStream = sendClient; 
        docSend["port"] = streamConfig.port;
        streamConfig.ip = sendClient->remoteIP();
      } else {
        response = F("Unsupported sampling type: ");
        response += typeC;
        docSend["msg"] = response;
        return;
      }

      // Set global sampling variable
      streamConfig.samplingRate = rate;

      // TIMER_MAX = CLOCK_SPEED/PRESCALER/samplingRate;

      calcChunkSize();      
      docSend["chunksize"] = streamConfig.chunkSize;
      docSend["samplingrate"] = streamConfig.samplingRate;
      docSend["conn_type"] = typeC;
      docSend["measurements"] = streamConfig.numValues;
      docSend["cmd"] = CMD_SAMPLE;


      next_state = SampleState::SAMPLE;

      if (ts != 0) {
        response += F("Should sample at: ");
        response += myTime.timeStr(ts, 0);
        // Wait random time to start ntp update s.t. not all devices start ntp request at the same time
        int waitMillis = random(200);
        delay(waitMillis);
        // Update ntp time actively wait for finish
        myTime.updateNTPTime(true);
        uint32_t delta = ts - myTime.timestamp().seconds;
        uint32_t nowMs = millis();
        delta *= 1000;
        delta -= myTime.timestamp().milliSeconds;
        if (delta > 20000 or delta < 500) {
          response += F("//nCannot start sampling in: "); response += delta; response += F("ms");
          streamConfig.countdown = 0;
        } else {
          response += F("//nStart sampling in: "); response += delta; response += F("ms");
          streamConfig.countdown = nowMs + delta;
          docSend["error"] = false;
          Timestamp timestamp;
          timestamp.seconds = ts;
          timestamp.milliSeconds = 0;
          docSend["startTs"] = myTime.timestampStr(timestamp);
        }
        docSend["msg"] = String(response);
        return;
      }
      docSend["error"] = false;
      state = next_state;
      // UDP packets are not allowed to exceed 1500 bytes, so keep size reasonable
      startSampling(false);
      docSend["startTs"] = myTime.timestampStr(streamConfig.startTs);
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** SWITCHING COMMAND ****************************/
  // {"cmd":"switch", "payload":{"value":"true"}}
  else if (strcmp(cmd, CMD_SWITCH) == 0) {
    // For switching we need value payload
    docSend["error"] = true;
    JsonVariant payloadValue = root["payload"]["value"];
    if (payloadValue.isNull()) {
      docSend["error"] = false;
      docSend["msg"] = F("Info:Not a valid \"switch\" command");
      return;
    }
    bool value = docRcv["payload"]["value"].as<bool>();
    docSend["msg"]["switch"] = value;
    response = F("Switching: ");
    response += value ? F("On") : F("Off");
    docSend["msg"] = response;
    docSend["error"] = false;
    relay.set(value);
  }

  /*********************** STOP COMMAND ****************************/
  // e.g. {"cmd":"stop"}
  else if (strcmp(cmd, CMD_STOP) == 0) {
    if (state == SampleState::IDLE) return;
    // State is reset in stopSampling
    stopSampling();
    // Write remaining chunks with tailr
    if (outputWriter != NULL) outputWriter(true);
    docSend["msg"] = F("Received stop command");
    docSend["sample_duration"] = samplingDuration;
    docSend["samples"] = totalSamples;
    docSend["sent_samples"] = sentSamples;
    docSend["start_ts"] = myTime.timestampStr(streamConfig.startTs);
    docSend["stop_ts"] = myTime.timestampStr(streamConfig.stopTs);
    docSend["ip"] = WiFi.localIP().toString();
    docSend["avg_rate"] = totalSamples/(samplingDuration/1000.0);
    docSend["cmd"] = CMD_STOP;
  }

  /*********************** LOG LEVEL COMMAND ****************************/
  else if (strcmp(cmd, CMD_LOG_LEVEL) == 0) {
    docSend["error"] = true;
    const char* level = root["level"];
    LogType newLogType = LogType::DEBUG;
    if (level == nullptr) {
      docSend["msg"] = "Not a valid Log level cmd, level missing";
      return;
    }
    if (strcmp(level, LOG_LEVEL_ALL) == 0) {
      newLogType = LogType::ALL;
    } else if (strcmp(level, LOG_LEVEL_DEBUG) == 0) {
      newLogType = LogType::DEBUG;
    } else if (strcmp(level, LOG_LEVEL_INFO) == 0) {
      newLogType = LogType::INFO;
    } else if (strcmp(level, LOG_LEVEL_WARNING) == 0) {
      newLogType = LogType::WARNING;
    } else if (strcmp(level, LOG_LEVEL_ERROR) == 0) {
      newLogType = LogType::ERROR;
    } else {
      response = "Unknown log level: ";
      response += level;
      docSend["msg"] = response;
      return;
    }
    int found = -2;
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (newGetter == (Stream*)&client[i]) {
        found = i;
        streamLog[i]->_type = newLogType;
        streamLog[i]->setLogType(newLogType);
      }
    }
    if (found <= -2) {
      docSend["msg"] = "getter not related to logger";
      return;
    }
    response = "Log Level set to: ";
    response += level;
    docSend["msg"] = response;
    docSend["error"] = false;
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":"restart"}
  else if (strcmp(cmd, CMD_RESTART) == 0) {
    ESP.restart();
  }

  /*********************** factoryReset COMMAND ****************************/
  // e.g. {"cmd":"factoryReset"}
  else if (strcmp(cmd, CMD_FACTORY_RESET) == 0) {
    config.makeDefault();
    config.store();
    ESP.restart();
  }

  /*********************** basicReset COMMAND ****************************/
  // e.g. {"cmd":"basicReset"}
  else if (strcmp(cmd, CMD_BASIC_RESET) == 0) {
    config.makeDefault(false);// Do not reset name
    config.store();
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  // e.g. {"cmd":"info"}
  else if (strcmp(cmd, CMD_INFO) == 0) {
    sendDeviceInfo(newGetter);
    // It is already sent, prevent sending again
    JsonObject obj = docSend.to<JsonObject>();
    obj.clear();
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":"mdns", "payload":{"name":"newName"}}
  else if (strcmp(cmd, CMD_MDNS) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newName = docRcv["payload"]["name"];
      if (newName == nullptr) {
        docSend["msg"] = F("MDNS name required in payload with key name");
        return;
      }
      if (strlen(newName) < MAX_NAME_LEN) {
        config.setName((char * )newName);
      } else {
        response = F("MDNS name too long, only string of size ");
        response += MAX_NAME_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * name = config.name;
      response = F("Set MDNS name to: ");
      response += name;
      //docSend["msg"] = snprintf( %s", name);
      docSend["msg"] = response;
      docSend["mdns_name"] = name;
      docSend["error"] = false;
      initMDNS();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** Stream Server COMMAND ****************************/
  // e.g. {"cmd":"streamServer", "payload":{"server":"<ServerAddress>"}}
  else if (strcmp(cmd, CMD_STREAM_SERVER) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newServer = docRcv["payload"]["server"];
      if (newServer == nullptr) {
        docSend["msg"] = F("StreamServer address required in payload with key server");
        return;
      }
      if (strlen(newServer) < MAX_IP_LEN) {
        config.setStreamServerAddress((char * )newServer);
      } else {
        response = F("StreamServer address too long, only string of size ");
        response += MAX_IP_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * address = config.streamServer;
      response = F("Set StreamServer address to: ");
      response += address;
      //docSend["msg"] = snprintf( %s", name);
      docSend["msg"] = response;
      docSend["stream_server"] = address;
      docSend["error"] = false;
      initStreamServer();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** Time Server COMMAND ****************************/
  // e.g. {"cmd":"streamServer", "payload":{"server":"<ServerAddress>"}}
  else if (strcmp(cmd, CMD_TIME_SERVER) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newServer = docRcv["payload"]["server"];
      if (newServer == nullptr) {
        docSend["msg"] = F("StreamServer address required in payload with key server");
        return;
      }
      if (strlen(newServer) < MAX_DNS_LEN) {
        config.setTimeServerAddress((char * )newServer);
      } else {
        response = F("TimeServer address too long, only string of size ");
        response += MAX_DNS_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * address = config.timeServer;
      response = F("Set TimeServer address to: ");
      response += address;
      //docSend["msg"] = snprintf( %s", name);
      docSend["msg"] = response;
      docSend["time_server"] = address;
      docSend["error"] = false;
      myTime.updateNTPTime(true);
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** ADD WIFI COMMAND ****************************/
  // e.g. {"cmd":"addWifi", "payload":{"ssid":"ssidName","pwd":"pwdName"}}
  else if (strcmp(cmd, CMD_ADD_WIFI) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newSSID = docRcv["payload"]["ssid"];
      const char* newPWD = docRcv["payload"]["pwd"];
      if (newSSID == nullptr or newPWD == nullptr) {
        docSend["msg"] = F("WiFi SSID and PWD required, for open networks, fill empty pwd");
        return;
      }
      int success = 0;
      if (strlen(newSSID) < MAX_SSID_LEN and strlen(newPWD) < MAX_PWD_LEN) {
        success = config.addWiFi((char * )newSSID, (char * )newPWD);
      } else {
        response = F("SSID or PWD too long, max: ");
        response += MAX_SSID_LEN;
        response += F(", ");
        response += MAX_PWD_LEN;
        docSend["msg"] = response;
        return;
      }
      if (success == 1)  {
        char * name = config.wifiSSIDs[config.numAPs-1];
        char * pwd = config.wifiPWDs[config.numAPs-1];
        response = F("New Ap, SSID: ");
        response += name;
        response += F(", PW: ");
        response += pwd;
        //docSend["msg"] = snprintf( %s", name);
        docSend["ssid"] = name;
        docSend["pwd"] = pwd;
        docSend["error"] = false;
      } else if (success == -1) {
        response = F("Wifi AP ");
        response += newSSID;
        response += F(" already in list");
      } else {
        response = F("MAX # APs reached, need to delete first");
      }

      docSend["msg"] = response;
      String ssids = "[";
      for (size_t i = 0; i < config.numAPs; i++) {
        ssids += config.wifiSSIDs[i];
        ssids += ", ";
      }
      ssids += "]";
      docSend["ssids"] = ssids;
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** DEl WIFI COMMAND ****************************/
  // e.g. {"cmd":"delWifi", "payload":{"ssid":"ssidName"}}
  else if (strcmp(cmd, CMD_REMOVE_WIFI) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newSSID = docRcv["payload"]["ssid"];
      if (newSSID == nullptr) {
        docSend["msg"] = F("Required SSID to remove");
        return;
      }
      bool success = false;
      if (strlen(newSSID) < MAX_SSID_LEN) {
        success = config.removeWiFi((char * )newSSID);
      } else {
        response = F("SSID too long, max: ");
        response += MAX_SSID_LEN;
        docSend["msg"] = response;
        return;
      }
      if (success)  {
        response = F("Removed SSID: ");
        response += newSSID;
        docSend["error"] = false;
      } else {
        response = F("SSID ");
        response += newSSID;
        response += F(" not found");
      }
      docSend["msg"] = response;
      String ssids = "[";
      for (size_t i = 0; i < config.numAPs; i++) {
        ssids += config.wifiSSIDs[i];
        ssids += ", ";
      }
      ssids += "]";
      docSend["ssids"] = ssids;
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** NTP COMMAND ****************************/
  // e.g. {"cmd":"ntp"}
  else if (strcmp(cmd, CMD_NTP) == 0) {
    if (myTime.updateNTPTime(true)) {
      docSend["msg"] = "Time synced";
      docSend["error"] = false;
    } else {
      docSend["msg"] = "Error syncing time";
      docSend["error"] = true;
    }
    char * timeStr = myTime.timeStr();
    docSend["current_time"] = timeStr;
  }

}

/****************************************************
 * Indicate that someone else is currently sampling 
 * and we cannot perform the requested action
 * This function just build the string msg for it
 ****************************************************/
void setBusyResponse() {
  if (sendClient != NULL) {
    response = "Device with IP: ";
    response += sendClient->localIP().toString();
    response += " currently sampling"; 
  } else {
    response = "Currently sampling";
  }
}
