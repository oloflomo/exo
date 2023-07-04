void WMSetup(){
  //wifimanager
  WiFi.mode(WIFI_STA);
  Serial.setDebugOutput(true);  
  Serial.println("\n Wifi Starting");

  wm.setConfigPortalBlocking(false);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  wm.setClass("invert");

  wm.setConfigPortalTimeout(120);
  
}


void WMStart(){  //initialize wifi manager
    if(wm.autoConnect("AutoConnectAP")){
      Serial.println("connected...yeey :)");
      ConnectedWiFi();
    }
    else {
      Serial.println("Configportal running");
      
      if (!wm.startConfigPortal("Exo","1234567890"))
      {
        Serial.println("failed to connect or hit timeout");
        led.flash(RGBLed::RED,50);  //LED failed to connect
        fun[2] = 2; //run service
        // ESP.restart();
      } 
      else
      {
        //if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
        ConnectedWiFi();
      }
    }
    getLocalTime(&timeinfo);
    
}


void ConnectedWiFi(){     //functions called when wifi is connected
        led.setColor(RGBLed::CYAN); //LED synchornizing time + Connected wifi
        while(!getLocalTime(&timeinfo)){};              //synchronize time

        led.setColor(RGBLed::BLUE); //LED Sending file to server and formating SD card

        CSVupdateserver();  //send csv data to server
        delay(1000);
        File root = SD.open("/");
        delay(1000);
        rm(root, rootpath); //delete all files from sd card
}