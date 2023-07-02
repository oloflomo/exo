struct tm timeinfo;       //current time

void SDSetup(){
    while(!SD.begin()){
        Serial.println("Card Mount Failed");
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

        Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);  //configure time and servers
}

void CSVWrite(float csvarray[], int csvarraylenght){
  getLocalTime(&timeinfo);
  char filename[40];
  char filedata[400];
  sprintf(filename, "/%04d-%02d-%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

  sprintf(filedata, "%d,%d,%d",timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  for(int i = 0; i < csvarraylenght; i++){
      sprintf(filedata, "%s,%f", filedata, csvarray[i]);
   }
  sprintf(filedata, "%s\n", filedata);
  Serial.print("File name:");
  Serial.println(filename);
  Serial.print("CSV data");
  Serial.println(filedata);
  appendFile(SD, filename, filedata);
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        led.flash(RGBLed::MAGENTA,100);  //LED saving data failed
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
        led.flash(RGBLed::MAGENTA,100);  //LED saving data failed
        delay(1000);
    }
    file.close();
}


void CSVupdateserver() {
  ftp.OpenConnection();  //start frp connection
  ftp.InitFile("Type A");
  ftp.ChangeWorkDir("/pliki");  //move from ftp root to pliki folder

  File root = SD.open("/");  //open sd card root
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();  //first folder is useless system volume information
  while (file) {
    Serial.println(file.name());
    if (!file.isDirectory()) {  //open every folder in sd card
      Serial.print(file.name());
      readAndSendBigBinFile(SD, file.name(), ftp);
    }
    file = root.openNextFile();
  }
  
ftp.CloseConnection();
}



// ReadFile Example from ESP32 SD_MMC Library within Core\Libraries
// Changed to also write the output to an FTP Stream
void readAndSendBigBinFile(fs::FS &fs, const char *path, ESP32_FTPClient ftpClient) {
  ftpClient.InitFile("Type I");
  ftpClient.NewFile(path);

  String fullPath = "/";
  fullPath.concat(path);
  Serial.printf("Reading file: %s\n", fullPath);

  File file = fs.open(fullPath);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");

  while (file.available()) {
    // Create and fill a buffer
    unsigned char buf[1024];
    int readVal = file.read(buf, sizeof(buf));
    ftpClient.WriteData(buf, sizeof(buf));
  }
  ftpClient.CloseFile();
  file.close();
}


void rm(File dir, String tempPath) {
  while (true) {
    File entry = dir.openNextFile();
    String localPath;

    Serial.println("");
    if (entry) {
      if (entry.isDirectory()) {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length());
        rm(entry, folderBuf);


        if (SD.rmdir(folderBuf)) {
          Serial.print("Deleted folder ");
          Serial.println(folderBuf);
        } else {
          Serial.print("Unable to delete folder ");
          Serial.println(folderBuf);
        }
      } else {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length());

        if (SD.remove(charBuf)) {
          Serial.print("Deleted ");
          Serial.println(localPath);
        } else {
          Serial.print("Failed to delete ");
          Serial.println(localPath);
        }
      }
    } else {
      // break out of recursion
      break;
    }
  }
}