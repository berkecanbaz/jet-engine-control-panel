#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// ESP32-S3 Donanımsal Serial2 Nesnesi
HardwareSerial mySerial(2); 
DFRobotDFPlayerMini myDFPlayer;

// J4 Konnektörü Pin Tanımlamaları
#define FPS_RX 44 // S3 RX (IO44) -> DFPlayer TX
#define FPS_TX 43 // S3 TX (IO43) -> DFPlayer RX

void printDetail(uint8_t type, int value);

void setup() {
  Serial.begin(115200);
  
  // S3 UART2 Başlatma: Baudrate, Protokol, RX Pin, TX Pin
  mySerial.begin(9600, SERIAL_8N1, FPS_RX, FPS_TX);
  
  delay(2000); 

  Serial.println(F("ESP32-S3 -> DFPlayer Mini Başlatılıyor..."));

  bool isConnected = false;
  for (int i = 0; i < 5; i++) {
    if (myDFPlayer.begin(mySerial)) {
      isConnected = true;
      break;
    }
    Serial.println(F("Bağlantı deneniyor..."));
    delay(1000);
  }

  if (!isConnected) {
    Serial.println(F("DFPlayer bağlantısı başarısız!"));
    while (true);
  }

  Serial.println(F("DFPlayer Mini başarıyla bağlandı!"));
  
  // Sesi maksimuma getir (0 - 30 arası)
  myDFPlayer.volume(30); 
  
  // 0001.mp3 dosyasını oynat (Tam olarak çalacak)
  myDFPlayer.play(1); 
}

void loop() {
  // 10 saniyelik geçiş kodunu kaldırdık, şarkı bitene kadar çalacak.

  // Modülden gelen bildirimleri (örneğin "Oynatma bitti") dinle
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read());
  }
}

// Durum geri bildirimlerini yazdıran yardımcı fonksiyon
void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Zaman aşımı (Time Out)!"));
      break;
    case WrongStack:
      Serial.println(F("Hatalı veri paketi."));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("SD Kart takıldı."));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("SD Kart çıkarıldı."));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Oynatma bitti. Parça No: "));
      Serial.println(value);
      // İstersen şarkı bittiğinde otomatik tekrar başlatmak için:
      // myDFPlayer.play(1);
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayer Hatası: "));
      switch (value) {
        case Busy:
          Serial.println(F("Kart meşgul"));
          break;
        case Sleeping:
          Serial.println(F("Uyku modunda"));
          break;
        case SerialWrongStack:
          Serial.println(F("Seri haberleşme hatası"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Checksum eşleşmedi"));
          break;
        case FileIndexOut:
          Serial.println(F("Dosya indeksi sınırların dışında"));
          break;
        case FileMismatch:
          Serial.println(F("Dosya bulunamadı"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}