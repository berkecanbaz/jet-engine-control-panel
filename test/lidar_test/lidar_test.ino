#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

// --- PIN TANIMLARI (Senin donanımına göre) ---
#define CUSTOM_SDA 1
#define CUSTOM_SCL 2

// --- NESNELER ---
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Lidar Veri Okuma Baslatiliyor...");

  // I2C Baslatma
  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
  Wire.setClock(400000); // Lidar için 400kHz daha stabil olabilir

  if (myImager.begin() == false) {
    Serial.println("Lidar sensoru bulunamadi! Baglantilari kontrol et.");
    while (1);
  }

  myImager.setResolution(8 * 8);      // 8x8 cozunurluk
  myImager.setRangingFrequency(15);   // 15 Hz guncelleme hizi
  myImager.startRanging();
}

void loop() {
  // Veri hazir mi kontrol et
  if (myImager.isDataReady()) {
    if (myImager.getRangingData(&measurementData)) {
      
      Serial.println("--- 8x8 Mesafe Matrisi (mm) ---");
      
      for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
          // 8x8 diziliminde her hucrenin indexi
          int index = r * 8 + c; 
          
          int mesafe = measurementData.distance_mm[index];
          uint8_t durum = measurementData.target_status[index];

          // Durum kodu 5, 6 veya 9 ise veri gecerlidir
          if (durum == 5 || durum == 6 || durum == 9) {
            Serial.print(mesafe);
          } else {
            Serial.print("X"); // Gecersiz veya cok uzak veri
          }
          
          Serial.print("\t"); // Sutunlar arasi bosluk
        }
        Serial.println(); // Alt satira gec
      }
      Serial.println("-------------------------------");
      delay(100); // Okunabilirlik icin kisa bir bekleme
    }
  }
}