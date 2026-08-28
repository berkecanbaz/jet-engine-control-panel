#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Boardoza_FT800.h"
#include "DFRobotDFPlayerMini.h"
#include "VNH5019A.h"
#include "T3A33BRG.h"
#include <SparkFun_VL53L5CX_Library.h>

// ═══════════════════════════════════════════════════════════════════════
//  JET MOTOR KONTROL MODULU  -  v3 (yeni arayuz)
// ═══════════════════════════════════════════════════════════════════════

const bool KULLAN_MOTOR = true;
const bool KULLAN_LED   = true;
const bool KULLAN_LIDAR = true;
const bool KULLAN_MP3   = true;

// ── LIDAR tetikli motor ──────────────────────────────────────────────────
const bool LIDAR_TETIK_KULLAN = true;
const int  LIDAR_TETIK_MESAFE = 300;   // mm - altinda motor calisir
const int  LIDAR_BIRAK_MESAFE = 400;   // mm - ustunde motor durur
const int  LIDAR_TETIK_HIZ    = 100;    // %
const unsigned long LIDAR_BIRAK_GECIKME = 1500;  // ms

bool lidarTetikAktif = false;
unsigned long lidarSonGormeZamani = 0;
bool otomatikMod = true;

// ── FT800 ────────────────────────────────────────────────────────────────
constexpr uint8_t FT800_INT  = 47;
constexpr uint8_t FT800_PD   = 48;
constexpr uint8_t FT800_CS   = 7;
constexpr uint8_t FT800_MISO = 16;
constexpr uint8_t FT800_MOSI = 15;
constexpr uint8_t FT800_SCK  = 17;

Boardoza_FT800 ft800(FT800_INT, FT800_PD, FT800_CS);

constexpr uint8_t I2C_SDA_PIN = 1;
constexpr uint8_t I2C_SCL_PIN = 2;
#define TOUCH_TD_STATUS (0x02)

// ── Motor ────────────────────────────────────────────────────────────────
constexpr uint8_t MOTOR_INA_PIN   = 36;
constexpr uint8_t MOTOR_INB_PIN   = 37;
constexpr uint8_t MOTOR_PWM_PIN   = 35;
constexpr uint8_t MOTOR_DIAGA_PIN = 38;
constexpr uint8_t MOTOR_DIAGB_PIN = 38;
constexpr uint8_t MOTOR_CS_PIN    = 6;

VNH5019A motor(MOTOR_INA_PIN, MOTOR_INB_PIN, MOTOR_PWM_PIN,
               MOTOR_DIAGA_PIN, MOTOR_DIAGB_PIN, MOTOR_CS_PIN);

float motorHiziAnlik = 0;
int   motorHiziHedef = 0;
int   motorRPM       = 0;
unsigned long sonSpoolZamani = 0;

// Otomatik (LIDAR) modda jet hissi icin yavas rampa
const float SPOOL_UP_HIZ   = 12.0;   // %/sn
const float SPOOL_DOWN_HIZ = 6.0;    // %/sn

// Manuel modda kullanici parmagini takip etsin diye HIZLI rampa
const float MANUEL_UP_HIZ   = 90.0;  // %/sn
const float MANUEL_DOWN_HIZ = 90.0;  // %/sn

const int   MAKS_RPM       = 20000;
const int   BASLANGIC_HIZ  = 45;

// ── Motor olu bolge telafisi ─────────────────────────────────────────────
// VNH5019 drive() araligi -400..400. Dusuk duty'de motor kalkis torku
// uretemiyor, o yuzden %1 bile MOTOR_MIN_PWM'den basliyor.
// Motor %25'te hala donmuyorsa MOTOR_MIN_PWM'i 160-200'e cikar.
const int  MOTOR_MIN_PWM = 130;
const int  MOTOR_MAX_PWM = 400;

// Duraktan her kalkista kisa sureli tam guc tekmesi
const int  KALKIS_PWM    = 320;
const int  KALKIS_SURESI = 300;   // ms
bool kalkisAktif = false;
unsigned long kalkisBaslangic = 0;

// Manuel surukleme durumu
bool barSurukleniyor = false;

// ── MP3 ──────────────────────────────────────────────────────────────────
constexpr uint8_t MP3_RX_PIN = 44;
constexpr uint8_t MP3_TX_PIN = 43;
constexpr uint8_t MP3_SES_SEVIYESI = 10;   // 0-30

constexpr uint8_t MP3_SPOOL_UP   = 1;
constexpr uint8_t MP3_IDLE_LOOP  = 2;
constexpr uint8_t MP3_SPOOL_DOWN = 3;
const bool MP3_TEK_DOSYA = true;

HardwareSerial mp3Serial(2);
DFRobotDFPlayerMini mp3Player;
bool mp3Hazir = false;
uint8_t mp3AktifParca = 0;

void mp3DetayYazdir(uint8_t type, int value);
void mp3SesGuncelle();

// ── LED ──────────────────────────────────────────────────────────────────
constexpr uint8_t LED_DATA_PIN  = 41;
constexpr uint8_t LED_CLOCK_PIN = 42;
constexpr uint8_t LED_ADET      = 4;

T3A33BRG ledSerit(LED_DATA_PIN, LED_CLOCK_PIN);

int ledParlakligiYuzde = 75;
int ledModu = 0;
const char* LED_MOD_ISIM[4] = { "RAINBOW", "CHASE", "KIRMIZI", "FADE" };

uint8_t  ledHueOffset = 0;
uint32_t ledAnimStep  = 0;
unsigned long ledSonAnimZamani = 0;
float ledFadeAcisi = 0;

// ── LIDAR ────────────────────────────────────────────────────────────────
SparkFun_VL53L5CX lidarImager;
VL53L5CX_ResultsData lidarVeri;
bool lidarHazir = false;
int  lidarEnYakinMesafe = 0;

const bool LIDAR_SERI_YAZ = false;
const unsigned long LIDAR_YAZ_ARALIGI = 500;
unsigned long lidarSonYazZamani = 0;

// ── Renkler ──────────────────────────────────────────────────────────────
#define CYAN       0x00C8FF
#define NEON_GREEN 0x39FF6E
#define ORANGE_RED 0xFF6A00
#define TEXT_DIM   0x4A7BA0
#define LINE_DIM   0x16222E
#define WHITE      0xFFFFFF

// ═══════════════════════════════════════════════════════════════════════
//  ARAYUZ YERLESIMI (480 x 272)
//  Font yuksekligi: 26 ~16px, 28 ~25px, 29 ~28px
//  drawText SOLDAN hizali ve UST noktadan konumlanir.
// ═══════════════════════════════════════════════════════════════════════
const int EKRAN_W = 480;

// Baslik bandi
const int Y_BASLIK   = 10;
const int Y_AYIRAC_1 = 42;

// Bilgi satiri
const int Y_ETIKET   = 52;    // "MOTOR" / "LED"
const int Y_DEGER    = 72;    // buyuk sayi
const int Y_DURUM    = 104;   // durum yazisi
const int X_SOL      = 20;
const int X_SAG      = 268;

// Buton satiri
const int BTN_Y      = 130;
const int BTN_H      = 27;
const int BTN_MOD_X  = 20;    // AUTO / MANUEL
const int BTN_MOD_W  = 110;
const int BTN_LED_X  = 268;   // LED modu
const int BTN_LED_W  = 192;

// Barlar
const int BAR_X      = 92;
const int BAR_W      = 368;
const int BAR_H      = 16;
const int BAR_HIZ_Y  = 176;
const int BAR_ISIK_Y = 212;

// Alt bilgi
const int Y_ALT      = 248;

// ── Ekran koruyucu ───────────────────────────────────────────────────────
unsigned long sonDokunusZamani = 0;
const unsigned long beklemeSuresi = 10000;
bool saatModu = false;
bool oncekiDokunma = false;

// Kutuphane "first_x_touch declared degil" derse asagidakileri ac:
// uint8_t first_x_touch = 0;
// uint8_t first_y_touch = 0;

// ═══════════════════════════════════════════════════════════════════════
//  Cizim yardimcilari
// ═══════════════════════════════════════════════════════════════════════

// Fontun yaklasik karakter genisligi - ortalama hizalama icin
int fontGenislik(int font) {
  switch (font) {
    case 26: return 8;
    case 27: return 9;
    case 28: return 11;
    case 29: return 13;
    case 30: return 17;
    case 31: return 22;
    default: return 10;
  }
}

// Metni verilen X merkezine gore ortalar
void drawTextOrta(int cx, int y, int font, uint32_t renk, const char* s) {
  int w = strlen(s) * fontGenislik(font);
  int x = cx - w / 2;
  if (x < 2) x = 2;
  ft800.drawText(x, y, font, renk, (char*)s);
}

void drawYatayCizgi(int x1, int x2, int y, uint32_t renk) {
  ft800.drawLine(x1, y, x2, y, renk, 1);
}

void drawCerceve(int x, int y, int w, int h, uint32_t renk) {
  ft800.drawLine(x, y,     x + w, y,     renk, 1);
  ft800.drawLine(x, y + h, x + w, y + h, renk, 1);
  ft800.drawLine(x, y,     x,     y + h, renk, 1);
  ft800.drawLine(x + w, y, x + w, y + h, renk, 1);
}

void drawButon(int x, int y, int w, int h, uint32_t renk, const char* etiket) {
  drawCerceve(x, y, w, h, renk);
  drawTextOrta(x + w / 2, y + (h - 16) / 2, 26, renk, etiket);
}

void drawProgressBar(int x, int y, int w, int h, int pct, uint32_t renk) {
  drawCerceve(x, y, w, h, LINE_DIM);
  int dolu = (w * pct) / 100;
  if (dolu > 2) {
    for (int i = 2; i < h - 1; i++) {
      ft800.drawLine(x + 2, y + i, x + dolu - 1, y + i, renk, 1);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
//  LED
// ═══════════════════════════════════════════════════════════════════════
void ledCerceveyiKilitle() {
  digitalWrite(LED_DATA_PIN, HIGH);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(LED_CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LED_CLOCK_PIN, LOW);
    delayMicroseconds(1);
  }
}

void ledRenkTekeri(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (pos < 85)       { r = pos * 3;       g = 255 - pos * 3; b = 0; }
  else if (pos < 170) { pos -= 85;  r = 255 - pos * 3; g = 0; b = pos * 3; }
  else                { pos -= 170; r = 0; g = pos * 3; b = 255 - pos * 3; }
}

void ledSeridiGuncelleVeCiz() {
  if (millis() - ledSonAnimZamani > 3) {
    ledHueOffset += 2;
    ledAnimStep++;
    ledFadeAcisi += 0.02;
    if (ledFadeAcisi > PI * 2) ledFadeAcisi = 0;
    ledSonAnimZamani = millis();
  }

  uint8_t aktifParlaklik = map(ledParlakligiYuzde, 0, 100, 0, 31);

  ledSerit.send32Bit(0, 0, 0, 0, 0);
  for (uint8_t i = 0; i < LED_ADET; i++) {
    uint8_t r = 0, g = 0, b = 0;
    switch (ledModu) {
      case 0:
        ledRenkTekeri((i * 256 / LED_ADET + ledHueOffset) & 0xFF, r, g, b);
        break;
      case 1:
        if ((i + ledAnimStep / 30) % 3 == 0) { r = 0; g = 255; b = 0; }
        break;
      case 2:
        r = 255; g = 0; b = 0;
        break;
      case 3: {
        float s = (sin(ledFadeAcisi) + 1.0) / 2.0;
        uint8_t val = (uint8_t)(s * 255.0);
        r = g = b = val;
        break;
      }
    }
    ledSerit.send32Bit(0x07, aktifParlaklik, b, g, r);
  }
  ledCerceveyiKilitle();
}

// ═══════════════════════════════════════════════════════════════════════
//  LIDAR
// ═══════════════════════════════════════════════════════════════════════
void lidarVeriyiOku() {
  if (!lidarHazir) return;

  // I2C hatti dokunmatikle ortak. Her donguce 64 hucre cekmek dokunma
  // tepkisini oldururdu; 100 ms'de bir okumak tetik icin fazlasiyla yeterli.
  static unsigned long sonLidarOkuma = 0;
  if (millis() - sonLidarOkuma < 100) return;
  sonLidarOkuma = millis();

  if (!lidarImager.isDataReady()) return;
  if (!lidarImager.getRangingData(&lidarVeri)) return;

  int enYakin = 9999;
  for (int i = 0; i < 64; i++) {
    uint8_t durum = lidarVeri.target_status[i];
    if (durum == 5 || durum == 6 || durum == 9) {
      int m = lidarVeri.distance_mm[i];
      if (m > 0 && m < enYakin) enYakin = m;
    }
  }
  lidarEnYakinMesafe = (enYakin == 9999) ? 0 : enYakin;

  if (!LIDAR_SERI_YAZ) return;
  if (millis() - lidarSonYazZamani < LIDAR_YAZ_ARALIGI) return;
  lidarSonYazZamani = millis();

  Serial.println(F("--- 8x8 Mesafe Matrisi (mm) ---"));
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      int idx = r * 8 + c;
      uint8_t d = lidarVeri.target_status[idx];
      if (d == 5 || d == 6 || d == 9) Serial.print(lidarVeri.distance_mm[idx]);
      else                            Serial.print("X");
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println(F("-------------------------------"));
}

// ═══════════════════════════════════════════════════════════════════════
//  MOTOR
// ═══════════════════════════════════════════════════════════════════════
void motorHedefBelirle(int yeniHedef) {
  if (yeniHedef < 0)   yeniHedef = 0;
  if (yeniHedef > 100) yeniHedef = 100;

  // Motor duruyorsa ve hareket isteniyorsa kalkis tekmesi ver
  if (motorHiziAnlik < 1 && yeniHedef > 0) {
    kalkisAktif = true;
    kalkisBaslangic = millis();
  }
  motorHiziHedef = yeniHedef;
}

void lidarTetikGuncelle() {
  if (!LIDAR_TETIK_KULLAN) return;
  if (!lidarHazir) return;
  if (!otomatikMod) return;

  bool gecerli = (lidarEnYakinMesafe > 0);

  if (gecerli && lidarEnYakinMesafe < LIDAR_TETIK_MESAFE) {
    lidarSonGormeZamani = millis();
    if (!lidarTetikAktif) {
      lidarTetikAktif = true;
      motorHedefBelirle(LIDAR_TETIK_HIZ);
      Serial.print(F("LIDAR tetik: "));
      Serial.print(lidarEnYakinMesafe);
      Serial.println(F(" mm"));
    }
  }
  else if (lidarTetikAktif) {
    bool uzaklasti = (!gecerli) || (lidarEnYakinMesafe > LIDAR_BIRAK_MESAFE);
    if (uzaklasti && (millis() - lidarSonGormeZamani > LIDAR_BIRAK_GECIKME)) {
      lidarTetikAktif = false;
      motorHedefBelirle(0);
      Serial.println(F("LIDAR tetik birakildi"));
    }
  }
}

void spoolGuncelle() {
  unsigned long simdi = millis();
  if (simdi - sonSpoolZamani < 20) return;
  float dt = (simdi - sonSpoolZamani) / 1000.0;
  sonSpoolZamani = simdi;

  // Manuel modda parmagi aninda takip et, otomatikte jet gibi yavas rampala
  float upHiz   = otomatikMod ? SPOOL_UP_HIZ   : MANUEL_UP_HIZ;
  float downHiz = otomatikMod ? SPOOL_DOWN_HIZ : MANUEL_DOWN_HIZ;

  if (motorHiziAnlik < motorHiziHedef) {
    motorHiziAnlik += upHiz * dt;
    if (motorHiziAnlik > motorHiziHedef) motorHiziAnlik = motorHiziHedef;
  } else if (motorHiziAnlik > motorHiziHedef) {
    motorHiziAnlik -= downHiz * dt;
    if (motorHiziAnlik < motorHiziHedef) motorHiziAnlik = motorHiziHedef;
  }

  // ── Olu bolge telafisi ────────────────────────────────────────────────
  // %0 -> gercek 0 (motor tamamen dursun)
  // %1..100 -> MOTOR_MIN_PWM..MOTOR_MAX_PWM
  int hiz;
  if (motorHiziAnlik < 0.5) {
    hiz = 0;
  } else {
    hiz = MOTOR_MIN_PWM +
          (int)((motorHiziAnlik / 100.0) * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
    if (hiz > MOTOR_MAX_PWM) hiz = MOTOR_MAX_PWM;
  }

  if (kalkisAktif) {
    if (simdi - kalkisBaslangic < KALKIS_SURESI) {
      if (hiz > 0 && KALKIS_PWM > hiz) hiz = KALKIS_PWM;
    } else {
      kalkisAktif = false;
    }
  }

  motor.drive(hiz);
  motorRPM = map((int)motorHiziAnlik, 0, 100, 0, MAKS_RPM);
}

// ═══════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("=== Jet Motor Kontrol Modulu ==="));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  ft800.begin(FT800_SCK, FT800_MISO, FT800_MOSI, FT800_CS);
  Serial.println(F("Ekran baslatildi."));

  if (KULLAN_MOTOR) {
    motor.begin();
    motor.drive(0);
    motorHiziAnlik = 0;
    sonSpoolZamani = millis();
    if (LIDAR_TETIK_KULLAN) {
      motorHedefBelirle(0);
      Serial.println(F("Motor hazir - LIDAR tetigi bekleniyor."));
    } else {
      motorHedefBelirle(BASLANGIC_HIZ);
      Serial.println(F("Motor baslatildi."));
    }
  } else {
    Serial.println(F("Motor kapali."));
  }

  if (KULLAN_LED) {
    ledSerit.begin();
    Serial.println(F("LED seridi baslatildi."));
  }

  if (KULLAN_LIDAR) {
    if (lidarImager.begin()) {
      lidarImager.setResolution(8 * 8);
      lidarImager.setRangingFrequency(15);
      lidarImager.startRanging();
      lidarHazir = true;
      Serial.println(F("LIDAR baglandi."));
    } else {
      Serial.println(F("LIDAR bulunamadi."));
    }
  } else {
    Serial.println(F("LIDAR kapali."));
  }

  if (KULLAN_MP3) {
    Serial.println(F("MP3 baslatiliyor..."));
    mp3Serial.begin(9600, SERIAL_8N1, MP3_RX_PIN, MP3_TX_PIN);
    delay(1000);
    for (int i = 0; i < 3 && !mp3Hazir; i++) {
      if (mp3Player.begin(mp3Serial, true, false)) { mp3Hazir = true; break; }
      delay(500);
    }
    if (mp3Hazir) {
      Serial.println(F("DFPlayer baglandi."));
      mp3Player.volume(MP3_SES_SEVIYESI);
      mp3AktifParca = MP3_TEK_DOSYA ? 1 : MP3_SPOOL_UP;
      mp3Player.play(mp3AktifParca);
    } else {
      Serial.println(F("DFPlayer bulunamadi."));
    }
  } else {
    Serial.println(F("MP3 kapali."));
  }

  sonDokunusZamani = millis();
  Serial.println(F("Sistem hazir."));
}

// ═══════════════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════════════
void loop() {

  lidarVeriyiOku();
  lidarTetikGuncelle();
  spoolGuncelle();
  mp3SesGuncelle();

  if (mp3Hazir && mp3Player.available()) {
    mp3DetayYazdir(mp3Player.readType(), mp3Player.read());
  }

  ledSeridiGuncelleVeCiz();

  // ── Dokunmatik ─────────────────────────────────────────────────────────
  uint8_t touchCount = ft800.read_i2c_register(TOUCH_TD_STATUS);
  bool dokunmaVar = false;
  uint16_t touchX = 0, touchY = 0;

  if (touchCount > 0) {
    touchX = ft800.touchXPosition(first_x_touch);
    touchY = ft800.touchYPosition(first_y_touch);
    dokunmaVar = (touchX < 480 && touchY < 272);
  }

  bool yeniDokunma = dokunmaVar && !oncekiDokunma;

  if (dokunmaVar) {
    sonDokunusZamani = millis();

    if (saatModu) {
      if (yeniDokunma) saatModu = false;
    } else {
      // Hiz bari (dokunma alani barin biraz disina tasar)
      if (touchY >= BAR_HIZ_Y - 14 && touchY <= BAR_HIZ_Y + BAR_H + 14 &&
          touchX >= BAR_X - 10 && touchX <= BAR_X + BAR_W + 10) {
        otomatikMod = false;
        lidarTetikAktif = false;
        barSurukleniyor = true;

        int x = constrain((int)touchX, BAR_X, BAR_X + BAR_W);
        motorHedefBelirle(map(x, BAR_X, BAR_X + BAR_W, 0, 100));
      }
      // Isik bari
      else if (touchY >= BAR_ISIK_Y - 12 && touchY <= BAR_ISIK_Y + BAR_H + 12 &&
               touchX >= BAR_X && touchX <= BAR_X + BAR_W) {
        ledParlakligiYuzde = map(touchX, BAR_X, BAR_X + BAR_W, 0, 100);
      }
      // AUTO / MANUEL butonu
      else if (yeniDokunma &&
               touchY >= BTN_Y && touchY <= BTN_Y + BTN_H &&
               touchX >= BTN_MOD_X && touchX <= BTN_MOD_X + BTN_MOD_W) {
        otomatikMod = !otomatikMod;
        if (otomatikMod) {
          lidarTetikAktif = false;
          motorHedefBelirle(0);
        }
      }
      // LED mod butonu
      else if (yeniDokunma &&
               touchY >= BTN_Y && touchY <= BTN_Y + BTN_H &&
               touchX >= BTN_LED_X && touchX <= BTN_LED_X + BTN_LED_W) {
        ledModu = (ledModu + 1) % 4;
      }
    }
  }
  if (!dokunmaVar) barSurukleniyor = false;
  oncekiDokunma = dokunmaVar;

  if (millis() - sonDokunusZamani > beklemeSuresi) saatModu = true;

  // ═══════════════════════════════════════════════════════════════════
  //  EKRAN
  // ═══════════════════════════════════════════════════════════════════
  ft800.startFrame();
  ft800.clearScreencolorBackground(0x000000, CLR_COL | CLR_STN | CLR_TAG);

  if (saatModu) {
    // ── Ekran koruyucu ──────────────────────────────────────────────
    unsigned long s = (millis() / 1000) % 60;
    unsigned long m = (millis() / 60000) % 60;
    unsigned long h = (millis() / 3600000) % 24;
    String t = (h < 10 ? "0" : "") + String(h) + ":" +
               (m < 10 ? "0" : "") + String(m) + ":" +
               (s < 10 ? "0" : "") + String(s);
    drawTextOrta(240, 90, 29, CYAN, t.c_str());

    String rpmMini = String(motorRPM) + " RPM";
    drawTextOrta(240, 140, 26, TEXT_DIM, rpmMini.c_str());
    drawTextOrta(240, 200, 26, TEXT_DIM, "DOKUNARAK DEVAM EDIN");

  } else {
    // ── Baslik ──────────────────────────────────────────────────────
    drawTextOrta(240, Y_BASLIK, 28, CYAN, "JET MOTOR KONTROL");
    drawYatayCizgi(20, 460, Y_AYIRAC_1, LINE_DIM);

    // ── Sol: Motor ──────────────────────────────────────────────────
    ft800.drawText(X_SOL, Y_ETIKET, 26, TEXT_DIM, (char*)"MOTOR");

    String rpmLabel = String(motorRPM) + " RPM";
    ft800.drawText(X_SOL, Y_DEGER, 29, CYAN, (char*)rpmLabel.c_str());

    const char* durum;
    uint32_t durumRenk;
    if ((int)motorHiziAnlik < motorHiziHedef) {
      durum = "SPOOLING UP";  durumRenk = ORANGE_RED;
    } else if ((int)motorHiziAnlik > motorHiziHedef) {
      durum = "SPOOLING DOWN"; durumRenk = ORANGE_RED;
    } else if (motorHiziHedef == 0) {
      durum = otomatikMod ? "HEDEF BEKLENIYOR" : "DURDU";
      durumRenk = TEXT_DIM;
    } else {
      durum = "STABIL"; durumRenk = NEON_GREEN;
    }
    ft800.drawText(X_SOL, Y_DURUM, 26, durumRenk, (char*)durum);

    // ── Sag: LED ────────────────────────────────────────────────────
    ft800.drawText(X_SAG, Y_ETIKET, 26, TEXT_DIM, (char*)"ISIK");

    String ledLabel = String(ledParlakligiYuzde) + " %";
    ft800.drawText(X_SAG, Y_DEGER, 29, NEON_GREEN, (char*)ledLabel.c_str());

    // LIDAR mesafesi - LED'in altina, sag sutuna
    if (lidarHazir) {
      String lidarLabel;
      if (lidarEnYakinMesafe > 0) lidarLabel = "MESAFE  " + String(lidarEnYakinMesafe) + " mm";
      else                        lidarLabel = "MESAFE  --";
      uint32_t lidarRenk = lidarTetikAktif ? ORANGE_RED : TEXT_DIM;
      ft800.drawText(X_SAG, Y_DURUM, 26, lidarRenk, (char*)lidarLabel.c_str());
    }

    // ── Butonlar ────────────────────────────────────────────────────
    uint32_t modRenk = otomatikMod ? NEON_GREEN : ORANGE_RED;
    drawButon(BTN_MOD_X, BTN_Y, BTN_MOD_W, BTN_H, modRenk,
              otomatikMod ? "AUTO" : "MANUEL");

    drawButon(BTN_LED_X, BTN_Y, BTN_LED_W, BTN_H, CYAN, LED_MOD_ISIM[ledModu]);

    // ── Barlar ──────────────────────────────────────────────────────
    ft800.drawText(X_SOL, BAR_HIZ_Y, 26, TEXT_DIM, (char*)"HIZ");
    drawProgressBar(BAR_X, BAR_HIZ_Y, BAR_W, BAR_H, (int)motorHiziAnlik, CYAN);

    // Hedef gostergesi: parmagin nereyi sectigini ANINDA gosterir,
    // motor oraya rampalanirken bile bar tepkisiz gorunmez.
    if (motorHiziHedef != (int)motorHiziAnlik) {
      int hx = BAR_X + (BAR_W * motorHiziHedef) / 100;
      hx = constrain(hx, BAR_X + 1, BAR_X + BAR_W - 1);
      ft800.drawLine(hx, BAR_HIZ_Y - 4, hx, BAR_HIZ_Y + BAR_H + 4,
                     ORANGE_RED, 2);
    }

    ft800.drawText(X_SOL, BAR_ISIK_Y, 26, TEXT_DIM, (char*)"ISIK");
    drawProgressBar(BAR_X, BAR_ISIK_Y, BAR_W, BAR_H, ledParlakligiYuzde, NEON_GREEN);

    // ── Alt bilgi ───────────────────────────────────────────────────
    drawYatayCizgi(20, 460, Y_ALT - 8, LINE_DIM);
    ft800.drawText(X_SOL, Y_ALT, 26, NEON_GREEN, (char*)"SISTEM AKTIF");
    ft800.drawText(378, Y_ALT, 26, ORANGE_RED, (char*)"BOARDOZA");
  }

  ft800.endFrame();
  ft800.waitForCommandCompletion();
}

// ═══════════════════════════════════════════════════════════════════════
//  MP3
// ═══════════════════════════════════════════════════════════════════════
void mp3SesGuncelle() {
  if (!mp3Hazir || MP3_TEK_DOSYA) return;

  uint8_t istenen;
  if ((int)motorHiziAnlik < motorHiziHedef)      istenen = MP3_SPOOL_UP;
  else if ((int)motorHiziAnlik > motorHiziHedef) istenen = MP3_SPOOL_DOWN;
  else                                           istenen = MP3_IDLE_LOOP;

  if (istenen != mp3AktifParca) {
    mp3AktifParca = istenen;
    mp3Player.play(mp3AktifParca);
  }
}

void mp3DetayYazdir(uint8_t type, int value) {
  switch (type) {
    case TimeOut:              Serial.println(F("MP3: Zaman asimi!")); break;
    case WrongStack:           Serial.println(F("MP3: Hatali paket.")); break;
    case DFPlayerCardInserted: Serial.println(F("MP3: SD Kart takildi.")); break;
    case DFPlayerCardRemoved:  Serial.println(F("MP3: SD Kart cikarildi.")); break;
    case DFPlayerPlayFinished:
      if (mp3Hazir) mp3Player.play(mp3AktifParca);
      break;
    case DFPlayerError:
      Serial.print(F("MP3: Hata: "));
      switch (value) {
        case Busy:             Serial.println(F("Kart mesgul")); break;
        case Sleeping:         Serial.println(F("Uyku modunda")); break;
        case SerialWrongStack: Serial.println(F("Seri haberlesme hatasi")); break;
        case CheckSumNotMatch: Serial.println(F("Checksum eslesmedi")); break;
        case FileIndexOut:     Serial.println(F("Dosya indeksi disinda")); break;
        case FileMismatch:     Serial.println(F("Dosya bulunamadi")); break;
        default: break;
      }
      break;
    default: break;
  }
}
