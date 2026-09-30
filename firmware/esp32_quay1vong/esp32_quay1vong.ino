// esp32_quay1vong.ino -- Phat xung STEP/DIR cho driver dong co buoc lai (hybrid servo)
// de dong co quay dung so vong, dung logic analyzer thu xung encoder va tinh thong so encoder.
//
// Noi day (xem huong dan): driver HDB CZ880 nhan tin hieu 5-24 V -> ESP32 (3.3 V) di qua transistor NPN:
//   GPIO25 -1k-> B (NPN), E -> GND, C -> PUL- ; PUL+ -> 5V.   Tuong tu GPIO26 -> DIR.
//   (GPIO muc 1 -> transistor dan -> co dong qua opto -> 1 xung; so xung khong doi)
//                           GPIO33 = MARK: len muc 1 trong luc dang phat xung (noi vao logic analyzer de trigger)
// Serial Monitor 115200, ket thuc dong bang "Newline". Lenh:
//   r            quay 1 vong TRUC DONG CO
//   g            quay 1 vong TRUC RA hop so (= XUNG_MOI_VONG * TI_SO_HOP_SO xung)
//   v <so vong>  quay nhieu vong truc dong co, vd: v 3   (nen quay >= 2 vong neu encoder co kenh Z)
//   n <so xung>  phat dung so xung, vd: n 4000
//   f / b        chieu thuan / nghich
//   s <Hz>       tan so xung toi da, vd: s 2000
//   e            bat / tat driver (chan ENA, chi dung neu da noi ENA)
//   ?            in cai dat hien tai

// ======= SUA CHO DUNG PHAN CUNG =======
const int  PIN_PUL  = 25;
const int  PIN_DIR  = 26;
const int  PIN_ENA  = 27;      // tuy chon; nhieu driver de trong ENA = luon bat
const int  PIN_MARK = 33;      // bao cho logic analyzer
long  XUNG_MOI_VONG = 800;     // so xung / 1 vong truc dong co = cai dat DIP SW1-SW4 tren driver HDB CZ880
float TI_SO_HOP_SO  = 10.0;    // hop so 1:10 -> 1 vong truc ra = 10 vong truc dong co = 8000 xung
float TAN_SO_MAX    = 2000;    // [Hz] tan so xung khi chay deu (bat dau cham de khong mat buoc)
const long XUNG_TANG_TOC = 200; // so xung tang toc / giam toc o dau va cuoi
const bool ENA_MUC_BAT = LOW;   // muc logic de BAT driver (tuy driver, thuong LOW)
// ======================================

bool chieuThuan = true;
bool driverBat  = true;

void phatXung(long n) {
  if (n <= 0) return;
  digitalWrite(PIN_DIR, chieuThuan ? HIGH : LOW);
  delayMicroseconds(20);                          // DIR phai on dinh truoc xung dau tien
  digitalWrite(PIN_MARK, HIGH);
  delayMicroseconds(200);
  uint32_t t0 = micros();
  for (long i = 0; i < n; i++) {
    long conLai = n - i;
    long k = min(i, conLai);                      // khoang cach toi dau / cuoi gan nhat
    float f = (k < XUNG_TANG_TOC) ? TAN_SO_MAX * (0.1f + 0.9f * k / XUNG_TANG_TOC) : TAN_SO_MAX;
    uint32_t nuaChuKy = (uint32_t)(500000.0f / f);
    digitalWrite(PIN_PUL, HIGH); delayMicroseconds(nuaChuKy);
    digitalWrite(PIN_PUL, LOW);  delayMicroseconds(nuaChuKy);
  }
  delayMicroseconds(200);
  digitalWrite(PIN_MARK, LOW);
  Serial.printf("Da phat %ld xung, chieu %s, mat %.3f s\n", n, chieuThuan ? "thuan" : "nghich",
                (micros() - t0) / 1e6);
}

void inCaiDat() {
  Serial.printf("XUNG_MOI_VONG=%ld  TI_SO_HOP_SO=%.3f  TAN_SO_MAX=%.0f Hz  chieu=%s  driver=%s\n",
                XUNG_MOI_VONG, TI_SO_HOP_SO, TAN_SO_MAX, chieuThuan ? "thuan" : "nghich", driverBat ? "bat" : "tat");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUL, OUTPUT);  digitalWrite(PIN_PUL, LOW);
  pinMode(PIN_DIR, OUTPUT);  digitalWrite(PIN_DIR, HIGH);
  pinMode(PIN_MARK, OUTPUT); digitalWrite(PIN_MARK, LOW);
  pinMode(PIN_ENA, OUTPUT);  digitalWrite(PIN_ENA, ENA_MUC_BAT);
  delay(500);
  Serial.println("\nESP32 - quay dong co de do encoder. Lenh: r g v n f b s e ?");
  inCaiDat();
}

void loop() {
  if (!Serial.available()) return;
  String dong = Serial.readStringUntil('\n'); dong.trim();
  if (dong.length() == 0) return;
  char lenh = dong.charAt(0);
  double so = dong.length() > 1 ? dong.substring(1).toDouble() : 0;
  switch (lenh) {
    case 'r': phatXung(XUNG_MOI_VONG); break;
    case 'g': phatXung(lround(XUNG_MOI_VONG * TI_SO_HOP_SO)); break;
    case 'v': phatXung(lround(XUNG_MOI_VONG * (so > 0 ? so : 1))); break;
    case 'n': phatXung(lround(so)); break;
    case 'f': chieuThuan = true;  inCaiDat(); break;
    case 'b': chieuThuan = false; inCaiDat(); break;
    case 's': if (so > 0) TAN_SO_MAX = so; inCaiDat(); break;
    case 'e': driverBat = !driverBat; digitalWrite(PIN_ENA, driverBat ? ENA_MUC_BAT : !ENA_MUC_BAT); inCaiDat(); break;
    case '?': inCaiDat(); break;
    default:  Serial.println("Lenh khong hop le. Dung: r g v n f b s e ?");
  }
}
