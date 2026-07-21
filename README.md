# Gerçek Zamanlı Quadcopter Hareket Kontrol ve Simülasyon Sistemi

## Proje Hakkında Genel Bilgiler

- **Proje Adı:** Quadcopter Motion Control (QMC) - Real-Time Quadcopter Simulation
- **Kurum:** Doğu Akdeniz Üniversitesi (DAÜ) / Eastern Mediterranean University (EMU)
- **Bölüm:** Bilgisayar Mühendisliği Bölümü (Department of Computer Engineering)
- **Ders:** CMSE443 - Real Time System Design (Gerçek Zamanlı Sistem Tasarımı)
- **Ders Sorumlusu:** Prof. Dr. Alexander Chefreanov
- **Geliştirici Takımı (Group 15):**
  - Zeynep Pelin Çolak (17300009)
  - Mehmethan Küçük (18330756)
  - Melisa Mehrüboğlu (18330757)
- **Dönem:** 2025-2026 Güz Dönemi

---

## 1. Projenin Amacı ve Tanımı

Bu projenin temel amacı, katı zaman kısıtlamaları (strict timing constraints) altında çalışan, gerçekçi matematiksel hareket modellerine dayalı, kararlı ve performanslı bir 3B Quadcopter (Dört Pervaneli İHA) simülasyon ve kontrol sistemi tasarlamak ve geliştirmektir.

Sistem, fiziksel bir drone'un uçuş dinamiklerini diferansiyel denklemler aracılığıyla sayısal olarak çözer. Kullanıcıya hem 3. Şahıs (Third Person) hem de 1. Şahıs Görüş (FPV - First Person View) perspektiflerinden gerçek zamanlı görsel geri bildirim sunar.

### Temel Hedefler
- **Gerçek Zamanlı Çalışma (Real-Time Execution):** Simülasyon adımlarının ve görselleştirmelerin sabit zaman aralıklarında (fixed time-step) kararlı bir şekilde yürütülmesi.
- **Sayısal İntegrasyon Teknikleri:** Euler ve 4. Derece Runge-Kutta (RK4) sayısal integrasyon yöntemlerinin uygulanması ve doğruluk-hesaplama yükü karşılaştırması.
- **Modüler Mimarı:** Simülasyon motoru, fizik modeli, PID kontrolör, çarpışma algılama ve kullanıcı arayüzünün birbirinden bağımsız modüller halinde tasarlanması.
- **Etkileşimli Kontrol:** Kullanıcının drone'u manuel olarak kumanda edebilmesi, otomatik süzülme (Auto-Hover) modunu kullanabilmesi ve simülasyon parametrelerini anlık olarak değiştirebilmesi.

---

## 2. Kullanılan Teknolojiler ve Araçlar

Proje geliştirme sürecinde aşağıdaki teknoloji yığını ve kütüphaneler kullanılmıştır:

- **Oyun ve Simülasyon Motoru:** Unity 3D (2022 / 6000 serisi URP)
  - Grafik işleme, 3B sahne yönetimi, kamera kontrolleri ve kullanıcı arayüzü sunumu için kullanılmıştır.
  - Unity'nin hazır fizik motoru yerine, drone dinamikleri özel sayısal integrasyon algoritmaları ile C# tarafında hesaplanmaktadır.
- **Programlama Dili:** C# (.NET / MonoBehaviour mimarisi)
  - Tüm matematiksel hesaplamalar, PID kontrol algoritmaları, entegrasyon yöntemleri ve arayüz mantığı C# diliyle yazılmıştır.
- **Kullanıcı Arayüzü (UI):** Unity UI & TextMeshPro (TMP)
  - Simülasyon durum panelleri, Start/Stop/Reset butonları ve uçuş verisi göstergeleri için kullanılmıştır.
- **Görsel Varlıklar (Assets):**
  - City Package (Şehir ortamı ve engeller)
  - Simple Drone Prefab (3B Drone modeli ve dönen pervaneler)

---

## 3. Sistem Mimarisi ve Bileşen Etkileşimi

Sistem üç temel katmandan oluşan modüler bir mimariye sahiptir:

1. **Kullanıcı Arayüzü (User Interface - UI):** Kullanıcıdan kumanda komutlarını (W/S, A/D, Q/E, Space/Ctrl) ve buton etkileşimlerini alır.
2. **Quadcopter Simülatörü (Quadcopter Simulator):** Girdileri işler, PID kontrolörler aracılığıyla itki ve tork değerlerini hesaplar, diferansiyel denklemleri seçilen integrasyon yöntemiyle (Euler/RK4) çözer ve drone durumunu günceller.
3. **Görselleştirme Sistemi (Visualization System):** Hesaplanan durum verisini okur, drone'un 3B sahnedeki konumunu ve rotasyonunu günceller, pervaneleri döndürür ve FPV/TPS kameralarını işler.

### Bileşen Etkileşim Akışı

1. Kullanıcı klavye veya arayüz üzerinden komut verir.
2. `DroneController` komutları işler; yükseklik PID'sinden Toplam İtki (`totalThrust`), tutum PID'sinden Tork Momenti (`moments`) üretir.
3. `DronePhysics` bu verileri kullanarak türevleri hesaplar (`ComputeDerivatives`).
4. `SimulationManager` seçilen entegratöre (Euler veya RK4) göre `DronePhysics` adımı atar ve durum vektörünü (`DroneState`) günceller.
5. `GroundClamp` drone'un yerin altına girmesini engeller.
6. `DroneCrashHandler` engellerle çarpışmayı tespit eder.
7. `SimulationManager` güncellenen konumu `FixedUpdate` döngüsünde görsel objeye aktarır ve `UpdateHUD` fonksiyonunu tetikler.

---

## 4. Matematiksel Model ve Fizik Motoru

Quadcopter'in hareketi non-linear (doğrusal olmayan) diferansiyel denklem takımları ile modellenmiştir.

### 4.1. Durum Vektörü (State Vector)

Drone'un anlık durumunu temsil eden vektör aşağıdaki 12 değişkenden oluşur:

$$\mathbf{S} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \boldsymbol{\Theta} \\ \boldsymbol{\omega} \end{bmatrix} = \begin{bmatrix} x, y, z \\ v_x, v_y, v_z \\ \text{pitch}, \text{yaw}, \text{roll} \\ \omega_x, \omega_y, \omega_z \end{bmatrix}$$

- $\mathbf{p}$: 3B Konum vektörü (Metre)
- $\mathbf{v}$: 3B Çizgisel Hız vektörü (Metre/Saniye)
- $\boldsymbol{\Theta}$: Euler Açıları (Derece)
- $\boldsymbol{\omega}$: Açısal Hız vektörü (Radyan/Saniye veya Derece/Saniye)

### 4.2. Çizgisel Hareket Dinamikleri (Translational Dynamics)

Newton'un ikinci hareket yasasına göre çizgisel ivme:

$$\mathbf{a} = \frac{\mathbf{F}_{thrust} + \mathbf{F}_{damping}}{m} + \mathbf{g}$$

Burada:
- $\mathbf{F}_{thrust}$: Drone gövdesinin Y ekseni yönünde uygulanan itki kuvvetinin Dünya koordinat sistemine dönüştürülmüş halidir. Rotasyon matrisi $R$ ile hesaplanır:

$$\mathbf{F}_{thrust} = R(\boldsymbol{\Theta}) \cdot \begin{bmatrix} 0 \\ T_{total} \\ 0 \end{bmatrix}$$

- $\mathbf{F}_{damping}$: Lineer aerodinamik sönümleme (sürtünme) kuvvetidir:

$$\mathbf{F}_{damping} = -c_{linear} \cdot \mathbf{v}$$

- $\mathbf{g}$: Yerçekimi ivmesidir $(0, -9.81, 0)^T$.
- $m$: Drone kütlesidir (Varsayılan: $1.0 \text{ kg}$).

### 4.3. Açısal Hareket Dinamikleri (Rotational Dynamics)

Euler'in katı cisim hareket denklemine göre açısal ivme:

$$\dot{\boldsymbol{\omega}} = \mathbf{M} - c_{angular} \cdot \boldsymbol{\omega}$$

Burada:
- $\mathbf{M} = (M_x, M_y, M_z)^T$: Pitch, Yaw ve Roll eksenlerindeki tork momentleridir.
- $c_{angular}$: Açısal sönümleme kat sayısıdır.

### 4.4. Açı ve Hız Sınırlamaları (Clamping)

Gerçekçi olmayan taklaları ve sayısal patlamaları önlemek için:
- Pitch (Eğim) ve Roll (Yatma) açıları $[-40^\circ, 40^\circ]$ aralığında tutulur (`maxTiltAngle = 40f`).
- Açısal hız genliği maksimum $200^\circ/s$ ile sınırlandırılır.

---

## 5. Sayısal İntegrasyon Yöntemleri

Simülasyonda diferansiyel denklemleri zaman içinde ilerletmek için iki farklı sayısal integrasyon yöntemi uygulanmıştır.

### 5.1. Euler Yöntemi (Birinci Derece Entegratör)

Euler yöntemi en basit sayısal entegrasyon yöntemidir. Bir sonraki durumu, mevcut durumun türevini zaman adımı ($\Delta t$) ile çarparak ekler:

$$\mathbf{S}_{n+1} = \mathbf{S}_n + f(\mathbf{S}_n, \mathbf{u}) \cdot \Delta t$$

- **Avantajı:** Hesaplama yükü son derece düşüktür.
- **Dezavantajı:** Adım boyutu büyüdükçe birikimli hata artar ve sayısal kararsızlığa neden olabilir.

### 5.2. 4. Derece Runge-Kutta Yöntemi (RK4)

RK4 yöntemi, zaman adımı içerisinde 4 farklı noktada türev alarak yüksek doğruluk ve sayısal kararlılık sağlar:

1. $k_1 = f(\mathbf{S}_n, \mathbf{u})$
2. $k_2 = f\left(\mathbf{S}_n + \frac{\Delta t}{2} k_1, \mathbf{u}\right)$
3. $k_3 = f\left(\mathbf{S}_n + \frac{\Delta t}{2} k_2, \mathbf{u}\right)$
4. $k_4 = f\left(\mathbf{S}_n + \Delta t \, k_3, \mathbf{u}\right)$

Yeni durum vektörü bu dört eğimin ağırlıklı ortalaması alınarak hesaplanır:

$$\mathbf{S}_{n+1} = \mathbf{S}_n + \frac{\Delta t}{6} \left( k_1 + 2k_2 + 2k_3 + k_4 \right)$$

- **Avantajı:** Yüksek doğruluk, düşük hata oranı ve uzun süreli simülasyonlarda yüksek kararlılık.
- **Dezavantajı:** Her adımda 4 kez türev hesabı yapıldığı için Euler'e göre daha fazla işlem yükü getirir.

---

## 6. Kontrol Sistemleri ve PID Stabilizatör

`DroneController` sınıfı, drone'un süzülmesini (hover) ve kullanıcı girdilerine doğru yanıt vermesini sağlayan kapalı döngü PID (Proportional-Integral-Derivative) kontrolörleri içerir.

### 6.1. Yükseklik Kontrolörü (Altitude PID)

Drone'un istenen yükseklikte kalması veya yükselip alçalması için gereken toplam itkiyi ($T$) hesaplar:

$$e_{alt} = y_{target} - y_{current}$$

$$T = (m \cdot g) + K_{p,alt} \cdot e_{alt} + K_{i,alt} \int e_{alt} dt + K_{d,alt} \cdot (-v_y)$$

Hesaplanan itki, motor güç sınırları ($0$ ile `maxThrust` arası) dahilinde tutulur.

### 6.2. Tutum Kontrolörü (Attitude PID)

Pitch, Roll ve Yaw açılarını kontrol ederek istenen yönelimi sağlar:

- **Pitch Momenti ($M_x$):** İstenen Pitch açısı ile mevcut Pitch açısı arasındaki hataya göre tork üretir.
- **Roll Momenti ($M_z$):** İstenen Roll açısı ile mevcut Roll açısı arasındaki hataya göre tork üretir.
- **Yaw Momenti ($M_y$):** İstenen Yaw açısı ile mevcut Yaw açısı arasındaki hataya göre tork üretir.

İntegral teriminde taşma olmaması için anti-windup sınırı (`integralLimit = 10f`) uygulanmıştır.

### 6.3. Kontrol Modları

1. **Manuel Mod (Manual Control):**
   - Kullanıcı W/S tuşlarıyla hedef Pitch açısını, A/D tuşlarıyla hedef Roll açısını belirler.
   - Q/E tuşlarıyla Yaw açısı değiştirilir.
   - Space/LCtrl tuşlarıyla hedef yükseklik arttırılır veya azaltılır.
2. **Otomatik Süzülme Modu (AutoHoverPID):**
   - Drone belirlenen `autoTargetPosition` noktasına (örneğin başlangıç konumunun 2 metre üstü) gitmek için yatay ve dikey hataları hesaplar ve kendini otomatik olarak stabilize eder.
   - Modlar arasında `Tab` tuşu ile geçiş yapılır.

---

## 7. Kod Yapısı ve Betik Detayları

Projenin `Assets/Scripts` dizininde bulunan temel C# betikleri ve görevleri aşağıda detaylandırılmıştır:

### 7.1. ControlInput.cs
İtki ve tork verilerini taşıyan hafif bir veri yapısıdır (struct):
- `totalThrust`: Toplam dikey itki kuvveti (Newton).
- `moments`: Pitch, Yaw, Roll eksenlerindeki tork momentleri (Vector3).

### 7.2. DronePhysics.cs
Fizik diferansiyel denklemlerinin çözüldüğü ana matematik betiğidir:
- Physical parametreler: `mass` (1.0 kg), `gravity` (9.81 m/s²), `linearDamping` (1.5), `angularDamping` (4.0).
- `ComputeDerivatives()`: Anlık durum ve kontrol girdisine göre türev vektörünü ($d\mathbf{S}/dt$) hesaplar.
- `EulerStep()`: Birinci derece Euler entegrasyon adımını icra eder.
- `RungeKutta4Step()`: 4. derece Runge-Kutta entegrasyon adımını icra eder.
- `ClampAngles()`: Aşırı açısal sapmaları önler.

### 7.3. DroneController.cs
PID kontrolörlerini ve girdi yönetkenini içerir:
- Tutum kazançları: `pitchKp`, `rollKp`, `yawKp`, `pitchKd`, `rollKd`, `yawKd`.
- Yükseklik kazançları: `altKp`, `altKd`.
- `GetControlInput()`: Manuel veya otomatik mod durumuna göre gerekli itki ve tork değerlerini oluşturur.
- `ToggleMode()`: Manuel ve AutoHover modları arasında geçiş yapar.

### 7.4. SimulationManager.cs
Simülasyon döngüsünü ve zamanlamasını yöneten ana orkestratördür:
- `simulationDt = 0.01f`: Entegrasyon adım boyutu (100 Hz).
- `visualizeDt = 0.05f`: Görselleştirme ve HUD yenileme hızı (20 Hz).
- `Update()`: Zaman akümülatörü (`simTimeAccumulator`) kullanarak adımların kaçırılmadan sabit frekansta çalışmasını sağlar.
- `FixedUpdate()`: Hesaplanan durumu Unity'nin `Rigidbody` bileşenine aktarır.
- `StartSimulation()`, `StopSimulation()`, `ResetSimulation()`: Simülasyon durum yönetimi.

### 7.5. DroneCrashHandler.cs
Gelişmiş çarpışma ve kaza yönetim sistemidir:
- `FixedUpdate()` döngüsünde `SphereCast` ile hareket yönündeki engelleri tarar.
- Hız eşiği (`crashSpeedThreshold = 1.0f`) aşıldığında ve bir nesneye çarpıldığında kaza coroutine'ini (`CrashCutFallAndReset`) başlatır.
- Motorları ve pervaneleri durdurur, kısa bir düşüş efekti uygular, ardından drone'u başlangıç konumuna yerleştirerek simülasyonu yeniden başlatır.

### 7.6. DroneCameraController.cs
Kamera takip ve açı yönetim sistemidir:
- **Third Person (TPS):** Drone'un arkasından ve biraz yukarısından yumuşak takip (`followSmooth`, `rotateSmooth`). FOV: 60 derece.
- **First Person (FPV):** Drone'un burun kısmına yerleştirilmiş kokpit görüşü. FOV: 75 derece.
- `C` tuşu ile kamera modları arasında geçiş yapılır.

### 7.7. GroundClamp.cs
Drone'un yerin altına girmesini veya yer kabuğunu delmesini önleyen güvenlik katmanıdır:
- `SphereCast` ile zemin mesafesini ölçer ve minimum zemin mesafesini (`clearance = 0.05f`) korur.

### 7.8. PropellerSpin.cs
Pervanelerin görsel dönme hareketini kontrol eder:
- İtki miktarına (`lastThrust`) bağlı olarak dönme hızını ayarlar. Kaza anında dönmeyi durdurur.

### 7.9. SimulationUI.cs
Arayüz üzerindeki Start, Stop ve Reset butonlarının tıklama olaylarını dinler ve durum metnini ("RUNNING" / "STOPPED") günceller.

---

## 8. Gerçek Zamanlı (Real-Time) Çalışma Mantığı

Sistem, **Soft Real-Time** (Yumuşak Gerçek Zamanlı) kısıtlar altında çalışacak şekilde tasarlanmıştır.

### Zaman Adımı Stratejisi (Fixed Time-Step)

Visual render hızı (FPS) bilgisayardan bilgisayara değişkenlik gösterebilir. Fizik simülasyonunun kare hızından bağımsız ve kararlı kalması için zaman akümülatör yöntemi kullanılmıştır:

- Fizik hesabı her $0.01 \text{ saniye}$ ($100 \text{ Hz}$) aralıklarla adımlanır.
- Görselleştirme ve telemetry güncellemeleri her $0.05 \text{ saniye}$ ($20 \text{ Hz}$) aralıklarla yapılır.
- `Update()` fonksiyonunda biriken süre (`simTimeAccumulator`), `simulationDt` miktarından büyük olduğu sürece `while` döngüsü içinde `StepSimulation()` çağrılarak zaman kayması (time drift) engellenir.

---

## 9. Projeyi Çalıştırma ve Kullanım Kılavuzu

### 9.1. Gereksinimler
- Unity Hub ve Unity 2022.3 LTS (veya üzeri) sürümü.
- Windows / macOS işletim sistemi.

### 9.2. Projeyi Başlatma Adımları

1. Unity Hub üzerinden projeyi açın (`Denemeler` klasörü).
2. `Assets/Drone/Scene` veya `Assets/Scenes` klasörü altındaki demo sahnesini açın.
3. Üst menüdeki **Play** (Oynat) butonuna basın.
4. Arayüzün sol üst kısmındaki **Start** butonuna basarak simülasyonu başlatın.

### 9.3. Kontrol Tuşları

| Tuş / Girdi | İşlev |
| :--- | :--- |
| **W** | Öne Doğru Eğim (Pitch Forward) |
| **S** | Arkaya Doğru Eğim (Pitch Backward) |
| **A** | Sola Doğru Yatma (Roll Left) |
| **D** | Sağa Doğru Yatma (Roll Right) |
| **Q** | Sola Dönüş (Yaw Left) |
| **E** | Sağa Dönüş (Yaw Right) |
| **Space (Boşluk)** | Yükselme (Altitude Up) |
| **Left Control (Sol Ctrl)** | Alçalma (Altitude Down) |
| **Tab** | Kontrol Modu Değiştirme (Manuel <-> AutoHoverPID) |
| **C** | Kamera Modu Değiştirme (3. Şahıs <-> 1. Şahıs FPV) |

### 9.4. Arayüz Butonları
- **Start:** Simülasyonu başlatır veya duraklatılmış simülasyonu devam ettirir.
- **Stop:** Simülasyonu duraklatır (pause).
- **Reset:** Drone'u başlangıç konumuna, hızını ve açılarını sıfıra getirir.

---

## 10. Test Yaklaşımı ve Sonuçlar

Sistem hem birim düzeyinde hem de entegrasyon düzeyinde doğrulama testlerine tabi tutulmuştur:

### 10.1. Birim Testleri (Unit Testing)
- **Serbest Düşme Testi:** İtki sıfırlandığında drone'un sadece yerçekimi ivmesi ($9.81 \text{ m/s}^2$) ile doğru hızlandığı doğrulandı.
- **Denge ve Süzülme Testi:** İtki $m \cdot g$ değerine eşitlendiğinde drone'un irtifasını koruduğu gözlemlendi.
- **İntegratör Karşılaştırma Testi:** Euler ve RK4 yöntemleri aynı senaryoda çalıştırıldı. RK4'ün keskin dönüşlerde daha pürüzsüz ve stabil sonuçlar verdiği doğrulandı.

### 10.2. Entegrasyon Testleri (Integration Testing)
- **Çarpışma ve Respawn Testi:** Drone binalara veya engellere belli bir hızın üzerinde çarptığında kaza modunun tetiklendiği, kontrolün kilitlendiği ve ardından başlangıç konumuna başarıyla resetlendiği doğrulandı.
- **Kamera Geçiş Testi:** `C` tuşuna basıldığında TPS ve FPV kameraları arasında sorunsuz geçiş yapıldığı ve FOV değerlerinin güncellendiği doğrulandı.

---

## 11. Takım Üyeleri ve Görev Dağılımı

- **Zeynep Pelin Çolak (17300009):**
  - Çarpışma algılama ve kaza yönetimi (`DroneCrashHandler`).
  - Görselleştirme kullanıcı arayüzü ve durum panelleri (`SimulationUI`).
  - Proje raporu ve dokümantasyon hazırlığı.

- **Mehmethan Küçük (18330756):**
  - Sistem bileşenlerinin ve mimarisinin oluşturulması.
  - Fizik motoru, diferansiyel denklemler ve sayısal integratörler (`DronePhysics` - Euler/RK4).
  - Kontrolör ve PID algoritmaları (`DroneController`, `SimulationManager`).
  - Proje raporu ve dokümantasyon hazırlığı.

- **Melisa Mehrüboğlu (18330757):**
  - Birim (Unit) ve Entegrasyon (Integration) testlerinin kurgulanması.
  - Test senaryolarının yürütülmesi ve sonuçların analizi.
  - Proje raporu ve dokümantasyon hazırlığı.

---

## 12. Sonuç ve Değerlendirme

Bu projede, teorik gerçek zamanlı sistem kavramları, diferansiyel denklemler ve sayısal integrasyon teknikleri pratik bir 3B drone simülasyonu üzerinde başarıyla uygulanmıştır.

Proje sonucunda:
- Katı zaman kısıtları altında çalışan modüler bir simülasyon mimarisi kurulmuştur.
- 4. Derece Runge-Kutta yöntemi ile yüksek doğrulukta fizik hesaplaması elde edilmiştir.
- Kullanıcıya hem manuel kumanda hem de otomatik süzülme imkanı sunan kararlı PID kontrolörleri geliştirilmiştir.
- FPV ve TPS kamera açıları ile gerçekçi bir uçuş deneyimi sağlanmıştır.

Gelecek geliştirmelerde sisteme otomatik otonom rota takip (waypoint navigation) ve rüzgar/fırtına gibi dış çevresel etmenler dahil edilebilir.
