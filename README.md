# Drone_Sekil_Algilama🚁
ROS Noetic ve Gazebo 11 kullanılarak geliştirilen otonom drone görev simülasyonu. Drone, kamera görüntüsünden kırmızı üçgen ve mavi altıgen tespit ederek şekle bağlı aksiyonlar uygular.

🟥 Kırmızı Üçgen tespit edilince: hedefi görüntü merkezine stabil şekilde ortalar, ardından LAND ile tam üstüne iniş yapar.

🟦 Mavi Altıgen tespit edilince: hedefi stabil ortalar, ardından 3 metreye alçalır, 5 saniye bekler, tekrar 10 metreye çıkar ve aramaya devam eder.

Algılama: HSV renk filtresi + kontur analizi

Kontrol: DroneKit + MAVLink velocity komutları ile visual servo merkezleme
.

### 🎯 Görev Senaryosu (Özet)

Drone 10 metre irtifaya kalkar. Belirli iki noktaya seyrüsefer yapar. Kuzey yönünde tarama yapar. Şekil bulunursa ilgili aksiyonu uygular:

Kırmızı Üçgen → merkezle → LAND

Mavi Altıgen → merkezle → 3m in → 5sn bekle → 10m çık → devam



### 🧰 Gereksinimler

Ubuntu (ROS Noetic uyumlu)

ROS Noetic

Gazebo 11

ArduPilot SITL + Gazebo Iris

Python 3

requirements.txt (repo içinde)


### ▶️ Çalıştırma (Adım Adım)
Çalıştırma sırası önemli:
Gazebo world

ArduPilot SITL

Python ROS node

### ✅ A) Gazebo’yu görev alanı dünyası ile aç

gorev_alani.world : Bu dünya simülasyona eklediğim şekillerle birlikte açılır.

roslaunch gazebo_ros empty_world.launch world_name:=/home/kullanici_adiniz/.../gorev_alani.world

Not: Lütfen /home/ dizinini kendi sisteminize göre güncelleyin.
<img width="604" height="554" alt="gorev_alani world" src="https://github.com/user-attachments/assets/e56b5a01-cdd7-4774-8776-6a90f31d53cf" />

### ✅ B) ArduPilot SITL’i başlat

cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map

### ✅ C) Python görev scriptini çalıştır
Önce ROS’u source ettiğinden emin ol:
source /opt/ros/noetic/setup.bash

Sonra script:
python3 src/Otonom_Gorev_ve_Sekil_Aksiyonu.py

### 🔧 Önemli Parametreler (Kod İçinden)
Kodun başındaki ayarlar:

ArduPilot bağlantısı:
CONNECTION_STRING = "127.0.0.1:14550"

Kamera topic:
CAM_TOPIC = "/iris_demo/gimbal/image_raw"

-------------------------------
🧠 Algoritma (Çalışma Mantığı)
--------------------------------
Bu proje, Gazebo simülasyonundaki drone kamerasından alınan görüntüleri işleyerek yerdeki renkli geometrik şekilleri tespit eder ve tespit edilen hedef üzerinde hassas konumlanma (visual servo) uygulayarak görev aksiyonlarını gerçekleştirir.

### 1) 🚀Görüntü Alma (ROS → OpenCV)

Drone kamerasından gelen görüntü, ROS üzerinden /iris_demo/gimbal/image_raw topic’ine yayınlanır.

Python tarafında CvBridge kullanılarak bu görüntü OpenCV formatına (BGR) dönüştürülür.

Her döngüde en güncel kare (frame) alınarak algılama işlemi gerçek zamanlı yapılır.


### 2) 📚Renk Tabanlı Ayırma (HSV Maskeleme)

Görüntü HSV renk uzayına çevrilir.
Kırmızı ve mavi renkler için ayrı eşik aralıkları kullanılarak iki maske üretilir:


Maskelerdeki gürültüyü azaltmak ve şekli daha düzgün hale getirmek için morfolojik işlemler uygulanır:

Open (açma): küçük gürültü noktalarını temizler

Close (kapama): kopuk bölgeleri birleştirir



### 3) ⚙️Şekil Tespiti (Kontur + Köşe Sayısı)

Her renk maskesi için findContours ile konturlar çıkarılır.

En büyük kontur seçilerek “hedef aday kontur” belirlenir (alanı küçük olanlar elenir).

Kontur approxPolyDP ile sadeleştirilir ve köşe sayısı hesaplanır:

### 🎥 3 köşe → 🟥 Kırmızı Üçgen
<img width="1846" height="862" alt="kirmizi_ucgen" src="https://github.com/user-attachments/assets/45fec3e9-45e3-4c23-8c98-7d72f748a261" />

### 🎥 6 veya 7 köşe → 🟦 Mavi Altıgen (simülasyon/kenar yumuşaması nedeniyle 7 köşe toleransı var)
<img width="1840" height="823" alt="mavi_altigen" src="https://github.com/user-attachments/assets/6194838f-54ce-4fb1-b3cc-515d53e4750d" />


Şeklin merkezi, konturun momentleriyle hesaplanır.


### 4) 🔍Hedefe Merkezleme (Visual Servo Kontrol)
Tespit edilen şeklin merkezi, görüntü merkezine oturtulana kadar drone’a hız komutu gönderilir.

Hata hesabı:

err_x = hedef_x - görüntü_merkez_x

err_y = hedef_y - görüntü_merkez_y


Bu hata, normalize edilerek drone’un LOCAL_NED hız komutlarına çevrilir:

Hedef sağdaysa drone doğuya,
Hedef aşağıdaysa drone güney yönüne hareket edecek şekilde hız vektörü üretilir.



### 5) 📍Stabil Merkez Onayı (Erken Tetiklemeyi Engelleme)
Sistemin “hedefi görür görmez” aksiyona geçmesini önlemek için iki güvenlik kriteri kullanılır:


Sıkı merkez penceresi (tight window)
Hedefin merkezinin, görüntü merkezine CENTER_TIGHT_PX toleransı içinde olması gerekir.


Ardışık kare doğrulaması (stable frames)
Hedef bu sıkı pencerede üst üste N kare (CENTER_STABLE_FRAMES) kalmadan “tam ortalandı” sayılmaz.


Alan oranı eşiği (min area fraction)
Şekil uzaktayken kadrajda küçük görünür. Bu nedenle:

kontur_alani / frame_alani oranı belirli bir eşikten küçükse, sistem hedefi “merkezde değil” kabul eder.
Böylece uzaktan görülen hedeflerde erken iniş/aksiyon engellenir.
<img width="981" height="650" alt="LAND" src="https://github.com/user-attachments/assets/235f6134-71f3-4291-9189-324c90db59ed" />
