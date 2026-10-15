# husky_traversability

Segmentacija lidarskog oblaka točaka i Nav2 costmap layer za procjenu prohodnosti terena.

Paket rješava problem standardnih Nav2 slojeva (`ObstacleLayer`, `VoxelLayer`) na neravnom terenu. Ti slojevi odlučuju je li točka prepreka prema apsolutnoj visini u fiksnom koordinatnom sustavu (`min_obstacle_height`, `max_obstacle_height`). Tlo na uzbrdici ispred robota je iznad tog praga, pa ga označe kao prepreku, a podizanje praga sakrije i stvarne niske prepreke. Ovaj paket umjesto toga najprije odvaja tlo od ostatka scene, a zatim cijenu ćelije računa iz nagiba terena, visine stepenice i visine prepreke **iznad lokalnog tla**.

Ciljna platforma: ROS 2 Jazzy, Nav2, Clearpath Husky A200 u Isaac Simu. Ništa u kodu nije vezano uz Husky, osim zadanih vrijednosti parametara.

---

## 1. Pregled sustava

Paket sadrži dvije komponente koje rade u nizu.

```
/lidar/point_cloud  (sim_lidar frame)
        │
        ▼
┌──────────────────────────────┐
│  ground_segmentation_node    │  transform to odom, segment
└──────────────────────────────┘
        │                         │
        ▼                         ▼
/ground_segmentation/ground   /ground_segmentation/obstacles   (odom frame)
        │                         │
        └────────────┬────────────┘
                     ▼
┌──────────────────────────────┐
│  TraversabilityLayer         │  Nav2 costmap plugin
│  (local_costmap,             │  slope / step / obstacle cost
│   global_costmap)            │
└──────────────────────────────┘
                     │  updateWithMax
                     ▼
        master costmap  →  planner, MPPI controller
```

**`ground_segmentation_node`** je samostalni ROS 2 node. Prima sirovi oblak s lidara, transformira ga u koordinatni sustav poravnat s gravitacijom (`odom`) i objavljuje dva oblaka: točke tla i sve ostale točke.

**`TraversabilityLayer`** je Nav2 plugin koji se učitava unutar `local_costmap` i/ili `global_costmap`. Pretplaćuje se na oba oblaka i u costmapu upisuje cijenu prolaska.

Obje komponente su izvedene u dva sloja. Algoritam je u biblioteci koja ovisi samo o Eigenu (`GroundSegmenter`, `TraversabilityGrid`), a ROS dio (`ground_segmentation_node`, `TraversabilityLayer`) je tanki omotač koji čita poruke, radi TF transformacije i objavljuje rezultat. Zato se algoritmi mogu testirati unit testovima bez simulatora i bez Nav2.

---

## 2. Struktura paketa

```
husky_traversability/
├── CMakeLists.txt
├── package.xml
├── traversability_layer_plugin.xml            # pluginlib description of the layer
├── README.md
├── include/husky_traversability/
│   ├── ground_segmenter.hpp                   # segmentation algorithm (Eigen only)
│   ├── traversability_grid.hpp                # cost algorithm (Eigen only)
│   └── traversability_layer.hpp               # Nav2 plugin
├── src/
│   ├── ground_segmenter.cpp
│   ├── ground_segmentation_node.cpp           # ROS 2 node
│   ├── traversability_grid.cpp
│   └── traversability_layer.cpp               # Nav2 plugin + PLUGINLIB_EXPORT_CLASS
├── config/
│   ├── ground_segmentation.yaml               # node parameters
│   └── nav2_traversability_example.yaml       # example Nav2 costmap configuration
├── launch/
│   └── ground_segmentation.launch.py
└── test/
    ├── test_ground_segmenter.cpp
    └── test_traversability_grid.cpp
```

Build proizvodi ove artefakte:

| Artefakt | Vrsta | Namjena |
|---|---|---|
| `libground_segmenter.so` | biblioteka | algoritam segmentacije |
| `libtraversability_grid.so` | biblioteka | izračun cijene |
| `ground_segmentation_node` | izvršna datoteka | ROS 2 node za segmentaciju |
| `libtraversability_layer.so` | biblioteka (plugin) | Nav2 costmap layer |
| `test_ground_segmenter`, `test_traversability_grid` | gtest | unit testovi |

---

## 3. Potrebni paketi

Paket je napisan za ROS 2 Jazzy na Ubuntu 24.04. Ovisnosti su navedene u `package.xml`:

| Ovisnost | Koristi je | apt paket |
|---|---|---|
| `ament_cmake` | build sustav | `ros-jazzy-ament-cmake` |
| `rclcpp`, `rclcpp_lifecycle` | node i plugin | `ros-jazzy-rclcpp`, `ros-jazzy-rclcpp-lifecycle` |
| `sensor_msgs` | `PointCloud2` | `ros-jazzy-sensor-msgs` |
| `tf2`, `tf2_ros`, `tf2_eigen` | transformacije | `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-eigen` |
| `nav2_costmap_2d` | bazna klasa layera | `ros-jazzy-nav2-costmap-2d` |
| `pluginlib` | registracija plugina | `ros-jazzy-pluginlib` |
| `eigen` | linearna algebra | `libeigen3-dev` |
| `launch_ros` | launch datoteka | `ros-jazzy-launch-ros` |
| `ament_cmake_gtest` | testovi | `ros-jazzy-ament-cmake-gtest` |

PCL se ne koristi. Oblaci se čitaju i pišu preko `sensor_msgs::PointCloud2Iterator`.

Najjednostavnije je instalirati sve ovisnosti preko rosdepa:

```bash
cd ~/husky_ws_isaacSim
rosdep install --from-paths src/husky_traversability -y --ignore-src
```

Za rad cijelog sustava potrebni su i Nav2 (`ros-jazzy-navigation2`) te lokalizacija koja objavljuje TF `map → odom → base_link`.

---

## 4. Build i testovi

```bash
conda deactivate        # a conda Python environment breaks Jazzy builds
cd ~/husky_ws_isaacSim
colcon build --packages-select husky_traversability --symlink-install
source install/setup.bash
```

`--symlink-install` preporučujem jer se tada izmjene u `config/` i `launch/` odmah vide bez ponovnog builda. Bez te opcije launch datoteke čitaju kopiju iz `install/`, koja ostaje stara dok ne napraviš novi build. To vrijedi i za Nav2 params file u drugim paketima.

Unit testovi:

```bash
colcon test --packages-select husky_traversability
colcon test-result --verbose
```

`test_ground_segmenter` generira scenu s ravnim tlom, rampom od 20°, platoom, kutijom visokom 0.5 m i zidom od 1 m. Provjerava tri slučaja: robot na ravnom, robot nagnut na rampi (sjeme s nagnutom normalom), i pogrešno postavljeno sjeme, kad se svi oblaci moraju objaviti kao prepreke.

`test_traversability_grid` provjerava izračun cijene na umjetnim oblacima:

1. Ravno tlo je slobodno.
2. Rampa od 17.5° ima srednju cijenu.
3. Rampa od 30° je LETHAL.
4. Stepenica od 0.30 m je LETHAL, a od 0.05 m nije.
5. Prepreka iznad tla je LETHAL.
6. Krivo klasificirane točke koje leže na površini rampe se ignoriraju.
7. Krošnja iznad robota se ignorira.
8. Prepreke bez tla u blizini ponašaju se prema parametru `mark_obstacles_without_ground`.

---

## 5. Teorijska podloga: segmentacija tla

Algoritam je inspiriran radom *GSeg3D: A High-Precision Grid-Based Algorithm for Safety-Critical Ground Segmentation in LiDAR Point Clouds* (Lodhi, Hertzberg, ICRCV 2025). Iz rada su preuzete tri ideje: lokalna PCA klasifikacija u ćelijama mreže, rast regije od sjemena ispod robota i pretraga susjeda po radijusu umjesto samo po susjednim ćelijama. Nekoliko dijelova je promijenjeno (vidi 5.7).

### 5.1 Priprema oblaka

Svaka točka se najprije filtrira po udaljenosti od senzora (`min_range`, `max_range`). `min_range` uklanja odraze od samog robota. Zatim se točka transformira u `target_frame` preko TF-a u trenutku vremenske oznake oblaka.

`target_frame` mora biti poravnat s gravitacijom, tj. njegova z-os mora gledati prema gore. Za `odom` to vrijedi samo ako EKF koji objavljuje `odom → base_link` radi s `two_d_mode: false` i koristi roll i pitch iz IMU-a. Ako je `two_d_mode: true`, TF nema nagib robota, pa je na rampi cijeli oblak zarotiran za kut nagiba i segmentacija krivo procjenjuje nagibe.

### 5.2 Mreža i lokalna klasifikacija

Točke se svrstaju u 2D mrežu s ćelijama veličine `cell_size`. U svakoj ćeliji uzimaju se samo najniže točke: one za koje vrijedi

```
z ≤ z_min + low_band
```

Tako trup stabla, bok kutije ili grm u istoj ćeliji ne kvare procjenu tla. Ćelija s manje od `min_points_per_cell` takvih točaka se ne klasificira.

Za preostale točke računa se kovarijacijska matrica

```
C = (1/N) Σ (p_i − p̄)(p_i − p̄)ᵀ
```

i njezine svojstvene vrijednosti λ₀ ≤ λ₁ ≤ λ₂ sa svojstvenim vektorima v₀, v₁, v₂. Raspored svojstvenih vrijednosti opisuje oblik skupa točaka:

| Uvjet | Tip ćelije | Odluka |
|---|---|---|
| λ₁ < `min_linearity_ratio` · λ₂ | linija (npr. jedan prsten lidara) | kandidat za tlo ako je linija gotovo horizontalna: \|v₂,z\| ≤ sin(`max_slope_deg`). Normala nije poznata. |
| λ₀ / (λ₀+λ₁+λ₂) ≤ `max_planarity` | ravnina | normala n = v₀ (okrenuta prema gore). Kandidat ako je n_z ≥ cos(`max_slope_deg`). |
| inače | nije površina (vegetacija, rubovi objekata) | nije kandidat |

`low_band` mora biti veći od visinske razlike koju sam nagib stvara unutar ćelije, tj. `low_band > cell_size · tan(max_slope_deg)`. Inače se na strmom terenu dio tla odbaci kao "previsok".

### 5.3 Sjeme ispod robota

Rast regije kreće od tla ispod robota. Točka sjemena i njezina normala dobivaju se iz TF-a za `seed_frame`:

```
seed_point  = T_target_seed · (0, 0, seed_z_offset)
seed_normal = R_target_seed · (0, 0, 1)
```

`seed_z_offset` je visina tla ispod `seed_frame` duž njegove z-osi (negativna vrijednost). Normala sjemena je z-os robota, jer je tlo ispod kotača približno ravnina baze robota, i to vrijedi i kad robot stoji na rampi.

Sjemenske ćelije su sve kandidatske ćelije unutar `seed_radius` čija se visina od ravnine robota razlikuje za najviše `seed_height_tolerance`. `seed_radius` mora biti veći od slijepe zone lidara oko robota, jer tamo nema točaka.

Ako se ne nađe nijedna sjemenska ćelija, sve točke se objavljuju kao prepreke i node ispisuje upozorenje. Takvo ponašanje je namjerno sigurno, ali skoro uvijek znači krivi TF ili krivi `seed_z_offset`.

### 5.4 Rast regije s provjerom nagiba

Iz reda se uzima ćelija c i pregledavaju se sve kandidatske ćelije n čiji je centroid na horizontalnoj udaljenosti d ≤ `search_radius`. Pretraga po radijusu premošćuje razmak između prstenova lidara, zbog kojeg obična pretraga susjednih ćelija puca na rjeđim oblacima.

Visina susjeda predviđa se iz ravnine trenutne ćelije:

```
z_pred = c_z − (n_x (x − c_x) + n_y (y − c_y)) / n_z
```

Susjed se prihvaća ako je |n_z − z_pred| ≤ tolerancija, gdje je:

| Trenutna ćelija | Tolerancija |
|---|---|
| ima normalu (ravnina) | `max_step` + d · tan(`max_slope_change_deg`) |
| nema normalu (linija) | `max_step` + d · tan(`max_slope_deg`), uz z_pred = c_z |

Kad je ravnina poznata, dopušta se samo mala promjena nagiba između susjednih dijelova terena. Zato rampa prolazi, a krov kutije ne prolazi iako je ravan.

### 5.5 Označavanje točaka

Za svaku ćeliju odabire se referentna ravnina tla: vlastita, ako je ćelija dosegnuta rastom regije, ili ravnina najbliže dosegnute ćelije unutar `search_radius`. Točka je tlo ako je

```
|z − z_pred(x, y)| ≤ ground_inlier_threshold
```

Sve ostale točke, i sve točke bez referentne ravnine u blizini, su prepreke. Ovaj korak vraća u tlo i točke iz ćelija koje su odbačene jer sadrže i objekt, npr. tlo oko podnožja stabla.

### 5.6 Izlaz

Node objavljuje `~/ground` i `~/obstacles` (puna imena `/ground_segmentation/ground` i `/ground_segmentation/obstacles`). Oba oblaka su u `target_frame`, s istom vremenskom oznakom kao ulazni oblak i poljima `x, y, z` (float32).

Svakih 5 sekundi node ispisuje statistiku:

```
points 28000 | ground 19000 | cells 2100 (candidate 1500, seed 60, ground 1300) | 12.3 ms
```

### 5.7 Razlike u odnosu na GSeg3D

| GSeg3D (rad) | Ovaj paket | Razlog |
|---|---|---|
| 3D mreža, dvije faze s različitom visinom ćelije | 2D mreža, fit na najnižem sloju točaka | isti učinak (objekti ne kvare fit tla) uz jednu fazu |
| Linijske ćelije s kutom ≥ 90° − prag prema z-osi označene kao prepreka (jednadžba 4) | horizontalne linije su kandidati za tlo | jednadžba u radu je obrnuta: horizontalan prsten na tlu je tlo |
| Faza II prihvaća susjede s razlikom visine centroida ≤ τ = 0.125 m | tolerancija raste s udaljenošću i ovisi o ravnini susjeda | fiksni τ implicitno ograničava nagib na oko arctan(0.125) ≈ 7° |
| Nagib se mjeri prema z-osi oblaka | oblak se prvo transformira u okvir poravnat s gravitacijom | na nagnutom robotu nagib bi se mjerio relativno prema robotu |
| Sintetičke točke ispod robota kao sjeme | ravnina robota kao sjeme s tolerancijom visine | sjeme vrijedi i kad je robot na rampi |
| Evaluacija optimizira preciznost tla | cijena u layeru mjeri visinu iznad lokalnog tla | krivo klasificirano tlo ne postaje prepreka (vidi 6.4) |

---

## 6. Teorijska podloga: izračun cijene u layeru

### 6.1 Ulazni podaci

Layer prima oba oblaka, transformira ih u `global_frame` costmape (za `local_costmap` je to obično `odom`, pa transformacija nije potrebna) i sprema ih u dva spremnika.

`observation_persistence` određuje koliko dugo se oblaci zadržavaju: `0.0` znači samo zadnji oblak, a npr. `1.0` spaja sve oblake iz zadnje sekunde. Spajanje popunjava slijepu zonu lidara oko robota dok vozi i smanjuje treperenje prepreka.

### 6.2 Mreža analize

Cijena se računa u vlastitoj mreži s ćelijama `cell_size`, neovisno o rezoluciji costmape. Preporuka je `cell_size` 2 do 5 puta veći od rezolucije costmape, tako da svaka ćelija sadrži dovoljno točaka za stabilan fit. Na kraju se svaka ćelija costmape čiji centar leži unutar ćelije analize oboji njezinom cijenom.

Za svaku ćeliju računaju se četiri veličine.

**Visina tla.** Ćelija ima tlo ako ima barem `min_ground_points` točaka tla. Visina je centroid tih točaka.

**Nagib.** Ravnina se fitta PCA metodom na sve točke tla iz ćelija unutar `slope_radius`, a nagib je

```
α = arccos(n_z)
```

Ako točke leže na liniji (λ₁ < `min_linearity_ratio` · λ₂), nagib poprečno na liniju nije određen i ćelija nema procjenu nagiba.

**Visina stepenice.** Za svaki od 8 susjeda računa se razlika između stvarne visine susjeda i visine koju predviđa ravnina ćelije. Stepenica je najveća takva razlika. Na rampi je razlika blizu nule jer ravnina objašnjava promjenu visine; na rubnjaku ostaje velika. Ako ćelija nema ravninu, od razlike visina oduzima se najveća promjena koju bi objasnio letalni nagib: max(0, |Δz| − d · tan(`slope_lethal_deg`)).

**Prepreke iznad tla.** Za ćeliju s točkama prepreka referentna visina tla je vlastita ravnina ili ravnina najbliže ćelije s tlom unutar `ground_search_radius`. Ćelija je LETHAL ako barem jedna točka prepreke zadovoljava

```
min_obstacle_height ≤ z − h_ground ≤ max_obstacle_height
```

Točke niže od `min_obstacle_height` (trava, šum) i više od `max_obstacle_height` (krošnje, grane) se ignoriraju. Ako ćelija nema tlo u blizini, ponašanje određuje `mark_obstacles_without_ground`.

### 6.3 Konačna cijena

| Stanje ćelije | Cijena |
|---|---|
| prepreka iznad tla | 254 (LETHAL) |
| stepenica > `max_step_height` | 254 (LETHAL) |
| nagib ≥ `slope_lethal_deg` | 254 (LETHAL) |
| `slope_safe_deg` < nagib < `slope_lethal_deg` | linearno od 1 do `max_slope_cost` |
| nagib ≤ `slope_safe_deg` | 0 (FREE) |
| nema podataka | 255 (NO_INFORMATION) |

`max_slope_cost` je ograničen na najviše 252, ispod vrijednosti *inscribed* (253). Zbog toga strm, ali prohodan teren nije prepreka: planer i MPPI ga izbjegavaju kad postoji lakši put, ali ga mogu koristiti. Inflation layer širi samo LETHAL ćelije, pa se cijena nagiba ne napuhuje.

### 6.4 Zašto relativna visina rješava problem uzbrdice

Zamisli točku na rampi 0.35 m iznad visine na kojoj robot stoji. `ObstacleLayer` s `min_obstacle_height: 0.20` označi je kao prepreku. `TraversabilityLayer` je uspoređuje s lokalnom ravninom rampe, prema kojoj je njezina visina približno 0 m, pa je ignorira čak i ako ju je segmentacija krivo proglasila preprekom. Cijenu te ćelije tada određuje samo nagib rampe.

### 6.5 Upis u glavnu costmapu

Layer nasljeđuje `nav2_costmap_2d::CostmapLayer`, dakle ima vlastitu kartu iste veličine kao glavna. U svakom ciklusu costmape:

1. **`updateBounds()`:** kod rolling window karte pomakne ishodište vlastite karte. Ako su stigli novi oblaci, preračuna mrežu, obriše područje obojeno u prošlom ciklusu, oboji novo i javi Nav2-u uniju ta dva područja.
2. **`updateCosts()`:** spoji vlastitu kartu s glavnom metodom `updateWithMax`. Ćelija u glavnoj karti dobiva veću od dvije cijene, a ćelije s NO_INFORMATION u layeru ne mijenjaju glavnu kartu.

Posljedica `updateWithMax` je važna za konfiguraciju: ako neki drugi layer u istoj costmapi obilježi uzbrdicu kao LETHAL, ta cijena ostaje, bez obzira na ovaj layer.

### 6.6 Dretve

Pretplate rade u callback grupi costmape (`callback_group_`), koju Nav2 izvršava u zasebnoj dretvi. Callbackovi samo transformiraju oblake i spremaju ih u spremnike zaštićene mutexom. Sav izračun radi se u dretvi ažuriranja costmape, u `updateBounds()`.

---

## 7. Kako se plugin registrira u Nav2

Nav2 učitava layere preko pluginliba, po imenu klase iz params filea. Da bi se klasa našla, četiri stvari moraju biti usklađene.

**1. Makro u izvornom kodu** (`src/traversability_layer.cpp`) registrira klasu kao implementaciju sučelja `nav2_costmap_2d::Layer`:

```cpp
PLUGINLIB_EXPORT_CLASS(husky_traversability::TraversabilityLayer, nav2_costmap_2d::Layer)
```

**2. Opis plugina** (`traversability_layer_plugin.xml`) povezuje ime klase s bibliotekom:

```xml
<library path="traversability_layer">
  <class type="husky_traversability::TraversabilityLayer" base_class_type="nav2_costmap_2d::Layer">
    <description>...</description>
  </class>
</library>
```

**3. CMake** upisuje opis u ament index pod paketom bazne klase:

```cmake
pluginlib_export_plugin_description_file(nav2_costmap_2d traversability_layer_plugin.xml)
```

Nakon builda nastaje datoteka

```
install/husky_traversability/share/ament_index/resource_index/nav2_costmap_2d__pluginlib__plugin/husky_traversability
```

Nav2 je pronalazi samo ako je workspace sourcan u terminalu iz kojeg se Nav2 pokreće.

**4. Params file** koristi isto ime klase:

```yaml
plugin: "husky_traversability::TraversabilityLayer"
```

Provjera koje layere pluginlib vidi (pokrenuti u terminalu iz kojeg se pokreće Nav2):

```bash
python3 -c "from ament_index_python import get_resources; print(sorted(get_resources('nav2_costmap_2d__pluginlib__plugin')))"
```

---

## 8. Pokretanje

### 8.1 Redoslijed

1. Isaac Sim (lidar, IMU, odometrija, `/clock`)
2. lokalizacija (`robot_localization`: `odom → base_link`, `map → odom`)
3. segmentacija
4. Nav2

### 8.2 Segmentacija

```bash
ros2 launch husky_traversability ground_segmentation.launch.py
```

Launch argumenti:

| Argument | Zadano | Opis |
|---|---|---|
| `params_file` | `share/husky_traversability/config/ground_segmentation.yaml` | parametri nodea |
| `use_sim_time` | `true` | koristiti `/clock` iz simulatora |

Isti node može se uključiti i u vlastiti launch:

```python
Node(
    package='husky_traversability',
    executable='ground_segmentation_node',
    name='ground_segmentation',
    output='screen',
    parameters=[params_file, {'use_sim_time': True}],
)
```

### 8.3 Provjera prije pokretanja Nav2

```bash
ros2 topic hz /lidar/point_cloud
ros2 topic echo /lidar/point_cloud --field header.frame_id --once
ros2 run tf2_ros tf2_echo odom sim_lidar
ros2 run tf2_ros tf2_echo base_link base_footprint     # ground height under base_link
ros2 topic hz /ground_segmentation/ground
```

### 8.4 Vizualizacija u RViz-u

Pokreni RViz sa `ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true` i dodaj ove displayeve:

| Display | Topic | Postavke |
|---|---|---|
| PointCloud2 | `/ground_segmentation/ground` | FlatColor zelena, Size 0.05 |
| PointCloud2 | `/ground_segmentation/obstacles` | FlatColor crvena, Size 0.05 |
| Map | `/local_costmap/costmap` | Color Scheme: costmap |
| Map | `/global_costmap/costmap` | Color Scheme: costmap |

Fixed Frame postavi na `odom` za segmentaciju i lokalnu kartu, a na `map` za globalnu.

---

## 9. Uključivanje u Nav2

### 9.1 Local costmap

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      update_frequency: 20.0
      publish_frequency: 20.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 8
      height: 8
      resolution: 0.1
      footprint: "[ [0.55, 0.45], [0.55, -0.45], [-0.55, -0.45], [-0.55, 0.45] ]"
      always_send_full_costmap: True
      plugins: ["traversability_layer", "inflation_layer"]

      traversability_layer:
        plugin: "husky_traversability::TraversabilityLayer"
        enabled: true
        ground_topic: /ground_segmentation/ground
        obstacles_topic: /ground_segmentation/obstacles
        observation_persistence: 1.0
        data_timeout: 0.0
        transform_tolerance: 0.2
        cell_size: 0.25
        min_ground_points: 3
        slope_radius: 0.75
        min_linearity_ratio: 0.05
        slope_safe_deg: 10.0
        slope_lethal_deg: 25.0
        max_slope_cost: 200
        max_step_height: 0.15
        min_obstacle_height: 0.15
        max_obstacle_height: 1.0
        ground_search_radius: 1.0
        mark_obstacles_without_ground: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7
```

### 9.2 Global costmap

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_link
      footprint: "[ [0.55, 0.45], [0.55, -0.45], [-0.55, -0.45], [-0.55, 0.45] ]"
      resolution: 0.2
      rolling_window: True
      width: 100
      height: 100
      track_unknown_space: false
      always_send_full_costmap: True
      plugins: ["traversability_layer", "inflation_layer"]

      traversability_layer:
        plugin: "husky_traversability::TraversabilityLayer"
        enabled: true
        ground_topic: /ground_segmentation/ground
        obstacles_topic: /ground_segmentation/obstacles
        observation_persistence: 3.0
        data_timeout: 0.0
        transform_tolerance: 0.5
        cell_size: 0.4
        min_ground_points: 3
        slope_radius: 0.8
        min_linearity_ratio: 0.05
        slope_safe_deg: 10.0
        slope_lethal_deg: 25.0
        max_slope_cost: 200
        max_step_height: 0.15
        min_obstacle_height: 0.15
        max_obstacle_height: 1.0
        ground_search_radius: 1.0
        mark_obstacles_without_ground: false

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 3.0
        inflation_radius: 1.5
```

Globalna karta ima grublje ćelije, dulje pamti oblake i ne označava prepreke bez tla u blizini, jer daleko od robota lidar ima rijetke prstenove koji bi inače ostavljali lažne LETHAL linije.

### 9.3 Pravila konfiguracije

**Lidar ne smije istovremeno ulaziti u `ObstacleLayer` ili `VoxelLayer` u istoj costmapi.** Zbog `updateWithMax` bi taj layer i dalje zadržavao uzbrdicu kao LETHAL. Drugi senzori (npr. dubinska kamera) mogu ostati u tim layerima.

**Remap topica nije potreban.** Layer koristi apsolutna imena topica iz parametara `ground_topic` i `obstacles_topic`.

**Parametri se čitaju samo pri pokretanju.** Nakon izmjene yamla treba restartati Nav2. Promjena parametara preko `ros2 param set` nema učinka.

**Nav2 mora čitati uređeni params file.** Ako se params file nalazi u paketu buildanom bez `--symlink-install`, Nav2 čita staru kopiju iz `install/`.

### 9.4 Provjera nakon pokretanja

U logu Nav2 za svaku costmapu moraju biti ove linije:

```
[local_costmap.local_costmap]: Using plugin "traversability_layer"
[local_costmap.local_costmap.traversability_layer]: TraversabilityLayer: ground '/ground_segmentation/ground', ... slope safe/lethal 10.0/25.0 deg, step 0.15 m
[local_costmap.local_costmap]: Initialized plugin "traversability_layer"
```

Zatim provjeri pretplate i parametre:

```bash
ros2 topic info /ground_segmentation/ground -v           # one subscription per costmap
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /local_costmap/local_costmap traversability_layer.slope_lethal_deg
```

---

## 10. Parametri

### 10.1 `ground_segmentation_node`

| Parametar | Zadano | Jedinica | Opis |
|---|---|---|---|
| `input_topic` | `/lidar/point_cloud` | | ulazni oblak |
| `target_frame` | `odom` | | izlazni okvir, mora biti poravnat s gravitacijom |
| `seed_frame` | `base_link` | | okvir robota za sjeme |
| `seed_z_offset` | −0.13 | m | visina tla ispod `seed_frame` duž njegove z-osi |
| `min_range` | 0.8 | m | odbacuje točke bliže senzoru (odrazi od robota) |
| `max_range` | 30.0 | m | odbacuje daleke točke |
| `tf_timeout` | 0.1 | s | čekanje na TF |
| `cell_size` | 0.5 | m | veličina ćelije mreže |
| `low_band` | 0.35 | m | debljina najnižeg sloja točaka za fit tla |
| `min_points_per_cell` | 5 | | minimalan broj točaka za klasifikaciju ćelije |
| `max_slope_deg` | 25.0 | ° | najstrmija ravnina koja može biti tlo |
| `max_planarity` | 0.05 | | λ₀/Σλ, veće vrijednosti prihvaćaju neravnije površine |
| `min_linearity_ratio` | 0.05 | | λ₁/λ₂ ispod kojeg je ćelija linija |
| `max_step` | 0.15 | m | osnovna tolerancija visine između ćelija tla |
| `max_slope_change_deg` | 10.0 | ° | dopuštena promjena nagiba između susjednih ćelija |
| `search_radius` | 1.5 | m | radijus pretrage susjeda |
| `seed_radius` | 4.0 | m | radijus traženja sjemena, veći od slijepe zone lidara |
| `seed_height_tolerance` | 0.25 | m | odstupanje sjemenske ćelije od ravnine robota |
| `ground_inlier_threshold` | 0.10 | m | najveća udaljenost točke od ravnine tla |

### 10.2 `TraversabilityLayer`

| Parametar | Zadano | Jedinica | Opis |
|---|---|---|---|
| `enabled` | true | | uključuje layer |
| `ground_topic` | `/ground_segmentation/ground` | | oblak tla |
| `obstacles_topic` | `/ground_segmentation/obstacles` | | oblak prepreka |
| `observation_persistence` | 0.0 | s | koliko dugo se oblaci spajaju; 0 = samo zadnji |
| `data_timeout` | 0.0 | s | briše layer i javlja "not current" ako oblaci ne stižu; 0 = isključeno |
| `transform_tolerance` | 0.2 | s | čekanje na TF do `global_frame` |
| `cell_size` | 0.25 | m | ćelija analize, 2–5× rezolucija costmape |
| `min_ground_points` | 3 | | točke tla potrebne za visinu ćelije |
| `slope_radius` | 0.75 | m | radijus fita ravnine za nagib |
| `min_linearity_ratio` | 0.05 | | λ₁/λ₂ ispod kojeg nagib nije određen |
| `slope_safe_deg` | 10.0 | ° | nagib bez cijene |
| `slope_lethal_deg` | 25.0 | ° | nagib od kojeg je ćelija LETHAL |
| `max_slope_cost` | 200 | | cijena neposredno ispod letalnog nagiba (1–252) |
| `max_step_height` | 0.15 | m | stepenica od koje je ćelija LETHAL |
| `min_obstacle_height` | 0.15 | m | najniža visina prepreke iznad lokalnog tla |
| `max_obstacle_height` | 1.0 | m | najviša visina prepreke iznad lokalnog tla |
| `ground_search_radius` | 1.0 | m | radijus traženja tla za ćelije bez vlastitog tla |
| `mark_obstacles_without_ground` | true | | LETHAL za prepreke bez tla u blizini |

### 10.3 Smjernice za podešavanje

`slope_lethal_deg` i `max_step_height` treba postaviti prema stvarnim mogućnostima robota, izmjerenima u simulaciji ili na terenu, a ne prema zadanim vrijednostima. `max_obstacle_height` postavi na visinu robota s lidarom plus sigurnosnu marginu, da grane koje robot može proći ispod ne blokiraju put.

Između parametara postoje ovisnosti. U segmentaciji treba vrijediti `low_band > cell_size · tan(max_slope_deg)`, a `max_slope_deg` u segmentaciji treba biti barem jednak `slope_lethal_deg` u layeru, jer strmije površine inače nikad ne stignu do layera kao tlo. `search_radius` mora premostiti razmak između susjednih prstenova lidara na udaljenostima koje su važne za navigaciju; razmak raste približno s kvadratom udaljenosti, pa `max_range` ima smisla ograničiti na doseg u kojem se prstenovi još spajaju.

---

## 11. Testiranje u simulaciji

Prije rada u realističnom okruženju isplati se napraviti kontroliranu scenu u Isaac Simu: rampe od 10°, 15°, 20°, 25° i 30°, rubnjak od 0.15–0.20 m, kutiju visoku 0.3–0.5 m i prepreku s prolazom ispod (npr. greda na 1.5 m).

| Test | Očekivani rezultat |
|---|---|
| robot miruje ispred rampi | rampe ≤ 10° slobodne, 15–20° srednja cijena, ≥ 25° LETHAL |
| teleop na rampu od 15° | rampa ostaje prohodna i kad je robot nagnut |
| rubnjak i kutija | LETHAL s inflacijom |
| greda na 1.5 m | nije prepreka |
| Nav2 cilj na vrhu rampe od 15° | robot se popne |
| Nav2 cilj iza rampe od 30° | planer je zaobiđe |

---

## 12. Rješavanje problema

| Simptom | Uzrok | Rješenje |
|---|---|---|
| `class husky_traversability::TraversabilityLayer ... does not exist` | pluginlib ne vidi ament index datoteku | `source install/setup.bash` u terminalu iz kojeg se pokreće Nav2; provjeri datoteku iz poglavlja 7; čisti build paketa |
| `Parameter not set` za parametre layera | plugin nije učitan ili Nav2 čita drugi params file | `ros2 param get .../local_costmap plugins`; provjeri log; rebuild paketa s params fileom |
| `Lifecycle node ... does not have error state implemented` | posljedica neuspjelog configurea | pogledaj prethodnu ERROR poruku u logu |
| `No ground found around the robot` | krivi TF, `seed_frame` ili `seed_z_offset` | `tf2_echo odom base_link`, provjeri visinu tla ispod `base_link` |
| `TF ... failed` u nodeu ili layeru | nema TF lanca ili sim time nije uključen | `use_sim_time: true` na svim nodeovima, provjeri `tf2_echo` |
| uzbrdica i dalje LETHAL | lidar još ulazi u drugi layer iste costmape | makni lidar iz `ObstacleLayer` / `VoxelLayer` |
| LETHAL samo na prijelazu ravno → rampa | stepenica zbog promjene nagiba | povećaj `max_step_height` ili `slope_radius` |
| cijela rampa krivo nagnuta ili LETHAL dok je robot nagnut | EKF u `two_d_mode: true` | `two_d_mode: false`, IMU roll/pitch u `imu0_config` |
| trava i neravan teren su prepreke | preniska tolerancija ravnosti ili visine | povećaj `max_planarity` u segmentaciji i `min_obstacle_height` u layeru |
| tlo daleko od robota nestaje | razmak između prstenova lidara | povećaj `search_radius`, smanji `max_range` |
| lažne LETHAL ćelije daleko | točke bez tla u blizini | `mark_obstacles_without_ground: false` |
| prepreke trepere, slijepa zona uz robota prazna | layer koristi samo zadnji oblak | povećaj `observation_persistence` |
| niske kutije postaju tlo | tolerancije segmentacije prevelike | smanji `max_step` i `ground_inlier_threshold` |
| visoko opterećenje CPU-a | gusti oblak ili sitne ćelije | veći `cell_size`, manji `max_range`, niži `update_frequency` globalne karte |

---

## 13. Ograničenja

**Negativne prepreke se ne detektiraju.** Točke ispod lokalnog tla (jarci, rupe, rubovi padina) layer ignorira, jer uzima u obzir samo visinu iznad tla. Jarak koji lidar ne vidi izravno ostaje bez podataka.

**Globalni layer nema dugoročnu memoriju.** Prepreka koja nije u vidnom polju dulje od `observation_persistence` nestaje iz globalne karte. Lokalni layer je ponovno otkrije kad joj se robot približi, pa planer preplanira.

**Vegetacija se tretira geometrijski.** Visoka trava iznad `min_obstacle_height` je prepreka, iako je robot možda može prijeći. Rješenje bi zahtijevalo semantičku informaciju, npr. iz segmentacije kamere.

**Hrapavost terena ne ulazi u cijenu.** Kamenje manje od `max_step_height` ne povećava cijenu.

**Parametri se ne mogu mijenjati tijekom rada.** Potreban je restart nodea ili Nav2.

**Rijedak lidar ograničava doseg.** Daleko od robota prstenovi su preudaljeni za rast regije, pa se tlo tamo ne prepoznaje.

**Pretpostavlja se okvir poravnat s gravitacijom.** Bez nagiba robota u TF-u rezultati na nagnutom terenu nisu ispravni.

---

## 14. Mogući nastavak rada

1. Memorija prepreka u globalnom layeru, npr. mreža koja pamti zadnju poznatu cijenu s postupnim zaboravljanjem.
2. Detekcija negativnih prepreka iz točaka ispod ravnine tla i iz nedostatka povrata lidara.
3. Hrapavost terena (rezidual fita ravnine) kao dodatni doprinos cijeni.
4. Dinamička promjena parametara preko `add_on_set_parameters_callback`.
5. Objava debug topica s nagibom i visinom stepenice po ćeliji za lakše podešavanje.
6. Spajanje sa semantičkom segmentacijom kamere, kako bi se razlikovala vegetacija i staza od stvarnih prepreka.
