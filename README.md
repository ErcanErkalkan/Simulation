# Heterogeneous Network Simulation

Bu proje, UAV ve hedef nesneleri uzerinde calisan bir simülasyon cekirdegi ile
eklenebilir algoritma mimarisini ayirir.

## Klasor yapisi

- `simulation_core/`: cekirdek simülasyon motoru, algoritma arayuzu, registry
- `simulation_app/`: domain nesneleri, yardimci moduller, UI, donanim adaptörleri
- `algorithms/`: yerlesik algoritmalar
- `user_algorithms/`: kullanici tarafindan eklenen algoritmalar
- `tests/`: `unittest` tabanli regresyon testleri

## Calistirma

Arayuz:

```bash
python ground_main.py
```

veya kurulumdan sonra:

```bash
simulation-ui
```

Testler:

```bash
python -m unittest -v
```

## Yeni algoritma ekleme

Yeni bir algoritma icin `user_algorithms/` altina bir `*.py` dosyasi ekleyin.
Kullanilacak sabit arayuz `simulation_api.py` icindedir.

Hazir ornek:

- `user_algorithms/example_user_algorithm.py`

## Opsiyonel donanim bagimliliklari

- CoDrone icin: `codrone-edu`
- Tello icin: `djitellopy`, `opencv-python`
