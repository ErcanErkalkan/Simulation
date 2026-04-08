# Heterogeneous Network Simulation

Bu proje, simülasyon çekirdeğini algoritmalardan ayıran paket yapısına sahiptir.

## Klasör Yapısı

- `simulation_core/`: çekirdek motor, config, environment ve algoritma API
- `simulation_app/`: domain nesneleri, UI, yardımcı modüller ve donanım adaptörleri
- `algorithms/`: yerleşik algoritmalar
- `user_algorithms/`: kullanıcı tarafından eklenen algoritmalar
- `tests/`: regresyon testleri

## Çalıştırma

Arayüz:

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

## Notlar

- UI içindeki `Speed` alanı, UAV'lerin her simülasyon adımında kaç birim ilerleyeceğini belirler.
- CoDrone için `codrone-edu`, Tello için `djitellopy` gerekir.
