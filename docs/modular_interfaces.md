# Модульные интерфейсы GUI / интегратора / автопилота

## Цель

Подготовить проект к отделению автопилота и/или оценщика положения в отдельный модуль без переделки GUI, UART и визуализации трека.

Стабильные обязанности, которые должны остаться внутри Python GUI:

- редактирование и отрисовка трека;
- настройки COM/UART и логирование;
- прием и разбор D0/D1/D2/F0;
- хранение и редактирование параметров геометрии, скоростей, ускорений и PWM-калибровки;
- выбор источника позы для отрисовки;
- выбор алгоритма автопилота;
- маршрутизация команд в `C0`.

## Текущие блоки проекта

| Блок | Текущая реализация | Роль |
| --- | --- | --- |
| UART transport | `comm/serial_worker.py` | Жизненный цикл порта, RX/TX очереди, поток чтения, метрики |
| Протокол | `comm/protocol.py` | Парсинг/сборка `D0/D1/D2/B0/F0/C0/A0/A1` |
| Синхронизация времени | `comm/time_sync.py` | Модель `t_mcu ≈ a * t_pc + b`, единый timestamp для `C0` |
| Главный runtime GUI | `app/gui_app.py` | Управление режимами, маршрутизация RX/TX, логирование, time sync |
| Планирование траектории и визуализация | `app/coordinate_tab.py` | Редактор трека, профиль скоростей, внутренняя интеграция позы, выбор режимов |
| Встроенный автопилот | `utils/pure_pursuit_controller.py` | Built-in алгоритм сопровождения трека |

## Точки стыковки

В коде выделены 4 стабильных контракта из `runtime/contracts.py`.

### 1. `MissionConfig`

Передается из GUI в любой модуль интегратора/автопилота при старте миссии.

Содержит:

- `track_points`: полилиния трека в мировых координатах;
- `geometry`: `a1`, `a2`, ширина базы, длина окружности гусеницы;
- `motion`: целевая скорость, ускорение, торможение, `dt`, минимальный радиус;
- `builtin_tuning`: текущие коэффициенты встроенного `Pure Pursuit`;
- `speed_map`: таблица `PWM <-> speed`;
- `pwm_correction`: shift/linear для левой и правой гусеницы;
- `pose_source`: `internal | trolley | external`;
- `autopilot`: `builtin_pure_pursuit | external`;
- `telemetry_subscription`: список телеметрических каналов, которые должны поступать наружу.

### 2. `TelemetrySnapshot`

Передается из RX-контура во внешний модуль после приема новых данных.

Содержит:

- `sync`: состояние time sync, текущий `pc_time_ms`, `mcu_rx_ms`, `mcu_est_ms`, коэффициенты `a` и `b`;
- `imu`: последний `D0`;
- `tacho`: последний `D1`;
- `motor`: последний `D2`.

Это базовый пакет, через который наружу уже можно отдавать:

- данные о треке через `MissionConfig`;
- показания телеги через `TelemetrySnapshot`;
- системные настройки через `MissionConfig.geometry`, `MissionConfig.motion`, `MissionConfig.speed_map`, `MissionConfig.pwm_correction`.

### 3. `PoseEstimate`

Возвращается от выбранного оценщика положения в GUI.

Содержит:

- `x_m`, `y_m`, `theta_rad`;
- `source`: `internal | trolley | external`;
- `pc_time_ms`;
- `mcu_time_ms` при наличии.

GUI использует этот контракт для отрисовки положения телеги без знания внутренней реализации оценщика.

### 4. `DriveCommand`

Возвращается от автопилота в GUI перед отправкой в UART.

Содержит:

- `left_pwm`, `right_pwm`;
- `duration_ms`;
- `source`;
- `created_pc_ms`.

Важно: значения в `DriveCommand` трактуются как команды для физических левой/правой гусеницы до аппаратного swap. Swap для coordinate mode остается на стороне GUI перед отправкой `C0`.

## Интерфейсы модулей

### `PoseEstimator`

Минимальный контракт для внешнего или встроенного оценщика положения:

1. `apply_mission(mission)`
2. `reset()`
3. `update_telemetry(snapshot) -> PoseEstimate | None`

### `AutopilotController`

Минимальный контракт для внешнего или встроенного автопилота:

1. `apply_mission(mission)`
2. `reset()`
3. `update(snapshot, pose, dt_s) -> DriveCommand | None`
4. `is_finished() -> bool`
5. `stop()`

## Что уже сделано в коде

- В `CoordinateTab` добавлен независимый выбор:
  - источника позы: `Internal integrator`, `Trolley data`, `External estimator`;
  - автопилота: `Built-in Pure Pursuit`, `External autopilot`.
- Состояние этих режимов сохраняется в `app_settings.json` в каноническом виде.
- `CoordinateTab.build_mission_config()` собирает типизированный пакет миссии.
- `VirtualControllerApp.build_telemetry_snapshot()` собирает типизированный снимок телеметрии.
- Добавлены точки подключения:
  - `VirtualControllerApp.set_external_pose_provider(...)`
  - `VirtualControllerApp.set_external_autopilot(...)`
- При выборе внешних режимов GUI теперь работает через контракты `PoseEstimator` и `AutopilotController`.
- Внутренний интегратор GUI может обновляться и от внешнего `DriveCommand`, а не только от встроенного `Pure Pursuit`.
- Все отправки `C0` переведены на единый timestamp через `comm.time_sync.control_timestamp_u32(...)`.

## Где стыковать будущий C++ модуль

Есть два безопасных пути.

### Вариант 1. Только внешний автопилот

- GUI оставляет внутренний или trolley pose source.
- Внешний модуль реализует только `AutopilotController`.
- На вход получает `MissionConfig` и `TelemetrySnapshot`.
- На выход отдает `DriveCommand`.

### Вариант 2. Внешний оценщик положения + внешний автопилот

- GUI выбирает `External estimator` и `External autopilot`.
- Внешний модуль реализует оба интерфейса: `PoseEstimator` и `AutopilotController`.
- GUI остается только редактором, визуализатором, транспортом и маршрутизатором.

## Рекомендация по пакетам для внешнего блока

Для первого рабочего контура наружу стоит передавать:

- `MissionConfig.track_points`
- `MissionConfig.geometry`
- `MissionConfig.motion`
- `MissionConfig.speed_map`
- `MissionConfig.pwm_correction`
- `TelemetrySnapshot.tacho`
- `TelemetrySnapshot.motor`
- `TelemetrySnapshot.imu`
- `TelemetrySnapshot.sync`

Минимальный practical subset для быстрого старта внешнего автопилота:

- трек;
- `D1` тахо;
- геометрия телеги;
- speed map;
- текущий sync state.

Если внешний модуль будет сам оценивать позу по IMU, тогда `D0` надо считать обязательным.
