# Модульные интерфейсы GUI и внешнего C++ runtime

## Цель

Подготовить проект к выносу всей бизнес-логики в единый внешний C++ runtime без переделки GUI, UART и визуализации.

Что должно остаться внутри Python GUI:

- редактирование и отрисовка трека;
- настройки COM/UART и логирование;
- прием и разбор `D0/D1/D2/F0`;
- хранение и редактирование геометрии, speed map, ускорений и PWM-калибровки;
- выбор источника позы и алгоритма автопилота;
- маршрутизация команд в `C0`.

Что должно уйти во внешний C++ runtime:

- интегратор позы;
- фильтры и обработка входных скоростей;
- автопилот;
- внутренняя публикация событий и логгер runtime;
- вычисление выходных скоростей/команд.

## Текущая целевая схема

Внешний модуль больше не рассматривается как два независимых блока `pose provider` и `autopilot`.

Целевой вариант такой:

1. GUI один раз передает `MissionConfig` при старте миссии.
2. По приходу `D1` тахо GUI отправляет `TelemetrySnapshot` как event-trigger.
3. Внутри C++ runtime это считается аналогом входного прерывания.
4. Runtime сам крутит интегратор, фильтры и автопилот.
5. Runtime асинхронно публикует наружу:
   - `pose_update` для GUI;
   - `control_command` для GUI.
6. GUI только отображает позу и отправляет пришедшую команду в `C0`.

Поза для GUI в этой схеме не является синхронным запросом. Это отдельный выход runtime, который в будущем будет естественно идти из внутреннего логгера/топика.

## Стабильные контракты

Все ключевые типы описаны в [runtime/contracts.py](/home/necrosii/Programming/Python/Telega_controller/runtime/contracts.py).

### 1. `MissionConfig`

Передается из GUI во внешний runtime при старте миссии.

Содержит:

- `track_points`: полилиния трека;
- `geometry`: геометрия телеги;
- `motion`: целевая скорость, ускорение, торможение, `dt`, минимальный радиус;
- `builtin_tuning`: текущие коэффициенты встроенного контроллера;
- `speed_map`: таблица `PWM <-> speed`;
- `pwm_correction`: коррекция левой и правой гусеницы;
- `pose_source`: выбранный источник позы;
- `autopilot`: выбранный режим автопилота;
- `telemetry_subscription`: набор каналов телеметрии.

### 2. `TelemetrySnapshot`

Передается из GUI во внешний runtime по входному событию.

Содержит:

- `sync`: текущее состояние time sync;
- `imu`: последний `D0`;
- `tacho`: последний `D1`;
- `motor`: последний `D2`.

Сейчас основным runtime-trigger является именно приход `D1` тахо. GUI отправляет снимок телеметрии с последними кешированными `D0/D1/D2`, но инициатором расчета считается новый speed sample.

### 3. `PoseEstimate`

Унифицированное описание позы, которое GUI может отрисовать без знания внутренней реализации интегратора:

- `x_m`, `y_m`, `theta_rad`;
- `source`;
- `pc_time_ms`;
- `mcu_time_ms`.

### 4. `DriveCommand`

Команда, которую GUI отправляет дальше в `C0`:

- `left_pwm`, `right_pwm`;
- `duration_ms`;
- `source`;
- `created_pc_ms`.

Важно: значения трактуются как команды для физических левой/правой гусеницы до аппаратного swap. Swap для coordinate mode остается на стороне GUI.

### 5. `ExternalRuntimeState`

Последний снимок выходов внешнего runtime:

- `pose`;
- `drive_command`;
- `finished`.

GUI использует его как локальный снимок последних опубликованных выходов C++ runtime.

## Основной интерфейс внешнего модуля

Целевой внешний seam теперь один: `ExternalRuntimeBridge`.

Минимальный контракт:

1. `apply_mission(mission)`
2. `reset()`
3. `ingest_telemetry(snapshot)`
4. `poll_state() -> ExternalRuntimeState`
5. `stop()`

Смысл методов:

- `apply_mission`: сохранить системную конфигурацию и трек внутри runtime;
- `reset`: сбросить внутреннее состояние миссии;
- `ingest_telemetry`: подать входное событие от GUI;
- `poll_state`: забрать последние опубликованные runtime выходы;
- `stop`: завершить активную сессию.

Разделенные `PoseEstimator` и `AutopilotController` оставлены в `runtime/contracts.py` только как совместимость с более ранней схемой. Для внешнего C++ runtime целевой контракт теперь именно единый `ExternalRuntimeBridge`.

## Что уже подготовлено в коде

- `CoordinateTab.build_mission_config()` собирает типизированный `MissionConfig`.
- `VirtualControllerApp.build_telemetry_snapshot()` собирает типизированный `TelemetrySnapshot`.
- `VirtualControllerApp.set_external_runtime(...)` подключает единый внешний runtime.
- GUI отправляет `TelemetrySnapshot` во внешний runtime только по приходу `D1`.
- GUI отдельно читает последние `pose` и `drive_command` из `ExternalRuntimeState`.
- При выборе `External autopilot` команда из внешнего runtime уходит в `C0`.
- При выборе `External estimator` GUI рисует позу из внешнего runtime.

## Первый транспортный мост GUI <-> C++

Текущий рабочий мост уже поднят:

- папка проекта: [cpp_autopilot](/home/necrosii/Programming/Python/Telega_controller/cpp_autopilot);
- Python-адаптер: [runtime/socket_runtime.py](/home/necrosii/Programming/Python/Telega_controller/runtime/socket_runtime.py);
- транспорт: TCP `127.0.0.1:8765`;
- формат: newline-delimited JSON;
- C++ stub: [cpp_autopilot/src/main.cpp](/home/necrosii/Programming/Python/Telega_controller/cpp_autopilot/src/main.cpp).

GUI по умолчанию использует:

- `TELEGA_CPP_RUNTIME_HOST`
- `TELEGA_CPP_RUNTIME_PORT`

Для обратной совместимости пока поддерживаются и старые переменные:

- `TELEGA_CPP_AUTOPILOT_HOST`
- `TELEGA_CPP_AUTOPILOT_PORT`

## Транспортный протокол первого этапа

### GUI -> C++

- `hello`
- `mission`
- `reset`
- `telemetry_event`
- `stop`
- `shutdown`

`telemetry_event` содержит:

- `trigger`: сейчас `tacho`;
- `telemetry`: сериализованный `TelemetrySnapshot`.

Семантика завершения:

- `stop` завершает активную runtime-сессию и закрывает текущий клиентский TCP-сеанс;
- `shutdown` завершает сам внешний C++ процесс и освобождает listening port;
- для совместимости `kill`, `terminate` и `exit` на стороне stub трактуются как aliases для `shutdown`.

### C++ -> GUI

- `pose_update`
- `control_command`
- `runtime_status` при необходимости

`pose_update` должен нести `PoseEstimate` или совместимый словарь.

`control_command` может вернуть:

- прямой `left_pwm` / `right_pwm`;
- или `left_speed_m_s` / `right_speed_m_s`, которые GUI переведет в PWM через `MissionConfig.speed_map` и `MissionConfig.pwm_correction`.

## Практический минимум данных для внешнего runtime

На первом реальном контуре достаточно:

- трек;
- геометрия телеги;
- speed map;
- PWM correction;
- `D1` тахо;
- sync state.

Если внутренний C++ интегратор будет использовать IMU, тогда обязательным становится и `D0`.

## Текущий статус stub

Текущий `cpp`-stub пока делает только следующее:

- принимает и печатает все входящие сообщения в своем терминале;
- сохраняет переданную миссию;
- на каждый `telemetry_event` публикует:
  - нулевую позу как `pose_update`;
  - нулевые скорости как `control_command`.

Этого достаточно, чтобы зафиксировать транспорт, жизненный цикл миссии и точки стыковки GUI <-> C++ до начала реальной разработки C++ runtime.
