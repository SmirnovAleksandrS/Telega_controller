Имеется: программа на питоне на компьютере, к компу подключен мощный радиопередатчик, соединенный с аналогичным на гусеничной тележке. Телега имеет ряд датчиков, которые агрегирует и посылает на компьютер. Комп в ответ говорит скорости гусениц в каждый момент времени.

# Формат сообщений

### Общий формат сообщений:
-  заголовок - 0x7E
- тип - 1 байт
- длинна - 1 байт
- полезная нагрузка - 0-255 байт
- crc32 - 4 байта

CRC считается только от изменяемой части, т.е. от всего кроме 0x7E.
### Небольшая договоренность в типах пакетов: 
Пакеты со стартовым байтом 
- A - administration  пакеты с настройками системы.
- B - команды на взаимодействие (запрос ответа)
- С - control пакеты с параметрами движения.
- D - данные от датчиков.  
- E - пакеты с кодами ошибки.
- F - ответы на запросы
Второй байт типа говорит о номере сообщения такого типа. Итоговые типы должны быть уникальны, т.е. тип пакета 0xD0, говорящий о данных с IMU, всегда должен нести эти данные, откуда бы он не пришел.

## Сообщение от телеги
- Тип - 0xD0
- Payload - all IMU:
	- в формате uint32_t - временная метка пакета в миллисекундах
	- в формате float - X Y Z, акселерометр, магнитометр и гироскоп.
- Длинна - 4* 3 * 3 + 4 = 40 байт

- Тип - 0xD1
- Payload - taho data:
	- в формате uint32_t - временная метка пакета в миллисекундах
	- в формате int32_t  левая и правая гусеницы rpm, 
- Длинна - 4 + 2 * 4 = 12 байт

- Тип - 0xD2
- Payload - motor data:
	- в формате uint32_t - временная метка пакета в миллисекундах
	- в формате int16_t ток левой и правой гусеницы 
	- в формате int16_t напряжение левой и правой гусеницы 
	- в формате int16_t температура левой и правой гусеницы 
- Длинна - 4 + 2 * 2 + 2 * 2+ 2 * 2 = 16 байт

- Тип - 0xD3
- Payload - sens tenzor data
	- в формате uint32_t - временная метка пакета в миллисекундах
	- в формате float три линейные скорости x y z
	- в формате float три угловые скорости $\phi$ $\psi$ $\theta$
	- в формате float три параметра качества для линейной скорости $k_{x} \space k_{y} \space k_{z}$
	- в формате float три параметра качества для линейной скорости $k_{\phi} \space k_{\psi} \space k_{\theta}$
- Длинна 52 байта

- Тип - 0xF0 - ответ на запрос синхронизации
- Payload
	- в формате uint32_t - время приема сообщения (максимально быстро и близко к моменту приема 0xB0)
	- в формате uint32_t - время отправки ответа, максимально близко к отправке сообщения
- Длинна = 8

- Тип - 0xF1 - ответ на запрос параметров ПИД регулятора
- Payload
	- в формате float - P компонента для левой
	- в формате float - I  компонента для левой
	- в формате float - D компонента для левой
	- в формате float - P компонента для правой
	- в формате float - I  компонента для правой
	- в формате float - D компонента для правой
- Длинна = 24
## Сообщения от компьютера:
- Тип - 0xС0
- Payload - скважность шима на левую и правую гусеницы
	- в формате uint32_t - временная метка пакета в миллисекундах
	- в формате int16_t на левую и правую гусеницы
	- в формате uint16_t сколько миллисекунд действительна команда
- Длинна = 10

- Тип 0xB0 - Запрос на синхронизацию времени
- Payload
	- в формате uint16_t - номер раунда
	- в формате uint32_t - время компа
- Длинна - 6 байт

- Тип 0xB0 - Запрос на данные ПИД регу
- Payload
	- в формате uint16_t - номер раунда
	- в формате uint32_t - время компа
- Длинна - 6 байт

- Тип 0xA0 - выключить отправку сообщений под типом....
- Payload
	- в формате uint32_t - временная метка пакета в миллисекундах
	- тип D сообщения, которое надо отключить (uint8_t)
- Длинна - 5 

- Тип 0xA1 - включить сообщения с периодом
- Payload
	- в формате uint32_t - временная метка пакета в миллисекундах
	- тип D сообщения, которое надо включить (uint8_t)
	- в формате uint16_t - период отправки сообщения

- Тип 0xA2 - установить ПИД двигателя
- Payload
	- в формате float - P компонента для левой
	- в формате float - I  компонента для левой
	- в формате float - D компонента для левой
	- в формате float - P компонента для правой
	- в формате float - I  компонента для правой
	- в формате float - D компонента для правой

# Протокол синхронизации времени PC↔MCU (NTP-like, оценка a и b на стороне PC)

## 0. Обозначения
- PC ведёт монотонное время `t_pc` (uint64, например мкс).
- MCU ведёт монотонное время `t_mcu` (uint64, те же единицы, что и на PC, или любые — главное постоянный масштаб).
- Модель соответствия времени:
  - `t_mcu ≈ a * t_pc + b`
  - `a` — масштаб (1 + skew), `b` — смещение (offset).
- Все таймстампы берутся максимально близко к событию:
  - `t1_pc` — при формировании SYNC_REQ на PC
  - `t2_mcu` — при приёме SYNC_REQ на MCU (RX обработчик)
  - `t3_mcu` — при формировании SYNC_RESP на MCU
  - `t4_pc` — при приёме SYNC_RESP на PC (RX обработчик)

---

## 1. Форматы пакетов

### 1.1 SYNC_REQ (PC → MCU)
Поля:
- `type = SYNC_REQ` (u8)
- `seq` (u16/u32) — номер раунда
- `t1_pc` (u64) — время PC в момент формирования этого пакета

### 1.2 SYNC_RESP (MCU → PC)
Поля:
- `type = SYNC_RESP` (u8)
- `seq` (u16/u32) — эхо номера раунда
- `t2_mcu` (u64) — время MCU в момент приёма SYNC_REQ
- `t3_mcu` (u64) — время MCU в момент формирования SYNC_RESP

### 1.3 Рабочий пакет MCU → PC (DATA, необязательный формат)
Поля (минимум для привязки времени):
- `type = DATA` (u8)
- `ts_mcu` (u64) — таймстамп события/формирования на MCU (что именно — фиксируете в протоколе прикладного уровня)

---

## 2. Расчёт одной синхронизационной точки на PC (для раунда i)

После получения SYNC_RESP{seq=i, t2_mcu, t3_mcu}:
- PC фиксирует `t4_pc = now_pc()` (при приёме)
- `t1_pc` берётся из ранее отправленного SYNC_REQ с тем же seq

Вычисляются:

### 2.1 Оценка качества (задержка канала)
`delta_i` (используется только для фильтрации “плохих” раундов):
- `delta_i = (t4_pc - t1_pc) - (t3_mcu - t2_mcu)`

### 2.2 Mid-пара (реперная точка соответствия времен)
- `tmid_pc_i  = (t1_pc + t4_pc) / 2`
- `tmid_mcu_i = (t2_mcu + t3_mcu) / 2`

Интерпретация: в момент `tmid_pc_i` по PC на MCU было примерно `tmid_mcu_i`.

---

## 3. Процедура первой инициализации (стартовая синхронизация)

Цель: получить первичные `a_hat`, `b_hat`.

1) PC выполняет N раундов SYNC (рекомендуемо N=10…30):
   - для i=0..N-1:
     - PC: сформировать SYNC_REQ, записать `t1_pc`, отправить
     - MCU: при приёме зафиксировать `t2_mcu`; сформировать ответ с `t3_mcu`; отправить SYNC_RESP
     - PC: при приёме зафиксировать `t4_pc`, посчитать `delta_i`, `tmid_pc_i`, `tmid_mcu_i`, сохранить запись

2) PC выбирает K лучших записей по минимальному `delta_i` (рекомендуемо K=5, K<=N).

3) Оценка `a_hat`, `b_hat` по двум крайним точкам среди этих K (максимально простой расчёт):
   - выбрать среди K точек самую раннюю и самую позднюю по `tmid_pc`:
     - `(tmid_pc_first, tmid_mcu_first)`
     - `(tmid_pc_last,  tmid_mcu_last)`
   - вычислить:
     - `a_hat = (tmid_mcu_last - tmid_mcu_first) / (tmid_pc_last - tmid_pc_first)`
     - `b_hat = tmid_mcu_first - a_hat * tmid_pc_first`

4) С этого момента PC считает, что время MCU задаётся моделью `t_mcu ≈ a_hat * t_pc + b_hat`.

---

## 4. Процедура последующей инициализации (подстройка во времени)

Цель: не дать модели “уплыть” из-за изменения дрейфа (температура/режимы/питание).

Периодически (например, раз в 10 секунд) выполнить один раунд SYNC и сгладить модель:

1) Выполнить 1 раунд SYNC, получить `(delta, tmid_pc, tmid_mcu)`.
2) (Опционально) отбросить раунд, если `delta` слишком велик (локальный порог или относительно недавних минимумов).
3) Обновить `a_hat` и `b_hat` по последней принятой “хорошей” точке и предыдущей опорной точке:
   - пусть предыдущая опорная точка: `(tmid_pc_prev, tmid_mcu_prev)`
   - текущая: `(tmid_pc_cur, tmid_mcu_cur)`
   - сырые оценки:
     - `a_raw = (tmid_mcu_cur - tmid_mcu_prev) / (tmid_pc_cur - tmid_pc_prev)`
     - `b_raw = tmid_mcu_cur - a_raw * tmid_pc_cur`
   - сглаживание (0<beta<=1, например 0.02…0.1):
     - `a_hat = (1 - beta) * a_hat + beta * a_raw`
     - `b_hat = (1 - beta) * b_hat + beta * b_raw`
   - обновить опорную точку: `prev = cur`

---

## 5. Формулы использования на PC (после синхронизации)

### 5.1 Оценка текущего времени MCU на PC
Для текущего времени PC `t_pc_now = now_pc()`:
- `t_mcu_est = a_hat * t_pc_now + b_hat`

### 5.2 Перевод таймстампа из пакета MCU → PC в шкалу времени PC
Если MCU прислал `ts_mcu` (u64) в DATA-пакете:
- `t_pc_est = (ts_mcu - b_hat) / a_hat`

(Дальше `t_pc_est` можно использовать как “оценку времени события по шкале PC”.)

## Общая концепция приложения

Кросплатформенное (Linux и Windows) графическое приложение на python, работающее с UART. Соответственно должна быть возможность задания настроек ком порта (выбор самого порта и скорости общения). Также должна быть возможность ведения и сохранения логов, полученных из ком порта. Сохраняться должны уже распарсенные данные в формате .log . 

Есть несколько отдельных окон/вкладок с разными методами управления и реализуемой математикой.
![[Pasted image 20260114163247.png]]

Можно переключаться между режимами, пока будем делать только Manual

![[Pasted image 20260114163417.png]]

Так же настройки на кнопках files, settings и Deviation Settings 

![[Pasted image 20260114163533.png]]

![[Pasted image 20260114163606.png]]

![[Pasted image 20260114163643.png]]
## Виртуальный джойстик

На всех рисунках кроме первого есть джойстик, который можно двигать мышкой. Он задает параметры для Left и Right. Shift и Linear это коэффициенты коррекции, задаваемые вручную отдельно (просто окно для ввода числа). Цветные значения в критикал параметрс это значения, полученные от микроконтроллера по радио каналу. Зеленые, если значение находиться в нормал +- 0.5 дельта, желтое если больше +- 0.5 и до +- дельта, красное если больше дельта. Нормаль и дельта задаются в окне Deviation Settings для каждого параметра отдельно. 

Логирование включается через вкладку файл. Если логгирование идет, то должна появиться надпись Log Running зеленым, вместо Log Stopped красным.

# Задачи

Требуется реализовать считывание данных из ком порта, обработку всех описанных выше сообщений и алгоритма синхронизации времени. Реализовать графическое приложение по приложенным референсам. Весь код должен быть модульным, разбитым по нескольким файлам. Математику расчета итоговых значений вывести в отдельные функции для легкой замены.

Во время работы над кодом надо вести чейнджлог, список того, что было сделано, использованный фреймворк и идеи. Весь код тщательно комментировать и пояснять смысл и работу каждой функции.

# Новое окно

# CODEX SPEC — Sensors / Magnetometer

## 0. Core rule

Do **not** rewrite the app.  
Extend the existing app incrementally.

The app already has:

- Python + Tkinter GUI
- UART via pyserial
- parsed packet pipeline
- global menus
- `Manual` and `Coordinate` tabs
- right-side system panel
- bottom `Log / RPM`
- time sync
- changelog discipline.

The new work must preserve all of that.

---

# 1. Final objective

Add a new top-level tab:

- `Sensors`

Inside it, add a sub-tab:

- `Magnetometer`

This sub-tab must become a working environment for:

- live magnetometer visualization,
- heading visualization,
- recording datasets,
- loading/editing/concatenating datasets,
- attaching external Python calibration/filter methods,
- offline calibration over loaded datasets,
- realtime production of derived streams,
- later comparison via metrics.

---

# 2. Mandatory product constraints

## 2.1 Technology

- Keep framework: `Tkinter`
- Keep UART stack: existing `pyserial` code
- Keep time sync: existing NTP-like PC↔MCU model
- Keep logging style and existing app state persistence approach.

## 2.2 UI consistency

The new tab must visually match current app patterns:

- top-level mode tabs
- center workspace
- right status/action area
- bottom notebook region
- same spacing philosophy
- same button hierarchy
- same “settings are in dialogs” pattern

## 2.3 Data source

Magnetometer data must come from the already existing IMU packet `0xD0`, which includes:

- packet timestamp
- accelerometer XYZ
- magnetometer XYZ
- gyro XYZ.

## 2.4 Time handling

All recorded and processed magnetometer datasets must use:

- MCU timestamp
- PC receive timestamp
- PC-estimated event time via existing `a_hat`, `b_hat` time model.

## 2.5 Stream rule

Derived algorithm outputs must never overwrite raw data.  
Each algorithm produces its own separate derived stream.

---

# 3. Final UI layout to implement

## 3.1 Existing top app frame

Keep unchanged:

- `Files`
- `Settings`
- `Physics`
- `Connect / Disconnect`
- `Test mode`
- `Log Running / Log Stopped`
- `Time Sync`
- `KILL SWITCH`
- right-side global system status panel.

## 3.2 New top-level mode tab

Add:

- `Sensors`

Placement:

- same level as `Manual`
- same level as `Coordinate`

## 3.3 Internal sub-tabs inside Sensors

Create internal notebook:

- `Magnetometer`

Only this first one must be working now.

## 3.4 Magnetometer tab split

The `Magnetometer` tab must use a 4-zone layout:

### Zone A — left panel

Width: about 20–25%  
Purpose: sources and methods list

### Zone B — center panel

Width: about 50–55%  
Purpose: main visualization canvas

### Zone C — right panel

Width: about 20–25%  
Purpose: current values, dataset actions, selected method actions

### Zone D — bottom panel

Full width under A+B+C content area  
Purpose: local notebook for `Log / Data / Metrics`

---

# 4. Required widgets and exact placement

## 4.1 Left panel — `Sources & Methods`

### Top row

Buttons:

- `+ Add`

Optional later:

- `Remove`
- `Reload`

### Main content

Scrollable vertical list of cards.

### Card types

Two card types:

- `SourceCard`
- `MethodCard`

### SourceCard fields

- title
- source type
- status indicator
- `Show` toggle
- `Record` toggle
- `Info` button

### MethodCard fields

- title
- version
- status indicator
- `Show` toggle
- `Calibrate`
- `Load params`
- `Save params`
- `Realtime On/Off`
- `Info`

### Card statuses

Mandatory statuses:

- gray = loaded but partial capability / inactive
- yellow = warning
- green = ready / success
- green-progress = currently calibrating
- red = error.

### Error behavior

Clicking a red card must open a diagnostics dialog containing:

- method name
- version
- file path
- last action
- error text
- traceback
- last warnings

---

## 4.2 Center panel — `Magnetometer View`

### Top toolbar over canvas

Controls:

- `3D`
- `XY`
- `XZ`
- `YZ`
- `Fit`
- `Reset`
- `Auto-fit` checkbox

### Main canvas content

Must support displaying:

- compass-like horizon plane
- cardinal marks: `N`, `S`, `W`, `E`
- vertical direction markers: `+Z`, `-Z`
- current raw magnetometer point
- current heading vector from raw magnetometer
- point cloud of loaded dataset
- corrected point cloud from selected method
- heading vectors from multiple enabled sources/methods simultaneously.

### Interaction

Must support:

- zoom
- pan
- reset
- fit loaded data
- layer toggles

Use interaction style consistent with current `Coordinate` canvas behavior where possible.

---

## 4.3 Right panel — `Magnetometer Control`

Split into stacked sections.

### Section 1 — Current Data

Read-only fields:

- `MCU ts`
- `PC rx ts`
- `PC est ts`
- `mx`
- `my`
- `mz`
- `|m|`
- `raw heading`
- `selected output heading`
- `selected source`
- `dataset status`

### Section 2 — View Options

Checkboxes:

- show raw points
- show corrected points
- show current point
- show raw heading
- show GNSS heading
- show derived headings
- show trails
- auto fit on load

### Section 3 — Dataset Actions

Buttons:

- `Start record`
- `Stop record`
- `Load CSV`
- `Load multiple`
- `Save current`
- `Save as`
- `Concatenate`
- `Trim selection`
- `Delete selection`

Read-only dataset summary:

- active dataset name
- number of rows
- source count
- time range

### Section 4 — Selected Method

Shown only when a method is selected.

Fields:

- method name
- method version
- method path
- status
- supported capabilities

Buttons:

- `Calibrate`
- `Load params`
- `Save params`
- `Enable realtime`
- `Disable realtime`

### Section 5 — Output Routing

Controls:

- choose visible heading streams
- choose primary heading stream
- choose streams to include in recording
- later placeholder for autopilot routing

---

## 4.4 Bottom panel — local notebook

Create local tabs:

- `Log`
- `Data`
- `Metrics`

### Log tab

Shows only sensor/magnetometer module logs:

- dataset load/save
- recording start/stop
- method load
- calibration start/finish
- warnings
- errors

### Data tab

Table view for active dataset.

Mandatory columns:

- row_id
- stream_id
- stream_type
- producer_name
- producer_version
- timestamp_mcu
- timestamp_pc_rx
- timestamp_pc_est
- mag_x
- mag_y
- mag_z
- heading
- flags

Table actions:

- row selection
- range selection
- filter by stream
- delete selection
- trim by selected range
- export current view

### Metrics tab

At first release:

- disabled or placeholder label only

Later:

- metrics table

This follows the user plan where metrics are explicitly future work.

---

# 5. Internal data model that must be introduced

## 5.1 Stream types

Create explicit stream typing:

- `raw`
- `reference`
- `derived`

## 5.2 Core entities

Create these entities as separate classes/dataclasses:

### `SampleRecord`

Represents one time-stamped sample.

Mandatory fields:

- stream_id
- stream_type
- producer_name
- producer_version
- timestamp_mcu
- timestamp_pc_rx
- timestamp_pc_est
- mag_x
- mag_y
- mag_z
- heading
- flags
- extra dict

### `Dataset`

Represents a collection of `SampleRecord`.

Mandatory methods:

- `append(record)`
- `extend(records)`
- `trim(start_idx, end_idx)`
- `delete_rows(indices)`
- `concatenate(other_dataset)`
- `to_csv(path)`
- `from_csv(path)`
- `summary()`

### `CalibrationProfile`

Mandatory fields:

- algorithm_name
- algorithm_version
- schema_version
- created_at
- params

### `StreamRegistry`

Responsibilities:

- register streams
- activate/deactivate visibility
- activate/deactivate recording
- resolve primary heading source

### `MagnetometerController`

Responsibilities:

- subscribe to parsed IMU messages
- convert IMU payload into raw stream records
- update UI view-model
- dispatch records to realtime methods
- record datasets
- manage selected dataset
- manage selected method

---

# 6. Plugin system specification

## 6.1 What must be loadable

External Python file chosen by user.

## 6.2 Required plugin API

Every plugin must export:

def get_info() -> dict: ...  
def calibrate(dataset, config=None): ...  
def load_params(path): ...  
def save_params(path, params): ...  
def process(sample, params): ...

## 6.3 Required keys in `get_info()`

Mandatory keys:

- `name`
- `version`
- `type`
- `supports_calibrate`
- `supports_load_params`
- `supports_save_params`
- `supports_process`
- `input_schema`
- `output_schema`

## 6.4 Optional plugin API

May export:

def validate_dataset(dataset): ...  
def get_default_config(): ...  
def get_last_report() -> str: ...

## 6.5 Plugin loader rules

- load module safely
- validate required functions
- catch import errors
- catch runtime errors
- never crash GUI if plugin is broken
- show all plugin errors in diagnostics dialog

## 6.6 Parameter file rules

Use JSON.  
Mandatory header:

- `algorithm_name`
- `algorithm_version`
- `schema_version`
- `created_at`

Everything else is method-specific. This matches the requirement that parameter compatibility between different methods is not required.

---

# 7. Built-in sources and methods that must exist before external plugins are useful

Implement these built-ins:

## Sources

- `Raw Magnetometer`
- `Raw Heading`

## Methods

- `HardIron Offset`

This is the minimum viable chain:

- receive IMU
- display raw point
- record dataset
- load dataset
- calibrate with simplest built-in method
- produce corrected realtime stream

The research plan also explicitly includes hard-iron-only correction as the baseline method.

---

# 8. CSV and dataset rules

## 8.1 Recording

Dataset recording is separate from global `.log` logging.  
Do not merge those two concepts.

## 8.2 CSV fields

The CSV schema must include at least:

- stream_id
- stream_type
- producer_name
- producer_version
- timestamp_mcu
- timestamp_pc_rx
- timestamp_pc_est
- mag_x
- mag_y
- mag_z
- heading
- flags

## 8.3 Multiple loaded files

The system must support:

- opening several CSV files at once
- holding them in memory as multiple datasets
- concatenating selected datasets into a new one.

## 8.4 Editing rules

Editing actions are non-destructive until explicit save:

- trim range
- delete rows
- concatenate
- save as new dataset

---

# 9. IMU integration rules

## 9.1 Existing packet use

Use the existing parsed `0xD0` IMU message as the source of:

- `mag_x`, `mag_y`, `mag_z`
- optional accelerometer-based future leveling
- optional gyro-based future methods.

## 9.2 Heading computation

For first implementation:

- compute raw heading from magnetometer plane projection

Keep this math in a separate utility module so it can be swapped later.  
This follows the current app rule that computational math should be isolated into replaceable functions/modules.

---

# 10. Time-sync integration rules

Use the existing time model:

- initial sync rounds
- best-by-delta filtering
- estimation of `a_hat`, `b_hat`
- periodic resync and smoothing.

For every dataset row:

- store MCU timestamp from incoming message
- store PC receive time
- store PC-estimated event time derived from time model

Do not introduce a second time-normalization mechanism.

---

# 11. Required repository/file structure

Create or extend using this layout:

app/  
  main.py  
  
  gui/  
    main_window.py  
    tabs/  
      manual_tab.py  
      coordinate_tab.py  
      sensors_tab.py  
      magnetometer_tab.py  
    dialogs/  
      add_source_method_dialog.py  
      method_diagnostics_dialog.py  
      dataset_editor_dialog.py  
      method_info_dialog.py  
    widgets/  
      source_card.py  
      method_card.py  
      magnetometer_canvas.py  
      dataset_table.py  
  
  telemetry/  
    uart_manager.py  
    frame_parser.py  
    packet_models.py  
    time_sync.py  
    stream_bus.py  
  
  sensors/  
    magnetometer/  
      controller.py  
      dataset.py  
      models.py  
      streams.py  
      heading_math.py  
      record_manager.py  
      plugin_loader.py  
      metrics.py  
      view_model.py  
  
  plugins/  
    builtin/  
      raw_magnetometer_source.py  
      raw_heading_source.py  
      hardiron_offset.py  
  
  utils/  
    csv_utils.py  
    json_utils.py  
    logging_utils.py

Do not collapse all magnetometer code into one file.

---

# 12. Implementation order — hard sequence

Follow this exact sequence.  
Do not skip ahead.

---

## PHASE 1 — Create UI shell

### Goal

Add `Sensors` and `Magnetometer` tabs only.

### Deliverables

- top-level `Sensors` tab
- internal `Magnetometer` sub-tab
- 4-zone layout
- placeholder widgets in all regions

### Must not do yet

- no live data
- no dataset logic
- no plugin loader

### Acceptance

- app launches
- `Manual` unchanged
- `Coordinate` unchanged
- `Sensors > Magnetometer` opens without errors

### Report required from CODEX

- changed files
- screenshot
- short note on layout structure

---

## PHASE 2 — Wire live IMU data

### Goal

Feed live raw magnetometer values into the new tab.

### Deliverables

- subscribe to existing IMU parser output
- display `mx`, `my`, `mz`
- display `timestamp_mcu`, `timestamp_pc_rx`, `timestamp_pc_est`
- compute and display raw heading
- append messages to local magnetometer log

### Must not do yet

- no dataset save/load
- no plugins
- no complex rendering

### Acceptance

- live values update without freezing
- time fields update correctly
- raw heading is visible

### Report required

- changed files
- manual test steps
- known limitations

---

## PHASE 3 — Implement visualization canvas

### Goal

Create main magnetometer visualizer.

### Deliverables

- canvas toolbar: `3D / XY / XZ / YZ / Fit / Reset`
- compass plane
- N/S/W/E labels
- vertical markers `+Z/-Z`
- current raw point
- raw heading vector
- zoom/pan/reset

### Must not do yet

- no point cloud history loading
- no correction overlays

### Acceptance

- point moves live
- heading vector updates
- projection switching works

### Report required

- changed files
- screenshot per projection
- known rendering limitations

---

## PHASE 4 — Implement dataset recording

### Goal

Record live samples into dataset and save CSV.

### Deliverables

- `Start record`
- `Stop record`
- create in-memory `Dataset`
- export CSV with required schema
- dataset summary panel update

### Must not do yet

- no editing
- no multi-load
- no concatenate

### Acceptance

- user can record session
- CSV saves successfully
- reopened file content is structurally valid

### Report required

- changed files
- example CSV schema
- test procedure

---

## PHASE 5 — Implement dataset loading and editing

### Goal

Allow offline work with datasets.

### Deliverables

- `Load CSV`
- `Load multiple`
- `Save current`
- `Save as`
- `Concatenate`
- `Trim selection`
- `Delete selection`
- bottom `Data` table
- active dataset summary

### Must not do yet

- no plugin calibration

### Acceptance

- several files can be loaded
- active dataset is viewable
- user can trim and save
- user can concatenate datasets

### Report required

- changed files
- editing workflow
- screenshot of table

---

## PHASE 6 — Implement source cards

### Goal

Create left panel as real stream manager.

### Deliverables

- `SourceCard` widget
- built-in source entries:
    - Raw Magnetometer
    - Raw Heading
- toggles:
    - Show
    - Record
- status coloring
- selection handling

### Must not do yet

- no external methods

### Acceptance

- built-in sources visible in list
- visibility toggle affects canvas
- record toggle affects dataset output

### Report required

- changed files
- card status logic summary

---

## PHASE 7 — Implement plugin loader

### Goal

Load external Python methods.

### Deliverables

- `+ Add` dialog
- choose `.py`
- validate plugin API
- create `MethodCard`
- `Info` dialog
- diagnostics dialog for failures

### Must not do yet

- no calibration execution

### Acceptance

- valid plugin loads
- invalid plugin does not crash app
- error details shown in diagnostics dialog

### Report required

- changed files
- plugin API contract
- example plugin file

---

## PHASE 8 — Implement offline calibration

### Goal

Run plugin calibration on loaded dataset.

### Deliverables

- `Calibrate` action
- method status progression
- async/non-blocking execution
- result stored in runtime session
- warning and error handling
- diagnostics integration

### Must not do yet

- no save/load params
- no realtime process yet

### Acceptance

- calibration uses active dataset
- GUI stays responsive
- status goes green/yellow/red correctly

### Report required

- changed files
- calibration state flow
- user test steps

---

## PHASE 9 — Implement param save/load

### Goal

Support reusable calibration profiles.

### Deliverables

- `Save params`
- `Load params`
- JSON header validation
- mismatch warnings
- attach loaded params to method runtime state

### Acceptance

- params can be saved and later reloaded
- wrong method/version is handled gracefully

### Report required

- changed files
- example JSON
- validation rules

---

## PHASE 10 — Implement realtime derived streams

### Goal

Process live raw samples through loaded methods.

### Deliverables

- `Enable realtime`
- `Disable realtime`
- `process(sample, params)` dispatch
- derived stream creation
- visibility toggle on canvas
- optional recording of derived stream into dataset
- no overwrite of raw stream

### Acceptance

- raw and corrected data coexist
- derived stream can be shown and recorded independently

### Report required

- changed files
- stream lifecycle summary
- performance note

---

## PHASE 11 — Multi-heading visualization and routing

### Goal

Show several heading sources at once and select primary output.

### Deliverables

- multiple heading vectors on canvas
- view toggles in right panel
- primary heading selector
- source legend

### Acceptance

- user can compare raw vs corrected vs reference headings visually
- primary source selection updates current data panel

### Report required

- changed files
- screenshot with 2+ heading vectors
- legend explanation

---

## PHASE 12 — Metrics tab

### Goal

Add first comparison layer.

### Deliverables

- `Metrics` tab UI
- for selected dataset and selected method:
    - RMSE vs GNSS heading if available
    - mean radius
    - radius std
    - center offset
    - calibration runtime
    - mean process runtime
- export metrics CSV

These metrics follow the explicitly stated evaluation plan: fit to circle, radius stability, center offset, and GNSS-based comparison such as RMSE.

### Acceptance

- metrics table renders
- missing GNSS handled gracefully
- export works

### Report required

- changed files
- formulas used
- edge cases

---

# 13. Explicit non-goals for first full cycle

Do not implement in the first cycle:

- all advanced calibration methods
- C/C++ plugin bridge
- full OpenGL 3D engine
- autopilot integration based on selected heading
- batch benchmark runner over all datasets
- automatic environment anomaly classifier

Those are later extensions.

---

# 14. Required built-in QA checklist after every phase

CODEX must provide this checklist in every handoff:

## Required handoff template

- Phase number
- Goal
- Files changed
- Files added
- User-visible changes
- Manual test steps
- Known limitations
- Next phase recommendation
- `CHANGELOG.md` updated: yes/no

The changelog requirement is mandatory because the existing project explicitly requires ongoing changelog maintenance and already contains structured implementation history.

---

# 15. Required coding rules

## Code organization

- no monolithic GUI file
- no business logic inside Tkinter widget callbacks beyond dispatch
- math in separate modules
- parser integration via controller/service layer
- dataset model separate from view
- plugin loader separate from controller

## Error handling

- no unhandled plugin exceptions
- no GUI crash on bad dataset
- no silent CSV schema mismatch
- no silent param file mismatch

## Comments

- every public function explained
- every nontrivial class documented
- every plugin API function documented
- every time-sync-derived timestamp field explained

## Persistence

Reuse existing persistence style for app settings where possible, since the current app already persists COM settings, deviations, joystick coefficients, and coordinate state.

---

# 16. Minimum acceptance for first useful release

A release is considered minimally useful only if all of the following work:

- `Sensors > Magnetometer` tab opens
- live IMU magnetometer data is shown
- raw heading is computed and displayed
- recording to dataset CSV works
- loading CSV back works
- trimming and concatenating datasets works
- built-in `HardIron Offset` works on dataset
- params save/load works
- realtime corrected stream works
- raw and corrected streams can be shown simultaneously
- local log shows magnetometer/module events

If any of those are missing, the feature is not done.

---

# 17. One-line execution summary for CODEX

Implement `Sensors > Magnetometer` as a modular, dataset-driven, plugin-capable magnetometer workspace on top of the existing Tkinter/UART/time-sync app, using existing IMU `0xD0` packets and separate raw/reference/derived streams, delivered strictly in the 12 ordered phases above.

Если хочешь, я следующим сообщением сделаю еще более утилитарную версию: **“копипастный master prompt для CODEX”**, который можно вставить агенту одним блоком без редактирования.