# Подключение новых топиков в `cpp_autopilot`

Этот документ описывает, как в текущем каркасе `cpp_autopilot` добавлять новые
топики, какие интерфейсы для этого уже существуют и каким требованиям должны
соответствовать payload-структуры и узлы.

Документ относится к stage-1 архитектуре:

- шина событий реализована в `cpp_autopilot/include/telega_cpp_autopilot/event_bus.hpp`;
- базовые payload-типы лежат в `cpp_autopilot/include/telega_cpp_autopilot/event_payloads.hpp`;
- сборка topology происходит в `cpp_autopilot/src/runtime_graph.cpp`;
- узлы разнесены по `cpp_autopilot/include/telega_cpp_autopilot/runtime_nodes/` и `cpp_autopilot/src/runtime_nodes/`.

## 1. Что такое топик в текущей реализации

Топик в этой системе состоит из четырех частей:

1. Идентификатор в `enum class TopicId`.
2. Payload-тип, который реально передается через шину.
3. `TopicDescriptor`, в котором зарегистрированы размер payload, флаги и подписчики.
4. Код, который публикует этот топик и код, который на него подписан.

Шина сама по себе не знает ничего про физический смысл данных. Для нее топик это:

- номер;
- фиксированный размер payload;
- фиксированное выравнивание payload;
- список подписчиков;
- флаги поведения.

## 2. Обязательные требования к payload

Текущий `EventBus` копирует payload через `std::memcpy` в double-buffer storage.
Из этого следуют жесткие требования.

Новый payload должен быть:

- `trivially copyable`;
- `standard layout`;
- по размеру не превышать `kEventBusSlotSize`;
- по выравниванию не превышать `kEventBusSlotAlignment`.

Именно эти ограничения уже зафиксированы в коде через `static_assert` в typed
методах `publish<...>()` и `publishFromISR<...>()`.

Практически это значит:

- используйте обычные поля и простые aggregate-структуры;
- не добавляйте виртуальные методы;
- не добавляйте `std::string`, `std::vector`, `std::optional` и другие типы с
  нестабильным внутренним состоянием прямо внутрь bus-payload;
- если нужно передавать сложные данные, сначала переведите их в фиксированную
  POD-структуру.

### Рекомендуемая форма payload

Для совместимости с текущими узлами и будущими блоками рекомендуется:

- первым полем держать `timestamp`;
- сохранять префикс `dat...` для bus payload;
- держать методы вида `getWeight()` только там, где они реально нужны логике.

`getWeight()` не является обязательным интерфейсом шины. Это только локальный
контракт между конкретным payload и конкретным узлом `TriggerStub`.

## 3. Какие интерфейсы уже есть

### 3.1. Интерфейс публикации в шину

Публикация обычного топика:

```cpp
bus.publish<TopicId::kPose>(pose, timestamp_ms);
```

Публикация из ISR-подобного контекста:

```cpp
bus.publishFromISR<TopicId::kHealth>(health, timestamp_ms);
```

Если используется `publishFromISR`, у топика обязательно должен стоять флаг:

```cpp
kTopicAllowISRPublish
```

### 3.2. Интерфейс подписчика

Подписчик в bus имеет сигнатуру:

```cpp
using SubscriberFn = void (*)(void* ctx, const TopicSampleRef& sample);
```

Где:

- `ctx` — произвольный указатель на объект узла;
- `sample.payload` — указатель на payload текущего топика;
- `sample.sequence` — номер публикации;
- `sample.publish_timestamp` — timestamp, переданный в `publish(...)`.

Обычно узел делает статический адаптер:

```cpp
static void HandlePose(void* ctx, const TopicSampleRef& sample) {
    static_cast<MyNode*>(ctx)->onPose(sample);
}
```

А внутри `onPose(...)` уже делает typed-cast:

```cpp
const auto* pose = static_cast<const datPosition*>(sample.payload);
```

### 3.3. Интерфейсы прямого hot-path вызова

Для прямой связи между узлами без прохода через subscriber-механику сейчас
заведены минимальные интерфейсы в
`cpp_autopilot/include/telega_cpp_autopilot/runtime_node_interfaces.hpp`:

- `IngressPacketConsumer`
- `VelocityConsumer`
- `TickPublisher`

Они нужны для участков, где важно сохранить будущую структуру pipeline:

- `UartRxStub -> MixerFilterStub`
- `MixerFilterStub -> IntegratorStub`
- `UartRxStub -> TickSourceStub`

Если новый узел должен стоять в прямой цепочке, лучше добавить новый узкий
интерфейс, а не связывать узлы через конкретный класс.

Пример:

```cpp
class CurvatureConsumer {
public:
    virtual ~CurvatureConsumer() = default;
    virtual void receiveNewData(const datCurvature& curvature) = 0;
};
```

## 4. Пошагово: как добавить новый топик

Ниже базовый алгоритм.

### Шаг 1. Создать payload

Добавьте структуру в `event_payloads.hpp` или в отдельный header, если payload
уже относится к отдельному модулю.

Пример:

```cpp
struct datCurvature {
    std::uint32_t timestamp = 0;
    float curvature = 0.0F;
};
```

Если новый тип стал самым большим по размеру или выравниванию, расширьте
`kEventBusSlotSize` и `kEventBusSlotAlignment`, включив его в `std::max(...)`.

### Шаг 2. Добавить идентификатор в `TopicId`

В `event_bus.hpp` добавьте новый элемент в `enum class TopicId`.

Пример:

```cpp
kCurvature = 6,
kTopicCount = 7,
```

Важно:

- `kTopicCount` всегда должен оставаться последним;
- порядок значений задает порядок обхода `bitMask`, то есть и приоритет dispatch.

### Шаг 3. Подготовить узел-издатель

В узле, который производит новый payload, добавьте публикацию:

```cpp
datCurvature curvature;
curvature.timestamp = sample.timestamp;
curvature.curvature = 0.0F;
bus_->publish<TopicId::kCurvature>(curvature, curvature.timestamp);
```

Если публикация должна быть разрешена из ISR-пути, используйте
`publishFromISR<...>()` и не забудьте флаг `kTopicAllowISRPublish`.

### Шаг 4. Подготовить подписчиков

Если другие узлы должны реагировать на этот топик через bus, для них нужно:

1. добавить статический adapter `Handle...`;
2. добавить внутренний метод `on...`;
3. зарегистрировать `SubscriberBinding`.

Пример binding:

```cpp
curvature_subscribers[0] = SubscriberBinding {
    "MyController",
    SubscriberMode::kInline,
    &MyController::HandleCurvature,
    &my_controller,
};
```

### Шаг 5. Зарегистрировать `TopicDescriptor`

В `RuntimeGraph::Impl` добавьте descriptor в `registry[...]`.

Пример:

```cpp
registry[TopicIndex(TopicId::kCurvature)] = MakeTopicDescriptor(
    TopicId::kCurvature,
    "Curvature",
    static_cast<std::uint16_t>(sizeof(datCurvature)),
    static_cast<std::uint16_t>(alignof(datCurvature)),
    static_cast<std::uint16_t>(kTopicLatestWins),
    curvature_subscribers.data(),
    static_cast<std::uint8_t>(curvature_subscribers.size())
);
```

Что должно совпадать строго:

- `TopicId`;
- `sizeof(payload)`;
- `alignof(payload)`;
- массив подписчиков именно этого топика.

### Шаг 6. Добавить storage/subscriber контейнеры в graph

Если у нового топика есть подписчики, в `RuntimeGraph::Impl` обычно добавляется
новый `std::array<SubscriberBinding, N>`.

Если подписчиков пока нет, можно передать:

- `nullptr`
- `0U`

### Шаг 7. Проверить сценарий dispatch

После добавления топика нужно понять, как он должен жить:

- только как bus-topic для fan-out;
- как hot-path direct-call плюс зеркало в bus;
- только как служебный ISR-топик.

Это решение влияет на:

- какой интерфейс узла нужен;
- нужен ли флаг `kTopicHotPathMirror`;
- вызывается ли публикация до или после прямого вызова следующего узла.

## 5. Что означают текущие флаги

### `kTopicLatestWins`

Если один и тот же топик публикуется повторно до dispatch, старая версия
считается перезаписанной. Это уже отражается в `PublishResult.overwritten_previous`.

### `kTopicAllowISRPublish`

Разрешает публиковать топик через `publishFromISR(...)`.

Без этого флага `publishFromISR(...)` вернет отказ.

### `kTopicTraceEnabled`

Зарезервировано на будущее. Сейчас трассировка отдельно не реализована.

### `kTopicHotPathMirror`

Маркер для топиков, которые уже участвуют в прямом hot-path потоке, но при этом
дублируются в шину для fan-out, диагностики или логирования.

Сейчас так отмечены `Velocity` и `Pose`.

## 6. Как понять, нужен ли новый интерфейс узла

Добавляйте новый direct-call интерфейс только если одновременно выполняются три условия:

1. узел находится в строгой детерминированной pipeline-цепочке;
2. следующему узлу нужен typed payload немедленно, а не через общий fan-out;
3. связь концептуально point-to-point, а не many-to-many.

Во всех остальных случаях предпочтительнее bus-подписка.

Иначе graph быстро превратится в смешанную сеть из случайных прямых указателей.

## 7. Что нельзя делать

Не стоит:

- передавать через bus ссылки на временные объекты;
- хранить в payload heap-owned поля вроде `std::string`;
- менять `TopicId` порядок без понимания, что это меняет приоритет dispatch;
- забывать обновлять `kTopicCount`;
- публиковать через `publishFromISR` без `kTopicAllowISRPublish`;
- сохранять `sample.payload` как долгоживущий указатель после callback.

Причина последнего пункта: payload хранится внутри slot-buffer шины и может быть
перезаписан следующей публикацией.

## 8. Минимальный checklist для нового топика

- Добавлен payload.
- Payload trivially copyable и standard layout.
- Обновлен `TopicId`.
- Обновлен `kTopicCount`.
- Descriptor зарегистрирован в `RuntimeGraph::Impl`.
- Добавлены subscriber arrays, если они нужны.
- Узел-публикатор реально вызывает `publish(...)` или `publishFromISR(...)`.
- Узлы-подписчики корректно кастуют `sample.payload`.
- Есть хотя бы один smoke-тест или существующий интеграционный сценарий, который
  проходит через новый топик.

## 9. Текущий рекомендуемый паттерн

Для этой кодовой базы сейчас лучший паттерн такой:

- upstream/hot-path блоки связывать через узкие direct-call интерфейсы;
- их же результаты зеркалить в event bus;
- downstream fan-out, trigger, logger, controller, health-monitor держать через bus.

Именно в таком режиме сейчас собран `RuntimeGraph`.
