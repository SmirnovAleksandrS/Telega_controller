# Реализация 2D ETS/AETS-подобной калибровки магнитометра

Документ описывает практическую реализацию 2D-варианта метода Extended Two-Step (ETS) для магнитометра. Цель — по набору сырых измерений двух осей магнитометра получить смещение, масштабные коэффициенты и неортогональность осей, а затем применять калибровку к новым данным.

Основная опора: C. C. Foster, G. H. Elkaim, *Extension of a Two-Step Calibration Methodology to Include Nonorthogonal Sensor Axes*, IEEE TAES, 2008. В этой статье 2D-случай формулируется через окружность горизонтальной компоненты магнитного поля, деформированную ошибками датчика в эллипс.

---

## 1. Входные данные и область применимости

Вход:

```python
M: np.ndarray, shape (N, 2)
# M[:, 0] = raw magnetometer x
# M[:, 1] = raw magnetometer y
```

Желательно:

```python
B_h: float | None
# Горизонтальная компонента магнитного поля Земли в тех же единицах,
# что и измерения магнитометра. Например, в µT.
```

Если `B_h` неизвестно, метод всё равно может привести данные к единичной окружности, но абсолютный масштаб магнитного поля будет произвольным.

Метод применим, если во время сбора данных датчик вращался в основном вокруг вертикальной оси, то есть истинная горизонтальная компонента поля в теле датчика проходила окружность:

```math
B_x^2 + B_y^2 = B_h^2.
```

Типовые движения:

- поворот платформы на месте на 360°;
- несколько оборотов с разной скоростью;
- движение по плоскости с достаточным изменением heading/yaw.

Метод плохо обусловлен, если данные покрывают только малую дугу, например 20–40° heading вместо полного или почти полного оборота.

---

## 2. Модель измерения

Используем 2D ETS-модель:

```math
\hat B_x = a B_x + x_0,
```

```math
\hat B_y = b(B_y\cos\rho + B_x\sin\rho) + y_0.
```

Где:

- `hat B_x, hat B_y` — сырые измерения;
- `B_x, B_y` — откалиброванные компоненты горизонтального магнитного поля;
- `x_0, y_0` — смещение центра эллипса, то есть hard iron + zero bias;
- `a, b` — эффективные scale factors;
- `rho` — эффективная неортогональность осей в 2D;
- `B_h` — радиус идеальной окружности.

В матричной форме:

```math
m = L B + o,
```

```math
m =
\begin{bmatrix}
\hat B_x \\
\hat B_y
\end{bmatrix},
\quad
B =
\begin{bmatrix}
B_x \\
B_y
\end{bmatrix},
\quad
o =
\begin{bmatrix}
x_0 \\
y_0
\end{bmatrix},
```

```math
L =
\begin{bmatrix}
a & 0 \\
b\sin\rho & b\cos\rho
\end{bmatrix}.
```

Калибровка нового измерения:

```math
B = L^{-1}(m-o).
```

В явном виде:

```math
B_x = \frac{\hat B_x - x_0}{a},
```

```math
B_y = \frac{\hat B_y-y_0}{b\cos\rho}
      - \frac{\hat B_x-x_0}{a}\tan\rho.
```

---

## 3. Геометрическая постановка: fitting эллипса

Идеальные откалиброванные точки лежат на окружности:

```math
B_x^2+B_y^2=B_h^2.
```

Сырые точки лежат на эллипсе:

```math
A u^2 + Buv + C v^2 + D u + E v + F = 0,
```

где:

```math
u = \hat B_x,
\quad
v = \hat B_y.
```

В матричной форме:

```math
m^T Q m + d^T m + F = 0,
```

```math
Q =
\begin{bmatrix}
A & B/2 \\
B/2 & C
\end{bmatrix},
\quad
d =
\begin{bmatrix}
D \\
E
\end{bmatrix}.
```

Центр эллипса:

```math
o = -\frac{1}{2}Q^{-1}d.
```

После переноса в центр:

```math
z = m-o,
```

```math
z^T Q z = q,
```

где:

```math
q = o^T Q o - F.
```

Если `q <= 0` или `Q` не положительно определена, fitting дал невалидный эллипс.

---

## 4. Численно устойчивая схема реализации

Рекомендуемая структура алгоритма:

1. Предобработка данных.
2. Нормализация координат.
3. Первичная оценка эллипса.
4. Проверка валидности эллипса.
5. Извлечение параметров калибровки.
6. Робастная дооценка через IRLS/RANSAC, если нужны реальные данные.
7. Финальная нелинейная оптимизация по физическому residual.
8. Контроль качества.

---

## 5. Предобработка

### 5.1. Очистка NaN/Inf

```python
import numpy as np

M = np.asarray(M, dtype=float)
mask = np.isfinite(M).all(axis=1)
M = M[mask]
```

### 5.2. Минимальный размер выборки

Теоретически эллипс задаётся пятью независимыми точками, но практически нужно сильно больше:

```python
if len(M) < 50:
    raise ValueError("Too few samples for robust 2D calibration")
```

Хорошие практические значения:

- минимум: `N >= 50`;
- нормально: `N >= 200`;
- хорошо: `N >= 1000`.

### 5.3. Проверка покрытия углов

До калибровки можно грубо оценить покрытие через центрирование по медиане:

```python
center0 = np.median(M, axis=0)
angles = np.unwrap(np.arctan2(M[:, 1] - center0[1], M[:, 0] - center0[0]))
coverage = angles.max() - angles.min()
```

Для хорошей калибровки желательно:

```python
coverage > 1.5 * np.pi
```

То есть желательно покрыть хотя бы около 270°.

---

## 6. Нормализация координат перед fitting

Fitting коники чувствителен к масштабу чисел. Перед оценкой эллипса лучше нормировать данные:

```python
mu = M.mean(axis=0)
s = M.std(axis=0)
s[s == 0] = 1.0
X = (M - mu) / s
```

После оценки эллипса в нормированных координатах его нужно преобразовать обратно в исходные координаты. Чтобы не ошибиться, можно оценивать эллипс в нормированных координатах, извлекать центр и матрицу квадратичной формы там, а затем переносить их обратно:

Пусть:

```math
x_n = S^{-1}(x-\mu),
```

где:

```math
S = \operatorname{diag}(s_x, s_y).
```

Если в нормированных координатах:

```math
(x_n-o_n)^T W_n (x_n-o_n) = B_h^2,
```

то в исходных координатах:

```math
o = \mu + S o_n,
```

```math
W = S^{-T} W_n S^{-1}.
```

В коде:

```python
S = np.diag(s)
S_inv = np.diag(1.0 / s)
o = mu + S @ o_n
W = S_inv.T @ W_n @ S_inv
```

---

## 7. Первичная оценка эллипса

### 7.1. Простой algebraic least squares

Строим design matrix:

```python
u = X[:, 0]
v = X[:, 1]
Dmat = np.column_stack([u*u, u*v, v*v, u, v, np.ones_like(u)])
```

Решаем:

```math
Dmat \theta \approx 0,
```

```math
\theta = [A, B, C, D, E, F]^T.
```

Простейшая оценка через SVD:

```python
_, _, vh = np.linalg.svd(Dmat, full_matrices=False)
theta = vh[-1]
```

Минус: без специального ограничения SVD может дать гиперболу или параболу. Поэтому для инженерной реализации лучше использовать direct least squares ellipse fitting с эллиптическим ограничением.

### 7.2. Рекомендуемый вариант: Direct Least Squares Ellipse Fit

Используется ограничение:

```math
B^2 - 4AC < 0.
```

Классический алгоритм Fitzgibbon/Halir-Flusser разделяет квадратичные и линейные члены:

```python
D1 = np.column_stack([u*u, u*v, v*v])
D2 = np.column_stack([u, v, np.ones_like(u)])
S1 = D1.T @ D1
S2 = D1.T @ D2
S3 = D2.T @ D2
```

Далее:

```python
T = -np.linalg.solve(S3, S2.T)
M = S1 + S2 @ T
```

Матрица ограничения:

```python
C1 = np.array([
    [0.0, 0.0, 2.0],
    [0.0, -1.0, 0.0],
    [2.0, 0.0, 0.0],
])
```

Решаем generalized eigenproblem:

```python
eigvals, eigvecs = scipy.linalg.eig(M, C1)
```

Выбираем собственный вектор `a1 = [A, B, C]`, для которого:

```python
4*A*C - B*B > 0
```

Затем:

```python
a2 = T @ a1
theta = np.r_[a1, a2]
```

Полный код функции см. ниже.

---

## 8. Преобразование коэффициентов эллипса в параметры калибровки

### 8.1. Получение `Q`, `d`, `F`

```python
A, Bc, C, D, E, F = theta
Q = np.array([[A, Bc / 2.0],
              [Bc / 2.0, C]])
d = np.array([D, E])
```

### 8.2. Центр

```python
o = -0.5 * np.linalg.solve(Q, d)
```

### 8.3. Проверка положительной определённости

```python
eig = np.linalg.eigvalsh(Q)
if np.any(eig <= 0):
    Q = -Q
    d = -d
    F = -F
    eig = np.linalg.eigvalsh(Q)

if np.any(eig <= 0):
    raise ValueError("Fitted conic is not an ellipse")
```

### 8.4. Нормировка на физический радиус

```python
q = o @ Q @ o - F
if q <= 0:
    raise ValueError("Invalid ellipse normalization")
```

Если известен `B_h`:

```python
W = Q * (B_h * B_h / q)
```

Тогда:

```math
(m-o)^T W (m-o) = B_h^2.
```

Если `B_h is None`, используем единичный радиус:

```python
W = Q / q
B_h = 1.0
```

Тогда:

```math
(m-o)^T W (m-o) = 1.
```

---

## 9. Извлечение `a`, `b`, `rho`

Для 2D ETS-модели:

```math
W = L^{-T}L^{-1}.
```

Где:

```math
L =
\begin{bmatrix}
a & 0 \\
b\sin\rho & b\cos\rho
\end{bmatrix}.
```

Отсюда для матрицы:

```math
W =
\begin{bmatrix}
W_{11} & W_{12} \\
W_{12} & W_{22}
\end{bmatrix}
```

получаем:

```math
\sin\rho = -\frac{W_{12}}{\sqrt{W_{11}W_{22}}},
```

```math
\rho = \arcsin\left(-\frac{W_{12}}{\sqrt{W_{11}W_{22}}}\right),
```

```math
a = \frac{1}{\cos\rho\sqrt{W_{11}}},
```

```math
b = \frac{1}{\cos\rho\sqrt{W_{22}}}.
```

В коде:

```python
W11, W12, W22 = W[0, 0], W[0, 1], W[1, 1]
sin_rho = -W12 / np.sqrt(W11 * W22)
sin_rho = np.clip(sin_rho, -1.0, 1.0)
rho = np.arcsin(sin_rho)
cos_rho = np.cos(rho)

a = 1.0 / (cos_rho * np.sqrt(W11))
b = 1.0 / (cos_rho * np.sqrt(W22))
```

Внимание: это извлечение предполагает именно треугольную ETS-параметризацию `L`. Если тебе нужен только калибровочный оператор, можно не извлекать `a,b,rho`, а сразу использовать матрицу `C_cal = sqrt(W)`.

---

## 10. Калибровочная матрица

Есть два эквивалентных варианта применения.

### Вариант A: через ETS-параметры

```python
L = np.array([
    [a, 0.0],
    [b * np.sin(rho), b * np.cos(rho)],
])
C_cal = np.linalg.inv(L)
```

Калибровка:

```python
B_cal = (C_cal @ (M_raw - o).T).T
```

### Вариант B: через симметрический корень матрицы `W`

```python
eval_, evec = np.linalg.eigh(W)
C_cal = evec @ np.diag(np.sqrt(eval_)) @ evec.T
B_cal = (C_cal @ (M_raw - o).T).T
```

Вариант B проще и устойчивее, но он может дополнительно повернуть систему координат относительно исходной ETS-параметризации. Для heading это обычно допустимо, если дальше heading считается согласованно. Для строгого соответствия модели Foster–Elkaim лучше использовать вариант A.

---

## 11. Робастность к выбросам

Реальные магнитные данные часто содержат выбросы: локальные ферромагнитные предметы, токи, скачки АЦП, плохие участки движения.

Рекомендуемая схема:

1. Первичный fit по всем данным.
2. Вычисление residual:

```math
r_i = \sqrt{(m_i-o)^T W (m_i-o)} - B_h.
```

3. Оценка масштаба шума через MAD:

```python
sigma = 1.4826 * np.median(np.abs(r - np.median(r)))
```

4. Отбраковка:

```python
inliers = np.abs(r) < 3.0 * sigma
```

5. Повторный fit по inliers.

Для более сильной робастности использовать IRLS или RANSAC.

---

## 12. IRLS: устойчивое уточнение эллипса

IRLS — хороший компромисс между скоростью и устойчивостью.

Псевдокод:

```python
theta = fit_ellipse_direct(M)
for _ in range(max_iter):
    o, W = ellipse_to_center_W(theta, B_h)
    r = np.sqrt(np.einsum("ni,ij,nj->n", M - o, W, M - o)) - B_h
    sigma = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-12
    t = r / (1.345 * sigma)
    weights = np.ones_like(t)
    mask = np.abs(t) > 1.0
    weights[mask] = 1.0 / np.abs(t[mask])
    theta = fit_ellipse_direct(M, weights=weights)
```

В функции `fit_ellipse_direct` веса можно внести как умножение строк design matrix на `sqrt(weights)`.

---

## 13. RANSAC: когда есть грубые выбросы

RANSAC полезен, если выбросов много и они не похожи на гауссов шум.

Минимальная выборка для эллипса — 5 точек, но практически лучше брать 8–20 точек, чтобы не ловить вырожденные решения.

Псевдокод:

```python
best_inliers = None
best_score = -1

for trial in range(max_trials):
    idx = rng.choice(N, size=sample_size, replace=False)
    try:
        theta = fit_ellipse_direct(M[idx])
        o, W = ellipse_to_center_W(theta, B_h)
    except Exception:
        continue

    r = np.sqrt(np.einsum("ni,ij,nj->n", M - o, W, M - o)) - B_h
    inliers = np.abs(r) < threshold
    score = inliers.sum()

    if score > best_score:
        best_score = score
        best_inliers = inliers

final_theta = fit_ellipse_direct(M[best_inliers])
```

Рекомендуемые параметры:

```python
sample_size = 12
max_trials = 200
threshold = max(0.03 * B_h, 3 * expected_noise_std)
```

Если `B_h` неизвестно, threshold задаётся в нормированных единицах.

---

## 14. Финальная нелинейная оптимизация

После аналитического fit полезно сделать нелинейное уточнение по физическому residual:

```math
r_i = \|C(m_i-o)\| - B_h.
```

Параметризуем:

```python
params = [o_x, o_y, l11, l21, l22]
```

где `C` — нижнетреугольная матрица:

```math
C =
\begin{bmatrix}
\exp(l_{11}) & 0 \\
l_{21} & \exp(l_{22})
\end{bmatrix}.
```

Экспоненты гарантируют положительную диагональ.

Код residual:

```python
from scipy.optimize import least_squares


def unpack_2d_params(p):
    ox, oy, s11, c21, s22 = p
    C = np.array([
        [np.exp(s11), 0.0],
        [c21, np.exp(s22)],
    ])
    o = np.array([ox, oy])
    return o, C


def residual_2d(p, M, B_h):
    o, C = unpack_2d_params(p)
    Z = (M - o) @ C.T
    return np.linalg.norm(Z, axis=1) - B_h
```

Запуск:

```python
res = least_squares(
    residual_2d,
    p0,
    args=(M, B_h),
    loss="soft_l1",
    f_scale=expected_noise_std,
    max_nfev=200,
)
```

Параметр `loss="soft_l1"` делает оптимизацию устойчивее к выбросам.

---

## 15. Рекомендуемый публичный API

```python
from dataclasses import dataclass
import numpy as np


@dataclass
class Calib2DResult:
    offset: np.ndarray        # shape (2,)
    W: np.ndarray             # shape (2, 2), quadratic form
    C: np.ndarray             # shape (2, 2), calibration matrix
    L: np.ndarray | None      # ETS forward matrix, if extracted
    a: float | None
    b: float | None
    rho: float | None
    B_h: float
    inlier_mask: np.ndarray
    rms_residual: float
    max_abs_residual: float
    coverage_rad: float
    condition_number: float


def calibrate_2d_ets(
    M: np.ndarray,
    B_h: float | None = None,
    robust: str = "irls",        # "none", "trim", "irls", "ransac"
    refine: bool = True,
    expected_noise_std: float | None = None,
    max_iter: int = 10,
    random_state: int | None = None,
) -> Calib2DResult:
    ...


def apply_calibration_2d(M_raw: np.ndarray, result: Calib2DResult) -> np.ndarray:
    return (result.C @ (M_raw - result.offset).T).T
```

---

## 16. Полный скелет ключевых функций

```python
import numpy as np
import scipy.linalg
from scipy.optimize import least_squares


def normalize_points_2d(M):
    mu = M.mean(axis=0)
    s = M.std(axis=0)
    s[s == 0] = 1.0
    X = (M - mu) / s
    return X, mu, s


def fit_ellipse_direct(X, weights=None):
    x = X[:, 0]
    y = X[:, 1]

    if weights is None:
        w = np.ones(len(X))
    else:
        w = np.asarray(weights, dtype=float)
        w = np.maximum(w, 0.0)

    sw = np.sqrt(w)

    D1 = np.column_stack([x*x, x*y, y*y]) * sw[:, None]
    D2 = np.column_stack([x, y, np.ones_like(x)]) * sw[:, None]

    S1 = D1.T @ D1
    S2 = D1.T @ D2
    S3 = D2.T @ D2

    # Небольшая регуляризация помогает при почти вырожденных данных.
    reg = 1e-12 * np.trace(S3) / 3.0
    S3 = S3 + reg * np.eye(3)

    T = -np.linalg.solve(S3, S2.T)
    Mmat = S1 + S2 @ T

    C1 = np.array([
        [0.0, 0.0, 2.0],
        [0.0, -1.0, 0.0],
        [2.0, 0.0, 0.0],
    ])

    eigvals, eigvecs = scipy.linalg.eig(Mmat, C1)
    eigvecs = np.real(eigvecs)

    candidates = []
    for k in range(eigvecs.shape[1]):
        a1 = eigvecs[:, k]
        A, B, C = a1
        if np.isfinite(a1).all() and 4*A*C - B*B > 0:
            a2 = T @ a1
            theta = np.r_[a1, a2]
            candidates.append(theta)

    if not candidates:
        raise ValueError("No valid ellipse candidate")

    # Выбираем кандидата с минимальным algebraic residual.
    Dfull = np.column_stack([x*x, x*y, y*y, x, y, np.ones_like(x)])
    theta = min(candidates, key=lambda th: np.mean((Dfull @ th) ** 2))
    return theta / np.linalg.norm(theta)


def conic_to_center_W(theta, B_h=1.0):
    A, B, C, D, E, F = theta
    Q = np.array([[A, B / 2.0], [B / 2.0, C]], dtype=float)
    d = np.array([D, E], dtype=float)

    # Исправляем общий знак, если нужно.
    if np.any(np.linalg.eigvalsh(Q) <= 0):
        Q = -Q
        d = -d
        F = -F

    if np.any(np.linalg.eigvalsh(Q) <= 0):
        raise ValueError("Conic is not a valid ellipse")

    o = -0.5 * np.linalg.solve(Q, d)
    q = o @ Q @ o - F
    if q <= 0:
        raise ValueError("Invalid ellipse radius parameter")

    W = Q * (B_h * B_h / q)
    return o, W


def denormalize_ellipse(o_n, W_n, mu, s):
    S = np.diag(s)
    S_inv = np.diag(1.0 / s)
    o = mu + S @ o_n
    W = S_inv.T @ W_n @ S_inv
    return o, W


def sqrt_spd(W):
    vals, vecs = np.linalg.eigh(W)
    if np.any(vals <= 0):
        raise ValueError("Matrix is not SPD")
    return vecs @ np.diag(np.sqrt(vals)) @ vecs.T


def extract_2d_ets_params(o, W):
    W11 = W[0, 0]
    W12 = W[0, 1]
    W22 = W[1, 1]

    sin_rho = -W12 / np.sqrt(W11 * W22)
    sin_rho = np.clip(sin_rho, -1.0, 1.0)
    rho = np.arcsin(sin_rho)
    cos_rho = np.cos(rho)

    a = 1.0 / (cos_rho * np.sqrt(W11))
    b = 1.0 / (cos_rho * np.sqrt(W22))

    L = np.array([
        [a, 0.0],
        [b * np.sin(rho), b * np.cos(rho)],
    ])
    C_cal = np.linalg.inv(L)
    return a, b, rho, L, C_cal


def radial_residual_2d(M, o, W, B_h):
    Z = M - o
    r2 = np.einsum("ni,ij,nj->n", Z, W, Z)
    r2 = np.maximum(r2, 0.0)
    return np.sqrt(r2) - B_h
```

---

## 17. Контроль качества результата

После калибровки:

```python
M_cal = apply_calibration_2d(M, result)
radius = np.linalg.norm(M_cal, axis=1)
residual = radius - result.B_h
```

Считать:

```python
rms = np.sqrt(np.mean(residual**2))
max_abs = np.max(np.abs(residual))
rel_rms = rms / result.B_h
```

Хороший результат на чистых синтетических данных:

```text
rel_rms < 1e-3 ... 1e-2
```

На реальных дешёвых датчиках:

```text
rel_rms ~ 0.5% ... 5%
```

Дополнительные проверки:

```python
np.linalg.cond(result.W)
np.linalg.det(result.W) > 0
np.all(np.linalg.eigvalsh(result.W) > 0)
```

Слишком большой condition number означает, что эллипс очень вытянут или данные плохо покрывают окружность.

---

## 18. Типовые ошибки реализации

### Ошибка 1. Использование полного модуля поля вместо горизонтальной компоненты

Для 2D нужно использовать `B_h`, а не полный модуль `B_total`.

```math
B_h = B_{total}\cos I,
```

где `I` — магнитное наклонение.

### Ошибка 2. Отсутствие нормализации данных

Без нормализации координат fitting может быть нестабильным, особенно если значения в µT имеют смещения порядка десятков или сотен единиц.

### Ошибка 3. Fit по малой дуге

Если покрытие heading маленькое, эллипс плохо определяется. Алгоритм может вернуть математически валидный, но физически бессмысленный результат.

### Ошибка 4. Слепое доверие algebraic residual

Малый algebraic residual не гарантирует хорошую физическую калибровку. Контролировать нужно radial residual:

```math
r_i = \|B_{cal,i}\| - B_h.
```

### Ошибка 5. Неправильный выбор квадратного корня `W`

Симметрический корень `sqrt(W)` хорошо приводит эллипс к окружности, но может задавать повернутую систему координат. Если нужна ETS-интерпретация `a,b,rho`, используй треугольную матрицу `L` и `C=L^{-1}`.

---

## 19. Минимальный сценарий использования

```python
M = load_raw_xy()              # shape (N, 2)
B_h = 18.5                     # пример: µT, взять из WMM/IGRF/NOAA

result = calibrate_2d_ets(
    M,
    B_h=B_h,
    robust="irls",
    refine=True,
    expected_noise_std=0.1,
)

M_cal = apply_calibration_2d(M, result)
heading = np.arctan2(M_cal[:, 1], M_cal[:, 0])
```

---

## 20. Что писать в ВКР

В работе можно описать метод так:

1. Идеальный 2D-магнитометр при плоском вращении даёт окружность радиуса `B_h`.
2. Аффинные ошибки датчика переводят окружность в эллипс.
3. Калибровка сводится к оценке эллипса и построению обратного аффинного преобразования.
4. Смещение берётся как центр эллипса.
5. Матрица компенсации берётся как квадратный корень нормированной квадратичной формы или как обратная матрица ETS-параметризации.
6. Для устойчивости на реальных данных используются нормализация, отбраковка выбросов, IRLS/RANSAC и финальная nonlinear least squares оптимизация.

---

## 21. Ссылки

1. C. C. Foster, G. H. Elkaim, *Extension of a Two-Step Calibration Methodology to Include Nonorthogonal Sensor Axes*, IEEE Transactions on Aerospace and Electronic Systems, 2008. URL: https://users.soe.ucsc.edu/~elkaim/Documents/nonOrthogonality.pdf
2. D. Gebre-Egziabher, G. H. Elkaim, J. D. Powell, B. W. Parkinson, *Calibration of Strapdown Magnetometers in Magnetic Field Domain*, Journal of Aerospace Engineering, 2006. URL: https://users.soe.ucsc.edu/~elkaim/Documents/magcal.pdf
