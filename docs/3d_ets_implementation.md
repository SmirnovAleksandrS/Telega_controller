# Реализация 3D ETS/AETS-подобной калибровки магнитометра

Документ описывает практическую реализацию 3D-калибровки магнитометра в духе Extended Two-Step (ETS): по сырым трёхосевым измерениям оценивается смещение и линейное преобразование, переводящее измеренный эллипсоид обратно в сферу известного радиуса магнитного поля.

Основная опора: C. C. Foster, G. H. Elkaim, *Extension of a Two-Step Calibration Methodology to Include Nonorthogonal Sensor Axes*, IEEE TAES, 2008. Метод расширяет двухшаговую калибровку магнитометров на случай неортогональности осей для 2D- и 3D-сенсоров.

---

## 1. Входные данные и область применимости

Вход:

```python
M: np.ndarray, shape (N, 3)
# M[:, 0] = raw magnetometer x
# M[:, 1] = raw magnetometer y
# M[:, 2] = raw magnetometer z
```

Желательно:

```python
B_total: float | None
# Полный модуль магнитного поля Земли в тех же единицах, что и измерения.
# Например, в µT.
```

Если `B_total` неизвестно, можно откалибровать данные к единичной сфере. Абсолютный масштаб будет произвольным.

Требования к движениям:

- датчик должен менять ориентацию в 3D;
- точки должны покрывать заметную часть сферы;
- недопустима чисто плоская траектория, если требуется полноценная 3D-калибровка.

Хорошая 3D-калибровка требует возбуждения всех трёх осей. Если данные собраны только при yaw-вращении, 3D-эллипсоид будет плохо обусловлен или вообще неидентифицируем.

---

## 2. Модель измерения

Общая аффинная модель:

```math
m = L B + o,
```

где:

```math
m =
\begin{bmatrix}
\hat B_x \\
\hat B_y \\
\hat B_z
\end{bmatrix},
\quad
B =
\begin{bmatrix}
B_x \\
B_y \\
B_z
\end{bmatrix},
\quad
o =
\begin{bmatrix}
x_0 \\
y_0 \\
z_0
\end{bmatrix}.
```

`o` — hard iron / zero bias.

`L` — эффективная матрица, объединяющая:

- scale factors;
- soft iron;
- nonorthogonality axes;
- возможную несогласованность осей датчика в рамках выбранной модели.

Идеальное магнитное поле имеет постоянный модуль:

```math
\|B\|^2 = B_{total}^2.
```

Подставляя `B = L^{-1}(m-o)`, получаем эллипсоид в пространстве сырых измерений:

```math
(m-o)^T W (m-o) = B_{total}^2,
```

где:

```math
W = L^{-T}L^{-1}.
```

Калибровочная матрица:

```math
C = L^{-1}.
```

Тогда:

```math
B_{cal} = C(m-o).
```

И должно выполняться:

```math
\|B_{cal}\| \approx B_{total}.
```

---

## 3. Почему это эллипсоид

Общее уравнение квадрики в 3D:

```math
A x^2 + B y^2 + C z^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0.
```

В матричной форме:

```math
m^T Q m + 2g^T m + J = 0,
```

где:

```math
Q =
\begin{bmatrix}
A & D & E \\
D & B & F \\
E & F & C
\end{bmatrix},
\quad
g =
\begin{bmatrix}
G \\
H \\
I
\end{bmatrix}.
```

Центр эллипсоида:

```math
o = -Q^{-1}g.
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
q = o^T Q o - J.
```

Если `Q` положительно определена и `q > 0`, это эллипсоид.

Нормированная форма:

```math
W = Q \frac{B_{total}^2}{q}.
```

Тогда:

```math
(m-o)^T W (m-o) = B_{total}^2.
```

---

## 4. Численно устойчивая схема реализации

Рекомендуемый pipeline:

1. Очистить данные от NaN/Inf.
2. Проверить число точек и пространственное покрытие.
3. Нормализовать координаты.
4. Оценить квадрику/эллипсоид линейным МНК.
5. Проверить положительную определённость.
6. Извлечь центр `o` и матрицу `W`.
7. Получить калибровочную матрицу `C`.
8. Отбросить выбросы через residual.
9. Повторить fit или запустить IRLS/RANSAC.
10. Сделать финальную нелинейную оптимизацию по radial residual.
11. Проверить качество результата.

---

## 5. Предобработка

### 5.1. Очистка данных

```python
import numpy as np

M = np.asarray(M, dtype=float)
mask = np.isfinite(M).all(axis=1)
M = M[mask]
```

### 5.2. Минимальный размер выборки

Теоретически общая квадрика имеет 10 коэффициентов с одним масштабным множителем, то есть 9 степеней свободы. Практически нужно намного больше точек:

```python
if len(M) < 100:
    raise ValueError("Too few samples for 3D magnetometer calibration")
```

Рекомендуемые значения:

- минимум: `N >= 100`;
- нормально: `N >= 500`;
- хорошо: `N >= 2000`.

### 5.3. Проверка 3D-покрытия

Грубая проверка через SVD центрированных данных:

```python
X0 = M - np.median(M, axis=0)
svals = np.linalg.svd(X0, compute_uv=False)
ratio = svals[-1] / svals[0]
```

Если:

```python
ratio < 0.05
```

данные почти плоские или линейные. Полноценная 3D-калибровка будет ненадёжной.

Более мягкие ориентиры:

```text
ratio > 0.10  — приемлемо;
ratio > 0.20  — хорошо;
ratio > 0.30  — отлично.
```

---

## 6. Нормализация координат

Перед fitting нужно нормализовать данные:

```python
mu = M.mean(axis=0)
s = M.std(axis=0)
s[s == 0] = 1.0
X = (M - mu) / s
```

Если в нормированных координатах получено:

```math
(x_n-o_n)^T W_n (x_n-o_n) = B^2,
```

то в исходных координатах:

```math
o = \mu + S o_n,
```

```math
W = S^{-T} W_n S^{-1},
```

где:

```math
S = \operatorname{diag}(s_x,s_y,s_z).
```

Код:

```python
S = np.diag(s)
S_inv = np.diag(1.0 / s)
o = mu + S @ o_n
W = S_inv.T @ W_n @ S_inv
```

---

## 7. Первичная оценка эллипсоида

### 7.1. Algebraic least squares

Для каждой точки `x,y,z` строим строку design matrix:

```python
Dmat = np.column_stack([
    x*x,
    y*y,
    z*z,
    2*x*y,
    2*x*z,
    2*y*z,
    2*x,
    2*y,
    2*z,
    np.ones_like(x),
])
```

И решаем:

```math
Dmat \theta \approx 0.
```

Простое решение через SVD:

```python
_, _, vh = np.linalg.svd(Dmat, full_matrices=False)
theta = vh[-1]
```

Минус: обычный SVD может дать не эллипсоид. Поэтому после fit обязательно проверяем `Q > 0` и `q > 0`. Если нужно максимально строгое ellipsoid-specific fitting, лучше использовать constrained least squares по Li–Griffiths, но для инженерной реализации часто хватает SVD + нормализация + нелинейное уточнение.

---

## 8. Преобразование квадрики в `o` и `W`

Пусть:

```python
A, B, C, D, E, F, G, H, I, J = theta
```

Тогда:

```python
Q = np.array([
    [A, D, E],
    [D, B, F],
    [E, F, C],
])

g = np.array([G, H, I])
```

Проверка и исправление общего знака:

```python
if np.any(np.linalg.eigvalsh(Q) <= 0):
    Q = -Q
    g = -g
    J = -J

if np.any(np.linalg.eigvalsh(Q) <= 0):
    raise ValueError("Fitted quadric is not an ellipsoid")
```

Центр:

```python
o = -np.linalg.solve(Q, g)
```

Нормировочный коэффициент:

```python
q = o @ Q @ o - J
if q <= 0:
    raise ValueError("Invalid ellipsoid normalization")
```

Если известен `B_total`:

```python
W = Q * (B_total * B_total / q)
```

Если неизвестен:

```python
W = Q / q
B_total = 1.0
```

---

## 9. Получение калибровочной матрицы

Нужно найти `C`, такую что:

```math
C^T C = W.
```

Тогда:

```math
\|C(m-o)\|^2 = (m-o)^T W (m-o).
```

### Вариант A: симметрический SPD-корень

```python
eval_, evec = np.linalg.eigh(W)
if np.any(eval_ <= 0):
    raise ValueError("W is not positive definite")
C_cal = evec @ np.diag(np.sqrt(eval_)) @ evec.T
```

Это самый простой и устойчивый вариант.

### Вариант B: Cholesky-корень

```python
C_cal = np.linalg.cholesky(W).T
```

Так как `np.linalg.cholesky(W)` возвращает нижнетреугольную `R`, для которой:

```math
W = R R^T,
```

можно взять:

```math
C = R^T,
```

и получить:

```math
C^T C = W.
```

Cholesky-вариант удобен, если хочешь получить треугольную ETS-подобную параметризацию.

---

## 10. Применение калибровки

```python
def apply_calibration_3d(M_raw, offset, C_cal):
    return (C_cal @ (M_raw - offset).T).T
```

После калибровки:

```python
M_cal = apply_calibration_3d(M, o, C_cal)
radius = np.linalg.norm(M_cal, axis=1)
residual = radius - B_total
```

---

## 11. Робастность к выбросам

Физический residual:

```math
r_i = \|C(m_i-o)\| - B_{total}.
```

Код:

```python
Z = (C_cal @ (M - o).T).T
r = np.linalg.norm(Z, axis=1) - B_total
```

Оценка масштаба:

```python
sigma = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-12
```

Отбраковка:

```python
inliers = np.abs(r) < 3.0 * sigma
```

Рекомендуемая простая схема:

```python
for _ in range(3):
    theta = fit_ellipsoid_svd(M[inliers])
    o, W = quadric_to_center_W(theta, B_total)
    C = sqrt_spd(W)
    r = radial_residual_3d(M, o, C, B_total)
    sigma = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-12
    inliers = np.abs(r) < 3.0 * sigma
```

---

## 12. IRLS для 3D

IRLS полезен, когда выбросы умеренные, а не катастрофические.

Веса Хьюбера:

```python
def huber_weights(r, sigma, c=1.345):
    t = r / (c * sigma + 1e-12)
    w = np.ones_like(t)
    mask = np.abs(t) > 1.0
    w[mask] = 1.0 / np.abs(t[mask])
    return w
```

Weighted SVD можно сделать через умножение строк design matrix на `sqrt(w)`.

Псевдокод:

```python
theta = fit_ellipsoid_svd(M)

for _ in range(max_iter):
    o, W = quadric_to_center_W(theta, B_total)
    C = sqrt_spd(W)
    r = radial_residual_3d(M, o, C, B_total)
    sigma = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-12
    w = huber_weights(r, sigma)
    theta = fit_ellipsoid_svd(M, weights=w)
```

---

## 13. RANSAC для 3D

RANSAC нужен, если много грубых выбросов.

Общая квадрика имеет 9 независимых степеней свободы с учётом масштабной неоднозначности. Минимальная выборка — 9 точек, но для устойчивости лучше брать 20–50 точек.

Псевдокод:

```python
best_inliers = None
best_score = -1

for trial in range(max_trials):
    idx = rng.choice(N, size=sample_size, replace=False)
    try:
        theta = fit_ellipsoid_svd(M[idx])
        o, W = quadric_to_center_W(theta, B_total)
        C = sqrt_spd(W)
    except Exception:
        continue

    r = radial_residual_3d(M, o, C, B_total)
    inliers = np.abs(r) < threshold
    score = inliers.sum()

    if score > best_score:
        best_score = score
        best_inliers = inliers

final_theta = fit_ellipsoid_svd(M[best_inliers])
```

Рекомендуемые параметры:

```python
sample_size = 30
max_trials = 300
threshold = max(0.03 * B_total, 3 * expected_noise_std)
```

---

## 14. Финальная нелинейная оптимизация

Линейный fitting эллипсоида даёт хорошую начальную оценку, но не оптимизирует физическую ошибку. Финальный шаг лучше делать через `scipy.optimize.least_squares`.

Минимизируем:

```math
r_i = \|C(m_i-o)\| - B_{total}.
```

Параметризуем `C` как нижнетреугольную матрицу с положительной диагональю:

```math
C =
\begin{bmatrix}
\exp(s_1) & 0 & 0 \\
c_{21} & \exp(s_2) & 0 \\
c_{31} & c_{32} & \exp(s_3)
\end{bmatrix}.
```

Параметры:

```python
p = [ox, oy, oz, s1, c21, s2, c31, c32, s3]
```

Код:

```python
from scipy.optimize import least_squares
import numpy as np


def unpack_3d_params(p):
    ox, oy, oz, s1, c21, s2, c31, c32, s3 = p
    o = np.array([ox, oy, oz])
    C = np.array([
        [np.exp(s1), 0.0, 0.0],
        [c21, np.exp(s2), 0.0],
        [c31, c32, np.exp(s3)],
    ])
    return o, C


def residual_3d_params(p, M, B_total):
    o, C = unpack_3d_params(p)
    Z = (C @ (M - o).T).T
    return np.linalg.norm(Z, axis=1) - B_total
```

Запуск:

```python
res = least_squares(
    residual_3d_params,
    p0,
    args=(M_inliers, B_total),
    loss="soft_l1",
    f_scale=expected_noise_std,
    max_nfev=300,
)

o_refined, C_refined = unpack_3d_params(res.x)
```

Почему это полезно:

- algebraic LS минимизирует не физическую ошибку;
- nonlinear LS минимизирует ошибку радиуса сферы;
- `soft_l1` или `huber` делает доводку устойчивой к оставшимся выбросам.

---

## 15. Как получить начальный `p0` для nonlinear optimization

После линейного fit есть `o` и `C_cal`.

Нужно представить `C_cal` нижнетреугольной матрицей. Если использовался симметрический корень, можно взять Cholesky от `W`:

```python
R = np.linalg.cholesky(W)       # W = R R.T, R lower
C0 = R.T                        # C0.T C0 = W, upper
```

Но наша параметризация выше нижнетреугольная. Можно либо изменить параметризацию на верхнетреугольную, либо взять:

```python
C0 = np.linalg.cholesky(W)      # lower
```

Тогда residual `||C0.T z||` и `||C0 z||` вообще говоря соответствуют разным матрицам. Для согласованности проще параметризовать `C` как верхнетреугольную:

```math
C =
\begin{bmatrix}
\exp(s_1) & c_{12} & c_{13} \\
0 & \exp(s_2) & c_{23} \\
0 & 0 & \exp(s_3)
\end{bmatrix}.
```

Тогда можно взять:

```python
R = np.linalg.cholesky(W)       # W = R R.T
C0 = R.T                        # C0.T C0 = W
```

Инициализация:

```python
p0 = np.array([
    o[0], o[1], o[2],
    np.log(C0[0, 0]), C0[0, 1], C0[0, 2],
    np.log(C0[1, 1]), C0[1, 2],
    np.log(C0[2, 2]),
])
```

Для верхнетреугольного варианта:

```python
def unpack_3d_params_upper(p):
    ox, oy, oz, s1, c12, c13, s2, c23, s3 = p
    o = np.array([ox, oy, oz])
    C = np.array([
        [np.exp(s1), c12, c13],
        [0.0, np.exp(s2), c23],
        [0.0, 0.0, np.exp(s3)],
    ])
    return o, C
```

---

## 16. Рекомендуемый публичный API

```python
from dataclasses import dataclass
import numpy as np


@dataclass
class Calib3DResult:
    offset: np.ndarray          # shape (3,)
    W: np.ndarray               # shape (3, 3), quadratic form
    C: np.ndarray               # shape (3, 3), calibration matrix
    B_total: float
    inlier_mask: np.ndarray
    rms_residual: float
    max_abs_residual: float
    rel_rms_residual: float
    coverage_ratio: float
    condition_number: float
    optimizer_success: bool
    optimizer_cost: float | None


def calibrate_3d_ets(
    M: np.ndarray,
    B_total: float | None = None,
    robust: str = "irls",        # "none", "trim", "irls", "ransac"
    refine: bool = True,
    expected_noise_std: float | None = None,
    max_iter: int = 10,
    random_state: int | None = None,
) -> Calib3DResult:
    ...


def apply_calibration_3d(M_raw: np.ndarray, result: Calib3DResult) -> np.ndarray:
    return (result.C @ (M_raw - result.offset).T).T
```

---

## 17. Полный скелет ключевых функций

```python
import numpy as np
from scipy.optimize import least_squares


def normalize_points_3d(M):
    mu = M.mean(axis=0)
    s = M.std(axis=0)
    s[s == 0] = 1.0
    X = (M - mu) / s
    return X, mu, s


def fit_ellipsoid_svd(X, weights=None):
    x = X[:, 0]
    y = X[:, 1]
    z = X[:, 2]

    D = np.column_stack([
        x*x,
        y*y,
        z*z,
        2*x*y,
        2*x*z,
        2*y*z,
        2*x,
        2*y,
        2*z,
        np.ones_like(x),
    ])

    if weights is not None:
        w = np.asarray(weights, dtype=float)
        w = np.maximum(w, 0.0)
        D = D * np.sqrt(w)[:, None]

    _, _, vh = np.linalg.svd(D, full_matrices=False)
    theta = vh[-1]
    return theta / np.linalg.norm(theta)


def quadric_to_center_W(theta, B_total=1.0):
    A, B, C, D, E, F, G, H, I, J = theta

    Q = np.array([
        [A, D, E],
        [D, B, F],
        [E, F, C],
    ], dtype=float)
    g = np.array([G, H, I], dtype=float)

    if np.any(np.linalg.eigvalsh(Q) <= 0):
        Q = -Q
        g = -g
        J = -J

    if np.any(np.linalg.eigvalsh(Q) <= 0):
        raise ValueError("Quadric is not a valid ellipsoid")

    o = -np.linalg.solve(Q, g)
    q = o @ Q @ o - J
    if q <= 0:
        raise ValueError("Invalid ellipsoid normalization")

    W = Q * (B_total * B_total / q)
    return o, W


def denormalize_ellipsoid(o_n, W_n, mu, s):
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


def radial_residual_3d(M, o, C, B_total):
    Z = (C @ (M - o).T).T
    return np.linalg.norm(Z, axis=1) - B_total


def huber_weights(r, sigma, c=1.345):
    t = r / (c * sigma + 1e-12)
    w = np.ones_like(t)
    mask = np.abs(t) > 1.0
    w[mask] = 1.0 / np.abs(t[mask])
    return w


def unpack_3d_params_upper(p):
    ox, oy, oz, s1, c12, c13, s2, c23, s3 = p
    o = np.array([ox, oy, oz])
    C = np.array([
        [np.exp(s1), c12, c13],
        [0.0, np.exp(s2), c23],
        [0.0, 0.0, np.exp(s3)],
    ])
    return o, C


def residual_3d_params_upper(p, M, B_total):
    o, C = unpack_3d_params_upper(p)
    Z = (C @ (M - o).T).T
    return np.linalg.norm(Z, axis=1) - B_total
```

---

## 18. Контроль качества

После калибровки:

```python
M_cal = apply_calibration_3d(M, result)
r = np.linalg.norm(M_cal, axis=1) - result.B_total
```

Считать:

```python
rms = np.sqrt(np.mean(r**2))
max_abs = np.max(np.abs(r))
rel_rms = rms / result.B_total
cond_W = np.linalg.cond(result.W)
```

Ориентиры:

```text
Синтетика без выбросов: rel_rms < 1e-3 ... 1e-2
Хорошие реальные данные: rel_rms ~ 0.5% ... 2%
Плохие реальные данные: rel_rms > 5%
```

Также нужно смотреть:

```python
np.linalg.eigvalsh(result.W)
```

Все собственные значения должны быть положительными.

Большой condition number может означать:

- сильные искажения;
- плохое покрытие ориентаций;
- данные почти плоские;
- выбросы;
- неверный fit.

---

## 19. Как проверять на синтетических данных

Сгенерировать истинные точки на сфере:

```python
rng = np.random.default_rng(0)
N = 5000
v = rng.normal(size=(N, 3))
v /= np.linalg.norm(v, axis=1, keepdims=True)
B_total = 50.0
B_true = B_total * v
```

Задать ошибки:

```python
o_true = np.array([10.0, -5.0, 3.0])
L_true = np.array([
    [1.20, 0.05, -0.02],
    [0.00, 0.85, 0.04],
    [0.00, 0.00, 1.10],
])
noise = rng.normal(scale=0.05, size=(N, 3))
M_raw = (L_true @ B_true.T).T + o_true + noise
```

Калибровать:

```python
result = calibrate_3d_ets(M_raw, B_total=50.0, robust="irls", refine=True)
M_cal = apply_calibration_3d(M_raw, result)
```

Проверить:

```python
np.mean(np.linalg.norm(M_cal, axis=1))
np.std(np.linalg.norm(M_cal, axis=1))
```

Важно: из данных на сфере невозможно восстановить абсолютный поворот калиброванной системы без дополнительной информации. Поэтому `C_cal @ L_true` может отличаться от единичной матрицы на ортогональный поворот. Но модуль поля должен стать правильным.

---

## 20. Что невозможно восстановить только из магнитометра

По условию постоянного модуля магнитного поля можно восстановить эллипсоид и привести его к сфере. Но нельзя однозначно восстановить произвольный ортогональный поворот калиброванной системы.

Если:

```math
C(m-o)
```

даёт сферу, то:

```math
R C(m-o)
```

для любой ортогональной матрицы `R` тоже даёт сферу.

Поэтому магнитометрическая калибровка по модулю определяет scale/soft iron/bias, но не абсолютную ориентацию осей относительно корпуса без дополнительных ограничений или внешнего референса.

Для компаса это важно: если дальше нужен heading в системе корпуса, нужно либо:

- сохранять согласованную треугольную ETS-параметризацию;
- использовать механическую информацию о монтаже;
- использовать совместную калибровку с акселерометром/гироскопом;
- использовать внешний heading/reference.

---

## 21. Типовые ошибки реализации

### Ошибка 1. Использовать 3D fit на плоских данных

Если данные собраны только при yaw-вращении, полноценная 3D-калибровка не определяется. Нужно использовать 2D-метод или добавлять движения с roll/pitch.

### Ошибка 2. Не нормализовать координаты

Без нормализации SVD может дать нестабильную квадрику.

### Ошибка 3. Не проверять положительную определённость

Любой SVD что-то вернёт, но это может быть не эллипсоид.

### Ошибка 4. Оптимизировать algebraic residual вместо radial residual

Физическая ошибка — это ошибка радиуса сферы:

```math
\|C(m-o)\| - B_{total}.
```

### Ошибка 5. Ожидать восстановления абсолютной ориентации

По одному только модулю магнитного поля абсолютный поворот неидентифицируем.

---

## 22. Минимальный сценарий использования

```python
M = load_raw_xyz()             # shape (N, 3)
B_total = 50.0                 # µT, взять из WMM/IGRF/NOAA

result = calibrate_3d_ets(
    M,
    B_total=B_total,
    robust="irls",
    refine=True,
    expected_noise_std=0.1,
)

M_cal = apply_calibration_3d(M, result)
```

---

## 23. Что писать в ВКР

Метод можно описать так:

1. При произвольном 3D-вращении идеальный магнитометр измеряет векторы, лежащие на сфере радиуса полного модуля магнитного поля.
2. Аффинные ошибки датчика переводят эту сферу в эллипсоид.
3. Калибровка сводится к оценке эллипсоида по сырым данным.
4. Центр эллипсоида даёт смещение магнитометра.
5. Нормированная квадратичная форма эллипсоида даёт матрицу компенсации scale/soft iron/nonorthogonality.
6. Полученная калибровочная матрица применяется к центрированным измерениям.
7. Для реальных данных применяются нормализация, отбраковка выбросов, IRLS/RANSAC и финальная nonlinear least squares оптимизация по ошибке радиуса сферы.

---

## 24. Ссылки

1. C. C. Foster, G. H. Elkaim, *Extension of a Two-Step Calibration Methodology to Include Nonorthogonal Sensor Axes*, IEEE Transactions on Aerospace and Electronic Systems, 2008. URL: https://users.soe.ucsc.edu/~elkaim/Documents/nonOrthogonality.pdf
2. D. Gebre-Egziabher, G. H. Elkaim, J. D. Powell, B. W. Parkinson, *Calibration of Strapdown Magnetometers in Magnetic Field Domain*, Journal of Aerospace Engineering, 2006. URL: https://users.soe.ucsc.edu/~elkaim/Documents/magcal.pdf
3. Q. Li, J. G. Griffiths, *Least Squares Ellipsoid Specific Fitting*, Geometric Modeling and Processing, 2004.
