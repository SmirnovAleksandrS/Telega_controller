# Математическая модель и алгоритм калибровки MEMS-магнитометрического компаса по плоскому кольцу и проезду холма

## 0. Назначение модели

Цель данного раздела — построить математически обоснованную процедуру калибровки магнитометрического MEMS-компаса в ситуации, естественной для ровера:

1. на ровной плоскости снимается полный поворот вокруг вертикальной оси, то есть плоское магнитометрическое кольцо;
2. затем ровер проезжает через холм в одном направлении;
3. затем ровер разворачивается примерно на $180^\circ$ и проезжает тот же холм в обратном направлении;
4. в расширенном варианте во время проездов используется акселерометр, позволяющий оценивать угол наклона ровера.

Нас интересует не полная трехмерная реконструкция магнитного поля, а только heading-функционал, то есть угол проекции измеренного магнитного вектора на плоскость $XY$ датчика:

$$
\psi = \operatorname{atan2}(m_y,m_x).
$$

Поэтому ниже отдельно выделяются только те компоненты полной линейной модели магнитометра, которые могут влиять на этот угол.

---

# 1. Исходная линейная модель магнитометра

Пусть истинный вектор магнитного поля в системе координат датчика равен

$$
\mathbf h =
\begin{bmatrix}
h_x \\
h_y \\
h_z
\end{bmatrix}.
$$

Сырое измерение магнитометра описывается аффинной моделью

$$
\mathbf m = \mathbf A\mathbf h + \mathbf b + \boldsymbol\varepsilon,
$$

где

$$
\mathbf m =
\begin{bmatrix}
m_x \\
m_y \\
m_z
\end{bmatrix},
$$

$$
\mathbf A =
\begin{bmatrix}
a_{11} & a_{12} & a_{13} \\
a_{21} & a_{22} & a_{23} \\
a_{31} & a_{32} & a_{33}
\end{bmatrix},
$$

$$
\mathbf b =
\begin{bmatrix}
b_x \\
b_y \\
b_z
\end{bmatrix},
$$

а $\boldsymbol\varepsilon$ — шум измерения.

Для дальнейшего анализа сначала будем рассматривать детерминированную часть модели:

$$
\mathbf m = \mathbf A\mathbf h + \mathbf b.
$$

Поскольку heading вычисляется только по первым двум координатам,

$$
\psi = \operatorname{atan2}(m_y,m_x),
$$

нам нужна только проекция модели на плоскость $XY$.

Введем

$$
\mathbf u =
\begin{bmatrix}
m_x \\
m_y
\end{bmatrix}.
$$

Тогда

$$
\mathbf u =
\begin{bmatrix}
a_{11} & a_{12} & a_{13} \\
a_{21} & a_{22} & a_{23}
\end{bmatrix}
\begin{bmatrix}
h_x \\
h_y \\
h_z
\end{bmatrix}
+
\begin{bmatrix}
b_x \\
b_y
\end{bmatrix}.
$$

Отделим горизонтальную и вертикальную части поля:

$$
\mathbf h_{xy}=
\begin{bmatrix}
h_x \\
h_y
\end{bmatrix}.
$$

Введем обозначения

$$
\mathbf C=
\begin{bmatrix}
a_{11} & a_{12} \\
a_{21} & a_{22}
\end{bmatrix},
$$

$$
\mathbf d=
\begin{bmatrix}
a_{13} \\
a_{23}
\end{bmatrix},
$$

$$
\mathbf c=
\begin{bmatrix}
b_x \\
b_y
\end{bmatrix}.
$$

Тогда heading-существенная часть модели имеет вид

$$
\boxed{
\mathbf u = \mathbf C\mathbf h_{xy}+\mathbf d h_z+\mathbf c
}
$$

или покомпонентно

$$
m_x = a_{11}h_x+a_{12}h_y+a_{13}h_z+b_x,
$$

$$
m_y = a_{21}h_x+a_{22}h_y+a_{23}h_z+b_y.
$$

---

# 2. Теорема о компонентах модели, существенных для heading

## 2.1. Формулировка теоремы

Пусть измерения магнитометра описываются моделью

$$
\mathbf m = \mathbf A\mathbf h + \mathbf b,
$$

где $\mathbf A\in\mathbb R^{3\times 3}$ и $\mathbf b\in\mathbb R^3$. Пусть heading определяется только направлением проекции измеренного вектора на плоскость $XY$:

$$
\psi = \operatorname{atan2}(m_y,m_x),
$$

при условии

$$
(m_x,m_y)\neq(0,0).
$$

Тогда:

1. значение $\psi$ зависит только от первых двух строк матрицы $\mathbf A$ и первых двух компонент вектора $\mathbf b$;
2. параметры $a_{31}$, $a_{32}$, $a_{33}$ и $b_z$ не влияют напрямую на heading-функционал $\operatorname{atan2}(m_y,m_x)$;
3. все heading-существенные искажения могут быть записаны в виде

$$
\boxed{
\mathbf u = \mathbf C\mathbf h_{xy}+\mathbf d h_z+\mathbf c
}
$$

с определениями

$$
\mathbf C=
\begin{bmatrix}
a_{11} & a_{12} \\
a_{21} & a_{22}
\end{bmatrix},
\qquad
\mathbf d=
\begin{bmatrix}
a_{13} \\
a_{23}
\end{bmatrix},
\qquad
\mathbf c=
\begin{bmatrix}
b_x \\
b_y
\end{bmatrix}.
$$

## 2.2. Доказательство

Введем матрицу проекции на первые две координаты:

$$
\mathbf P=
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0
\end{bmatrix}.
$$

Тогда

$$
\mathbf u = \mathbf P\mathbf m.
$$

Подставим полную модель:

$$
\mathbf u = \mathbf P(\mathbf A\mathbf h+\mathbf b).
$$

Раскроем скобки:

$$
\mathbf u = \mathbf P\mathbf A\mathbf h+\mathbf P\mathbf b.
$$

При этом

$$
\mathbf P\mathbf A=
\begin{bmatrix}
a_{11} & a_{12} & a_{13} \\
a_{21} & a_{22} & a_{23}
\end{bmatrix},
$$

а

$$
\mathbf P\mathbf b=
\begin{bmatrix}
b_x \\
b_y
\end{bmatrix}.
$$

Следовательно,

$$
\mathbf u=
\begin{bmatrix}
a_{11} & a_{12} & a_{13} \\
a_{21} & a_{22} & a_{23}
\end{bmatrix}
\begin{bmatrix}
h_x \\
h_y \\
h_z
\end{bmatrix}
+
\begin{bmatrix}
b_x \\
b_y
\end{bmatrix}.
$$

Компоненты $a_{31}$, $a_{32}$, $a_{33}$ и $b_z$ входят только в выражение для $m_z$:

$$
m_z=a_{31}h_x+a_{32}h_y+a_{33}h_z+b_z.
$$

Но $m_z$ не входит в функцию

$$
\psi=\operatorname{atan2}(m_y,m_x).
$$

Значит, при прямом вычислении heading по $m_x$ и $m_y$ эти параметры не могут изменить результат. Теорема доказана.

## 2.3. Следствие о несущественном масштабе

Пусть двумерный вектор $\mathbf u$ заменяется на

$$
\tilde{\mathbf u}=k\mathbf u,
\qquad k>0.
$$

Тогда

$$
\operatorname{atan2}(k u_y,k u_x)=\operatorname{atan2}(u_y,u_x).
$$

Значит, общий положительный масштаб в плоскости $XY$ не влияет на heading. Однако неравные масштабы по осям,

$$
\tilde u_x=s_xu_x,
\qquad
\tilde u_y=s_yu_y,
\qquad
s_x\neq s_y,
$$

в общем случае меняют угол:

$$
\operatorname{atan2}(s_yu_y,s_xu_x)\neq \operatorname{atan2}(u_y,u_x).
$$

Именно поэтому для heading важна не абсолютная длина вектора, а форма и направление его двумерной проекции.

---

# 3. Геометрия плоского кольца

## 3.1. Идеальное кольцо на плоскости

На ровной плоскости при повороте ровера вокруг вертикальной оси вертикальная компонента магнитного поля в системе датчика остается постоянной:

$$
h_z=H_v.
$$

Горизонтальная проекция истинного магнитного поля имеет постоянный модуль:

$$
H_h=\sqrt{h_x^2+h_y^2}.
$$

Ее можно параметризовать углом $\gamma$:

$$
h_x=H_h\cos\gamma,
$$

$$
h_y=H_h\sin\gamma.
$$

Тогда

$$
\mathbf h_{xy}=H_h
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}.
$$

Подставим в heading-модель:

$$
\mathbf u(\gamma)=\mathbf C H_h
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}
+\mathbf dH_v+\mathbf c.
$$

То есть

$$
\boxed{
\mathbf u(\gamma)=H_h\mathbf C
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}
+\mathbf q_0
}
$$

где

$$
\boxed{
\mathbf q_0=\mathbf c+\mathbf dH_v
}
$$

— центр плоского кольца.

## 3.2. Что можно оценить из одного кольца

Из одного плоского кольца можно оценить:

1. центр наблюдаемого эллипса $\mathbf q_0$;
2. двумерную форму эллипса, задаваемую матрицей $\mathbf C$ с точностью до общего масштаба и постоянного поворота фазы;
3. матрицу выпрямления эллипса в окружность.

Но из одного кольца нельзя разделить

$$
\mathbf c
$$

и

$$
\mathbf dH_v.
$$

Действительно, все точки кольца содержат один и тот же постоянный член

$$
\mathbf c+\mathbf dH_v.
$$

Поэтому для данных одного плоского кольца любой набор

$$
\mathbf c' = \mathbf c+\Delta\mathbf c,
$$

$$
\mathbf d' = \mathbf d-\frac{\Delta\mathbf c}{H_v}
$$

дает тот же центр:

$$
\mathbf c'+\mathbf d'H_v
=
\mathbf c+\Delta\mathbf c+\left(\mathbf d-\frac{\Delta\mathbf c}{H_v}\right)H_v
=
\mathbf c+\mathbf dH_v.
$$

Следовательно, плоское кольцо дает только эффективный центр $\mathbf q_0$, но не чистый hard-iron $\mathbf c$.

## 3.3. Почему из эллипса не извлекается сама матрица $\mathbf C$ однозначно

Пусть

$$
\mathbf v=\mathbf u-\mathbf q_0.
$$

Тогда

$$
\mathbf v=H_h\mathbf C
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}.
$$

Обозначим

$$
\mathbf e(\gamma)=
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix},
\qquad
\mathbf e^T\mathbf e=1.
$$

Если $\mathbf C$ обратима, то

$$
\mathbf e=\frac{1}{H_h}\mathbf C^{-1}\mathbf v.
$$

Из условия $\mathbf e^T\mathbf e=1$ получаем

$$
\frac{1}{H_h^2}\mathbf v^T\mathbf C^{-T}\mathbf C^{-1}\mathbf v=1.
$$

Следовательно,

$$
\mathbf v^T\mathbf Q_{2D}\mathbf v=1,
$$

где

$$
\mathbf Q_{2D}=\frac{1}{H_h^2}\mathbf C^{-T}\mathbf C^{-1}.
$$

Матрица $\mathbf Q_{2D}$ симметрична и имеет три независимых параметра. Матрица $\mathbf C$ имеет четыре параметра. Поэтому по геометрии одного эллипса нельзя однозначно восстановить все элементы $\mathbf C$.

Если заменить $\mathbf C$ на

$$
\mathbf C' = \mathbf C\mathbf R,
$$

где $\mathbf R$ — произвольная ортогональная матрица,

$$
\mathbf R^T\mathbf R=\mathbf I,
$$

то множество точек

$$
\mathbf C'\mathbf e(\gamma)=\mathbf C\mathbf R\mathbf e(\gamma)
$$

совпадает с исходным эллипсом, потому что $\mathbf R\mathbf e(\gamma)$ также пробегает единичную окружность. Значит, одно кольцо определяет форму эллипса, но не абсолютную фазу истинного угла $\gamma$.

Практически это означает: по плоскому кольцу можно построить калибровку, которая переводит эллипс в окружность, но нуль heading остается заданным с точностью до постоянной добавки.

---

# 4. Плоская калибровка кольца

## 4.1. Уравнение эллипса

Пусть сырые точки плоского кольца имеют вид

$$
\mathbf u_i=
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}.
$$

Эллипс можно задать квадратичной формой

$$
(\mathbf u-\mathbf q_0)^T\mathbf Q(\mathbf u-\mathbf q_0)=1,
$$

где

$$
\mathbf q_0=
\begin{bmatrix}
q_x \\
q_y
\end{bmatrix}
$$

— центр эллипса, а $\mathbf Q$ — симметричная положительно определенная матрица:

$$
\mathbf Q=\mathbf Q^T>0.
$$

Эквивалентная запись через общий конический вид:

$$
Ax^2+Bxy+Cy^2+Dx+Ey+F=0.
$$

В матричной форме:

$$
\mathbf u^T\mathbf G\mathbf u+\mathbf g^T\mathbf u+F=0,
$$

где

$$
\mathbf G=
\begin{bmatrix}
A & B/2 \\
B/2 & C
\end{bmatrix},
\qquad
\mathbf g=
\begin{bmatrix}
D \\
E
\end{bmatrix}.
$$

Центр эллипса находится из условия обращения в нуль линейной части после переноса:

$$
2\mathbf G\mathbf q_0+\mathbf g=0.
$$

Отсюда

$$
\boxed{
\mathbf q_0=-\frac{1}{2}\mathbf G^{-1}\mathbf g
}
$$

при условии, что $\mathbf G$ обратима.

После переноса $\mathbf u=\mathbf q_0+\mathbf v$ получаем

$$
\mathbf v^T\mathbf G\mathbf v=\rho^2,
$$

где

$$
\rho^2=\mathbf q_0^T\mathbf G\mathbf q_0-F.
$$

Тогда нормированная матрица эллипса равна

$$
\boxed{
\mathbf Q=\frac{\mathbf G}{\rho^2}
}
$$

и уравнение принимает вид

$$
(\mathbf u-\mathbf q_0)^T\mathbf Q(\mathbf u-\mathbf q_0)=1.
$$

## 4.2. Матрица выпрямления эллипса

Поскольку $\mathbf Q$ симметрична и положительно определена, ее можно разложить как

$$
\mathbf Q=\mathbf U\boldsymbol\Lambda\mathbf U^T,
$$

где

$$
\boldsymbol\Lambda=\operatorname{diag}(\lambda_1,\lambda_2),
\qquad
\lambda_1>0,
\quad
\lambda_2>0.
$$

Определим матрицу

$$
\boxed{
\mathbf W=\mathbf Q^{1/2}=\mathbf U\boldsymbol\Lambda^{1/2}\mathbf U^T
}
$$

где

$$
\boldsymbol\Lambda^{1/2}=\operatorname{diag}(\sqrt{\lambda_1},\sqrt{\lambda_2}).
$$

Тогда калиброванная двумерная точка равна

$$
\boxed{
\mathbf r=\mathbf W(\mathbf u-\mathbf q_0)
}
$$

и для точек плоского кольца выполняется

$$
\|\mathbf r\|^2=1.
$$

Таким образом, $\mathbf W$ переводит плоский эллипс в единичную окружность.

## 4.3. Важная интерпретация

После плоской калибровки мы работаем не с исходной $\mathbf u$, а с

$$
\mathbf r=\mathbf W(\mathbf u-\mathbf q_0).
$$

В этой системе плоское кольцо имеет вид

$$
\mathbf r_0(\gamma)=
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}
$$

с точностью до постоянного поворота нуля heading.

Далее все формулы будут записываться именно для этой нормализованной системы.

---

# 5. Модель вертикального leakage после плоской калибровки

Исходная heading-модель:

$$
\mathbf u=\mathbf C\mathbf h_{xy}+\mathbf d h_z+\mathbf c.
$$

Центр плоского кольца:

$$
\mathbf q_0=\mathbf c+\mathbf dH_v.
$$

Вычтем его:

$$
\mathbf u-\mathbf q_0=\mathbf C\mathbf h_{xy}+\mathbf d(h_z-H_v).
$$

Применим матрицу $\mathbf W$:

$$
\mathbf r=\mathbf W\mathbf C\mathbf h_{xy}+\mathbf W\mathbf d(h_z-H_v).
$$

Матрица $\mathbf W$ выбиралась так, чтобы на плоском кольце выполнялось

$$
\mathbf W\mathbf C H_h
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}
\approx
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}.
$$

То есть в выбранной нормализованной системе можно писать

$$
\mathbf W\mathbf C\mathbf h_{xy}\approx \frac{\mathbf h_{xy}}{H_h}.
$$

Введем параметр вертикального leakage в нормализованной системе:

$$
\boxed{
\boldsymbol\eta=H_h\mathbf W\mathbf d
}
$$

Тогда

$$
\mathbf W\mathbf d(h_z-H_v)=\boldsymbol\eta\frac{h_z-H_v}{H_h}.
$$

Итоговая модель после плоской калибровки:

$$
\boxed{
\mathbf r
=
\frac{\mathbf h_{xy}}{H_h}
+
\boldsymbol\eta\frac{h_z-H_v}{H_h}
+
\boldsymbol\nu
}
$$

где $\boldsymbol\nu$ — остаточный шум и ошибки плоской калибровки.

Главная задача проезда холма — оценить вектор

$$
\boldsymbol\eta=
\begin{bmatrix}
\eta_x \\
\eta_y
\end{bmatrix}.
$$

Если $\boldsymbol\eta=\mathbf 0$, то изменение вертикальной компоненты магнитного поля не вызывает дополнительного смещения точки в плоскости $XY$ после плоской калибровки.

Если $\boldsymbol\eta\neq\mathbf 0$, то при изменении наклона ровер получает систематическую heading-ошибку, потому что точка $\mathbf r$ смещается в плоскости $XY$.

---

# 6. Кинематическая модель проезда холма с акселерометром

## 6.1. Обозначения

Пусть после плоской калибровки направление ровера на ровном участке перед холмом задается углом $\gamma$:

$$
\mathbf r_0(\gamma)=
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}.
$$

Пусть угол pitch равен $\theta$. Будем считать, что pitch происходит вокруг оси $Y$ датчика. Знак $\theta$ выбирается согласно принятой в системе координат конвенции. В дальнейшем важно не абсолютное соглашение о знаке, а согласованность формул и данных акселерометра.

Обозначим отношение вертикальной и горизонтальной компонент магнитного поля Земли:

$$
\boxed{
k=\frac{H_v}{H_h}
}
$$

где $H_h$ — горизонтальная составляющая, а $H_v$ — вертикальная составляющая поля в локальной системе.

## 6.2. Компоненты истинного поля при pitch

Для направления $\gamma$ и pitch-угла $\theta$ компоненты поля в системе ровера имеют вид

$$
\frac{h_x(\theta,\gamma)}{H_h}
=
\cos\gamma\cos\theta+k\sin\theta,
$$

$$
\frac{h_y(\theta,\gamma)}{H_h}
=
\sin\gamma,
$$

$$
\frac{h_z(\theta,\gamma)-H_v}{H_h}
=
-\cos\gamma\sin\theta+k(\cos\theta-1).
$$

Эти формулы следуют из поворота вектора магнитного поля при наклоне корпуса вокруг оси $Y$.

## 6.3. Модель точки холмовой дуги после плоской калибровки

Подставим компоненты поля в модель

$$
\mathbf r
=
\frac{\mathbf h_{xy}}{H_h}
+
\boldsymbol\eta\frac{h_z-H_v}{H_h}.
$$

Получаем

$$
\boxed{
\mathbf r(\theta,\gamma)=
\begin{bmatrix}
\cos\gamma\cos\theta+k\sin\theta \\
\sin\gamma
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma\sin\theta+k(\cos\theta-1)\right]
}
$$

На ровном участке $\theta=0$:

$$
\mathbf r(0,\gamma)=
\begin{bmatrix}
\cos\gamma \\
\sin\gamma
\end{bmatrix}.
$$

Поэтому полезно рассматривать разность

$$
\Delta\mathbf r(\theta,
\gamma)=\mathbf r(\theta,
\gamma)-\mathbf r(0,
\gamma).
$$

Она равна

$$
\boxed{
\Delta\mathbf r(\theta,
\gamma)=
\begin{bmatrix}
\cos\gamma(\cos\theta-1)+k\sin\theta \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma\sin\theta+k(\cos\theta-1)\right]
}
$$

Первый член — идеальное изменение горизонтальной проекции поля из-за pitch. Второй член — систематическое смещение из-за вертикального leakage.

---

# 7. Оценивание leakage при известном pitch и известном $k$

Если акселерометр дает $\theta_i$, а $k=H_v/H_h$ известно, например из модели магнитного поля Земли или из внешнего измерения, задача становится линейной по $\boldsymbol\eta$.

Для каждого измерения вычисляем

$$
\mathbf r_i=\mathbf W(\mathbf u_i-\mathbf q_0).
$$

Пусть $\gamma_i$ — азимут соответствующего проезда на плоском участке. Для одного проезда обычно можно считать $\gamma_i=\gamma$.

Вычисляем

$$
\Delta\mathbf r_i=\mathbf r_i-\mathbf r(0,\gamma_i).
$$

Определим известный геометрический член

$$
\mathbf g_i=
\begin{bmatrix}
\cos\gamma_i(\cos\theta_i-1)+k\sin\theta_i \\
0
\end{bmatrix}.
$$

Определим скалярный регрессор

$$
z_i=-\cos\gamma_i\sin\theta_i+k(\cos\theta_i-1).
$$

Тогда модель становится

$$
\Delta\mathbf r_i-\mathbf g_i=\boldsymbol\eta z_i+\boldsymbol\nu_i.
$$

Обозначим

$$
\mathbf e_i=\Delta\mathbf r_i-\mathbf g_i.
$$

Тогда

$$
\mathbf e_i=\boldsymbol\eta z_i+\boldsymbol\nu_i.
$$

Оценка weighted least squares:

$$
\boxed{
\hat{\boldsymbol\eta}=
\frac{\sum_i w_i z_i\mathbf e_i}{\sum_i w_i z_i^2}
}
$$

где $w_i$ — вес измерения.

Ковариационная оценка при изотропном шуме $\sigma^2$:

$$
\operatorname{Cov}(\hat{\boldsymbol\eta})\approx
\frac{\sigma^2}{\sum_i w_i z_i^2}\mathbf I.
$$

Из этой формулы видно, что качество оценки определяется возбуждением

$$
\sum_i w_i z_i^2.
$$

Если все $z_i$ малы, то $\boldsymbol\eta$ плохо наблюдаем.

---

# 8. Два встречных проезда через холм

## 8.1. Общая модель для двух направлений

Обозначим два направления проезда:

$$
\gamma_+ \approx \gamma,
$$

$$
\gamma_- \approx \gamma+\pi.
$$

Для каждого измерения с индексом $i$ и направлением $s\in\{+,-\}$ имеем

$$
\Delta\mathbf r_{s,i}
=
\begin{bmatrix}
\cos\gamma_s(\cos\theta_{s,i}-1)+k\sin\theta_{s,i} \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma_s\sin\theta_{s,i}+k(\cos\theta_{s,i}-1)
\right]
+
\boldsymbol\nu_{s,i}.
$$

Эта формула является основной для реализации при наличии акселерометра. Она не требует предполагать, что $\theta_-$ равен $\theta_+$ или $-\theta_+$. Нужно просто подставлять реально измеренный pitch.

## 8.2. Частный случай: одинаковые pitch-профили по знаку

Если для сопоставленных точек выполнено

$$
\theta_{-,i}=\theta_{+,i}=\theta_i,
$$

и

$$
\gamma_- = \gamma_+ + \pi,
$$

то

$$
\cos\gamma_-=-\cos\gamma_+.
$$

Обозначим

$$
\gamma_+=\gamma.
$$

Тогда антисимметричная комбинация

$$
\frac{\Delta\mathbf r_{-,i}-\Delta\mathbf r_{+,i}}{2}
$$

дает

$$
\frac{\Delta\mathbf r_{-,i}-\Delta\mathbf r_{+,i}}{2}
=
\begin{bmatrix}
-\cos\gamma(\cos\theta_i-1) \\
0
\end{bmatrix}
+
\boldsymbol\eta\cos\gamma\sin\theta_i.
$$

Отсюда

$$
\mathbf y_i=
\frac{\Delta\mathbf r_{-,i}-\Delta\mathbf r_{+,i}}{2}
+
\begin{bmatrix}
\cos\gamma(\cos\theta_i-1) \\
0
\end{bmatrix}
$$

и

$$
\mathbf y_i=\boldsymbol\eta p_i+oldsymbol\nu_i,
$$

где

$$
p_i=\cos\gamma\sin\theta_i.
$$

Тогда

$$
\boxed{
\hat{\boldsymbol\eta}=
\frac{\sum_i w_i p_i\mathbf y_i}{\sum_i w_i p_i^2}
}
$$

В этом частном случае член с $k$ сокращается, поэтому $\boldsymbol\eta$ можно оценивать без знания $k$.

## 8.3. Частный случай: обратный проезд дает противоположный pitch

На реальном холме при движении в противоположную сторону pitch часто меняет знак. Тогда для сопоставленных физических точек может быть ближе условие

$$
\theta_{-,i}=-\theta_{+,i}=-\theta_i.
$$

В этом случае при $\gamma_- = \gamma+\pi$ получается

$$
\Delta\mathbf r_{+,i}=
\begin{bmatrix}
\cos\gamma(\cos\theta_i-1)+k\sin\theta_i \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma\sin\theta_i+k(\cos\theta_i-1)\right],
$$

$$
\Delta\mathbf r_{-,i}=
\begin{bmatrix}
-\cos\gamma(\cos\theta_i-1)-k\sin\theta_i \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma\sin\theta_i+k(\cos\theta_i-1)\right].
$$

Тогда leakage содержится уже в симметричной комбинации:

$$
\frac{\Delta\mathbf r_{+,i}+\Delta\mathbf r_{-,i}}{2}
=
\boldsymbol\eta
\left[-\cos\gamma\sin\theta_i+k(\cos\theta_i-1)\right].
$$

Если $k$ известно, то снова получаем линейную оценку

$$
\mathbf y_i=\frac{\Delta\mathbf r_{+,i}+\Delta\mathbf r_{-,i}}{2},
$$

$$
z_i=-\cos\gamma\sin\theta_i+k(\cos\theta_i-1),
$$

$$
\boxed{
\hat{\boldsymbol\eta}=
\frac{\sum_i w_i z_i\mathbf y_i}{\sum_i w_i z_i^2}
}
$$

Главный практический вывод: при наличии акселерометра не нужно заранее решать, какая из этих двух симметрий верна. Надежнее использовать общую модель из раздела 8.1 и подставлять реальные измеренные $\theta_{s,i}$.

---

# 9. Модель без акселерометра

## 9.1. Что теряется без угла pitch

Без акселерометра для точки холма неизвестно значение $\theta_i$. Тогда в модели

$$
\Delta\mathbf r_i=
\begin{bmatrix}
\cos\gamma_i(\cos\theta_i-1)+k\sin\theta_i \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma_i\sin\theta_i+k(\cos\theta_i-1)\right]
+
\boldsymbol\nu_i
$$

неизвестны одновременно:

1. глобальный параметр $\boldsymbol\eta$;
2. возможно глобальный параметр $k$;
3. локальные скрытые углы $\theta_i$.

Один проезд без акселерометра, как правило, не позволяет устойчиво отделить физическое изменение проекции поля из-за наклона от вертикального leakage.

При малых углах

$$
\sin\theta\approx\theta,
$$

$$
\cos\theta-1\approx-\frac{\theta^2}{2}.
$$

Тогда для одного проезда

$$
\Delta\mathbf r
\approx
k\theta
\begin{bmatrix}
1 \\
0
\end{bmatrix}
-
\boldsymbol\eta\cos\gamma\theta.
$$

То есть наблюдается только комбинация

$$
\left(
k
\begin{bmatrix}
1 \\
0
\end{bmatrix}
-
\boldsymbol\eta\cos\gamma
\right)\theta.
$$

Без знания $\theta$ масштаб этой комбинации неоднозначен.

## 9.2. Что может дать пара встречных проездов без акселерометра

Если есть два встречных проезда и их точки можно сопоставить по положению на холме, то можно использовать скрытую переменную $\theta_i$ и оценивать параметры через нелинейную оптимизацию.

Для каждой пары измерений задается модель

$$
\Delta\mathbf r_{+,i}=
\begin{bmatrix}
\cos\gamma_+(\cos\theta_{+,i}-1)+k\sin\theta_{+,i} \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma_+\sin\theta_{+,i}+k(\cos\theta_{+,i}-1)\right],
$$

$$
\Delta\mathbf r_{-,i}=
\begin{bmatrix}
\cos\gamma_-(\cos\theta_{-,i}-1)+k\sin\theta_{-,i} \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma_-\sin\theta_{-,i}+k(\cos\theta_{-,i}-1)\right].
$$

Если предполагается, что при обратном проходе pitch меняет знак, можно наложить связь

$$
\theta_{-,i}=-\theta_{+,i}.
$$

Если предполагается одинаковая параметризация по модулю, но знак неизвестен, можно использовать

$$
\theta_{-,i}=s_\theta\theta_{+,i},
\qquad
s_\theta\in\{-1,+1\}.
$$

Тогда оптимизируется задача

$$
\boxed{
\min_{\boldsymbol\eta,k,\{\theta_i\}}
\sum_i
\left\|
\Delta\mathbf r_{+,i}-\mathbf f(\theta_i,\gamma_+,k,\boldsymbol\eta)
\right\|^2
+
\left\|
\Delta\mathbf r_{-,i}-\mathbf f(s_\theta\theta_i,
\gamma_-,k,\boldsymbol\eta)
\right\|^2
+\lambda\mathcal R(\{\theta_i\})
}
$$

где

$$
\mathbf f(\theta,\gamma,k,\boldsymbol\eta)=
\begin{bmatrix}
\cos\gamma(\cos\theta-1)+k\sin\theta \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma\sin\theta+k(\cos\theta-1)\right].
$$

Регуляризатор $\mathcal R$ нужен для физической правдоподобности профиля холма. Например, можно использовать гладкость:

$$
\mathcal R(\{\theta_i\})=
\sum_i(\theta_{i+1}-\theta_i)^2
$$

или гладкость второй разности:

$$
\mathcal R(\{\theta_i\})=
\sum_i(\theta_{i+1}-2\theta_i+\theta_{i-1})^2.
$$

## 9.3. Практический вывод для случая без акселерометра

Без акселерометра метод возможен, но он существенно слабее:

1. один проезд холма почти неидентифицируем;
2. два встречных проезда дают больше информации, но требуют сопоставления точек профиля;
3. желательно знать $k=H_v/H_h$ из внешнего источника;
4. желательно ограничить диапазон возможных $\theta_i$ физически правдоподобными значениями;
5. обязательно нужно оценивать кондиционирование задачи и неопределенность параметров.

Поэтому вариант без акселерометра стоит рассматривать как резервный. Для надежной реализации предпочтителен вариант с акселерометром.

---

# 10. Полный алгоритм калибровки с акселерометром

## 10.1. Входные данные

Необходимы следующие данные.

Для плоского кольца:

$$
\mathcal D_{ring}=\{\mathbf u_i\}_{i=1}^{N_r},
\qquad
\mathbf u_i=
\begin{bmatrix}
m_{x,i} \\
m_{y,i}
\end{bmatrix}.
$$

Для прямого проезда холма:

$$
\mathcal D_+=\{\mathbf u_{+,i},\theta_{+,i}\}_{i=1}^{N_+}.
$$

Для обратного проезда:

$$
\mathcal D_-=\{\mathbf u_{-,i},\theta_{-,i}\}_{i=1}^{N_-}.
$$

Также желательно знать или оценить

$$
k=\frac{H_v}{H_h}.
$$

Если используется частная симметричная формула, $k$ может сократиться. Но для общей модели лучше иметь оценку $k$.

## 10.2. Предобработка

Рекомендуется:

1. удалить явно насыщенные измерения магнитометра;
2. удалить точки с резкими магнитными выбросами;
3. синхронизировать магнитометр и акселерометр;
4. отфильтровать акселерометр низкочастотным фильтром;
5. отбрасывать точки, где акселерометр не квазистатичен:

$$
\left|\|\mathbf a_i\|-g\right|>\delta_a.
$$

## 10.3. Шаг 1 — фитинг плоского эллипса

По точкам $\mathbf u_i$ плоского кольца оценить эллипс

$$
(\mathbf u-\mathbf q_0)^T\mathbf Q(\mathbf u-\mathbf q_0)=1.
$$

Получить:

$$
\hat{\mathbf q}_0,
\qquad
\hat{\mathbf Q}.
$$

Затем вычислить

$$
\hat{\mathbf W}=\hat{\mathbf Q}^{1/2}.
$$

## 10.4. Шаг 2 — перевод всех данных в нормализованную систему

Для кольца:

$$
\mathbf r_i=\hat{\mathbf W}(\mathbf u_i-\hat{\mathbf q}_0).
$$

Для прямого проезда:

$$
\mathbf r_{+,i}=\hat{\mathbf W}(\mathbf u_{+,i}-\hat{\mathbf q}_0).
$$

Для обратного проезда:

$$
\mathbf r_{-,i}=\hat{\mathbf W}(\mathbf u_{-,i}-\hat{\mathbf q}_0).
$$

## 10.5. Шаг 3 — оценка базовых heading-углов проездов

На ровном участке перед холмом или на участках с $\theta\approx0$ оценить средние векторы

$$
\mathbf r_{+,0}=\operatorname{mean}\{\mathbf r_{+,i}: |\theta_{+,i}|<\theta_0\},
$$

$$
\mathbf r_{-,0}=\operatorname{mean}\{\mathbf r_{-,i}: |\theta_{-,i}|<\theta_0\}.
$$

Нормировать их:

$$
\bar{\mathbf r}_{+,0}=\frac{\mathbf r_{+,0}}{\|\mathbf r_{+,0}\|},
$$

$$
\bar{\mathbf r}_{-,0}=\frac{\mathbf r_{-,0}}{\|\mathbf r_{-,0}\|}.
$$

Тогда

$$
\hat\gamma_+=\operatorname{atan2}(\bar r_{+,0,y},\bar r_{+,0,x}),
$$

$$
\hat\gamma_-=\operatorname{atan2}(\bar r_{-,0,y},\bar r_{-,0,x}).
$$

Для хорошего разворота на $180^\circ$ должно выполняться

$$
\bar{\mathbf r}_{+,0}+\bar{\mathbf r}_{-,0}\approx\mathbf 0.
$$

## 10.6. Шаг 4 — построение регрессии для $\boldsymbol\eta$

Для каждого измерения прямого и обратного проездов вычислить

$$
\Delta\mathbf r_{s,i}=\mathbf r_{s,i}-\bar{\mathbf r}_{s,0},
\qquad
s\in\{+,-\}.
$$

Далее для каждого измерения вычислить

$$
\mathbf g_{s,i}=
\begin{bmatrix}
\cos\hat\gamma_s(\cos\theta_{s,i}-1)+k\sin\theta_{s,i} \\
0
\end{bmatrix},
$$

$$
z_{s,i}=-\cos\hat\gamma_s\sin\theta_{s,i}+k(\cos\theta_{s,i}-1).
$$

Затем

$$
\mathbf e_{s,i}=\Delta\mathbf r_{s,i}-\mathbf g_{s,i}.
$$

Объединив оба проезда в один набор индексов $j$, получаем модель

$$
\mathbf e_j=\boldsymbol\eta z_j+\boldsymbol\nu_j.
$$

Оценка:

$$
\boxed{
\hat{\boldsymbol\eta}=\frac{\sum_j w_j z_j\mathbf e_j}{\sum_j w_j z_j^2}
}
$$

## 10.7. Шаг 5 — итоговая коррекция произвольного измерения

Для нового сырого измерения

$$
\mathbf u=
\begin{bmatrix}
m_x \\
m_y
\end{bmatrix}
$$

сначала выполнить плоскую коррекцию:

$$
\mathbf r=\hat{\mathbf W}(\mathbf u-
\hat{\mathbf q}_0).
$$

Если во время работы известен pitch $\theta$ и текущий приближенный heading $\gamma$, то вычислить

$$
z(\theta,
\gamma)=-\cos\gamma\sin\theta+k(\cos\theta-1).
$$

Затем выполнить leakage-коррекцию:

$$
\boxed{
\mathbf r_{corr}=\mathbf r-
\hat{\boldsymbol\eta}z(\theta,
\gamma)
}
$$

После этого heading вычисляется как

$$
\boxed{
\psi=\operatorname{atan2}(r_{corr,y},r_{corr,x})+\\psi_0
}
$$

где $\psi_0$ — постоянная поправка нуля курса, определяемая по внешнему ориентиру, магнитному склонению или другому референсу.

На практике $\gamma$ в $z(\theta,
\gamma)$ можно брать итерационно:

1. сначала вычислить грубый heading

$$
\gamma^{(0)}=\operatorname{atan2}(r_y,r_x);
$$

2. вычислить $z^{(0)}$;
3. получить $\mathbf r_{corr}^{(0)}$;
4. пересчитать heading

$$
\gamma^{(1)}=\operatorname{atan2}(r_{corr,y}^{(0)},r_{corr,x}^{(0)});
$$

5. при необходимости повторить 1--2 раза.

---

# 11. Полный алгоритм без акселерометра

## 11.1. Входные данные

Данные те же, но без $\theta_i$:

$$
\mathcal D_{ring}=\{\mathbf u_i\}_{i=1}^{N_r},
$$

$$
\mathcal D_+=\{\mathbf u_{+,i}\}_{i=1}^{N_+},
$$

$$
\mathcal D_- =\{\mathbf u_{-,i}\}_{i=1}^{N_-}.
$$

Желательно иметь сопоставление точек прямого и обратного проездов по положению на холме:

$$
i \leftrightarrow i.
$$

## 11.2. Шаги, совпадающие с вариантом с акселерометром

Сначала выполняются те же действия:

1. фитинг плоского эллипса;
2. получение $\hat{\mathbf q}_0$ и $\hat{\mathbf W}$;
3. перевод всех точек в нормализованные координаты $\mathbf r$;
4. оценка $\hat\gamma_+$ и $\hat\gamma_-$ на ровных участках.

## 11.3. Нелинейная задача для скрытого pitch

Далее вводятся скрытые переменные $\theta_i$.

Если предполагается, что обратный проезд имеет противоположный pitch, используем

$$
\theta_{-,i}=-\theta_i,
\qquad
\theta_{+,i}=\theta_i.
$$

Если знак неизвестен, можно рассмотреть оба варианта $s_\theta=+1$ и $s_\theta=-1$ и выбрать тот, который дает меньший остаток.

Общая модель:

$$
\Delta\mathbf r_{+,i}=\mathbf f(\theta_i,
\hat\gamma_+,
k,
\boldsymbol\eta)+\boldsymbol\nu_{+,i},
$$

$$
\Delta\mathbf r_{-,i}=\mathbf f(s_\theta\theta_i,
\hat\gamma_-,
k,
\boldsymbol\eta)+\boldsymbol\nu_{-,i},
$$

где

$$
\mathbf f(\theta,
\gamma,
k,
\boldsymbol\eta)=
\begin{bmatrix}
\cos\gamma(\cos\theta-1)+k\sin\theta \\
0
\end{bmatrix}
+
\boldsymbol\eta
\left[-\cos\gamma\sin\theta+k(\cos\theta-1)\right].
$$

Оптимизационная задача:

$$
\boxed{
\min_{\boldsymbol\eta,k,\{\theta_i\}}
\sum_i
\rho\left(
\left\|
\Delta\mathbf r_{+,i}-\mathbf f(\theta_i,
\hat\gamma_+,
k,
\boldsymbol\eta)
\right\|
\right)
+
\rho\left(
\left\|
\Delta\mathbf r_{-,i}-\mathbf f(s_\theta\theta_i,
\hat\gamma_-,
k,
\boldsymbol\eta)
\right\|
\right)
+
\lambda\mathcal R(\{\theta_i\})
}
$$

где $\rho$ — робастная функция потерь, например Huber loss, а $\mathcal R$ — регуляризатор гладкости профиля.

Если $k$ известно, его лучше зафиксировать. Тогда задача становится существенно устойчивее.

## 11.4. Почему этот вариант менее устойчив

Без акселерометра параметр $\theta_i$ может компенсировать ошибки $\boldsymbol\eta$. Поэтому возникает сильная корреляция параметров.

Показателем плохой наблюдаемости является высокая обусловленность матрицы Гаусса--Ньютона:

$$
\kappa(\mathbf J^T\mathbf J)\gg 1,
$$

где $\mathbf J$ — якобиан остатков по оцениваемым параметрам.

Если задача плохо обусловлена, результат может сильно зависеть от начального приближения, регуляризации и выбросов.

---

# 12. Метрики качества и достаточности данных

## 12.1. Метрики плоского кольца

### 12.1.1. Радиальный остаток после калибровки

После плоской коррекции точки кольца должны лежать на единичной окружности:

$$
\mathbf r_i=\hat{\mathbf W}(\mathbf u_i-\hat{\mathbf q}_0).
$$

Радиальный остаток:

$$
\epsilon_i^{ring}=\|\mathbf r_i\|-1.
$$

Среднеквадратичный остаток:

$$
\operatorname{RMS}_{ring}=\sqrt{\frac{1}{N_r}\sum_i (\epsilon_i^{ring})^2}.
$$

Также полезны:

$$
\operatorname{MAD}_{ring}=\operatorname{median}\left(|\epsilon_i^{ring}-\operatorname{median}(\epsilon^{ring})|\right),
$$

$$
\epsilon_{max}^{ring}=\max_i|\epsilon_i^{ring}|.
$$

### 12.1.2. Покрытие углов

После коррекции вычислить

$$
\alpha_i=\operatorname{atan2}(r_{i,y},r_{i,x}).
$$

Данные должны покрывать почти весь диапазон $[-\pi,\pi)$.

Можно отсортировать углы и оценить максимальный угловой разрыв:

$$
\Delta\alpha_{max}=\max_i \operatorname{wrap}_{2\pi}(\alpha_{i+1}-\alpha_i).
$$

Чем меньше $\Delta\alpha_{max}$, тем лучше покрытие кольца.

### 12.1.3. Обусловленность эллипса

Собственные значения $\hat{\mathbf Q}$:

$$
\lambda_1,
\lambda_2.
$$

Отношение полуосей эллипса:

$$
\chi=\sqrt{\frac{\max(\lambda_1,
\lambda_2)}{\min(\lambda_1,
\lambda_2)}}.
$$

Слишком большое $\chi$ может указывать на сильные искажения или плохое покрытие.

## 12.2. Метрики проезда холма с акселерометром

### 12.2.1. Амплитуда возбуждения leakage

Для каждого измерения:

$$
z_i=-\cos\gamma_i\sin\theta_i+k(\cos\theta_i-1).
$$

Суммарное возбуждение:

$$
\boxed{
E_z=\sum_i w_i z_i^2
}
$$

или нормированное:

$$
\bar E_z=\frac{1}{N}\sum_i z_i^2.
$$

Если $E_z$ мало, оценка $\boldsymbol\eta$ будет шумной.

### 12.2.2. Максимальный pitch

$$
\theta_{max}=\max_i |\theta_i|.
$$

Слишком малый $\theta_{max}$ означает слабое возбуждение $h_z$.

### 12.2.3. Качество разворота на $180^\circ$

Проверка антипараллельности базовых векторов:

$$
\epsilon_{180}=\|\bar{\mathbf r}_{+,0}+\bar{\mathbf r}_{-,0}\|.
$$

В угловой форме:

$$
\epsilon_{\gamma}=\left|\operatorname{wrap}_{\pi}(\hat\gamma_- - \hat\gamma_+ - \pi)\right|.
$$

### 12.2.4. Остаток модели холма

После оценки $\hat{\boldsymbol\eta}$ остаток:

$$
\mathbf e_i^{hill}=\Delta\mathbf r_i-\mathbf g_i-\hat{\boldsymbol\eta}z_i.
$$

Среднеквадратичный остаток:

$$
\operatorname{RMS}_{hill}=\sqrt{\frac{1}{N}\sum_i \|\mathbf e_i^{hill}\|^2}.
$$

Робастный остаток:

$$
\operatorname{MAD}_{hill}=\operatorname{median}\left(\left|\|\mathbf e_i^{hill}\|-\operatorname{median}(\|\mathbf e^{hill}\|)\right|\right).
$$

### 12.2.5. Улучшение после leakage-коррекции

До коррекции остаток:

$$
\mathbf e_i^{before}=\Delta\mathbf r_i-\mathbf g_i.
$$

После коррекции:

$$
\mathbf e_i^{after}=\Delta\mathbf r_i-\mathbf g_i-\hat{\boldsymbol\eta}z_i.
$$

Коэффициент улучшения:

$$
\boxed{
G_{hill}=1-\frac{\sum_i\|\mathbf e_i^{after}\|^2}{\sum_i\|\mathbf e_i^{before}\|^2}
}
$$

Если $G_{hill}$ близок к нулю или отрицателен, то leakage-модель не объясняет данные или данные недостаточно возбуждают параметр.

### 12.2.6. Неопределенность оценки $\boldsymbol\eta$

Оценка дисперсии остатка:

$$
\hat\sigma^2=\frac{1}{2N-2}\sum_i\|\mathbf e_i^{hill}\|^2.
$$

Ковариация:

$$
\boxed{
\operatorname{Cov}(\hat{\boldsymbol\eta})\approx
\frac{\hat\sigma^2}{\sum_i w_i z_i^2}\mathbf I
}
$$

Стандартные ошибки:

$$
\operatorname{SE}(\hat\eta_x)=\sqrt{\left[\operatorname{Cov}(\hat{\boldsymbol\eta})\right]_{11}},
$$

$$
\operatorname{SE}(\hat\eta_y)=\sqrt{\left[\operatorname{Cov}(\hat{\boldsymbol\eta})\right]_{22}}.
$$

## 12.3. Метрики акселерометра

Акселерометр можно использовать для pitch только в квазистатике. Поэтому нужно проверять

$$
\epsilon_a=\left|\|\mathbf a_i\|-g\right|.
$$

Критерий отбора:

$$
\epsilon_a<\delta_a.
$$

Также полезно смотреть дисперсию pitch на ровном участке:

$$
\sigma_\theta^2=\operatorname{Var}(\theta_i: |\theta_i|<\theta_0).
$$

Большая дисперсия на ровном участке означает шум или динамические ускорения.

## 12.4. Метрики для варианта без акселерометра

### 12.4.1. Остаток нелинейной модели

После решения нелинейной задачи:

$$
\operatorname{RMS}_{noacc}=
\sqrt{\frac{1}{2N}\sum_i
\left\|
\Delta\mathbf r_{+,i}-\mathbf f(\hat\theta_i,
\hat\gamma_+,
\hat k,
\hat{\boldsymbol\eta})
\right\|^2
+
\left\|
\Delta\mathbf r_{-,i}-\mathbf f(s_\theta\hat\theta_i,
\hat\gamma_-,
\hat k,
\hat{\boldsymbol\eta})
\right\|^2
}.
$$

### 12.4.2. Обусловленность

Вычислить якобиан остатков по параметрам:

$$
\mathbf J=\frac{\partial \mathbf e}{\partial \boldsymbol\xi},
$$

где

$$
\boldsymbol\xi=\begin{bmatrix}
\eta_x & \eta_y & k & \theta_1 & \dots & \theta_N
\end{bmatrix}^T.
$$

Показатель обусловленности:

$$
\kappa=\operatorname{cond}(\mathbf J^T\mathbf J).
$$

Большое значение $\kappa$ означает слабую идентифицируемость.

### 12.4.3. Физическая правдоподобность скрытого pitch

Оцененные углы должны удовлетворять физическим ограничениям:

$$
|\hat\theta_i|<\theta_{phys,max}.
$$

Профиль должен быть гладким:

$$
\sum_i(\hat\theta_{i+1}-2\hat\theta_i+
\hat\theta_{i-1})^2
$$

не должен быть аномально большим.

Если для хорошего fit-а оптимизация требует резких скачков $\theta_i$, значит модель подгоняет шум или магнитные выбросы.

---

# 13. Робастность метода

## 13.1. Робастный фитинг эллипса

Обычный fit эллипса чувствителен к выбросам. Рекомендуется:

1. сначала выполнить грубый fit;
2. вычислить радиальные остатки $\epsilon_i^{ring}$;
3. удалить точки с $|\epsilon_i^{ring}|>c\cdot\operatorname{MAD}_{ring}$;
4. повторить fit;
5. при необходимости использовать робастную функцию потерь.

## 13.2. Робастная оценка $\boldsymbol\eta$

Вместо обычного МНК можно решить

$$
\boxed{
\min_{\boldsymbol\eta}
\sum_i
\rho\left(\left\|\mathbf e_i-
\boldsymbol\eta z_i\right\|\right)
}
$$

где $\rho$ — Huber loss:

$$
\rho(t)=
\begin{cases}
\frac{1}{2}t^2, & |t|\leq \delta, \\
\delta(|t|-\frac{1}{2}\delta), & |t|>\delta.
\end{cases}
$$

Практически это можно реализовать через iteratively reweighted least squares.

## 13.3. Веса измерений

Полезный выбор веса:

$$
w_i=w_i^{acc}w_i^{exc}w_i^{mag}.
$$

Компонента качества акселерометра:

$$
w_i^{acc}=\exp\left(-\frac{(\|\mathbf a_i\|-g)^2}{2\sigma_a^2}\right).
$$

Компонента возбуждения:

$$
w_i^{exc}=\min\left(1,\frac{z_i^2}{z_{ref}^2}\right).
$$

Компонента магнитных выбросов может задаваться через отклонение модуля после коррекции:

$$
w_i^{mag}=\exp\left(-\frac{(\|\mathbf r_i\|-1)^2}{2\sigma_r^2}\right).
$$

## 13.4. Проверка локальных магнитных возмущений

Локальное внешнее магнитное возмущение часто проявляется как резкое отклонение $\mathbf r$ от ожидаемой дуги. Можно контролировать скорость изменения:

$$
\Delta_i=\|\mathbf r_{i+1}-\mathbf r_i\|.
$$

Аномально большие $\Delta_i$ при плавном движении являются признаком выбросов.

---

# 14. Итоговая инструкция по реализации

## 14.1. Калибровка с акселерометром

1. Снять плоское кольцо на ровной поверхности.
2. По точкам $\mathbf u_i=[m_{x,i},m_{y,i}]^T$ оценить эллипс.
3. Получить $\hat{\mathbf q}_0$ и $\hat{\mathbf W}$.
4. Проверить качество кольца по $\operatorname{RMS}_{ring}$, $\Delta\alpha_{max}$ и обусловленности эллипса.
5. Снять проезд холма в прямом направлении, записывая $\mathbf u_{+,i}$ и $\theta_{+,i}$.
6. Развернуться на $180^\circ$ и снять обратный проезд, записывая $\mathbf u_{-,i}$ и $\theta_{-,i}$.
7. Перевести все точки в нормализованные координаты:

$$
\mathbf r=\hat{\mathbf W}(\mathbf u-\hat{\mathbf q}_0).
$$

8. Оценить базовые углы $\hat\gamma_+$ и $\hat\gamma_-$ на ровных участках.
9. Для каждой точки вычислить $\Delta\mathbf r_i$, $\mathbf g_i$ и $z_i$.
10. Оценить $\hat{\boldsymbol\eta}$ по формуле

$$
\hat{\boldsymbol\eta}=\frac{\sum_i w_i z_i(\Delta\mathbf r_i-\mathbf g_i)}{\sum_i w_i z_i^2}.
$$

11. Проверить $E_z$, $\operatorname{RMS}_{hill}$, $G_{hill}$ и $\operatorname{Cov}(\hat{\boldsymbol\eta})$.
12. Использовать итоговую коррекцию

$$
\mathbf r_{corr}=\hat{\mathbf W}(\mathbf u-
\hat{\mathbf q}_0)-\hat{\boldsymbol\eta}z(\theta,
\gamma).
$$

13. Вычислять heading:

$$
\psi=\operatorname{atan2}(r_{corr,y},r_{corr,x})+\psi_0.
$$

## 14.2. Калибровка без акселерометра

1. Выполнить плоскую калибровку так же, как в варианте с акселерометром.
2. Снять прямой и обратный проезды холма.
3. Перевести данные в нормализованные координаты $\mathbf r$.
4. Оценить $\hat\gamma_+$ и $\hat\gamma_-$.
5. Сопоставить точки прямого и обратного профиля.
6. Задать физическую модель $\mathbf f(\theta,
\gamma,k,
\boldsymbol\eta)$.
7. Решить нелинейную задачу по $\boldsymbol\eta$, $k$ и скрытым $\theta_i$ либо по $\boldsymbol\eta$ и $\theta_i$ при фиксированном $k$.
8. Использовать робастную функцию потерь и регуляризацию гладкости $\theta_i$.
9. Проверить обусловленность $\mathbf J^T\mathbf J$.
10. Если задача плохо обусловлена, признать оценку $\boldsymbol\eta$ ненадежной и требовать акселерометр или дополнительный проезд под другим углом.

---

# 15. Краткий итог

Процедура «плоское кольцо + холм туда + холм обратно» позволяет построить heading-ориентированную калибровку магнитометрического MEMS-компаса без полного трехмерного вращения датчика.

Плоское кольцо дает двумерную калибровку:

$$
\mathbf r=\mathbf W(\mathbf u-\mathbf q_0),
$$

которая переводит эллипс в окружность.

Однако центр плоского кольца равен

$$
\mathbf q_0=\mathbf c+\mathbf dH_v,
$$

поэтому он не разделяет чистый hard-iron и вертикальный leakage.

Проезд холма изменяет $h_z$ и тем самым возбуждает компоненты

$$
a_{13},a_{23},
$$

которые в нормализованной heading-модели представлены вектором

$$
\boldsymbol\eta=H_h\mathbf W\mathbf d.
$$

Акселерометр делает дугу параметризованной по $\theta$ и превращает оценку $\boldsymbol\eta$ в линейную или почти линейную регрессию. Встречный проезд через тот же холм повышает наблюдаемость и позволяет отделить leakage от обычного геометрического изменения проекции магнитного поля при pitch.

Итоговая рабочая коррекция имеет вид

$$
\boxed{
\mathbf r_{corr}=\mathbf W(\mathbf u-\mathbf q_0)-\hat{\boldsymbol\eta}z(\theta,
\gamma)
}
$$

где

$$
z(\theta,
\gamma)=-\cos\gamma\sin\theta+k(\cos\theta-1).
$$

После этого heading вычисляется как

$$
\boxed{
\psi=\operatorname{atan2}(r_{corr,y},r_{corr,x})+\psi_0.
}
$$

