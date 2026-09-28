# Solver V2: план исследования и реализации

Этот план заменяет план XPBD V1. Внешние требования остаются в `docs/SPEC.md`, архитектурные
правила — в `AGENTS.md`. V1 не удаляется: V2 разрабатывается параллельно и включается явным
переключателем.

## 1. Причина V2

Длительный прогон при неподвижных якорях показал не ошибку настройки, а несогласованность
алгоритма:

- rest length — 40 м, длина ломаной — 43,61 м;
- локальная ошибка stretch — около 132%;
- натяжение — около 254 Н;
- одновременно меняются более 600 контактов;
- колебания не затухают без движения якорей.

V1 последовательно применяет XPBD stretch, LRA, DER-bending, жёсткие positional projections,
отдельный velocity/friction pass и изменение топологии катушки. Каждый проход исправляет
результат предыдущего; часть коррекций исключается из скорости, часть нет. В плотном контакте
это не является дискретизацией одной системы уравнений и может подкачивать энергию.

Предыдущая работа подтверждает несколько отдельных дефектов — выдачу кабеля в покое, скачки
от ориентации якоря, неустойчивый per-particle sleep, короткий хвостовой сегмент катушки и
неполный velocity-response segment-world контактов. Их локальные исправления улучшили простые
сценарии, но не устранили узлы и плотный self-contact. Дальнейший подбор damping, stiffness и
substeps не является направлением V2.

## 2. Проверка первоисточников

### Discrete Elastic Rods

Bergou et al. задают центральную линию вершинами и одну twist-переменную на ребро. Bending и
twist выводятся из одной энергии; material frame строится parallel transport и поворотом на
twist. Работа не содержит готового friction/contact solver и не доказывает realtime для
узлов. Полная DER-конфигурация для 572 рёбер имеет 2291 DOF; centerline-only — 1719 DOF.

Источник: <https://www.cs.jhu.edu/~misha/Fall09/Bergou08.pdf>.

### PyElastica / Cosserat rods

PyElastica — полезный эталон Cosserat rod со stretch, shear, bend и twist, но использует
explicit Position Verlet и penalty contacts. Self-contact основан на segment queries и не
даёт нужной fully implicit контактной системы или CCD. Полный Cosserat state избыточен для
круглого кабеля без наблюдаемого torsion.

Источники: <https://docs.cosseratrods.org/en/latest/>,
<https://docs.cosseratrods.org/en/latest/guide/discretization.html>.

### IMC и fully implicit friction

IMC вводит гладкую segment-segment contact energy, её gradient и Hessian в общий backward
Euler Newton solve. Продолжение работы добавляет fully implicit regularized Coulomb friction,
включая производную контактной силы. Именно coupled solve, а не форма отдельной силы, даёт
устойчивость.

Опубликованная стоимость не подтверждает realtime: пример с 10 flagella по 67 рёбер
(670 рёбер суммарно), `dt = 1 ms`, без трения занимает в среднем 389,4 мс на timestep.
Это не прямой benchmark нашего Apple Silicon и геометрия другая, но разница с 16,67 мс на весь
physics tick исключает слепой перенос полного IMC.

Источники:

- <https://doi.org/10.1115/1.4050238>;
- <https://arxiv.org/html/2205.10309v3>;
- <https://github.com/QuantuMope/imc-der>;
- <https://github.com/StructuresComp/rod-contact-sim>.

### C-IPC

C-IPC объединяет inertia, elasticity, barrier contact и friction в incremental potential,
ограничивает Newton line search через CCD/ACCD и при корректном исходном состоянии даёт
непересечение. Цена — глобальная sparse Hessian, collision set и CCD на каждой line-search
итерации. Опубликованные времена измеряются секундами и минутами на timestep для больших
примеров, поэтому целиком C-IPC не является realtime-зависимостью.

Переносимы идеи barrier activation, collision-free step length и conservative advancement.
Источник кода Codim-IPC — Apache-2.0; более модульный IPC Toolkit — MIT.

Источники: <https://ipc-sim.github.io/C-IPC/>,
<https://github.com/ipc-sim/Codim-IPC>, <https://github.com/ipc-sim/ipc-toolkit>.

### DisMech

DisMech объединяет DER, implicit integration, IMC contact и friction и годится как
архитектурный эталон. Это не зависимость: GPLv3, SymEngine/LLVM, FCL, oneMKL/Pardiso и другие
тяжёлые компоненты несовместимы с компактным распространяемым GDExtension. Публичные
benchmarks не подтверждают 60 Гц при 572 рёбрах и 600 контактах.

Источник: <https://github.com/structurescomp/dismech-rods>.

### High-order contact detection

Работа Crespel et al. подтверждает: скачки контактной силы возникают при переходе closest
point между линейными сегментами и усиливаются при большой кривизне. High-order detector
работает между гладкими кривыми и не может быть просто применён к нашей ломаной. Полный
branch-and-bound для всех пар пока не принимается; V2 должен сначала обеспечить persistent
manifold, hysteresis и плавный перенос параметров контакта. High-order reconstruction остаётся
измеряемой альтернативой для контактов с большой кривизной.

Источники: <https://doi.org/10.1145/3658191>,
<https://hal.science/hal-04364565>.

## 3. Решение и границы утверждений

Базовый V2 — **centerline-only implicit isotropic DER без twist**:

- позиции и скорости узлов;
- физические stretch и bend energies;
- barrier contact для segment-world и segment-segment;
- persistent contact manifold;
- regularized Coulomb friction как lagged incremental potential;
- Newton с sparse SPD approximation, line search и CCD step cap;
- изменение топологии только между принятыми timestep.

Material frames в физическом state сейчас не оправданы:

- сечение кабеля круглое;
- anisotropic bending не требуется;
- API не задаёт torque или material orientation на концах;
- mesh использует независимый rotation-minimizing frame;
- удаление twist уменьшает state с 2291 до 1719 DOF и упрощает Hessian.

Цена решения: V2 не моделирует torsion, roll resistance, anisotropic section и twist boundary
conditions. Если это станет наблюдаемым требованием, одна twist-переменная на ребро
добавляется после стабилизации centerline solver; API и раскладка резервируют этот extension
point.

Полный IMC friction Jacobian несимметричен и не совместим с выбранным SPD factorization.
Поэтому первая версия V2 не называется «полностью implicit Coulomb» без оговорки: normal
contact решается внутри Newton, а normal force и tangent basis для friction фиксируются на
внешней friction-итерации. При сходимости внешнего цикла получается coupled решение; при
остановке по бюджету — контролируемая lagged approximation. Альтернатива — nonsymmetric
Newton + SparseLU/Pardiso — остаётся только benchmark-кандидатом.

## 4. Performance gate до масштабной реализации

Число 1 мс из старого SPEC — желаемая цель, а не подтверждённый бюджет. Реальный бюджет:

```
cable_budget_p95 =
    measured_physics_headroom_p95_in_target_level
    - safety_reserve
```

Он измеряется на целевом Mac в существующей игровой сцене при 60 Гц: сначала без кабеля, затем
с V1, затем с прототипом V2. Пока целевой уровень не предоставлен, абсолютный предел —
16,67 мс на весь physics tick, но он не является допустимым бюджетом одного кабеля.

До полной физики делаются три instrumented spike без нового test scene и без automated tests:

1. 573 узла, inertia + stretch + bending, без контакта;
2. тот же state и фиксированный manifold из 600 world contacts;
3. плотный segment-segment manifold, friction и меняющиеся пары.

Используются существующая demo/manual сцена и сохранённые диагностические состояния. Для
каждого прогона пишутся p50/p95/p99:

- broad phase и narrow phase;
- assembly gradient/Hessian;
- symbolic analysis и numeric factorization;
- linear solve;
- число Newton/friction/line-search итераций;
- CCD;
- число активных контактов и смен contact key;
- количество аллокаций и peak memory;
- residual, stretch error, minimum gap и принятая доля timestep.

Go/no-go принимается по измеренному budget, а не по числу итераций. Если full 572/600 state не
входит в budget, порядок упрощений такой:

1. sleeping islands и active window с корректными граничными условиями;
2. reuse symbolic sparsity при неизменном manifold;
3. centerline-only SPD/Gauss-Newton;
4. adaptive substepping только проблемного timestep;
5. уменьшение контактного activation distance.

Нельзя возвращаться к несогласованным positional projections. Если после этих мер V2 не
проходит budget, честный результат gate — implicit V2 непригоден для целевого hardware; тогда
выбирается отдельный упрощённый solver, а не выдаётся IMC-ярлык XPBD-патчу.

## 5. Параллельная архитектура и откат

V1 остаётся без функциональных изменений. Новые файлы располагаются в `src/solver/v2/`,
поэтому будущая реализация явно добавит этот каталог в `SConstruct`.

```
src/solver/v2/
  implicit_cable_state.{h,cpp}
  implicit_cable_params.h
  elastic_energy.{h,cpp}
  contact_manifold.{h,cpp}
  contact_energy.{h,cpp}
  friction_energy.{h,cpp}
  ccd.{h,cpp}
  sparse_system.{h,cpp}
  newton_solver.{h,cpp}
  implicit_cable_solver.{h,cpp}
  topology_transition.{h,cpp}
```

`CableBody3D` владеет фасадом с backend:

```
LEGACY_XPBD
IMPLICIT_V2
```

На этапе разработки default остаётся `LEGACY_XPBD`. После прохождения quality и performance
gate default можно поменять отдельным решением. Публичный API центральной линии, якорей,
катушки, натяжения, world snapshot, mesh builder, debug draw и логирование сохраняется.

Переключение допускается только на границе timestep. Нейтральный transfer state содержит
positions, velocities, rest lengths, deployed/stowed length и anchors. Persistent contacts и
внутренние lambda не переносятся между backend. Переключение выполняет controlled
reinitialization и помечается в диагностике; бесшовность не обещается.

Один неуспешный Newton не включает V1 автоматически: V2 откатывает последний timestep и
уменьшает шаг. Автоматический backend fallback скрывал бы дефекты и давал физический скачок.
Переход на V1 разрешён только явной policy после заданного числа failures или вручную.

## 6. Данные V2

`ImplicitCableState` хранит плоские массивы с заранее зарезервированной capacity:

```
position[node]
previous_position[node]
velocity[node]
mass[node]
inv_mass[node]
flags[node]

rest_length[edge]
stretch_stiffness[edge]
bending_stiffness[interior_vertex]

trial_position[node]
gradient[3 * node_count]
newton_step[3 * node_count]
```

Отдельно хранятся accepted state и trial state. Line search никогда не портит accepted state.
Pinned DOF исключаются из линейной системы либо задаются elimination, а не огромной
stiffness.

`ContactManifold` — flat vector + hash `ContactKey -> index`:

```
ContactKey:
  kind                    world или self
  ordered edge ids
  collider stable id
  collider feature id

Contact:
  edge indices/stencil
  closest parameters
  local points on moving collider
  gap and normal
  tangent basis
  accumulated tangential displacement
  lagged normal force
  age, inactive_age, stick/slip state
```

World contact использует два cable nodes; self-contact — четыре. Key не включает
floating-point closest parameters. Контакт остаётся жив в release distance, большем activation
distance, что даёт hysteresis. При переходе closest point через вершину соседние candidates
коротко сосуществуют и blend-ятся; резкая замена одной пары другой запрещена. Дубликаты
одного геометрического feature объединяются до assembly, иначе barrier энергии создадут
искусственную «стену».

## 7. Incremental potential и elastic energies

Для шага `h` predictor:

```
y = x_n + h * v_n + h² * M⁻¹ * f_external
```

Ищется минимум:

```
Phi(x) =
    1 / (2 h²) * (x - y)^T M (x - y)
    + E_stretch(x)
    + E_bend(x)
    + E_contact(x)
    + E_friction(x; lagged contact state)
```

Residual и Newton matrix:

```
r = M / h² * (x - y) + grad(E)
H = M / h² + Hessian(E)
H * dx = -r
```

### Stretch

Для ребра длины `l`, rest length `l0`:

```
E_stretch = 0.5 * EA / l0 * (l - l0)²
```

`EA` — физическая axial stiffness, а не iteration-dependent stiffness. Бесконечная
нерастяжимость создаёт плохо обусловленную систему. Если измеренный физический `EA` не
удерживает length tolerance, применяется augmented Lagrangian outer update; отдельная LRA и
жёсткая positional projection в V2 отсутствуют.

### Bending

Для изотропного круглого кабеля используется centerline DER curvature binormal и Voronoi
length. В V2 stress-free кабель прямой, поэтому `kappa_rest = 0`:

```
E_bend = 0.5 * EI / l_bar_rest * ||kappa||²
```

Формулы gradient/Hessian реализуются независимо по статье, без копирования GPL-кода.
`l_bar_rest` — Voronoi length в rest-конфигурации: текущая длина ошибочно связывала бы
stretch с bending stiffness.
Ненулевая rest curvature без material/Bishop frame была бы необъективной: она менялась бы при
глобальном повороте кабеля. Она не поддерживается centerline-only state.
Конфигурация около 180° сингулярна для `tan(theta / 2)`: line search не пересекает именованный
angle limit, а вырожденный trial отклоняется. Это failure condition, не место для случайного
epsilon.

Minimum bend radius не решается отдельной проекцией. Если обычного `EI` недостаточно, вводится
smooth curvature barrier для физической дискретной кривизны
`kappa_physical = ||kappa_binormal|| / l_bar` перед `1 / R_min`; его вклад находится в той же
gradient/Hessian и line search, что остальные энергии.

### Якоря

Позиции концов — Dirichlet boundary conditions. Направление выхода, если включено, задаётся
boundary bending energy на первом/последнем ребре. Ни виртуальная частица, ни ориентационная
поправка не превращаются в скорость отдельным pass.

### Совместимость параметров и диагностики

Существующие `stretch_compliance`, `bending_compliance` и `damping` не удаляются из
`CableBody3D`, чтобы не ломать сцены V1. Однако XPBD compliance зависит от дискретизации и не
имеет однозначного физического преобразования в `EA`/`EI`. V2 добавляет параметры с единицами:

- `axial_stiffness` — `EA`, Н;
- `bending_stiffness` — `EI`, Н·м²;
- `internal_damping` — коэффициент относительного strain-rate damping.

Legacy compliance используется только backend V1; при выборе V2 редактор явно предупреждает,
если заданы только legacy-параметры. Runtime-изменение `EA/EI` инвалидирует только численные
коэффициенты, а изменение rest length или дискретизации требует rebuild.

Внутреннее демпфирование V2 — согласованная edge-relative viscous energy на изменении длины и,
при необходимости, кривизны за timestep. Она находится в том же residual/Hessian, не гасит
rigid translation и не заменяет контактное трение.

Натяжение наружу — axial force конечного ребра `EA * (l - l0) / l0`; для закреплённого конца
дополнительно сохраняется полная реакция Dirichlet DOF до elimination. В API возвращаются
модуль и направление реакции вдоль кабеля.

## 8. Newton, sparse system и line search

Одна Newton iteration:

1. обновить geometry текущего trial state без изменения manifold topology;
2. собрать `Phi`, residual и локальные Hessian blocks;
3. спроецировать неопределённые contact blocks в PSD либо использовать Gauss-Newton;
4. добавить diagonal regularization, если factorization не положительно определена;
5. решить `H dx = -r`;
6. вычислить collision-free upper bound `alpha_ccd`;
7. backtracking line search по `Phi` и residual на `[0, alpha_ccd]`;
8. принять trial только при конечных значениях, уменьшении merit function и положительном gap.

Критерии Newton convergence учитывают norm residual и относительный step. Max stretch error
и minimum gap проверяются отдельно как quality/acceptance gates: при конечном `EA` ненулевой
strain является частью физического решения и не может считаться numerical non-convergence.
Один малый residual при недопустимом проникновении не позволяет принять timestep.

Манifold discovery выполняется перед Newton и повторяется, если line search приводит новые
пары в activation distance. Нельзя менять sparse contact topology посреди assembly.

## 9. Sparse linear solver и лицензии

Первый кандидат — Eigen 3.4 `SimplicialLDLT`:

- MPL-2.0, допустима для линковки и распространения при выполнении file-level obligations;
- header-only и работает на macOS arm64;
- SPD approximation позволяет direct factorization без nonsymmetric runtime;
- symbolic analysis можно переиспользовать, пока contact sparsity не меняется.

Eigen не добавляется до performance spike. Сравниваются:

- Eigen `SimplicialLDLT` для SPD/Gauss-Newton;
- Eigen `SparseLU` только как оценка полного friction Jacobian;
- собственный block-CSR + preconditioned CG как более лёгкая альтернатива, если direct numeric
  factorization окажется bottleneck.

PCG нельзя выбрать по идеологии: плотные узлы дают плохую обусловленность, поэтому решение
принимается по residual/time measurements. Apple Accelerate может быть optional backend, но
не baseline: он привяжет pure solver к одной платформе. SuiteSparse/Pardiso не принимаются на
первом этапе из-за размера, сборки и неоднородных лицензий. oneMKL не является решением для
macOS arm64.

Допустимые optional зависимости после отдельного gate:

- Eigen — MPL-2.0;
- Tight Inclusion CCD — MIT;
- IPC Toolkit — MIT, только нужные модули; optional LGPL `filib` выключен.

Для Eigen фиксируются exact version, текст MPL-2.0, notices и способ получить исходники
Eigen вместе с изменениями MPL-covered файлов. Собственные изменения внутри Eigen не
планируются; используемые модули и их лицензии проверяются перед вендорингом.

Не используются как зависимости:

- DisMech и `rod-contact-sim` — GPLv3;
- `curly-splits` high-order detector — GPLv3;
- полный Codim-IPC, несмотря на Apache-2.0, — слишком тяжёл и не нужен целиком;
- устаревшие сборочные зависимости IMC-DER.

## 10. World SDF и Hessian

Для world gap `g_world = d(segment, shape) - cable_radius`; для self-contact
`g_self = d(segment_a, segment_b) - 2 * cable_radius`. Для contact potential `b(g)`:

```
grad E_contact = b'(g) * grad(g)
H_contact = b''(g) * grad(g) * grad(g)^T + b'(g) * Hessian(g)
```

Sphere, plane, capsule, box, cylinder и convex shape дают distance/gradient и, где функция
гладкая, analytic Hessian. Для convex shape хранится active face/edge/vertex feature; при его
смене действует manifold hysteresis. На рёбрах и углах closest feature меняется, поэтому
exact Hessian кусочно-гладкая, а baseline использует Gauss-Newton. Conservative CCD для
convex выполняется по distance oracle с bounded advancement либо через отдельный convex cast,
если первый вариант не проходит gate. Для этого `WorldSnapshot` расширяется кешированными
convex vertices, edges, face planes и feature adjacency; текущего набора одних плоскостей
недостаточно для Euclidean distance и CCD.

Baseline V2 использует Gauss-Newton:

```
H_contact ~= b''(g) * grad(g) * grad(g)^T
```

Она PSD и сохраняет совместимость с `SimplicialLDLT`. Exact distance Hessian включается
отдельно для гладких primitive regions только если измерения показывают сокращение Newton
iterations без потери устойчивости.

Для triangle mesh `TriangleMeshDistance` переиспользуется для distance/closest feature.
Hessian distance field из него не предполагается; используется Gauss-Newton и persistent
triangle feature. Для незамкнутой mesh прежняя эвристика знака недостаточна для barrier
guarantee: такие поверхности обрабатываются как двусторонние unsigned shells с толщиной.

Движущийся collider передаёт previous/current transform и линейную/угловую скорость. Gap и
friction считаются относительно интерполированного transform данного timestep; contact point
хранится в local space collider.

## 11. Contact barrier и persistence

Контакт активируется при `0 < g < d_hat`. Barrier стремится к бесконечности при `g -> 0`;
его stiffness масштабируется относительно inertia и не меняется произвольно между Newton
iterations. Начальный accepted state обязан иметь положительный gap.

Перед solve broad phase строит candidates по swept AABB сегментов. Narrow phase выдаёт все
пары в activation distance, а не только одну ближайшую пару на узел. Self-пары одного ребра
и рёбер с общей вершиной (`|edge_a - edge_b| <= 1`) исключаются одинаково в broad phase,
narrow phase, CCD и manifold. Пары через одно ребро и дальше не исключаются: при сильном
изгибе они должны удерживать толщину кабеля. Persistent manifold:

- warm-start normal force и tangential state;
- сохраняет пару до release distance;
- плавно переносит closest parameters;
- инвалидируется при удалении collider, изменении topology или несовместимой смене feature;
- будит sleeping island при движении collider или росте barrier force.

High-order contact не входит в первую реализацию. Перед его добавлением измеряется частота
смены segment pair/feature и корреляция со скачками force. Более простой вариант —
smooth centerline reconstruction только для narrow phase на контактах большой кривизны.

## 12. Friction

Persistent tangential displacement `u_t` обновляется в contact tangent plane с учётом
движения collider. Stick определяется отдельным пробным решением с выпуклой anchor energy:

```
E_stick = 0.5 * k_t * dot(u_t - u_anchor, u_t - u_anchor)
F_trial = k_t * ||u_t - u_anchor||
```

Если `F_trial <= mu_static * lambda_n`, пробное stick-решение допустимо. Иначе режим меняется
между внешними friction iterations, anchor проецируется на Coulomb boundary, а следующий solve
использует выпуклую regularized sliding work:

```
E_slide = mu_dynamic * lambda_n
        * (sqrt(dot(delta_u_t, delta_u_t) + epsilon_t²) - epsilon_t)
```

Обе Hessian PSD. Обратный переход имеет hysteresis. Внутри одной Newton solve режим не
меняется: Stribeck-падение силы и отрицательная tangent stiffness в baseline отсутствуют.

Внешняя friction iteration:

1. зафиксировать normal force, normal и tangent basis из текущего contact state;
2. решить incremental potential Newton;
3. обновить normal force и stick/slip state;
4. повторить до friction tolerance или лимита.

Warm start переносится только при совпадении stable `ContactKey`. При сильном повороте нормали
tangent state parallel-transport-ится; при несовместимой смене feature сбрасывается. Friction
никогда не применяется отдельной коррекцией скорости после принятого шага. Принятие проверяет
неотрицательную dissipated friction work; переход режима не может добавить энергию.

Если iteration/time budget исчерпан, принимается только состояние, прошедшее residual,
non-penetration и energy checks. Иначе timestep откатывается; частично решённое состояние
запрещено.

## 13. CCD и принятие timestep

CCD ограничивает Newton line-search step, а не исправляет проникновение после него:

- segment-segment conservative advancement для self-contact;
- segment-primitive swept query для analytic shapes;
- segment-triangle conservative advancement для mesh candidates;
- swept AABB обновляет candidate set.

`alpha = safety * time_of_impact`, если TOI найден. При `alpha` ниже минимального порога
Newton step отклоняется, diagonal regularization увеличивается либо timestep делится.

Полная гарантия C-IPC не заявляется, пока собственный CCD не проверен на changing feature,
parallel motion, zero-length edge и moving collider. Tight Inclusion рассматривается как
MIT-зависимость, если собственный segment CCD не даёт консервативного результата в измерениях.

## 14. Катушка и изменение topology

Topology не меняется внутри Newton, friction iteration или line search. Порядок:

1. контроллер сначала получает collision-resolved target без изменения transform игрока;
2. `prepare_motion(from, target)` ограничивает его по stowed length и синхронно решает trial
   timestep, но не публикует state;
3. только успешный prepare возвращает token и точный разрешённый target;
4. контроллер перемещает игрока к этому target и вызывает `commit_prepared_motion(token)`;
5. commit атомарно публикует accepted solver state, вычисляет payout и применяет
   `TopologyTransaction`;
6. failure или несовпадение фактического target отменяет token: ни player transform, ни solver
   state не должны быть закоммичены.

Одного старого `constrain_motion(from, to) -> to` недостаточно для атомарности: после движения
игрока Newton уже поздно откатывать. Метод сохраняется для V1 и read-only ограничения длины,
но V2 добавляет обязательный transactional handshake. Контроллер должен сначала выполнить
Godot collision test без движения; если он не умеет откатить несовпавшее перемещение, V2 не
может гарантировать согласованность и сообщает configuration error.

При split конечного ребра transaction переносит:

- `rest_length` так, чтобы сумма точно сохранилась;
- position новой вершины интерполяцией accepted edge;
- массу по длинам прилегающих half-edges;
- momentum существующей размотанной части;
- momentum новой массы, входящей с катушки со скоростью spool anchor;
- velocities только затронутых хвостовых узлов через constrained least-change remap: сохранить
  целевой локальный momentum и минимизировать изменение kinetic energy;
- contact history старого ребра на соответствующий child с пересчётом параметра.

В centerline-only V2 перенос material frame не нужен. Если twist появится позже, transaction
обязан parallel-transport frame и разделить integrated twist; простой copy запрещён.

Под натяжением сначала выдаётся доступная длина, затем выполняется split. Нельзя создавать
micro-segment короче минимального rest length. Если за кадр нужно несколько сегментов, число
transactions ограничивается; остаток anchor motion либо отклоняется через `constrain_motion`,
либо классифицируется как teleport. При неуспешном Newton topology transaction не применяется.

Полностью размотанная катушка не увеличивает rest length. Ограничение игрока продолжает
работать до его движения по геометрическому пути и доступной длине.

## 15. Failure policy

Для каждого timestep хранится accepted snapshot. При Newton/line-search failure:

1. отбросить trial state;
2. разделить timestep пополам до `max_retry_substeps`;
3. повторить с тем же accepted state и неизменённой topology;
4. при исчерпании retry/time budget сохранить accepted state, вернуть diagnostic failure и
   отклонить prepared motion до изменения player transform;
5. после серии failures применить только явно настроенную policy: pause V2, controlled rebuild
   или fallback в V1.

Нельзя принять penetrating state, растянуть rest length, удалить «трудные» контакты или
превысить cable length ради продолжения кадра.

## 16. Edge cases

- **Пустой state / меньше двух узлов.** Нет energies и контактов; API возвращает нулевые
  diagnostics без factorization.
- **Один сегмент.** Есть stretch и world contact, но нет bending/twist.
- **Нулевая длина ребра.** Trial отклоняется; topology transaction не создаёт такой сегмент.
- **Соседние self-пары.** Рёбра с общей вершиной исключаются во всех contact/CCD путях; иначе
  их нулевое расстояние делает обычную цепочку недопустимой для barrier.
- **Изгиб около 180°.** DER curvature сингулярна; angle-safe line search или timestep split,
  без NaN и без выбора случайной оси.
- **Плотный узел.** Дубликаты contact features объединяются; manifold не сбрасывается каждый
  Newton step; контролируются conditioning и barrier stiffness.
- **Сотни контактов.** Preallocated flat stencils, reuse sparsity, никакой аллокации на contact
  в горячем цикле.
- **Moving collider.** Relative velocity и local contact point; swept transform входит в CCD.
- **Удалённый collider.** Все его keys инвалидируются до assembly.
- **Teleport anchor.** Не превращается в velocity. Solve замораживается и требует controlled
  rebuild либо внешнего ограничения перемещения; CCD не обязан протащить кабель через сцену.
- **Полностью размотанная катушка.** Rest length не растёт, наружу отдаётся limit state.
- **Topology change под натяжением.** Только transaction между принятыми timestep с переносом
  массы и momentum.
- **Newton divergence / NaN / non-descent direction.** Regularization, rollback, timestep split.
- **Line search не находит шаг.** Candidate refresh один раз, затем rollback; нельзя принять
  `alpha = 0`.
- **Исчерпан iteration/time budget.** Принимается только валидно сошедшееся состояние, иначе
  остаётся предыдущий accepted state.
- **Смена backend.** Только граница timestep; persistent contacts теряются явно; автоматический
  бесшовный rollback не обещается.
- **Начальное пересечение.** Barrier не может стартовать из `g <= 0`; требуется отдельный
  initialization repair или отказ и controlled rebuild.
- **Открытая/неводонепроницаемая mesh.** Двусторонняя shell, а не ненадёжный signed interior.
- **Concurrent access.** Solver V2 однопоточный до отдельного этапа; snapshot immutable на время
  step. Параллельная assembly позже требует per-thread buffers и deterministic reduction.
- **Большой cable_length.** Capacity и memory проверяются до rebuild; превышение возвращает
  ошибку, а не частичный state.
- **Permission/license gap.** GPL reference code не копируется; у каждой vendored зависимости
  фиксируются версия, LICENSE и NOTICE.
- **Migration rollback.** Старый state не уничтожается во время переключения, пока новый backend
  не построил валидный accepted state.

## 17. Этапы

### Этап 0. Измерение и прототип математики

- добавить только profiling counters и backend skeleton;
- измерить доступный physics budget;
- сравнить centerline SPD, full friction Jacobian и linear solvers;
- зафиксировать go/no-go по 572/600.

Масштабная реализация до этого gate запрещена.

### Этап 1. Contact-free centerline solver

- state, inertia, stretch, bend;
- Newton, sparse assembly, line search;
- Dirichlet anchors;
- transfer в существующий mesh/debug/API.

### Этап 2. World contact

- SDF gradient, Gauss-Newton Hessian;
- barrier, moving colliders, persistent manifold;
- segment-world CCD.

### Этап 3. Self-contact и friction

- segment-segment broad/narrow phase;
- barrier Hessian stencils;
- persistent tangent state и friction outer iteration;
- CCD и knot diagnostics.

### Этап 4. Катушка

- transactional split;
- mass/momentum/contact-history transfer;
- exhausted-length feedback.

### Этап 5. Sleeping islands и performance

- связные islands, а не per-particle `inv_mass = 0`;
- wake propagation от contacts, anchors и tension;
- повторный p95/p99 gate на целевой сцене.

### Этап 6. Переключение default

Только после проверки length, non-penetration, rest stability, topology и measured budget.
V1 удаляется не в этом плане.

## 18. Критерии готовности V2

- неподвижные anchors и colliders не дают систематического роста kinetic energy;
- accepted state не имеет отрицательного gap;
- длина и max local strain остаются в физическом tolerance без LRA/projection pass;
- persistent contacts не churn-ятся сотнями без геометрической причины;
- knot не проходит сам через себя;
- moving collider не создаёт неучтённую скорость;
- topology transaction сохраняет rest length, массу и ожидаемый momentum;
- любой failure оставляет последний accepted state валидным;
- p95 укладывается в измеренный cable budget на целевом macOS arm64;
- `LEGACY_XPBD` остаётся доступен для явного отката.
