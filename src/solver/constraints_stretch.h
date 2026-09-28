#pragma once

#include "solver/cable_params.h"
#include "solver/cable_state.h"

namespace cable {

// Каноническая XPBD distance constraint: C = |x1 - x0| - rest_length,
// compliance масштабируется как alpha / h^2, lambda накапливается.
void solve_stretch(CableState &r_state, const CableParams &p_params, float p_h);

// Long Range Attachments (Kim et al., 2012): расстояние частицы до закреплённого
// конца не больше дуговой длины до него. Односторонний, без compliance, по одному
// проходу в каждую сторону — снимает медленное расхождение растяжения по длинной цепи.
void solve_lra(CableState &r_state);

} // namespace cable
