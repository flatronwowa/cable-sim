#pragma once

#include "solver/cable_params.h"
#include "solver/cable_state.h"
#include "solver/world_snapshot.h"

namespace cable {

// Позиционный проход: выталкивание центральной линии на cable_radius + collision_margin
// от поверхности, плюс статическое трение относительно якоря контакта.
// Идёт последним среди позиционных проходов, кроме самопересечения: непроникновение
// не должно разрушаться последующими проходами.
void solve_world_contacts(CableState &r_state, const CableParams &p_params,
		const WorldSnapshot &p_world, const Broadphase &p_broadphase);

// Velocity pass: гашение нормальной компоненты и динамическое трение по Кулону.
void world_contacts_velocity_pass(CableState &r_state, const CableParams &p_params, float p_h);

} // namespace cable
