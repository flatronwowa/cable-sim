#pragma once

#include "solver/cable_params.h"
#include "solver/cable_state.h"

namespace cable {

// Остаток на катушке считается по сумме rest_length, то есть по длине реально
// проложенного пути: игрок, обошедший колонну, тратит кабеля больше.
inline float stowed_length(const CableState &p_state, const CableParams &p_params) {
	return maxf(p_params.cable_length - p_state.deployed_length(), 0.0f);
}

// Раз в кадр перед солвером: если последний сегмент растянут сверх rest_length и на
// катушке ещё есть кабель, его rest_length увеличивается ровно настолько, чтобы снять
// натяжение. Когда он превышает target_segment_length, отщепляется новая частица.
// Размотка без сопротивления, обратной смотки нет: для катушки без пружины это верно.
// p_end_target — позиция якоря катушки на текущий кадр, а не решённая позиция
// прошлого: иначе выдача отстаёт от игрока на кадр и кабель тянется как резина.
// p_previous_end_target нужен, чтобы ограничить выдачу реальным удалением разъёма
// от опоры: тангенциальное движение и вращение игрока кабель не разматывают.
void spool_update(CableState &r_state, const CableParams &p_params, const Vec3 &p_end_target,
		const Vec3 &p_previous_end_target);

// Вызывается контроллером игрока ДО перемещения, по аналогии с move_and_slide.
// Гасится только компонента перемещения, направленная прочь от последней точки
// перегиба; движение вбок и обратно не ограничивается.
Vec3 constrain_motion(const CableState &p_state, const CableParams &p_params,
		const Vec3 &p_from, const Vec3 &p_to, bool &r_limited);

} // namespace cable
