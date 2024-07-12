#include "jacobians.h"

rm_65_example::rcg::Jacobians::Jacobians()
{}

void rm_65_example::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

