#ifndef RCG_RM_65_EXAMPLE_MODEL_CONSTANTS_H_
#define RCG_RM_65_EXAMPLE_MODEL_CONSTANTS_H_

// #include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace rm_65_example {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar rx_joint4 = -1.5707999467849731;
const Scalar sin_rx_joint4 = ScalarTraits::sin(rx_joint4);
const Scalar cos_rx_joint4 = ScalarTraits::cos(rx_joint4);
const Scalar rx_joint5 = 1.5707999467849731;
const Scalar sin_rx_joint5 = ScalarTraits::sin(rx_joint5);
const Scalar cos_rx_joint5 = ScalarTraits::cos(rx_joint5);
const Scalar rx_joint6 = -1.5707999467849731;
const Scalar sin_rx_joint6 = ScalarTraits::sin(rx_joint6);
const Scalar cos_rx_joint6 = ScalarTraits::cos(rx_joint6);
const Scalar rz_fr_Link2_COM = 1.5708036422729492;
const Scalar sin_rz_fr_Link2_COM = ScalarTraits::sin(rz_fr_Link2_COM);
const Scalar cos_rz_fr_Link2_COM = ScalarTraits::cos(rz_fr_Link2_COM);
const Scalar rz_fr_Link3_COM = -3.1415815353393555;
const Scalar sin_rz_fr_Link3_COM = ScalarTraits::sin(rz_fr_Link3_COM);
const Scalar cos_rz_fr_Link3_COM = ScalarTraits::cos(rz_fr_Link3_COM);
const Scalar rz_fr_Link4_COM = -3.1415889263153076;
const Scalar sin_rz_fr_Link4_COM = ScalarTraits::sin(rz_fr_Link4_COM);
const Scalar cos_rz_fr_Link4_COM = ScalarTraits::cos(rz_fr_Link4_COM);
const Scalar rz_fr_Link5_COM = -3.1415815353393555;
const Scalar sin_rz_fr_Link5_COM = ScalarTraits::sin(rz_fr_Link5_COM);
const Scalar cos_rz_fr_Link5_COM = ScalarTraits::cos(rz_fr_Link5_COM);
const Scalar rz_fr_Link6_COM = -3.1415889263153076;
const Scalar sin_rz_fr_Link6_COM = ScalarTraits::sin(rz_fr_Link6_COM);
const Scalar cos_rz_fr_Link6_COM = ScalarTraits::cos(rz_fr_Link6_COM);
const Scalar tz_joint1 = 0.24050000309944153;
const Scalar tx_joint3 = -1.8806809976013028E-6;
const Scalar ty_joint3 = 0.25600001215934753;
const Scalar tx_joint4 = -2.3141192286857404E-6;
const Scalar ty_joint4 = 0.20999999344348907;
const Scalar tx_joint6 = -1.5868246237005224E-6;
const Scalar ty_joint6 = 0.14399999380111694;
const Scalar tx_fr_base_link_COM = -4.332773096393794E-4;
const Scalar ty_fr_base_link_COM = -3.546644074958749E-5;
const Scalar tz_fr_base_link_COM = 0.05994276702404022;
const Scalar tx_fr_Link1_COM = 1.2226305301510365E-8;
const Scalar ty_fr_Link1_COM = 0.021107997745275497;
const Scalar tz_fr_Link1_COM = -0.025185422971844673;
const Scalar tx_fr_Link2_COM = -1.5939220929794828E-6;
const Scalar ty_fr_Link2_COM = 0.15225645899772644;
const Scalar tz_fr_Link2_COM = -0.006202603690326214;
const Scalar tx_fr_Link3_COM = -5.709813649446005E-6;
const Scalar ty_fr_Link3_COM = 0.05959256738424301;
const Scalar tz_fr_Link3_COM = 0.010569069534540176;
const Scalar tx_fr_Link4_COM = -1.2214395610499196E-6;
const Scalar ty_fr_Link4_COM = 0.018042447045445442;
const Scalar tz_fr_Link4_COM = -0.0215394739061594;
const Scalar tx_fr_Link5_COM = -3.852301688311854E-6;
const Scalar ty_fr_Link5_COM = 0.05938083678483963;
const Scalar tz_fr_Link5_COM = 0.007368042599409819;
const Scalar tx_fr_Link6_COM = -7.142359972931445E-4;
const Scalar ty_fr_Link6_COM = 3.9671611739322543E-4;
const Scalar tz_fr_Link6_COM = -0.01267236564308405;
const Scalar m_base_link = 0.8410707712173462;
const Scalar comx_base_link = -4.332773096393794E-4;
const Scalar comy_base_link = -3.546644074958749E-5;
const Scalar comz_base_link = 0.05994276702404022;
const Scalar ix_base_link = 0.0030220821499824524;
const Scalar ixy_base_link = 1.292456985879653E-8;
const Scalar ixz_base_link = -2.1844154616701417E-5;
const Scalar iy_base_link = 0.003022239077836275;
const Scalar iyz_base_link = -1.788079998732428E-6;
const Scalar iz_base_link = 1.5895152216671704E-7;
const Scalar m_Link1 = 0.593563437461853;
const Scalar comx_Link1 = 1.2226305301510365E-8;
const Scalar comy_Link1 = 0.021107997745275497;
const Scalar comz_Link1 = -0.025185422971844673;
const Scalar ix_Link1 = 6.409613415598869E-4;
const Scalar ixy_Link1 = 1.5318259161123393E-10;
const Scalar ixz_Link1 = -1.8277282465284372E-10;
const Scalar iy_Link1 = 3.7650056765414774E-4;
const Scalar iyz_Link1 = -3.1554652377963066E-4;
const Scalar iz_Link1 = 2.6446074480190873E-4;
const Scalar m_Link2 = 0.864175021648407;
const Scalar comx_Link2 = -1.5939220929794828E-6;
const Scalar comy_Link2 = 0.15225645899772644;
const Scalar comz_Link2 = -0.006202603690326214;
const Scalar ix_Link2 = 0.020066577941179276;
const Scalar ixy_Link2 = -2.0972225911464193E-7;
const Scalar ixz_Link2 = 8.54363779723144E-9;
const Scalar iy_Link2 = 3.324679346405901E-5;
const Scalar iyz_Link2 = -8.1611517816782E-4;
const Scalar iz_Link2 = 0.020033329725265503;
const Scalar m_Link3 = 0.28963369131088257;
const Scalar comx_Link3 = -5.709813649446005E-6;
const Scalar comy_Link3 = 0.05959256738424301;
const Scalar comz_Link3 = 0.010569069534540176;
const Scalar ix_Link3 = 0.0010609221644699574;
const Scalar ixy_Link3 = -9.855146743120713E-8;
const Scalar ixz_Link3 = -1.7478646086033223E-8;
const Scalar iy_Link3 = 3.235361145925708E-5;
const Scalar iyz_Link3 = 1.8242229998577386E-4;
const Scalar iz_Link3 = 0.0010285686003044248;
const Scalar m_Link4 = 0.23941977322101593;
const Scalar comx_Link4 = -1.2214395610499196E-6;
const Scalar comy_Link4 = 0.018042447045445442;
const Scalar comz_Link4 = -0.0215394739061594;
const Scalar ix_Link4 = 1.8901683506555855E-4;
const Scalar ixy_Link4 = -5.2762754165769365E-9;
const Scalar ixz_Link4 = 6.298934263071487E-9;
const Scalar iy_Link4 = 1.1107854516012594E-4;
const Scalar iyz_Link4 = -9.304446575697511E-5;
const Scalar iz_Link4 = 7.793828990543261E-5;
const Scalar m_Link5 = 0.21879975497722626;
const Scalar comx_Link5 = -3.852301688311854E-6;
const Scalar comy_Link5 = 0.05938083678483963;
const Scalar comz_Link5 = 0.007368042599409819;
const Scalar ix_Link5 = 7.83384486567229E-4;
const Scalar ixy_Link5 = -5.0051077948864986E-8;
const Scalar ixz_Link5 = -6.210395309125261E-9;
const Scalar iy_Link5 = 1.1878215445904061E-5;
const Scalar iyz_Link5 = 9.572938870405778E-5;
const Scalar iz_Link5 = 7.715062820352614E-4;
const Scalar m_Link6 = 0.06490180641412735;
const Scalar comx_Link6 = -7.142359972931445E-4;
const Scalar comy_Link6 = 3.9671611739322543E-4;
const Scalar comz_Link6 = -0.01267236564308405;
const Scalar ix_Link6 = 1.0432720955577679E-5;
const Scalar ixy_Link6 = -1.838985674851301E-8;
const Scalar ixz_Link6 = 5.87430122322985E-7;
const Scalar iy_Link6 = 1.0455614756210707E-5;
const Scalar iyz_Link6 = -3.262829011418944E-7;
const Scalar iz_Link6 = 4.3323044707221925E-8;

}
}
#endif
