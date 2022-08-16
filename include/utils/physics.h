// Copyright (C) 2022 Martin Scheiber, Alessandro Fornasier, and others,
// Control of Networked Systems, University of Klagenfurt, Austria
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <martin.scheiber@ieee.org>
// and <alessandro.fornasier@ieee.org>.

#ifndef UTILS_PHYSICS_H_
#define UTILS_PHYSICS_H_

namespace utils
{
namespace physics
{
/// Barometric constants
constexpr double M_ = 0.0289644;  //!< molar mass of Earth's air [kg/mol]
constexpr double r_ = 8.31432;    //!< universal gas constant [Nm/mol*K]
constexpr double g_ = 9.80665;    //!< gravitational acceleration [m/s^2]
constexpr double T_ = 298.15;     //!< default reference temperature (25 degC) [K]

/// Pre-Calculated constants
constexpr double rOverMg_ = 29.2712671552;   //!< r_/(M_*g_) [m/K]
constexpr double rTOverMg_ = 8727.22830231;  //!< assuming ref teperature (r_*T_)/(M_*g_) [m]

}  // namespace physics
}  // namespace utils

#endif  // UTILS_PHYSICS_H_
