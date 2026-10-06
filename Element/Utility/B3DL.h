/*******************************************************************************
 * Copyright (C) 2017-2026 Theodore Chang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
/**
 * @class B3DL
 * @brief A B3DL class.
 *
 * See Spacone's thesis.
 *
 * Flexibility-based finite element models for the nonlinear static and dynamic analysis of concrete frame structures
 *
 * The ends of the element may be offset from its nodes by rigid arms, given in global
 * coordinates, which model eccentric connections, insertion points away from the centroid and
 * rigid zones at joints. The element then spans the offset points.
 *
 * Order of local quantities:
 *   uniform axial
 *   strong axis bending near node
 *   strong axis bending far node
 *   weak axis bending near node
 *   weak axis bending far node
 *   uniform torsion
 *
 * @author tlc
 * @date 27/06/2018
 * @version 0.1.0
 * @file B3DL.h
 * @addtogroup Utility
 * @ingroup Element
 * @{
 */

#ifndef B3DL_H
#define B3DL_H

#include <Element/Utility/Orientation.h>

class B3DL : public Orientation {
    vec offset_i, offset_j;

    [[nodiscard]] bool has_offset() const;
    [[nodiscard]] mat rigid_arm() const;

protected:
    void update_transformation() override;

public:
    using Orientation::Orientation;
    B3DL(unsigned, vec&&, vec&&);

    [[nodiscard]] Type type() const override;

    unique_ptr<Orientation> unique_copy() override;

    [[nodiscard]] vec to_local_vec(const vec&) const override;
    [[nodiscard]] vec to_global_vec(const vec&) const override;
    [[nodiscard]] mat to_global_mass_mat(const mat&) const override;
    [[nodiscard]] mat to_global_stiffness_mat(const mat&) const override;
};

#endif

//! @}
