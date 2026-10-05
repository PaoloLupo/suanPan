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
 * @class EB31
 * @brief The EB31 class.
 *
 * Elastic 3D Euler--Bernoulli beam element with uniform torsion.
 *
 * Unlike B31 and F31, whose sections carry no torsion and which are assigned an artificial
 * torsional stiffness, EB31 takes GJ explicitly, so that torsion and the torsional modes of
 * structures are captured.
 *
 * @author PaoloLupo
 * @date 05/10/2026
 * @version 0.1.0
 * @file EB31.h
 * @addtogroup Beam
 * @ingroup Element
 * @{
 */

#ifndef EB31_H
#define EB31_H

#include <Element/SectionElement.h>
#include <Element/Utility/Orientation.h>

class EB31 final : public SectionElement3D {
    static constexpr unsigned b_node = 2u, b_dof = 6u, b_size = b_dof * b_node;

    const unsigned orientation_tag;

    const double length = 0.;

    unique_ptr<Orientation> b_trans;

    const vec property; // [E, G, A, IZ, IY, J]

    mat local_stiff;

public:
    EB31(
        unsigned,    // tag
        uvec&&,      // node tags
        vec&&,       // properties
        unsigned,    // orientation tag
        bool = false // nonlinear geometry switch
    );

    int initialize(const shared_ptr<DomainBase>&) override;

    int update_status() override;

    int commit_status() override;
    int clear_status() override;
    int reset_status() override;

    [[nodiscard]] std::vector<vec> record(OutputType) const override;

    void print() override;

#ifdef SUANPAN_VTK
    [[nodiscard]] vtkSmartPointer<vtkCell> GetCell() const override;

    mat GetData(OutputType) override;
    mat GetDeformation(double) override;
#endif
};

#endif

//! @}
