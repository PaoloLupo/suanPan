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

#include "EB31.h"

#include <Domain/DomainBase.h>
#include <Recorder/OutputType.h>

EB31::EB31(const unsigned T, uvec&& N, vec&& P, const unsigned O, const bool F)
    : SectionElement3D(T, b_node, b_dof, std::move(N), uvec{}, F)
    , orientation_tag(O)
    , property(std::move(P)) {}

int EB31::initialize(const shared_ptr<DomainBase>& D) {
    if(!D->find_orientation(orientation_tag)) {
        suanpan_warning("Element {} cannot find the assigned transformation {}.\n", get_tag(), orientation_tag);
        return SUANPAN_FAIL;
    }

    b_trans = D->get_orientation(orientation_tag)->unique_copy();

    if(b_trans->is_nlgeom() != is_nlgeom()) {
        suanpan_warning("Element {} is assigned with an inconsistent transformation {}.\n", get_tag(), orientation_tag);
        return SUANPAN_FAIL;
    }
    if(Orientation::Type::B3D != b_trans->type()) {
        suanpan_warning("Element {} is assigned with an inconsistent transformation {}, use B3DL or B3DC only.\n", get_tag(), orientation_tag);
        return SUANPAN_FAIL;
    }

    b_trans->set_element_ptr(this);

    access::rw(length) = b_trans->get_length();

    const auto& E = property(0);
    const auto& G = property(1);
    const auto& A = property(2);
    const auto& IZ = property(3);
    const auto& IY = property(4);
    const auto& J = property(5);

    // uniform axial
    // strong axis bending near node
    // strong axis bending far node
    // weak axis bending near node
    // weak axis bending far node
    // uniform torsion

    local_stiff.zeros(6, 6);
    local_stiff(0, 0) = E * A / length;

    auto factor = 2. * E * IZ / length;
    local_stiff(1, 2) = local_stiff(2, 1) = factor;
    factor *= 2.;
    local_stiff(1, 1) = local_stiff(2, 2) = factor;

    factor = 2. * E * IY / length;
    local_stiff(3, 4) = local_stiff(4, 3) = factor;
    factor *= 2.;
    local_stiff(3, 3) = local_stiff(4, 4) = factor;

    local_stiff(5, 5) = G * J / length;

    trial_stiffness = current_stiffness = initial_stiffness = b_trans->to_global_stiffness_mat(local_stiff);

    if(!nlgeom) ConstantStiffness(this);

    return SUANPAN_SUCCESS;
}

int EB31::update_status() {
    b_trans->update_status();

    const vec local_force = local_stiff * b_trans->to_local_vec(get_trial_displacement());

    trial_resistance = b_trans->to_global_vec(local_force);

    if(nlgeom) {
        trial_stiffness = b_trans->to_global_stiffness_mat(local_stiff);
        trial_geometry = b_trans->to_global_geometry_mat(local_force);
    }

    return SUANPAN_SUCCESS;
}

int EB31::commit_status() {
    b_trans->commit_status();
    return SUANPAN_SUCCESS;
}

int EB31::clear_status() {
    b_trans->clear_status();
    return SUANPAN_SUCCESS;
}

int EB31::reset_status() {
    b_trans->reset_status();
    return SUANPAN_SUCCESS;
}

std::vector<vec> EB31::record(const OutputType P) const {
    if(P == OutputType::BEAME) return {b_trans->to_local_vec(get_current_displacement())};
    if(P == OutputType::BEAMS) return {vec{local_stiff * b_trans->to_local_vec(get_current_displacement())}};

    return {};
}

void EB31::print() {
    suanpan_info("An elastic spatial beam element{}", nlgeom ? " with corotational formulation.\n" : ".\n");
}

#ifdef SUANPAN_VTK
#include <vtkLine.h>

vtkSmartPointer<vtkCell> EB31::GetCell() const { return vtkSmartPointer<vtkLine>::New(); }

mat EB31::GetData(const OutputType P) {
    if(OutputType::A == P) return reshape(get_current_acceleration(), b_dof, b_node);
    if(OutputType::V == P) return reshape(get_current_velocity(), b_dof, b_node);
    if(OutputType::U == P) return reshape(get_current_displacement(), b_dof, b_node);

    return {};
}

mat EB31::GetDeformation(const double amplifier) { return get_coordinate(3).t() + amplifier * reshape(get_current_displacement(), b_dof, b_node).eval().head_rows(3); }

#endif
