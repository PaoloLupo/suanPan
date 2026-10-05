#include "CatchHeader.h"

#include <Domain/MetaMat/Dense.Single/FullMat.hpp>
#include <Toolbox/arpack.h>
#include <Toolbox/utility.h>

TEST_CASE("Eigensolver", "[Utility.Eigen]") {
    constexpr auto N = 100;

    const vec D = regspace(1, 1, N);

    for(auto L = 0; L < N; ++L) {
        constexpr auto Q = 6;
        const mat P = orth(randn(D.n_elem, D.n_elem));

        mat K = P * diagmat(D) * P.t();

        mat M = 2. * eye(size(K));

        auto KK = std::make_shared<FullMat<double>>(D.n_elem, D.n_elem);

        for(uword I{0}; I < D.n_elem; ++I)
            for(uword J{0}; J < D.n_elem; ++J) KK->at(J, I) = K(J, I);

        auto MM = std::make_shared<FullMat<double>>(D.n_elem, D.n_elem);

        for(uword I{0}; I < D.n_elem; ++I) MM->at(I, I) = M(I, I);

        vec eigval;
        mat eigvec;

        REQUIRE(eig_solve(eigval, eigvec, KK, MM->unique_copy(), Q, "SM") == 0);

        for(auto I = 0; I < Q; ++I)
            REQUIRE(Approx(eigval(I)) == .5 * I + .5);

        REQUIRE(eig_solve(eigval, eigvec, KK, MM->unique_copy(), Q, "LM") == 0);

        for(auto I = 0; I < Q; ++I)
            REQUIRE(Approx(eigval(Q - 1 - I)) == .5 * (N - I));

        cx_vec cx_eigval;
        cx_mat cx_eigvec;

        REQUIRE(eig_solve(cx_eigval, cx_eigvec, KK->unique_copy(), MM->unique_copy(), Q, "LM") == 0);

        for(auto I = 0; I < Q; ++I)
            REQUIRE(Approx(cx_eigval(I).real()) == .5 * I + .5);
    }
}

TEST_CASE("Constrained Eigensolver", "[Utility.Eigen]") {
    constexpr auto N = 60;
    constexpr auto C = 5;
    constexpr auto Q = 6;

    const mat P = orth(randn(N, N));
    const mat K = P * diagmat(regspace(1, 1, N)) * P.t();
    const vec D = 1. + randu(N);
    const sp_mat B(randn(N, C));

    // reference eigenvalues of the problem reduced to the null space of B^T
    const mat T = null(mat(B.t()));
    const mat L = inv(trimatl(chol(T.t() * diagmat(D) * T, "lower")));
    const mat R = L * T.t() * K * T * L.t();
    const vec reference = eig_sym(.5 * (R + R.t()));

    auto KK = std::make_shared<FullMat<double>>(N, N);

    for(uword I{0}; I < N; ++I)
        for(uword J{0}; J < N; ++J) KK->at(J, I) = K(J, I);

    auto MM = std::make_shared<FullMat<double>>(N, N);

    for(uword I{0}; I < N; ++I) MM->at(I, I) = D(I);

    vec eigval;
    mat eigvec;

    REQUIRE(eig_solve(eigval, eigvec, KK, MM->unique_copy(), Q, "SM", B) == 0);

    for(auto I = 0; I < Q; ++I)
        REQUIRE(Approx(eigval(I)) == reference(I));

    REQUIRE(norm(B.t() * eigvec, "inf") < 1E-10);

    REQUIRE(eig_solve(eigval, eigvec, KK, MM->unique_copy(), Q, "LM", B) == 0);

    for(auto I = 0; I < Q; ++I)
        REQUIRE(Approx(eigval(Q - 1 - I)) == reference(reference.n_elem - 1 - I));

    REQUIRE(norm(B.t() * eigvec, "inf") < 1E-10);
}
