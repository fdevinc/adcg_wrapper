/******************************************************************************
 *
 * @file adcg_wrapper/autodiff/support/quaternion.hpp
 * @author Flavio De Vincenti (flavio.devincenti@inf.ethz.ch)
 *
 * @section LICENSE
 * -----------------------------------------------------------------------
 *
 * Copyright 2023 Flavio De Vincenti
 *
 * -----------------------------------------------------------------------
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 ******************************************************************************/

#ifndef _ADCG_WRAPPER__AUTODIFF__SUPPORT__QUATERNION_HPP_
#define _ADCG_WRAPPER__AUTODIFF__SUPPORT__QUATERNION_HPP_

#include "adcg_wrapper/autodiff/data_types.hpp"

namespace Eigen {

template <>
inline Quaternion<ADCGWrapper::ad_scalar_t> QuaternionBase<ADCGWrapper::Quaternionad>::inverse()
    const {
    Scalar n2 = this->squaredNorm();
    return Quaternion<Scalar>{conjugate().coeffs() /
                              CppAD::CondExpGt(n2, RealScalar{0}, n2, RealScalar{1.0})};
}
template <>
inline Quaternion<ADCGWrapper::ad_scalar_t>
QuaternionBase<Eigen::Map<ADCGWrapper::Quaternionad>>::inverse() const {
    Scalar n2 = this->squaredNorm();
    return Quaternion<Scalar>{conjugate().coeffs() /
                              CppAD::CondExpGt(n2, RealScalar{0}, n2, RealScalar{1.0})};
}
template <>
inline Quaternion<ADCGWrapper::ad_scalar_t>
QuaternionBase<Eigen::Map<const ADCGWrapper::Quaternionad>>::inverse() const {
    Scalar n2 = this->squaredNorm();
    return Quaternion<Scalar>{conjugate().coeffs() /
                              CppAD::CondExpGt(n2, RealScalar{0}, n2, RealScalar{1.0})};
}

template <>
inline void MatrixBase<ADCGWrapper::Vector4ad>::normalize() {
    RealScalar z = squaredNorm();
    derived() /= CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}
template <>
inline void MatrixBase<Eigen::Map<ADCGWrapper::Vector4ad>>::normalize() {
    RealScalar z = squaredNorm();
    derived() /= CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}

template <>
inline const typename MatrixBase<ADCGWrapper::Vector4ad>::PlainObject
MatrixBase<ADCGWrapper::Vector4ad>::normalized() const {
    using _Nested = const Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, 4, 1, 0>&;
    _Nested n(derived());
    RealScalar z = n.squaredNorm();
    return n / CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}
template <>
inline const typename MatrixBase<Eigen::Map<ADCGWrapper::Vector4ad>>::PlainObject
MatrixBase<Eigen::Map<ADCGWrapper::Vector4ad>>::normalized() const {
    using _Nested = const Eigen::Map<Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, 4, 1, 0>, 0>;
    _Nested n(derived());
    RealScalar z = n.squaredNorm();
    return n / CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}
template <>
inline const typename MatrixBase<Eigen::Map<const ADCGWrapper::Vector4ad>>::PlainObject
MatrixBase<Eigen::Map<const ADCGWrapper::Vector4ad>>::normalized() const {
    using _Nested =
        const Eigen::Map<const Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, 4, 1, 0>, 0>;
    _Nested n(derived());
    RealScalar z = n.squaredNorm();
    return n / CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}

template <>
inline const typename MatrixBase<ADCGWrapper::Vector3ad>::PlainObject
MatrixBase<ADCGWrapper::Vector3ad>::normalized() const {
    using _Nested = const Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, 3, 1, 0>&;
    _Nested n(derived());
    RealScalar z = n.squaredNorm();
    return n / CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}
template <>
inline const typename MatrixBase<Eigen::Map<ADCGWrapper::Vector3ad>>::PlainObject
MatrixBase<Eigen::Map<ADCGWrapper::Vector3ad>>::normalized() const {
    using _Nested = const Eigen::Map<Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, 3, 1, 0>, 0>;
    _Nested n(derived());
    RealScalar z = n.squaredNorm();
    return n / CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}
template <>
inline const typename MatrixBase<Eigen::Map<const ADCGWrapper::Vector3ad>>::PlainObject
MatrixBase<Eigen::Map<const ADCGWrapper::Vector3ad>>::normalized() const {
    using _Nested =
        const Eigen::Map<const Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, 3, 1, 0>, 0>;
    _Nested n(derived());
    RealScalar z = n.squaredNorm();
    return n / CppAD::CondExpGt(z, RealScalar{0}, numext::sqrt(z), RealScalar{1.0});
}

namespace internal {

template <>
struct quaternionbase_assign_impl<ADCGWrapper::Matrix3ad, 3, 3> {
    typedef typename ADCGWrapper::Matrix3ad::Scalar Scalar;
    template <class Derived>
    static inline void run(QuaternionBase<Derived>& q, const ADCGWrapper::Matrix3ad& a_mat) {
        static_assert(ADCGWrapper::dependent_false<Derived>,
                      "The construction of unit quaternions from rotation matrices "
                      "with scalar "
                      "type 'ad_scalar_t' is not implemented.");
    }
};

}  // namespace internal

template <>
template <class OtherDerived>
Quaternion<ADCGWrapper::ad_scalar_t> QuaternionBase<ADCGWrapper::Quaternionad>::slerp(
    const Scalar& t, const QuaternionBase<OtherDerived>& other) const {
    using std::acos;
    using std::sin;
    const Scalar one  = Scalar{1.0} - NumTraits<Scalar>::epsilon();
    const Scalar d    = this->dot(other);
    const Scalar absD = numext::abs(d);

    Scalar scale0;
    Scalar scale1;

    scale0 = CppAD::CondExpGe(
        absD, one, Scalar{1.0} - t, sin((Scalar{1.0} - t) * acos(absD)) / sin(acos(absD)));
    scale1 = CppAD::CondExpGe(absD, one, t, sin((t * acos(absD))) / sin(acos(absD)));
    scale1 = CppAD::CondExpLt(d, Scalar{0.0}, -scale1, scale1);

    return Quaternion<Scalar>{scale0 * coeffs() + scale1 * other.coeffs()};
}
template <>
template <class OtherDerived>
Quaternion<ADCGWrapper::ad_scalar_t> QuaternionBase<Eigen::Map<ADCGWrapper::Quaternionad>>::slerp(
    const Scalar& t, const QuaternionBase<OtherDerived>& other) const {
    using std::acos;
    using std::sin;
    const Scalar one  = Scalar{1.0} - NumTraits<Scalar>::epsilon();
    const Scalar d    = this->dot(other);
    const Scalar absD = numext::abs(d);

    Scalar scale0;
    Scalar scale1;

    scale0 = CppAD::CondExpGe(
        absD, one, Scalar{1.0} - t, sin((Scalar{1.0} - t) * acos(absD)) / sin(acos(absD)));
    scale1 = CppAD::CondExpGe(absD, one, t, sin((t * acos(absD))) / sin(acos(absD)));
    scale1 = CppAD::CondExpLt(d, Scalar{0.0}, -scale1, scale1);

    return Quaternion<Scalar>{scale0 * coeffs() + scale1 * other.coeffs()};
}
template <>
template <class OtherDerived>
Quaternion<ADCGWrapper::ad_scalar_t>
QuaternionBase<Eigen::Map<const ADCGWrapper::Quaternionad>>::slerp(
    const Scalar& t, const QuaternionBase<OtherDerived>& other) const {
    using std::acos;
    using std::sin;
    const Scalar one  = Scalar{1.0} - NumTraits<Scalar>::epsilon();
    const Scalar d    = this->dot(other);
    const Scalar absD = numext::abs(d);

    Scalar scale0;
    Scalar scale1;

    scale0 = CppAD::CondExpGe(
        absD, one, Scalar{1.0} - t, sin((Scalar{1.0} - t) * acos(absD)) / sin(acos(absD)));
    scale1 = CppAD::CondExpGe(absD, one, t, sin((t * acos(absD))) / sin(acos(absD)));
    scale1 = CppAD::CondExpLt(d, Scalar{0.0}, -scale1, scale1);

    return Quaternion<Scalar>{scale0 * coeffs() + scale1 * other.coeffs()};
}

template <>
template <typename Derived1, typename Derived2>
inline ADCGWrapper::Quaternionad& QuaternionBase<ADCGWrapper::Quaternionad>::setFromTwoVectors(
    const MatrixBase<Derived1>& a, const MatrixBase<Derived2>& b) {
    static_assert(ADCGWrapper::dependent_false<Derived1, Derived2>,
                  "The construction of unit quaternions with scalar type "
                  "'ad_scalar_t' from two "
                  "vectors is not implemented.");
    return derived();
}
template <>
template <typename Derived1, typename Derived2>
inline Eigen::Map<ADCGWrapper::Quaternionad>&
QuaternionBase<Eigen::Map<ADCGWrapper::Quaternionad>>::setFromTwoVectors(
    const MatrixBase<Derived1>& a, const MatrixBase<Derived2>& b) {
    static_assert(ADCGWrapper::dependent_false<Derived1, Derived2>,
                  "The construction of unit quaternions with scalar type "
                  "'ad_scalar_t' from two "
                  "vectors is not implemented.");
    return derived();
}
template <>
template <typename Derived1, typename Derived2>
inline Eigen::Map<const ADCGWrapper::Quaternionad>&
QuaternionBase<Eigen::Map<const ADCGWrapper::Quaternionad>>::setFromTwoVectors(
    const MatrixBase<Derived1>& a, const MatrixBase<Derived2>& b) {
    static_assert(ADCGWrapper::dependent_false<Derived1, Derived2>,
                  "The construction of unit quaternions with scalar type "
                  "'ad_scalar_t' from two "
                  "vectors is not implemented.");
    return derived();
}

}  // namespace Eigen

#endif /* _ADCG_WRAPPER__AUTODIFF__SUPPORT__QUATERNION_HPP_ */
