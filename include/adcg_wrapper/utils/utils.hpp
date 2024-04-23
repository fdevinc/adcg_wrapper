/******************************************************************************
 *
 * @file adcg_wrapper/utils/utils.hpp
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

#ifndef _ADCG_WRAPPER__UTILS__UTILS_HPP_
#define _ADCG_WRAPPER__UTILS__UTILS_HPP_

#include <filesystem>
#include <locale>
#include <random>

#include "adcg_wrapper/assert.hpp"
#include "adcg_wrapper/data_types.hpp"

#include "adcg_wrapper/autodiff/support/quaternion.hpp"

namespace ADCGWrapper {
namespace Concepts {

template <typename _Scalar>
concept Scalar = std::convertible_to<_Scalar, real_t> || std::convertible_to<_Scalar, ad_scalar_t>;

}  // namespace Concepts

[[noreturn]] inline void Unreachable() {
    // Uses compiler specific extensions if possible.
    // Even if no extension is used, undefined behavior is still raised by
    // an empty function body and the noreturn attribute.
#ifdef __GNUC__  // GCC, Clang, ICC
    __builtin_unreachable();
#elif defined _MSC_VER  // MSVC
    __assume(false);
#endif
}

namespace Utils {

/**
 * @brief Convert a real matrix into a dynamic array of triplets.
 *
 * @tparam _CLEAR_TRIPLETS   If true, the output dynamic array of triplets is
 *                           cleared before the input matrix gets converted.
 * @param[in] matrix         Input matrix to be converted into triplets.
 * @param[out] triplets      Dynamic array of triplets generated from the
 *                           input matrix.
 * @param[in] rowOffset      Offset applied to all the row indices of the
 *                           triplets generated from the input matrix.
 * @param[in] colOffset      Offset applied to all the column indices of
 *                           the triplets generated from the input matrix.
 */
template <bool _CLEAR_TRIPLETS = false>
void MatrixToTriplets(const RefToConstMatrixXr& matrix,
                      std::vector<MutableTriplet<real_t>>& triplets,
                      const index_t rowOffset = 0_idx,
                      const index_t colOffset = 0_idx) {
    if constexpr (_CLEAR_TRIPLETS) {
        triplets.clear();
    }

    for (index_t j = 0_idx; j < matrix.cols(); ++j) {
        for (index_t i = 0_idx; i < matrix.rows(); ++i) {
            triplets.emplace_back(i + rowOffset, j + colOffset, matrix(i, j));
        }
    }
}

/**
 * @brief Convert a real sparse matrix into a dynamic array of triplets.
 *
 * @tparam _LOWER_TRIANGULAR If true, only the lower triangular part
 *                           of the input matrix gets converted.
 * @tparam _CLEAR_TRIPLETS   If true, the output dynamic array of triplets is
 *                           cleared before the input matrix gets converted.
 * @param[in] sparseMatrix   Input matrix to be converted into triplets.
 * @param[out] triplets      Dynamic array of triplets generated from the
 *                           input matrix.
 * @param[in] rowOffset      Offset applied to all the row indices of the
 *                           triplets generated from the input matrix.
 * @param[in] colOffset      Offset applied to all the column indices of
 *                           the triplets generated from the input matrix.
 */
template <bool _LOWER_TRIANGULAR = false, bool _CLEAR_TRIPLETS = false>
void SparseMatrixToTriplets(const SparseMatrix<real_t>& sparseMatrix,
                            std::vector<MutableTriplet<real_t>>& triplets,
                            const index_t rowOffset = 0_idx,
                            const index_t colOffset = 0_idx) {
    if constexpr (_CLEAR_TRIPLETS) {
        triplets.clear();
    }

    for (int i = 0; i < sparseMatrix.outerSize(); ++i) {
        for (SparseMatrix<real_t>::InnerIterator it(sparseMatrix, i); it; ++it) {
            if constexpr (_LOWER_TRIANGULAR) {
                if (it.row() < it.col()) {
                    continue;
                }
            }
            triplets.emplace_back(it.row() + rowOffset, it.col() + colOffset, it.value());
        }
    }
}

namespace Internal {
namespace Concepts {

// clang-format off
template <typename _SparseMatrix>
concept HasNonZeros = requires (const _SparseMatrix sparseMatrix) {
    sparseMatrix.nonZeros();
};
// clang-format on

}  // namespace Concepts
}  // namespace Internal

inline std::string ToSnakeCase(std::string_view input) {
    std::string output{};
    for (size_t i = 1UL; const char c : input) {
        if (std::isdigit(c, std::locale())) {
            output.append(1UL, c);
        } else if (std::isalpha(c, std::locale())) {
            if ((i < input.size() && std::islower(c, std::locale()) &&
                 std::isupper(input[i], std::locale())) ||
                (i + 1UL < input.size() && std::isupper(c, std::locale()) &&
                 std::isupper(input[i], std::locale()) &&
                 std::islower(input[i + 1UL], std::locale()))) {
                output.append(static_cast<char>(std::tolower(c)) + "_"s);
            } else {
                output.append(1UL, std::tolower(c));
            }
        } else {
            output.append(1UL, '_');
        }

        ++i;
    }

    return output;
}

namespace Internal {

template <typename _Scalar>
struct MutableVectorSegmenter {
  public:
    template <typename _Vector>
    MutableVectorSegmenter(Eigen::MatrixBase<_Vector> const& vector)
        : _end{vector.const_cast_derived().data()} {
    }

    MutableVectorSegmenter(_Scalar* const begin) : _end{begin} {
    }

    _Scalar& Next() {
        _Scalar* const it = _end;
        _end              = std::ranges::next(_end);
        return *it;
    }

    auto Next(const std::integral auto size) {
        _Scalar* const begin = _end;
        _end                 = std::ranges::next(_end, size);
        return Eigen::Map<VectorX<_Scalar>>{begin, static_cast<index_t>(size)};
    }

    MutableVectorSegmenter& Skip(const std::integral auto size) {
        _end = std::ranges::next(_end, size);
        return *this;
    }

    auto End() const {
        return _end;
    }

  private:
    _Scalar* _end;
};

template <class _Vector>
MutableVectorSegmenter(Eigen::MatrixBase<_Vector> const&)
    -> MutableVectorSegmenter<typename _Vector::Scalar>;

template <typename _Scalar>
struct ConstVectorSegmenter {
  public:
    template <typename _Vector>
    ConstVectorSegmenter(const Eigen::MatrixBase<_Vector>& vector) : _end{vector.derived().data()} {
    }

    ConstVectorSegmenter(const _Scalar* const begin) : _end{begin} {
    }

    const _Scalar& Next() {
        const _Scalar* const it = _end;
        _end                    = std::ranges::next(_end);
        return *it;
    }

    auto Next(const std::integral auto size) {
        const _Scalar* const begin = _end;
        _end                       = std::ranges::next(_end, size);
        return Eigen::Map<const VectorX<_Scalar>>{begin, static_cast<index_t>(size)};
    }

    ConstVectorSegmenter& Skip(const std::integral auto size) {
        _end = std::ranges::next(_end, size);
        return *this;
    }

    auto End() const {
        return _end;
    }

  private:
    const _Scalar* _end;
};

template <class _Vector>
ConstVectorSegmenter(const Eigen::MatrixBase<_Vector>&)
    -> ConstVectorSegmenter<typename _Vector::Scalar>;

}  // namespace Internal

template <typename _Vector>
inline auto VectorSegmenter(Eigen::MatrixBase<_Vector> const& vector) {
    static_assert(dependent_false<_Vector>,
                  "This function is not implemented for the given input type; "
                  "consider wrapping it "
                  "into an Eigen::Ref (or Eigen::Map) object.");
}

template <typename _Vector>  // clang-format off
requires std::same_as<_Vector, const typename std::remove_cvref_t<_Vector>::PlainMatrix&> ||
         Concepts::EigenConstTypeWrapper<std::remove_cvref_t<_Vector>>
inline auto VectorSegmenter(_Vector&& vector) {  // clang-format on
    return Internal::ConstVectorSegmenter{std::forward<_Vector>(vector)};
}

template <typename _Scalar>
inline Internal::ConstVectorSegmenter<_Scalar> VectorSegmenter(const _Scalar* const begin) {
    return {begin};
}

template <typename _Vector>  // clang-format off
requires std::same_as<_Vector, typename std::remove_cvref_t<_Vector>::PlainMatrix&> ||
         Concepts::EigenMutableTypeWrapper<std::remove_cvref_t<_Vector>>
inline auto VectorSegmenter(_Vector&& vector) {  // clang-format on
    return Internal::MutableVectorSegmenter{std::forward<_Vector>(vector)};
}

template <typename _Scalar>
inline Internal::MutableVectorSegmenter<_Scalar> VectorSegmenter(_Scalar* const begin) {
    return {begin};
}

inline std::filesystem::path TemporaryDirectoryPath(const std::string& directoryNamePrefix = ""s,
                                                    const size_t maxAttempts = 1024UL) {
    const auto tmp = std::filesystem::temp_directory_path();

    std::random_device r;
    std::mt19937 randomEngine{r()};
    std::uniform_int_distribution<uint64_t> uniformDistribution{0};

    size_t i = 1UL;
    std::filesystem::path temporaryDirectoryPath;
    while (true) {
        if (i > maxAttempts) {
            throw std::runtime_error("Could not generate nonexistent temporary directory path.");
        }

        std::stringstream ss;
        ss << std::hex << uniformDistribution(randomEngine);
        temporaryDirectoryPath = tmp / (directoryNamePrefix + ss.str());

        if (!std::filesystem::is_directory(temporaryDirectoryPath)) {
            break;
        }

        ++i;
    }

    return temporaryDirectoryPath;
}

template <typename _Derived>
inline Vector3<typename _Derived::Scalar> EstimateLinearVelocity(
    const Eigen::MatrixBase<_Derived>& position,
    const Eigen::MatrixBase<_Derived>& previousPosition,
    const real_t stepSize) {
    return (position - previousPosition) / stepSize;
}

template <Concepts::Scalar _Scalar = real_t>
inline Vector3<_Scalar> EstimateBodyFrameAngularVelocity(
    const Quaternion<_Scalar>& orientation,
    const Quaternion<_Scalar>& previousOrientation,
    const real_t stepSize) {
    return 2.0 * (previousOrientation.conjugate() * orientation).vec() / stepSize;
}

template <Concepts::Scalar _Scalar = real_t>
inline Vector3<_Scalar> EstimateInertialFrameAngularVelocity(
    const Quaternion<_Scalar>& orientation,
    const Quaternion<_Scalar>& previousOrientation,
    const real_t stepSize) {
    return 2.0 * (orientation * previousOrientation.conjugate()).vec() / stepSize;
}

template <typename _Vector>
inline Quaternion<typename _Vector::Scalar> ExponentialMap(const Eigen::MatrixBase<_Vector>& v) {
    static_assert(_Vector::RowsAtCompileTime == 3);
    using ScalarType = typename _Vector::Scalar;
    Quaternion<ScalarType> q;

    if constexpr (std::convertible_to<ScalarType, real_t>) {
        const ScalarType vNorm = v.norm();
        q.vec()                = vNorm > 0.0 ? Vector3<ScalarType>{v * sin(0.5 * vNorm) / vNorm}
                                             : Vector3<ScalarType>{v * 0.5};
        q.w()                  = vNorm > 0.0 ? cos(0.5 * vNorm) : 1.0;
    }
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
    else if constexpr (std::convertible_to<ScalarType, ad_scalar_t>) {
        static_assert(dependent_false<ScalarType>,
                      "The 'ExponentialMap' function is not implemented for vectors with "
                      "scalar "
                      "type 'ad_scalar_t'. Use 'ApproximateExponentialMap' instead.");
        /**
         * @todo Implement non-approximate version of 'ExponentialMap' for vectors
         * with scalar type 'ad_scalar_t'.
         */
        // Here is an example implementation that does not work:
        // const ad_scalar_t vNorm = v.norm();
        // const ad_scalar_t vNormIsNotZero =
        //     CppAD::CondExpGt(vNorm, ad_scalar_t{0.0}, ad_scalar_t{1},
        //     ad_scalar_t{0});
        // const ad_scalar_t vNormIsZero = 1.0 - vNormIsNotZero;
        // q.vec() = v * (CppAD::azmul(vNormIsNotZero, sin(0.5 * vNorm) / vNorm) +
        //                CppAD::azmul(vNormIsZero, ad_scalar_t{0.5}));
        // q.w()   = CppAD::azmul(vNormIsNotZero, cos(0.5 * vNorm)) +
        //         CppAD::azmul(vNormIsZero, ad_scalar_t{1.0});
    }
#endif
    else {
        static_assert(dependent_false<ScalarType>,
                      "The 'ExponentialMap' function is not implemented for "
                      "vectors with the given "
                      "scalar type.");
    }
    return q;
}

template <typename _Vector>
inline typename _Vector::Scalar ApproximateNorm(const Eigen::MatrixBase<_Vector>& v) {
    using ScalarType = typename _Vector::Scalar;

    return Eigen::numext::sqrt(v.squaredNorm() + Eigen::NumTraits<ScalarType>::epsilon());
}

template <typename _Vector>
inline Quaternion<typename _Vector::Scalar> ApproximateExponentialMap(
    const Eigen::MatrixBase<_Vector>& v) {
    static_assert(_Vector::RowsAtCompileTime == 3);
    using ScalarType = typename _Vector::Scalar;
    Quaternion<ScalarType> q;

    const ScalarType vApproximateNorm = ApproximateNorm(v);
    q.vec()                           = v * sin(0.5 * vApproximateNorm) / vApproximateNorm;
    q.w()                             = cos(0.5 * vApproximateNorm);
    return q;
}

template <Concepts::Scalar _Scalar>
inline Quaternion<_Scalar> UnitQuaternionInverse(const Quaternion<_Scalar>& q) {
    return q.conjugate();
}

template <Concepts::Scalar _Scalar>
inline Quaternion<_Scalar> UnitQuaternionInverse(const Eigen::Map<const Quaternion<_Scalar>>& q) {
    return q.conjugate();
}

template <Concepts::Scalar _Scalar>
inline Quaternion<_Scalar> UnitQuaternionInverse(const Eigen::Map<Quaternion<_Scalar>>& q) {
    return q.conjugate();
}

/**
 * @brief Compute the inverse of a 3x3 invertible matrix.
 */
template <typename _Matrix>  // clang-format off
requires (_Matrix::RowsAtCompileTime == 3 || _Matrix::RowsAtCompileTime == Eigen::Dynamic) &&
         (_Matrix::ColsAtCompileTime == 3 || _Matrix::ColsAtCompileTime == Eigen::Dynamic)
inline Matrix3< typename _Matrix::Scalar> Inverse3(const Eigen::MatrixBase<_Matrix>& m) {  // clang-format on
    using ScalarType = typename _Matrix::Scalar;
    if constexpr (_Matrix::RowsAtCompileTime == Eigen::Dynamic ||
                  _Matrix::ColsAtCompileTime == Eigen::Dynamic) {
        ADCG_WRAPPER_ASSERT(m.rows() == 3_idx && m.cols() == 3_idx);
    }

    const ScalarType determinant = m.determinant();
    Matrix3<ScalarType> mInverse;
    mInverse << (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)), -(m(0, 1) * m(2, 2) - m(0, 2) * m(2, 1)),
        (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)), -(m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)),
        (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)), -(m(0, 0) * m(1, 2) - m(0, 2) * m(1, 0)),
        (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0)), -(m(0, 0) * m(2, 1) - m(0, 1) * m(2, 0)),
        (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0));
    mInverse /= determinant;
    return mInverse;
}

/**
 * @brief Compute the inverse of a 6x6 upper triangular block matrix.
 *
 * The input matrix has the form:
 *      [A B]
 *      [0 C],
 * where A and C are 3x3 invertible matrices.
 */
template <typename _Matrix>  // clang-format off
requires (_Matrix::RowsAtCompileTime == 6 || _Matrix::RowsAtCompileTime == Eigen::Dynamic) &&
         (_Matrix::ColsAtCompileTime == 6 || _Matrix::ColsAtCompileTime == Eigen::Dynamic)
inline Eigen::Matrix<typename _Matrix::Scalar, 6, 6> Inverse6(const Eigen::MatrixBase<_Matrix>& m) {  // clang-format on
    using ScalarType = typename _Matrix::Scalar;
    if constexpr (_Matrix::RowsAtCompileTime == Eigen::Dynamic ||
                  _Matrix::ColsAtCompileTime == Eigen::Dynamic) {
        ADCG_WRAPPER_ASSERT(m.rows() == 6_idx && m.cols() == 6_idx);
    }
    if constexpr (std::same_as<ScalarType, real_t>) {
        ADCG_WRAPPER_ASSERT((m.template bottomLeftCorner<3, 3>().isZero()));
    }

    const Matrix3<ScalarType> aInverse = Inverse3(m.derived().template topLeftCorner<3, 3>());
    const Matrix3<ScalarType> cInverse = Inverse3(m.derived().template bottomRightCorner<3, 3>());

    Eigen::Matrix<ScalarType, 6, 6> mInverse;
    mInverse << aInverse, -aInverse * m.template topRightCorner<3, 3>() * cInverse,
        Matrix3<ScalarType>::Zero(), cInverse;
    return mInverse;
}

template <Concepts::Scalar _Base, typename _Exponent>
inline auto Pow(const _Base base, const _Exponent exponent) {
    if constexpr (std::convertible_to<_Base, real_t>) {
        return std::pow(base, exponent);
    }
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
    else if constexpr (std::convertible_to<_Base, ad_scalar_t>) {
        if constexpr (std::integral<_Exponent>) {
            return CppAD::pow(base, static_cast<int>(exponent));
        } else {
            return CppAD::pow(base, exponent);
        }
    }
#endif
    else {
        Unreachable();
    }
}

template <Concepts::Scalar _Scalar>
inline auto Sqrt(const _Scalar a) {
    if constexpr (std::convertible_to<_Scalar, real_t>) {
        return std::sqrt(a);
    }
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
    else if constexpr (std::convertible_to<_Scalar, ad_scalar_t>) {
        return CppAD::sqrt(a);
    }
#endif
    else {
        Unreachable();
    }
}

inline constexpr std::string_view DASH_LINE_SEPARATOR =
    "----------------------------------------------------------------"sv;
inline constexpr std::string_view STAR_LINE_SEPARATOR =
    "****************************************************************"sv;

template <typename _Matrix>
inline bool HasNaN(const Eigen::MatrixBase<_Matrix>& matrix) {
    return matrix.hasNaN();
}

template <typename _SparseMatrix>
inline bool HasNaN(const Eigen::SparseMatrixBase<_SparseMatrix>& sparseMatrix) {
    for (int k = 0; k < sparseMatrix.outerSize(); ++k) {
        for (typename _SparseMatrix::InnerIterator it{sparseMatrix.derived(), k}; it; ++it) {
            if (!(it.value() == it.value())) {
                return true;
            }
        }
    }
    return false;
}

inline auto ElementaryXRotationMatrix(const Concepts::Scalar auto angle) {
    using ScalarType   = std::remove_cvref_t<decltype(angle)>;
    const ScalarType c = cos(angle);
    const ScalarType s = sin(angle);
    // clang-format off
    return Matrix3<ScalarType>{{ScalarType{1.0},  ScalarType{0.0},  ScalarType{0.0}},
                               {ScalarType{0.0},  c,                -s},
                               {ScalarType{0.0},  s,                c}};  // clang-format on
}

inline auto ElementaryYRotationMatrix(const Concepts::Scalar auto angle) {
    using ScalarType   = std::remove_cvref_t<decltype(angle)>;
    const ScalarType c = cos(angle);
    const ScalarType s = sin(angle);
    // clang-format off
    return Matrix3<ScalarType>{{c,                ScalarType{0.0},  s},
                               {ScalarType{0.0},  ScalarType{1.0},  ScalarType{0.0}},
                               {-s,               ScalarType{0.0},  c}};  // clang-format on
}

inline auto ElementaryZRotationMatrix(const Concepts::Scalar auto angle) {
    using ScalarType   = std::remove_cvref_t<decltype(angle)>;
    const ScalarType c = cos(angle);
    const ScalarType s = sin(angle);
    // clang-format off
    return Matrix3<ScalarType>{{c,                -s,               ScalarType{0.0}},
                               {s,                c,                ScalarType{0.0}},
                               {ScalarType{0.0},  ScalarType{0.0},  ScalarType{1.0}}};  // clang-format on
}

inline auto ElementaryXQuaternion(const Concepts::Scalar auto angle) {
    using ScalarType   = std::remove_cvref_t<decltype(angle)>;
    const ScalarType c = cos(angle / ScalarType{2.0});
    const ScalarType s = sin(angle / ScalarType{2.0});
    return Quaternion<ScalarType>{c, s, ScalarType{0.0}, ScalarType{0.0}};
}

inline auto ElementaryYQuaternion(const Concepts::Scalar auto angle) {
    using ScalarType   = std::remove_cvref_t<decltype(angle)>;
    const ScalarType c = cos(angle / ScalarType{2.0});
    const ScalarType s = sin(angle / ScalarType{2.0});
    return Quaternion<ScalarType>{c, ScalarType{0.0}, s, ScalarType{0.0}};
}

inline auto ElementaryZQuaternion(const Concepts::Scalar auto angle) {
    using ScalarType   = std::remove_cvref_t<decltype(angle)>;
    const ScalarType c = cos(angle / ScalarType{2.0});
    const ScalarType s = sin(angle / ScalarType{2.0});
    return Quaternion<ScalarType>{c, ScalarType{0.0}, ScalarType{0.0}, s};
}

/// @brief A yaw-pitch-roll angles vector 'ypr' is defined so that
///        the yaw, pitch and roll angles correspond to 'ypr.z()',
///        'ypr.y()' and 'ypr.x()', respectively.
template <typename _Vector>  // clang-format off
requires (_Vector::RowsAtCompileTime == 3)
inline auto RotationMatrixFromYawPitchRoll(const Eigen::MatrixBase<_Vector>& ypr) {  // clang-format on
    return Matrix3<typename _Vector::Scalar>{ElementaryZRotationMatrix(ypr.z()) *
                                             ElementaryYRotationMatrix(ypr.y()) *
                                             ElementaryXRotationMatrix(ypr.x())};
}

/// @brief A yaw-pitch-roll angles vector 'ypr' is defined so that
///        the yaw, pitch and roll angles correspond to 'ypr.z()',
///        'ypr.y()' and 'ypr.x()', respectively.
template <typename _Vector>  // clang-format off
requires (_Vector::RowsAtCompileTime == 3)
inline auto QuaternionFromYawPitchRoll(const Eigen::MatrixBase<_Vector>& ypr) {  // clang-format on
    return Quaternion<typename _Vector::Scalar>{ElementaryZQuaternion(ypr.z()) *
                                                ElementaryYQuaternion(ypr.y()) *
                                                ElementaryXQuaternion(ypr.x())};
}

template <typename _Quaternion>
inline auto QuaternionToYawPitchRoll(const Eigen::QuaternionBase<_Quaternion>& q) {
    return Vector3<typename _Quaternion::Scalar>{
        q.toRotationMatrix().eulerAngles(2_idx, 1_idx, 0_idx).reverse()};
}

#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
template <typename _Quaternion>  // clang-format off
requires std::convertible_to<typename _Quaternion::Scalar, ad_scalar_t>
inline Vector3<typename _Quaternion::Scalar> QuaternionToYawPitchRoll(
    const Eigen::QuaternionBase<_Quaternion>& q) {  // clang-format on
    const Matrix3<typename _Quaternion::Scalar> R = q.toRotationMatrix();
    return {
        CppAD::atan2(R(2_idx, 1_idx), R(2_idx, 2_idx)),
        CppAD::atan2(-R(2_idx, 0_idx),
                     CppAD::sqrt(Utils::Pow(R(2_idx, 1_idx), 2) + Utils::Pow(R(2_idx, 2_idx), 2))),
        CppAD::atan2(R(1_idx, 0_idx), R(0_idx, 0_idx))};
}
#endif

template <Concepts::Scalar _Scalar>
inline _Scalar Min(const _Scalar& a, const std::type_identity_t<_Scalar>& b) {
    if constexpr (std::convertible_to<_Scalar, real_t>) {
        return std::min(a, b);
    }
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
    else if constexpr (std::convertible_to<_Scalar, ad_scalar_t>) {
        return CppAD::CondExpGt(a, b, b, a);
    }
#endif
    else {
        Unreachable();
    }
}

template <Concepts::Scalar _Scalar>
inline _Scalar SmoothMin(const _Scalar& a,
                         const std::type_identity_t<_Scalar>& b,
                         const std::type_identity_t<_Scalar>& alpha = _Scalar{8.0}) {
    return (a * exp(-alpha * a) + b * exp(-alpha * b)) / (exp(-alpha * a) + exp(-alpha * b));
}

template <Concepts::Scalar _Scalar>
inline _Scalar Sign(const _Scalar& a) {
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
    return CppAD::CondExpGt(a, _Scalar{0.0}, _Scalar{1.0}, _Scalar{0.0}) -
           CppAD::CondExpLt(a, _Scalar{0.0}, _Scalar{1.0}, _Scalar{0.0});
#else
    return static_cast<real_t>(a > 0.0) - static_cast<real_t>(a < 0.0);
#endif
}

template <Concepts::Scalar _Scalar>
inline _Scalar Abs(const _Scalar& a) {
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
    if constexpr (std::convertible_to<_Scalar, ad_scalar_t>) {
        return CppAD::abs(a);
    } else
#else
    if (std::convertible_to<_Scalar, real_t>) {
        return std::abs(a);
    } else
#endif
    {
        Unreachable();
    }
}

template <Concepts::Scalar _Scalar>
inline _Scalar SmoothAbs(const _Scalar& a,
                         const _Scalar& epsilon = Eigen::NumTraits<_Scalar>::epsilon()) {
    return Utils::Sqrt(Utils::Pow(a, 2) + epsilon);
}

template <typename _Matrix>  // clang-format off
requires (_Matrix::RowsAtCompileTime == 1 || _Matrix::RowsAtCompileTime == Eigen::Dynamic) &&
         (_Matrix::ColsAtCompileTime == 1 || _Matrix::ColsAtCompileTime == Eigen::Dynamic)
inline auto Squeeze(const Eigen::MatrixBase<_Matrix>& m) {  // clang-format on
    if constexpr (_Matrix::RowsAtCompileTime == Eigen::Dynamic) {
        ADCG_WRAPPER_ASSERT(m.rows() == 1_idx);
    }
    if constexpr (_Matrix::ColsAtCompileTime == Eigen::Dynamic) {
        ADCG_WRAPPER_ASSERT(m.cols() == 1_idx);
    }

    return m.x();
}

#ifdef ADCG_WRAPPER_CONFIG_ENABLE_AUTODIFF
namespace Internal {

template <typename _AutodiffFunction>
struct RealFunctionHelper {
    RealFunctionHelper(const _AutodiffFunction& autodiffFunction_)
        : autodiffFunction{autodiffFunction_} {
    }

    VectorXr operator()(auto&&... realVectors) const {
        return autodiffFunction(
                   std::forward<decltype(realVectors)>(realVectors).template cast<ad_scalar_t>()...)
            .unaryExpr([](const auto& el) { return CppAD::Value(el).getValue(); });
    }

    _AutodiffFunction autodiffFunction;
};

}  // namespace Internal

inline auto ToRealFunction(const auto& autodiffFunction) {
    return Internal::RealFunctionHelper{autodiffFunction};
}
#endif

template <bool _LOWER_TRIANGULAR = false, typename _Matrix>  // clang-format off
requires (Concepts::DenseMatrixExpression<_Matrix> || Concepts::SparseMatrixExpression<_Matrix>) && (!_Matrix::IsRowMajor)
[[nodiscard]] bool CompareMatrices(const _Matrix& testMatrix,
                                   std::string_view testEntriesLabel,
                                   const MatrixXr& groundTruthMatrix,
                                   std::string_view groundTruthEntriesLabel) {  // clang-format on
    bool success = true;

    const real_t relativeTolerance = 1e-2;
    const real_t absoluteTolerance = 1e-3;
    const real_t epsilon           = 1e-12;

    int mismatchCounter     = 0;
    const int maxMismatches = 20;
    for (index_t j = 0_idx; j < groundTruthMatrix.cols(); ++j) {
        for (index_t i = 0_idx; i < groundTruthMatrix.rows(); ++i) {
            if constexpr (_LOWER_TRIANGULAR) {
                if (j > i) {
                    continue;
                }
            }

            const real_t absoluteError = std::abs(groundTruthMatrix(i, j) - testMatrix.coeff(i, j));
            const real_t relativeError =
                2.0 * absoluteError /
                (epsilon + std::abs(groundTruthMatrix(i, j)) + std::abs(testMatrix.coeff(i, j)));

            if ((relativeError > relativeTolerance && absoluteError > absoluteTolerance) ||
                std::isnan(testMatrix.coeff(i, j)) != std::isnan(groundTruthMatrix(i, j))) {
                if (success) {
                    ADCG_WRAPPER_LOG(debug, "Some mismatches were found.");
                    ADCG_WRAPPER_LOG(debug, "The following mismatches were found.");
                    ADCG_WRAPPER_LOG(debug,
                                     "{:<10}{:<10}{:<20}{:<20}{:<20}",
                                     "Row",
                                     "Column",
                                     testEntriesLabel,
                                     groundTruthEntriesLabel,
                                     "Error");
                    success = false;
                }

                ADCG_WRAPPER_LOG(debug,
                                 "{:<10}{:<10}{:<20.4f}{:<20.4f}{:<10.4f} ({:.4}%)",
                                 i,
                                 j,
                                 testMatrix.coeff(i, j),
                                 groundTruthMatrix(i, j),
                                 absoluteError,
                                 relativeError * 100.0);
                ++mismatchCounter;

                if (mismatchCounter > maxMismatches) {
                    const auto etc = "..."sv;
                    ADCG_WRAPPER_LOG(
                        debug, "{:<10}{:<10}{:<20}{:<20}{:<10}  {}", etc, etc, etc, etc, etc, etc);

                    ADCG_WRAPPER_LOG(debug, "More than {} mismatches were found.", maxMismatches);
                    return success;
                }
            }
        }
    }

    if (success) {
        ADCG_WRAPPER_LOG(debug, "No mismatches were found.");
    }

    return success;
}

}  // namespace Utils
}  // namespace ADCGWrapper

#endif /* _ADCG_WRAPPER__UTILS__UTILS_HPP_ */
