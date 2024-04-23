/******************************************************************************
 *
 * @file adcg_wrapper/data_types.hpp
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

#ifndef _ADCG_WRAPPER__DATA_TYPES_HPP_
#define _ADCG_WRAPPER__DATA_TYPES_HPP_

#include <ranges>
#include <type_traits>

#include "Eigen/Geometry"
#include "Eigen/Sparse"

#ifndef ADCG_WRAPPER_CODEGEN_FOLDER
#define ADCG_WRAPPER_CODEGEN_FOLDER \
    (std::filesystem::temp_directory_path() / "adcg_wrapper_codegen").c_str()
#endif

namespace ADCGWrapper {

using namespace std::literals;

using real_t = double;

using index_t     = Eigen::Index;
using time_step_t = index_t;

template <typename _T, template <typename...> typename _TemplateType>
struct is_specialization_of : std::false_type {};
template <template <typename...> typename _TemplateType, typename... _Args>
struct is_specialization_of<_TemplateType<_Args...>, _TemplateType> : std::true_type {};
template <typename _T, template <typename...> typename _TemplateType>
constexpr bool is_specialization_of_v = is_specialization_of<_T, _TemplateType>::value;

template <class _T>
struct is_std_array : std::false_type {};
template <class _T, std::size_t _N>
struct is_std_array<std::array<_T, _N>> : std::true_type {};
template <class _T>
constexpr bool is_std_array_v = is_std_array<_T>::value;

template <class _T>
struct std_array_size;
template <class _T, std::size_t _N>
struct std_array_size<std::array<_T, _N>> : std::integral_constant<size_t, _N> {};
template <class _T>
constexpr bool std_array_size_v = std_array_size<_T>::value;

template <class _T>
struct std_array_empty;
template <class _T, std::size_t _N>
struct std_array_empty<std::array<_T, _N>> : std::integral_constant<bool, !_N> {};
template <class _T>
constexpr bool std_array_empty_v = std_array_empty<_T>::value;

template <class _T>
struct is_std_optional : std::false_type {};
template <class _T>
struct is_std_optional<std::optional<_T>> : std::true_type {};
template <class _T>
constexpr bool is_std_optional_v = is_std_optional<_T>::value;

namespace Concepts {

template <typename _T, typename... _U>
concept AnyOf = (std::same_as<_T, _U> || ...);

template <typename _T, typename... _U>
concept Same = (std::same_as<_T, _U> && ...);

template <typename _STDArray>
concept STDArray = is_std_array_v<_STDArray>;

template <typename _STDOptional>
concept STDOptional = is_std_optional_v<_STDOptional>;

// clang-format off
template <typename _ForwardRangeOf, typename _T>
concept ForwardRangeOf = std::ranges::forward_range<_ForwardRangeOf> &&
                         std::same_as<std::ranges::range_value_t<_ForwardRangeOf>, _T>;

template <typename _RandomAccessRangeOf, typename _T>
concept RandomAccessRangeOf = std::ranges::random_access_range<_RandomAccessRangeOf> &&
                              std::same_as<std::ranges::range_value_t<_RandomAccessRangeOf>, _T>;

template <typename _ContiguousRangeOf, typename _T>
concept ContiguousRangeOf =
    std::ranges::contiguous_range<_ContiguousRangeOf> &&
    std::same_as<std::ranges::range_value_t<_ContiguousRangeOf>, _T>;
// clang-format on

template <typename _T, typename _U>
concept Constructs = std::constructible_from<_U, _T>;

template <typename _Matrix>
concept DenseMatrixExpression = std::derived_from<std::remove_cvref_t<_Matrix>,
                                                  Eigen::MatrixBase<std::remove_cvref_t<_Matrix>>>;

template <typename _Vector>
concept DenseVectorExpression =
    DenseMatrixExpression<_Vector> && (std::remove_cvref_t<_Vector>::ColsAtCompileTime == 1);

template <typename _Matrix>
concept SparseMatrixExpression =
    std::derived_from<std::remove_cvref_t<_Matrix>,
                      Eigen::SparseMatrixBase<std::remove_cvref_t<_Matrix>>>;

template <typename _T, template <typename...> typename _TemplateType>
concept SpecializationOf = is_specialization_of_v<_T, _TemplateType>;

}  // namespace Concepts

#define _ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS_IMPL(Type, TypeSuffix, SIZE, SizeSuffix)              \
    using Matrix##SizeSuffix##TypeSuffix    = Eigen::Matrix<Type, SIZE, SIZE>;                  \
    using Vector##SizeSuffix##TypeSuffix    = Eigen::Matrix<Type, SIZE, 1>;                     \
    using RowVector##SizeSuffix##TypeSuffix = Eigen::Matrix<Type, 1, SIZE>;                     \
                                                                                                \
    using RefToMatrix##SizeSuffix##TypeSuffix    = Eigen::Ref<Eigen::Matrix<Type, SIZE, SIZE>>; \
    using RefToVector##SizeSuffix##TypeSuffix    = Eigen::Ref<Eigen::Matrix<Type, SIZE, 1>>;    \
    using RefToRowVector##SizeSuffix##TypeSuffix = Eigen::Ref<Eigen::Matrix<Type, 1, SIZE>>;    \
    using RefToConstMatrix##SizeSuffix##TypeSuffix =                                            \
        Eigen::Ref<const Eigen::Matrix<Type, SIZE, SIZE>>;                                      \
    using RefToConstVector##SizeSuffix##TypeSuffix =                                            \
        Eigen::Ref<const Eigen::Matrix<Type, SIZE, 1>>;                                         \
    using RefToConstRowVector##SizeSuffix##TypeSuffix =                                         \
        Eigen::Ref<const Eigen::Matrix<Type, 1, SIZE>>;                                         \
                                                                                                \
    using MapToMatrix##SizeSuffix##TypeSuffix    = Eigen::Map<Eigen::Matrix<Type, SIZE, SIZE>>; \
    using MapToVector##SizeSuffix##TypeSuffix    = Eigen::Map<Eigen::Matrix<Type, SIZE, 1>>;    \
    using MapToRowVector##SizeSuffix##TypeSuffix = Eigen::Map<Eigen::Matrix<Type, 1, SIZE>>;    \
    using MapToConstMatrix##SizeSuffix##TypeSuffix =                                            \
        Eigen::Map<const Eigen::Matrix<Type, SIZE, SIZE>>;                                      \
    using MapToConstVector##SizeSuffix##TypeSuffix =                                            \
        Eigen::Map<const Eigen::Matrix<Type, SIZE, 1>>;                                         \
    using MapToConstRowVector##SizeSuffix##TypeSuffix =                                         \
        Eigen::Map<const Eigen::Matrix<Type, 1, SIZE>>
#define ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(Type, TypeSuffix)          \
    _ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS_IMPL(Type, TypeSuffix, 2, 2); \
    _ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS_IMPL(Type, TypeSuffix, 3, 3); \
    _ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS_IMPL(Type, TypeSuffix, 4, 4); \
    _ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS_IMPL(Type, TypeSuffix, Eigen::Dynamic, X)

ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(real_t, r);
#undef ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS
#undef _ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS_IMPL

#define ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(SIZE, SizeSuffix)                                     \
    template <typename _Scalar>                                                                \
    using Matrix##SizeSuffix = Eigen::Matrix<_Scalar, SIZE, SIZE>;                             \
    template <typename _Scalar>                                                                \
    using Vector##SizeSuffix = Eigen::Matrix<_Scalar, SIZE, 1>;                                \
    template <typename _Scalar>                                                                \
    using RowVector##SizeSuffix = Eigen::Matrix<_Scalar, 1, SIZE>;                             \
                                                                                               \
    template <typename _Scalar>                                                                \
    using RefToMatrix##SizeSuffix = Eigen::Ref<Eigen::Matrix<_Scalar, SIZE, SIZE>>;            \
    template <typename _Scalar>                                                                \
    using RefToVector##SizeSuffix = Eigen::Ref<Eigen::Matrix<_Scalar, SIZE, 1>>;               \
    template <typename _Scalar>                                                                \
    using RefToRowVector##SizeSuffix = Eigen::Ref<Eigen::Matrix<_Scalar, 1, SIZE>>;            \
    template <typename _Scalar>                                                                \
    using RefToConstMatrix##SizeSuffix = Eigen::Ref<const Eigen::Matrix<_Scalar, SIZE, SIZE>>; \
    template <typename _Scalar>                                                                \
    using RefToConstVector##SizeSuffix = Eigen::Ref<const Eigen::Matrix<_Scalar, SIZE, 1>>;    \
    template <typename _Scalar>                                                                \
    using RefToConstRowVector##SizeSuffix = Eigen::Ref<const Eigen::Matrix<_Scalar, 1, SIZE>>; \
                                                                                               \
    template <typename _Scalar>                                                                \
    using MapToMatrix##SizeSuffix = Eigen::Map<Eigen::Matrix<_Scalar, SIZE, SIZE>>;            \
    template <typename _Scalar>                                                                \
    using MapToVector##SizeSuffix = Eigen::Map<Eigen::Matrix<_Scalar, SIZE, 1>>;               \
    template <typename _Scalar>                                                                \
    using MapToRowVector##SizeSuffix = Eigen::Map<Eigen::Matrix<_Scalar, 1, SIZE>>;            \
    template <typename _Scalar>                                                                \
    using MapToConstMatrix##SizeSuffix = Eigen::Map<const Eigen::Matrix<_Scalar, SIZE, SIZE>>; \
    template <typename _Scalar>                                                                \
    using MapToConstVector##SizeSuffix = Eigen::Map<const Eigen::Matrix<_Scalar, SIZE, 1>>;    \
    template <typename _Scalar>                                                                \
    using MapToConstRowVector##SizeSuffix = Eigen::Map<const Eigen::Matrix<_Scalar, 1, SIZE>>

ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(2, 2);
ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(3, 3);
ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(4, 4);
ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(Eigen::Dynamic, X);
#undef ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS

#define ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(SIZE)                            \
    template <typename _Scalar>                                           \
    using Matrix##SIZE##X = Eigen::Matrix<_Scalar, SIZE, Eigen::Dynamic>; \
    template <typename _Scalar>                                           \
    using Matrix##X##SIZE = Eigen::Matrix<_Scalar, Eigen::Dynamic, SIZE>;

ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(2);
ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(3);
ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS(4);
#undef ADCG_WRAPPER_MAKE_EIGEN_TYPEDEFS

template <typename _Scalar, int SIZE>
using Vector = Eigen::Matrix<_Scalar, SIZE, 1>;
template <typename _Scalar, int SIZE>
using RowVector = Eigen::Matrix<_Scalar, 1, SIZE>;
template <typename _Scalar, int SIZE>
using RefToVector = Eigen::Ref<Eigen::Matrix<_Scalar, SIZE, 1>>;
template <typename _Scalar, int SIZE>
using RefToRowVector = Eigen::Ref<Eigen::Matrix<_Scalar, 1, SIZE>>;
template <typename _Scalar, int SIZE>
using RefToConstVector = Eigen::Ref<const Eigen::Matrix<_Scalar, SIZE, 1>>;
template <typename _Scalar, int SIZE>
using RefToConstRowVector = Eigen::Ref<const Eigen::Matrix<_Scalar, 1, SIZE>>;
template <typename _Scalar>
using SparseMatrix = Eigen::SparseMatrix<_Scalar>;

template <typename _Scalar>
using Quaternion  = Eigen::Quaternion<_Scalar>;
using Quaternionr = Eigen::Quaternion<real_t>;
template <typename _Scalar>
using MapToQuaternion  = Eigen::Map<Eigen::Quaternion<_Scalar>>;
using MapToQuaternionr = Eigen::Map<Eigen::Quaternion<real_t>>;
template <typename _Scalar>
using MapToConstQuaternion  = Eigen::Map<const Eigen::Quaternion<_Scalar>>;
using MapToConstQuaternionr = Eigen::Map<const Eigen::Quaternion<real_t>>;

template <typename _Scalar>
using AngleAxis  = Eigen::AngleAxis<_Scalar>;
using AngleAxisr = Eigen::AngleAxis<real_t>;

template <typename _Scalar>
using Rotation2D  = Eigen::Rotation2D<_Scalar>;
using Rotation2Dr = Eigen::Rotation2D<real_t>;

inline namespace Literals {

constexpr index_t operator"" _idx(const unsigned long long index) {
    return static_cast<index_t>(index);
}

constexpr time_step_t operator"" _step(const unsigned long long timeStep) {
    return static_cast<time_step_t>(timeStep);
}

}  // namespace Literals

template <typename _Scalar,
          typename _StorageIndex = typename Eigen::SparseMatrix<_Scalar>::StorageIndex>
class MutableTriplet final : public Eigen::Triplet<_Scalar, _StorageIndex> {
  public:
    using Eigen::Triplet<_Scalar, _StorageIndex>::Triplet;

    _StorageIndex& row() {
        return m_row;
    }

    _StorageIndex& col() {
        return m_col;
    }

    _Scalar& value() {
        return m_value;
    }

  private:
    using BaseType = Eigen::Triplet<_Scalar, _StorageIndex>;
    using BaseType::m_col;
    using BaseType::m_row;
    using BaseType::m_value;
};

template <bool VALUE, typename... _Args>
inline constexpr bool dependent_bool_value = VALUE;

template <typename... _Args>
inline constexpr bool dependent_false = dependent_bool_value<false, _Args...>;

template <typename _T, typename... _Args>
using dependent_type = _T;

template <std::integral _Integral>
constexpr auto enumerate(const _Integral n) {
    return std::views::iota(static_cast<_Integral>(0), n);
}

template <typename _T>
constexpr auto cast_to = std::views::transform(
    [](auto&& el) -> decltype(auto) { return static_cast<_T>(std::forward<decltype(el)>(el)); });

template <typename _T, bool _B>
using add_const_if = std::conditional<_B, std::add_const_t<_T>, _T>;
template <typename _T, bool _B>
using add_const_if_t = std::conditional_t<_B, std::add_const_t<_T>, _T>;

template <typename _T>
using is_const_ref =
    std::conjunction<std::is_reference<_T>, std::is_const<std::remove_reference_t<_T>>>;
template <typename _T>
constexpr bool is_const_ref_v = is_const_ref<_T>::value;

namespace Internal {

template <typename _EigenWrapper>
struct EigenWrapperTraits;

template <template <typename, int, typename> typename _EigenWrapper,
          typename _EigenWrappedType,
          int _OPTIONS,
          typename _StrideType>
struct EigenWrapperTraits<_EigenWrapper<_EigenWrappedType, _OPTIONS, _StrideType>> {
    using WrappedType = _EigenWrappedType;
};

template <template <typename, int> typename _EigenWrapper, typename _EigenWrappedType, int SIZE>
struct EigenWrapperTraits<_EigenWrapper<_EigenWrappedType, SIZE>> {
    using WrappedType = _EigenWrappedType;
};

template <typename _EigenWrapper>
using EigenWrappedType = typename EigenWrapperTraits<_EigenWrapper>::WrappedType;

}  // namespace Internal

namespace Concepts {

template <typename _EigenConstTypeWrapper>
concept EigenConstTypeWrapper = std::is_const_v<Internal::EigenWrappedType<_EigenConstTypeWrapper>>;

template <typename _EigenMutableTypeWrapper>
concept EigenMutableTypeWrapper =
    !std::is_const_v<Internal::EigenWrappedType<_EigenMutableTypeWrapper>>;

}  // namespace Concepts

template <size_t _N>
struct fixed_string;

template <class _T>
struct is_fixed_string : std::false_type {};
template <size_t _N>
struct is_fixed_string<fixed_string<_N>> : std::true_type {};
template <class _T>
constexpr bool is_fixed_string_v = is_fixed_string<_T>::value;

template <typename _T>
struct to_fixed_string_impl;

template <typename _T>
inline constexpr auto to_fixed_string = to_fixed_string_impl<_T>{};

namespace Concepts {

template <typename _FixedString>
concept FixedString = is_fixed_string_v<_FixedString>;

template <typename _T>
concept ConvertibleToFixedString = requires() {
    { to_fixed_string_impl<_T>{}() } -> FixedString;
};

}  // namespace Concepts

template <size_t _N>
struct fixed_string {
    template <Concepts::ConvertibleToFixedString _T>
    constexpr fixed_string(const _T& other) : fixed_string{to_fixed_string<_T>()} {
    }

    constexpr fixed_string(const char (&str)[_N]) {
        std::copy_n(str, _N, data);
    }

    constexpr char& operator[](const size_t i) {
        return data[i];
    }

    constexpr const char& operator[](const size_t i) const {
        return data[i];
    }

    constexpr size_t size() const {
        return _N;
    }

    char data[_N];
};

template <Concepts::ConvertibleToFixedString _T>
fixed_string(const _T& other) -> fixed_string<to_fixed_string<_T>().size()>;

template <class _T>
constexpr _T& as_mutable(const _T& t) noexcept {
    return const_cast<_T&>(t);
}

}  // namespace ADCGWrapper

#endif /* _ADCG_WRAPPER__DATA_TYPES_HPP_ */
