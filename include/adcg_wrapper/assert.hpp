/******************************************************************************
 *
 * @file adcg_wrapper/assert.hpp
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

#ifndef _ADCG_WRAPPER__ASSERT_HPP_
#define _ADCG_WRAPPER__ASSERT_HPP_

#include <cstdio>
#include <cstdlib>
#include <source_location>

#ifdef ADCG_WRAPPER_CONFIG_ENABLE_LOGGING
#include "adcg_wrapper/io/logging.hpp"
#endif

namespace ADCGWrapper {

inline constexpr long expect(long exp, long c) {
    if (exp == c) [[likely]] {
        return c;
    } else {
        return exp;
    }
}

inline void assertion_failed(
    char const* expr, const std::source_location location = std::source_location::current()) {
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_LOGGING
    ADCG_WRAPPER_LOG(error,
                     "{}:{}: {}: Assertion `{}` failed.",
                     location.file_name(),
                     location.line(),
                     location.function_name(),
                     expr);
#else
    std::fprintf(stderr,
                 "%s:%u: %s: Assertion `%s` failed.",
                 location.file_name(),
                 location.line(),
                 location.function_name(),
                 expr);
#endif
    std::abort();
}

inline void assertion_failed_msg(
    char const* expr,
    char const* msg,
    const std::source_location location = std::source_location::current()) {
#ifdef ADCG_WRAPPER_CONFIG_ENABLE_LOGGING
    ADCG_WRAPPER_LOG(error,
                     "{}:{}: {}: Assertion `{} && \"{}\"` failed.",
                     location.file_name(),
                     location.line(),
                     location.function_name(),
                     expr,
                     msg);
#else
    std::fprintf(stderr,
                 "%s:%u: %s: Assertion `%s && \"%s\"` failed.",
                 location.file_name(),
                 location.line(),
                 location.function_name(),
                 expr,
                 msg);
#endif
    std::abort();
}

}  // namespace ADCGWrapper

#define ADCG_WRAPPER_LIKELY(x) ::ADCGWrapper::expect(x, 1)
#define ADCG_WRAPPER_UNLIKELY(x) ::ADCGWrapper::expect(x, 0)

#ifdef ADCG_WRAPPER_CONFIG_ENABLE_RELEASE_MODE
#define ADCG_WRAPPER_ASSERT(expr) ((void)0)
#define ADCG_WRAPPER_ASSERT_MSG(expr, msg) ((void)0)
#define ADCG_WRAPPER_ASSERT_EXPLICIT(expr) ((void)0)
#define ADCG_WRAPPER_ASSERT_EXPLICIT_MSG(expr, msg) ((void)0)
#else
#define ADCG_WRAPPER_ASSERT(expr) \
    (ADCG_WRAPPER_LIKELY(!!(expr)) ? ((void)0) : ::ADCGWrapper::assertion_failed(#expr))
#define ADCG_WRAPPER_ASSERT_MSG(expr, msg) \
    (ADCG_WRAPPER_LIKELY(!!(expr)) ? ((void)0) : ::ADCGWrapper::assertion_failed_msg(#expr, msg))
#define ADCG_WRAPPER_ASSERT_EXPLICIT(expr) ADCG_WRAPPER_ASSERT(static_cast<bool>(expr))
#define ADCG_WRAPPER_ASSERT_EXPLICIT_MSG(expr, msg) \
    ADCG_WRAPPER_ASSERT_MSG(static_cast<bool>(expr), msg)
#endif

#endif /* _ADCG_WRAPPER__ASSERT_HPP_ */
