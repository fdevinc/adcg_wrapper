/******************************************************************************
 *
 * @file adcg_wrapper/io/logging.hpp
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

#ifndef _ADCG_WRAPPER__IO__LOGGING_HPP_
#define _ADCG_WRAPPER__IO__LOGGING_HPP_

#ifndef ADCG_WRAPPER_CONFIG_ENABLE_RELEASE_MODE
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace ADCGWrapper {
namespace Internal {

class Logger {
  public:
    static constexpr const char* NAME = "adcg_wrapper";

  private:
    Logger()                         = delete;
    Logger(const Logger&)            = delete;
    Logger(Logger&&)                 = delete;
    Logger& operator=(const Logger&) = delete;
    Logger& operator=(Logger&&)      = delete;

    static auto InitializeLogger() {
        static bool initialized = false;
        if (initialized) {
            throw std::logic_error("Can initialize ADCGWrapper's logger only once.");
        }

        auto logger = spdlog::stdout_color_mt(NAME);
#ifndef ADCG_WRAPPER_CONFIG_ENABLE_RELEASE_MODE
        logger->set_level(spdlog::level::trace);
#endif
        logger->set_pattern("[%H:%M:%S.%e] [%n] [%^%l%$] %v");

        initialized = true;
        return logger;
    }

  public:
    static auto& Get() {
        static auto logger = InitializeLogger();
        return logger;
    }
};

}  // namespace Internal
}  // namespace ADCGWrapper

#define _ADCG_WRAPPER_LOG_IMPL_trace SPDLOG_LOGGER_TRACE
#define _ADCG_WRAPPER_LOG_IMPL_debug SPDLOG_LOGGER_DEBUG
#define _ADCG_WRAPPER_LOG_IMPL_info SPDLOG_LOGGER_INFO
#define _ADCG_WRAPPER_LOG_IMPL_warn SPDLOG_LOGGER_WARN
#define _ADCG_WRAPPER_LOG_IMPL_error SPDLOG_LOGGER_ERROR
#define _ADCG_WRAPPER_LOG_IMPL_critical SPDLOG_LOGGER_CRITICAL
#define ADCG_WRAPPER_LOG(level, ...) \
    _ADCG_WRAPPER_LOG_IMPL_##level(::ADCGWrapper::Internal::Logger::Get(), __VA_ARGS__)

#endif /* _ADCG_WRAPPER__IO__LOGGING_HPP_ */
