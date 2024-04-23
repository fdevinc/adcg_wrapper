/******************************************************************************
 *
 * @file adcg_wrapper/example/autodiff/function.example.cpp
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

#include "adcg_wrapper/autodiff/function.hpp"

int main() {
    using namespace ADCGWrapper;

    /***************   Generate derivatives for functions of variable maps. */
    const index_t xSize = 24_idx;
    const index_t pSize = 12_idx;
    const Autodiff::Function::Blueprint functionBlueprint{
        [&](const VectorXad& xp, VectorXad& y) {
            y = VectorXr::Zero(1);
            y[0] += xp.squaredNorm();
        },
        xSize,
        pSize,
        "function_example"sv,
        EnabledDerivatives::JACOBIAN | EnabledDerivatives::HESSIAN};
    const auto function = Autodiff::MakeFunction(functionBlueprint, true);

    VectorXr xp                         = VectorXr::Random(xSize + pSize);
    const VectorXr value                = function(xp);
    const SparseMatrix<real_t> jacobian = function.Jacobian(xp);
    const SparseMatrix<real_t> hessian  = function.Hessian(xp);

    ADCG_WRAPPER_ASSERT(value.size() == function.DependentVariableSize() && value.size() == 1);
    ADCG_WRAPPER_ASSERT(function.TestJacobian(xp));
    ADCG_WRAPPER_ASSERT(function.TestHessian(xp));

    return 0;
}
