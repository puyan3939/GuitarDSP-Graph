#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace guitardsp::circuit {

// Fixed-pattern sparse Gaussian/LU-style solver for prepared MNA systems.
//
// Symbolic work happens in prepare():
// - find a structural row permutation that gives every column a diagonal entry
// - compute the exact no-pivot elimination fill pattern
// - compile CSR rows and a dense row/column -> sparse-slot lookup table
//
// Numeric solve() performs no allocation. It copies only the prepared structural
// entries from the dense correctness matrix into CSR storage, factors that fixed
// pattern in double precision, and solves the permuted-row system. Because the
// numeric phase deliberately avoids dynamic pivoting, callers can validate the
// accepted Newton result against the original dense equations using a scaled
// backward-error residual. An unsafe pivot or residual falls back to the dense
// partial-pivot reference solver.
// Columns are never permuted, so the output vector remains in original MNA order.
class FixedPatternSparseSolver {
public:
    bool prepare(std::size_t dimension,
                 const std::vector<std::uint8_t>& structuralPattern,
                 const std::vector<float>& numericHint) {
        clear();
        dimension_ = dimension;
        if (dimension_ == 0U || structuralPattern.size() != dimension_ * dimension_ ||
            numericHint.size() != dimension_ * dimension_)
            return false;

        constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();
        std::vector<std::size_t> rowMatch(dimension_, npos);
        std::vector<std::uint8_t> visited(dimension_, 0U);

        const auto augment = [&](auto&& self, std::size_t column) -> bool {
            for (std::size_t attempt = 0; attempt < dimension_; ++attempt) {
                std::size_t bestRow = npos;
                float bestScore = -1.0f;
                for (std::size_t row = 0; row < dimension_; ++row) {
                    if (visited[row] != 0U ||
                        structuralPattern[row * dimension_ + column] == 0U)
                        continue;
                    const float hint = std::abs(numericHint[row * dimension_ + column]);
                    const float diagonalBonus = row == column ? 1.0e-12f : 0.0f;
                    const float score = hint + diagonalBonus;
                    if (bestRow == npos || score > bestScore) {
                        bestRow = row;
                        bestScore = score;
                    }
                }
                if (bestRow == npos) break;
                visited[bestRow] = 1U;
                if (rowMatch[bestRow] == npos || self(self, rowMatch[bestRow])) {
                    rowMatch[bestRow] = column;
                    return true;
                }
            }
            return false;
        };

        for (std::size_t column = 0; column < dimension_; ++column) {
            std::fill(visited.begin(), visited.end(), 0U);
            if (!augment(augment, column)) {
                clear();
                return false;
            }
        }

        rowPermutation_.assign(dimension_, npos);
        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto column = rowMatch[row];
            if (column == npos || column >= dimension_) {
                clear();
                return false;
            }
            rowPermutation_[column] = row;
        }
        for (const auto row : rowPermutation_) {
            if (row == npos) {
                clear();
                return false;
            }
        }

        std::vector<std::uint8_t> fill(dimension_ * dimension_, 0U);
        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto originalRow = rowPermutation_[row];
            for (std::size_t column = 0; column < dimension_; ++column)
                fill[row * dimension_ + column] =
                    structuralPattern[originalRow * dimension_ + column];
        }

        for (std::size_t k = 0; k < dimension_; ++k) {
            if (fill[k * dimension_ + k] == 0U) {
                clear();
                return false;
            }
            for (std::size_t row = k + 1U; row < dimension_; ++row) {
                if (fill[row * dimension_ + k] == 0U) continue;
                for (std::size_t column = k + 1U; column < dimension_; ++column) {
                    if (fill[k * dimension_ + column] != 0U)
                        fill[row * dimension_ + column] = 1U;
                }
            }
        }

        originalNonZeros_ = 0U;
        factorNonZeros_ = 0U;
        for (const auto entry : structuralPattern)
            originalNonZeros_ += entry != 0U ? 1U : 0U;
        for (const auto entry : fill)
            factorNonZeros_ += entry != 0U ? 1U : 0U;

        rowOffsets_.assign(dimension_ + 1U, 0U);
        columns_.clear();
        columns_.reserve(factorNonZeros_);
        slotLookup_.assign(dimension_ * dimension_, npos);
        for (std::size_t row = 0; row < dimension_; ++row) {
            rowOffsets_[row] = columns_.size();
            for (std::size_t column = 0; column < dimension_; ++column) {
                if (fill[row * dimension_ + column] == 0U) continue;
                slotLookup_[row * dimension_ + column] = columns_.size();
                columns_.push_back(column);
            }
        }
        rowOffsets_[dimension_] = columns_.size();

        // Keep the original sparsity separately from the LU fill pattern. Both
        // numeric loading and residual verification only need actual A entries;
        // scanning dimension^2 zeros on every Newton iteration dominated pedal
        // processing despite the prepared factor being structurally sparse.
        originalRowOffsets_.assign(dimension_ + 1U, 0U);
        originalColumns_.clear();
        originalSlots_.clear();
        originalDenseIndices_.clear();
        originalColumns_.reserve(originalNonZeros_);
        originalSlots_.reserve(originalNonZeros_);
        originalDenseIndices_.reserve(originalNonZeros_);
        for (std::size_t row = 0; row < dimension_; ++row) {
            originalRowOffsets_[row] = originalColumns_.size();
            const auto originalRow = rowPermutation_[row];
            for (std::size_t column = 0; column < dimension_; ++column) {
                if (structuralPattern[originalRow * dimension_ + column] == 0U)
                    continue;
                originalColumns_.push_back(column);
                originalSlots_.push_back(slotLookup_[row * dimension_ + column]);
                originalDenseIndices_.push_back(originalRow * dimension_ + column);
            }
        }
        originalRowOffsets_[dimension_] = originalColumns_.size();

        // Symbolic elimination also knows exactly which rows contain each lower
        // column. Avoid another dense row scan during every numeric factorization.
        eliminationOffsets_.assign(dimension_ + 1U, 0U);
        eliminationRows_.clear();
        eliminationLowerSlots_.clear();
        eliminationTargetOffsets_.clear();
        eliminationTargets_.clear();
        diagonalSlots_.assign(dimension_, npos);
        for (std::size_t column = 0; column < dimension_; ++column) {
            eliminationOffsets_[column] = eliminationRows_.size();
            const auto diagonalSlot = slotLookup_[column * dimension_ + column];
            if (diagonalSlot == npos) {
                clear();
                return false;
            }
            diagonalSlots_[column] = diagonalSlot;
            for (std::size_t row = column + 1U; row < dimension_; ++row) {
                const auto lowerSlot = slotLookup_[row * dimension_ + column];
                if (lowerSlot == npos) continue;
                eliminationRows_.push_back(row);
                eliminationLowerSlots_.push_back(lowerSlot);
                eliminationTargetOffsets_.push_back(eliminationTargets_.size());
                for (std::size_t index = diagonalSlot + 1U;
                     index < rowOffsets_[column + 1U]; ++index) {
                    const auto target = slotLookup_[row * dimension_ + columns_[index]];
                    if (target == npos) {
                        clear();
                        return false;
                    }
                    eliminationTargets_.push_back(target);
                }
            }
        }
        eliminationOffsets_[dimension_] = eliminationRows_.size();

        values_.assign(columns_.size(), 0.0f);
        workRhs_.assign(dimension_, 0.0f);
        solutionWork_.assign(dimension_, 0.0);
        available_ = true;
        return true;
    }

    bool solve(const std::vector<float>& denseMatrix,
               const std::vector<float>& rhs,
               std::vector<float>& solution,
               bool verifyResidual = true) noexcept {
        if (!available_ || denseMatrix.size() != dimension_ * dimension_ ||
            rhs.size() != dimension_ || solution.size() != dimension_)
            return false;

        constexpr double pivotFloor = 1.0e-14;
        std::fill(values_.begin(), values_.end(), 0.0);

        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto originalRow = rowPermutation_[row];
            workRhs_[row] = rhs[originalRow];
            for (std::size_t index = originalRowOffsets_[row];
                 index < originalRowOffsets_[row + 1U]; ++index) {
                values_[originalSlots_[index]] = denseMatrix[originalDenseIndices_[index]];
            }
        }

        for (std::size_t k = 0; k < dimension_; ++k) {
            const auto diagonalSlot = diagonalSlots_[k];
            const double diagonal = values_[diagonalSlot];
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return false;

            for (std::size_t entryIndex = eliminationOffsets_[k];
                 entryIndex < eliminationOffsets_[k + 1U]; ++entryIndex) {
                const auto row = eliminationRows_[entryIndex];
                const auto lowerSlot = eliminationLowerSlots_[entryIndex];
                const double entry = values_[lowerSlot];
                if (entry == 0.0) continue;
                const double factor = entry / diagonal;
                if (!std::isfinite(factor)) return false;
                values_[lowerSlot] = factor;

                auto targetOffset = eliminationTargetOffsets_[entryIndex];
                for (std::size_t index = diagonalSlot + 1U;
                     index < rowOffsets_[k + 1U]; ++index) {
                    values_[eliminationTargets_[targetOffset++]] -= factor * values_[index];
                }
                workRhs_[row] -= factor * workRhs_[k];
            }
        }

        for (std::size_t row = dimension_; row-- > 0U;) {
            double sum = workRhs_[row];
            const auto diagonalSlot = diagonalSlots_[row];
            const double diagonal = values_[diagonalSlot];
            for (std::size_t index = diagonalSlot + 1U;
                 index < rowOffsets_[row + 1U]; ++index) {
                const auto column = columns_[index];
                sum -= values_[index] * solutionWork_[column];
            }
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return false;
            solutionWork_[row] = sum / diagonal;
            if (!std::isfinite(solutionWork_[row])) return false;
        }

        for (std::size_t i = 0; i < dimension_; ++i)
            solution[i] = static_cast<float>(solutionWork_[i]);

        return !verifyResidual || validate(denseMatrix, rhs, solution);
    }

    bool validate(const std::vector<float>& denseMatrix,
                  const std::vector<float>& rhs,
                  const std::vector<float>& solution) const noexcept {
        if (!available_ || denseMatrix.size() != dimension_ * dimension_ ||
            rhs.size() != dimension_ || solution.size() != dimension_)
            return false;

        // A fixed ordering can encounter poor numeric pivots even when every pivot
        // remains formally non-zero. Validate the candidate against A*x=b before
        // exposing it to Newton. The row-scaled backward error is dimensionless and
        // works for both millivolt pedals and hundreds-of-volts tube circuits.
        constexpr double residualFloor = 1.0e-18;
        constexpr double maximumBackwardError = 2.0e-4;
        double worstBackwardError = 0.0;
        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto originalRow = rowPermutation_[row];
            double residual = -static_cast<double>(rhs[originalRow]);
            double scale = std::abs(static_cast<double>(rhs[originalRow]));
            for (std::size_t index = originalRowOffsets_[row];
                 index < originalRowOffsets_[row + 1U]; ++index) {
                const auto column = originalColumns_[index];
                const double a = static_cast<double>(denseMatrix[originalDenseIndices_[index]]);
                const double x = static_cast<double>(solution[column]);
                residual += a * x;
                scale += std::abs(a) * std::abs(x);
            }
            const double backwardError = std::abs(residual) / std::max(residualFloor, scale);
            if (!std::isfinite(backwardError)) return false;
            worstBackwardError = std::max(worstBackwardError, backwardError);
        }
        return worstBackwardError <= maximumBackwardError;
    }

    bool available() const noexcept { return available_; }
    std::size_t dimension() const noexcept { return dimension_; }
    std::size_t originalNonZeros() const noexcept { return originalNonZeros_; }
    std::size_t factorNonZeros() const noexcept { return factorNonZeros_; }
    float factorDensity() const noexcept {
        if (dimension_ == 0U) return 1.0f;
        const auto full = static_cast<double>(dimension_) * static_cast<double>(dimension_);
        return static_cast<float>(static_cast<double>(factorNonZeros_) / full);
    }

private:
    void clear() {
        available_ = false;
        dimension_ = 0U;
        originalNonZeros_ = 0U;
        factorNonZeros_ = 0U;
        rowPermutation_.clear();
        rowOffsets_.clear();
        columns_.clear();
        slotLookup_.clear();
        originalRowOffsets_.clear();
        originalColumns_.clear();
        originalSlots_.clear();
        originalDenseIndices_.clear();
        eliminationOffsets_.clear();
        eliminationRows_.clear();
        eliminationLowerSlots_.clear();
        eliminationTargetOffsets_.clear();
        eliminationTargets_.clear();
        diagonalSlots_.clear();
        values_.clear();
        workRhs_.clear();
        solutionWork_.clear();
    }

    bool available_ = false;
    std::size_t dimension_ = 0U;
    std::size_t originalNonZeros_ = 0U;
    std::size_t factorNonZeros_ = 0U;
    std::vector<std::size_t> rowPermutation_;
    std::vector<std::size_t> rowOffsets_;
    std::vector<std::size_t> columns_;
    std::vector<std::size_t> slotLookup_;
    std::vector<std::size_t> originalRowOffsets_;
    std::vector<std::size_t> originalColumns_;
    std::vector<std::size_t> originalSlots_;
    std::vector<std::size_t> originalDenseIndices_;
    std::vector<std::size_t> eliminationOffsets_;
    std::vector<std::size_t> eliminationRows_;
    std::vector<std::size_t> eliminationLowerSlots_;
    std::vector<std::size_t> eliminationTargetOffsets_;
    std::vector<std::size_t> eliminationTargets_;
    std::vector<std::size_t> diagonalSlots_;
    std::vector<double> values_;
    std::vector<double> workRhs_;
    std::vector<double> solutionWork_;
};

} // namespace guitardsp::circuit
