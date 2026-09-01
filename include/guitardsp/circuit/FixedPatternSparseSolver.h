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
// - order invariant linear equations ahead of the nonlinear boundary
// - choose minimum-fill pivots inside each of those matched partitions
// - compute the exact no-pivot elimination fill pattern
// - compile CSR rows and a dense row/column -> sparse-slot lookup table
// - cache the exact elimination of the invariant linear prefix
//
// Numeric solve() performs no allocation. It restores only the mutable Schur
// boundary, adds the current nonlinear device stamps, factors the remaining
// suffix in double precision, and reconstructs all original circuit unknowns.
// Because numeric elimination deliberately avoids dynamic pivoting, callers can
// validate the result against the original dense equations using a scaled
// backward-error residual. An unsafe pivot or residual falls back to the dense
// partial-pivot reference solver.
// Internal rows and columns may be permuted for sparsity; solved unknowns are
// scattered back into the caller's original MNA/component order.
class FixedPatternSparseSolver {
public:
    bool prepare(std::size_t dimension,
                 const std::vector<std::uint8_t>& structuralPattern,
                 const std::vector<float>& numericHint,
                 const std::vector<std::uint8_t>& nonlinearPattern = {}) {
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

        // The user-visible MNA numbering follows schematic construction rather
        // than a useful elimination order. Keep all device-touched rows/columns
        // at the end, then use Markowitz pivots inside the linear and nonlinear
        // partitions. The exact linear prefix can consequently be eliminated
        // once and reused by every Newton iteration; no component or unknown is
        // removed from the original equations.
        std::vector<std::uint8_t> orderingFill(dimension_ * dimension_, 0U);
        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto originalRow = rowPermutation_[row];
            for (std::size_t column = 0; column < dimension_; ++column)
                orderingFill[row * dimension_ + column] =
                    structuralPattern[originalRow * dimension_ + column];
        }

        const bool partitionNonlinear = nonlinearPattern.size() == dimension_ * dimension_;
        std::vector<std::uint8_t> nonlinearRows(dimension_, 0U);
        std::vector<std::uint8_t> nonlinearColumns(dimension_, 0U);
        if (partitionNonlinear) {
            for (std::size_t row = 0; row < dimension_; ++row) {
                for (std::size_t column = 0; column < dimension_; ++column) {
                    if (nonlinearPattern[row * dimension_ + column] == 0U) continue;
                    nonlinearRows[row] = 1U;
                    nonlinearColumns[column] = 1U;
                }
            }
        }

        std::vector<std::uint8_t> eliminated(dimension_, 0U);
        columnPermutation_.clear();
        columnPermutation_.reserve(dimension_);
        for (std::size_t step = 0; step < dimension_; ++step) {
            std::size_t best = npos;
            std::size_t bestScore = npos;
            bool bestNonlinear = true;
            for (std::size_t candidate = 0; candidate < dimension_; ++candidate) {
                if (eliminated[candidate] != 0U) continue;
                std::size_t rowDegree = 0U;
                std::size_t columnDegree = 0U;
                for (std::size_t other = 0; other < dimension_; ++other) {
                    if (other == candidate || eliminated[other] != 0U) continue;
                    rowDegree += orderingFill[candidate * dimension_ + other] != 0U ? 1U : 0U;
                    columnDegree += orderingFill[other * dimension_ + candidate] != 0U ? 1U : 0U;
                }
                const auto score = rowDegree * columnDegree;
                const bool nonlinear = partitionNonlinear &&
                    (nonlinearRows[rowPermutation_[candidate]] != 0U ||
                     nonlinearColumns[candidate] != 0U);
                if (best == npos || (bestNonlinear && !nonlinear)
                    || (bestNonlinear == nonlinear && score < bestScore)) {
                    best = candidate;
                    bestScore = score;
                    bestNonlinear = nonlinear;
                }
            }
            if (best == npos) {
                clear();
                return false;
            }
            columnPermutation_.push_back(best);

            for (std::size_t row = 0; row < dimension_; ++row) {
                if (row == best || eliminated[row] != 0U ||
                    orderingFill[row * dimension_ + best] == 0U)
                    continue;
                for (std::size_t column = 0; column < dimension_; ++column) {
                    if (column == best || eliminated[column] != 0U ||
                        orderingFill[best * dimension_ + column] == 0U)
                        continue;
                    orderingFill[row * dimension_ + column] = 1U;
                }
            }
            eliminated[best] = 1U;
        }

        const auto matchedRows = rowPermutation_;
        for (std::size_t row = 0; row < dimension_; ++row)
            rowPermutation_[row] = matchedRows[columnPermutation_[row]];

        linearPrefix_ = 0U;
        if (partitionNonlinear) {
            while (linearPrefix_ < dimension_
                   && nonlinearRows[rowPermutation_[linearPrefix_]] == 0U
                   && nonlinearColumns[columnPermutation_[linearPrefix_]] == 0U)
                ++linearPrefix_;
        }

        std::vector<std::uint8_t> fill(dimension_ * dimension_, 0U);
        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto originalRow = rowPermutation_[row];
            for (std::size_t column = 0; column < dimension_; ++column)
                fill[row * dimension_ + column] =
                    structuralPattern[originalRow * dimension_ + columnPermutation_[column]];
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
        nonlinearSlots_.clear();
        nonlinearDenseIndices_.clear();
        nonlinearSlots_.reserve(originalNonZeros_);
        nonlinearDenseIndices_.reserve(originalNonZeros_);
        for (std::size_t row = 0; row < dimension_; ++row) {
            originalRowOffsets_[row] = originalColumns_.size();
            const auto originalRow = rowPermutation_[row];
            for (std::size_t column = 0; column < dimension_; ++column) {
                const auto originalColumn = columnPermutation_[column];
                if (structuralPattern[originalRow * dimension_ + originalColumn] == 0U)
                    continue;
                originalColumns_.push_back(originalColumn);
                originalSlots_.push_back(slotLookup_[row * dimension_ + column]);
                originalDenseIndices_.push_back(originalRow * dimension_ + originalColumn);
                if (partitionNonlinear
                    && nonlinearPattern[originalRow * dimension_ + originalColumn] != 0U) {
                    nonlinearSlots_.push_back(slotLookup_[row * dimension_ + column]);
                    nonlinearDenseIndices_.push_back(originalRow * dimension_ + originalColumn);
                }
            }
        }
        originalRowOffsets_[dimension_] = originalColumns_.size();

        validationRows_.clear();
        validationRows_.reserve(dimension_);
        nonlinearRows_.clear();
        nonlinearRows_.reserve(dimension_);
        if (partitionNonlinear) {
            for (std::size_t row = 0; row < dimension_; ++row) {
                if (nonlinearRows[rowPermutation_[row]] != 0U) {
                    validationRows_.push_back(row);
                    nonlinearRows_.push_back(row);
                }
            }
        }
        for (std::size_t row = 0; row < dimension_; ++row) {
            if (!partitionNonlinear || nonlinearRows[rowPermutation_[row]] == 0U)
                validationRows_.push_back(row);
        }

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

        mutableFactorSlots_.clear();
        mutableFactorSlots_.reserve(factorNonZeros_);
        for (std::size_t row = linearPrefix_; row < dimension_; ++row) {
            for (std::size_t slot = rowOffsets_[row];
                 slot < rowOffsets_[row + 1U]; ++slot) {
                if (columns_[slot] >= linearPrefix_)
                    mutableFactorSlots_.push_back(slot);
            }
        }

        values_.assign(columns_.size(), 0.0f);
        cachedLinearValues_.assign(columns_.size(), 0.0);
        nonlinearStaticValues_.assign(nonlinearDenseIndices_.size(), 0.0f);
        workRhs_.assign(dimension_, 0.0f);
        cachedSampleRhs_.assign(dimension_, 0.0);
        sampleBaselineRhs_.assign(dimension_, 0.0f);
        solutionWork_.assign(dimension_, 0.0);
        available_ = true;
        refreshStaticFactor(numericHint);
        return true;
    }

    void refreshStaticFactor(const std::vector<float>& staticMatrix) noexcept {
        cachedLinearFactorValid_ = false;
        cachedSampleRhsValid_ = false;
        if (!available_ || linearPrefix_ == 0U
            || staticMatrix.size() != dimension_ * dimension_)
            return;

        std::fill(cachedLinearValues_.begin(), cachedLinearValues_.end(), 0.0);
        for (std::size_t index = 0; index < originalDenseIndices_.size(); ++index)
            cachedLinearValues_[originalSlots_[index]] =
                static_cast<double>(staticMatrix[originalDenseIndices_[index]]);
        for (std::size_t index = 0; index < nonlinearDenseIndices_.size(); ++index)
            nonlinearStaticValues_[index] = staticMatrix[nonlinearDenseIndices_[index]];

        // This is ordinary sparse Gaussian elimination, stopped exactly where
        // nonlinear rows or unknowns begin. Its suffix is the static Schur
        // complement, not a reduced or approximate replacement circuit.
        constexpr double pivotFloor = 1.0e-14;
        for (std::size_t column = 0; column < linearPrefix_; ++column) {
            const auto diagonalSlot = diagonalSlots_[column];
            const double diagonal = cachedLinearValues_[diagonalSlot];
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return;

            for (std::size_t entry = eliminationOffsets_[column];
                 entry < eliminationOffsets_[column + 1U]; ++entry) {
                const auto lowerSlot = eliminationLowerSlots_[entry];
                const double value = cachedLinearValues_[lowerSlot];
                if (value == 0.0) continue;
                const double factor = value / diagonal;
                if (!std::isfinite(factor)) return;
                cachedLinearValues_[lowerSlot] = factor;

                auto target = eliminationTargetOffsets_[entry];
                for (std::size_t index = diagonalSlot + 1U;
                     index < rowOffsets_[column + 1U]; ++index)
                    cachedLinearValues_[eliminationTargets_[target++]] -=
                        factor * cachedLinearValues_[index];
            }
        }
        std::copy(cachedLinearValues_.begin(), cachedLinearValues_.end(), values_.begin());
        cachedLinearFactorValid_ = true;
    }

    void prepareSampleRhs(const std::vector<float>& rhs) noexcept {
        cachedSampleRhsValid_ = false;
        if (!cachedLinearFactorValid_ || rhs.size() != dimension_) return;

        std::copy(rhs.begin(), rhs.end(), sampleBaselineRhs_.begin());
        for (std::size_t row = 0; row < dimension_; ++row)
            cachedSampleRhs_[row] = rhs[rowPermutation_[row]];

        // Capacitor/source history changes per audio sample, but it does not
        // change across that sample's Newton iterations. Apply cached linear
        // elimination once and add only nonlinear equivalent sources later.
        for (std::size_t column = 0; column < linearPrefix_; ++column) {
            for (std::size_t entry = eliminationOffsets_[column];
                 entry < eliminationOffsets_[column + 1U]; ++entry)
                cachedSampleRhs_[eliminationRows_[entry]] -=
                    cachedLinearValues_[eliminationLowerSlots_[entry]]
                    * cachedSampleRhs_[column];
        }

        // Suffix factorization never changes an already-eliminated linear row.
        // Install that immutable prefix once per audio sample instead of copying
        // it again for every Newton iteration of the same component circuit.
        std::copy(cachedSampleRhs_.begin(),
                  cachedSampleRhs_.begin() + static_cast<std::ptrdiff_t>(linearPrefix_),
                  workRhs_.begin());
        cachedSampleRhsValid_ = true;
    }

    bool solve(const std::vector<float>& denseMatrix,
               const std::vector<float>& rhs,
               std::vector<float>& solution,
               bool verifyResidual = true) noexcept {
        if (!available_ || denseMatrix.size() != dimension_ * dimension_ ||
            rhs.size() != dimension_ || solution.size() != dimension_)
            return false;

        constexpr double pivotFloor = 1.0e-14;
        if (cachedLinearFactorValid_) {
            for (const auto slot : mutableFactorSlots_)
                values_[slot] = cachedLinearValues_[slot];
            for (std::size_t index = 0; index < nonlinearDenseIndices_.size(); ++index) {
                values_[nonlinearSlots_[index]] +=
                    static_cast<double>(denseMatrix[nonlinearDenseIndices_[index]])
                    - static_cast<double>(nonlinearStaticValues_[index]);
            }
        } else {
            std::fill(values_.begin(), values_.end(), 0.0);
            for (std::size_t index = 0; index < originalDenseIndices_.size(); ++index)
                values_[originalSlots_[index]] = denseMatrix[originalDenseIndices_[index]];
        }

        const std::size_t cachedPrefix = cachedLinearFactorValid_ ? linearPrefix_ : 0U;
        if (cachedSampleRhsValid_) {
            const auto suffix = static_cast<std::ptrdiff_t>(cachedPrefix);
            std::copy(cachedSampleRhs_.begin() + suffix,
                      cachedSampleRhs_.end(), workRhs_.begin() + suffix);
            for (const auto row : nonlinearRows_) {
                const auto originalRow = rowPermutation_[row];
                workRhs_[row] += static_cast<double>(rhs[originalRow])
                    - static_cast<double>(sampleBaselineRhs_[originalRow]);
            }
        } else {
            for (std::size_t row = 0; row < dimension_; ++row)
                workRhs_[row] = rhs[rowPermutation_[row]];

            for (std::size_t column = 0; column < cachedPrefix; ++column) {
                for (std::size_t entry = eliminationOffsets_[column];
                     entry < eliminationOffsets_[column + 1U]; ++entry)
                    workRhs_[eliminationRows_[entry]] -=
                        values_[eliminationLowerSlots_[entry]] * workRhs_[column];
            }
        }

        for (std::size_t k = cachedPrefix; k < dimension_; ++k) {
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
            // Prefix pivots were verified when its exact factor was cached;
            // every mutable suffix pivot was checked above before elimination.
            // No subsequent operation can mutate an already-eliminated pivot.
            solutionWork_[row] = sum / diagonal;
            if (!std::isfinite(solutionWork_[row])) return false;
        }

        for (std::size_t i = 0; i < dimension_; ++i)
            solution[columnPermutation_[i]] = static_cast<float>(solutionWork_[i]);

        return !verifyResidual || validate(denseMatrix, rhs, solution);
    }

    bool validate(const std::vector<float>& denseMatrix,
                  const std::vector<float>& rhs,
                  const std::vector<float>& solution,
                  double maximumBackwardError = 2.0e-4) const noexcept {
        return validateRows(denseMatrix, rhs, solution, validationRows_,
                            maximumBackwardError);
    }

    // A complete sparse Newton solve already satisfies every invariant linear
    // equation. Only device-touched rows change when its candidate is restamped,
    // so checking those rows is the same nonlinear KCL convergence test without
    // re-evaluating the eliminated linear prefix.
    bool validateNonlinear(const std::vector<float>& denseMatrix,
                           const std::vector<float>& rhs,
                           const std::vector<float>& solution,
                           double maximumBackwardError = 2.0e-4) const noexcept {
        return validateRows(denseMatrix, rhs, solution, nonlinearRows_,
                            maximumBackwardError);
    }

    bool available() const noexcept { return available_; }
    std::size_t dimension() const noexcept { return dimension_; }
    std::size_t originalNonZeros() const noexcept { return originalNonZeros_; }
    std::size_t factorNonZeros() const noexcept { return factorNonZeros_; }
    std::size_t cachedLinearUnknowns() const noexcept {
        return cachedLinearFactorValid_ ? linearPrefix_ : 0U;
    }
    float factorDensity() const noexcept {
        if (dimension_ == 0U) return 1.0f;
        const auto full = static_cast<double>(dimension_) * static_cast<double>(dimension_);
        return static_cast<float>(static_cast<double>(factorNonZeros_) / full);
    }

private:
    bool validateRows(const std::vector<float>& denseMatrix,
                      const std::vector<float>& rhs,
                      const std::vector<float>& solution,
                      const std::vector<std::size_t>& rows,
                      double maximumBackwardError) const noexcept {
        if (!available_ || denseMatrix.size() != dimension_ * dimension_ ||
            rhs.size() != dimension_ || solution.size() != dimension_)
            return false;

        // A fixed ordering can encounter poor numeric pivots even when every pivot
        // remains formally non-zero. Validate the candidate against A*x=b before
        // exposing it to Newton. The row-scaled backward error is dimensionless and
        // works for both millivolt pedals and hundreds-of-volts tube circuits.
        constexpr double residualFloor = 1.0e-18;
        for (const auto row : rows) {
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
            if (!std::isfinite(backwardError)
                || backwardError > maximumBackwardError)
                return false;
        }
        return true;
    }
    void clear() {
        available_ = false;
        dimension_ = 0U;
        originalNonZeros_ = 0U;
        factorNonZeros_ = 0U;
        linearPrefix_ = 0U;
        cachedLinearFactorValid_ = false;
        cachedSampleRhsValid_ = false;
        rowPermutation_.clear();
        columnPermutation_.clear();
        rowOffsets_.clear();
        columns_.clear();
        slotLookup_.clear();
        originalRowOffsets_.clear();
        originalColumns_.clear();
        originalSlots_.clear();
        originalDenseIndices_.clear();
        validationRows_.clear();
        nonlinearRows_.clear();
        nonlinearSlots_.clear();
        nonlinearDenseIndices_.clear();
        eliminationOffsets_.clear();
        eliminationRows_.clear();
        eliminationLowerSlots_.clear();
        eliminationTargetOffsets_.clear();
        eliminationTargets_.clear();
        mutableFactorSlots_.clear();
        diagonalSlots_.clear();
        values_.clear();
        cachedLinearValues_.clear();
        nonlinearStaticValues_.clear();
        workRhs_.clear();
        cachedSampleRhs_.clear();
        sampleBaselineRhs_.clear();
        solutionWork_.clear();
    }

    bool available_ = false;
    std::size_t dimension_ = 0U;
    std::size_t originalNonZeros_ = 0U;
    std::size_t factorNonZeros_ = 0U;
    std::size_t linearPrefix_ = 0U;
    bool cachedLinearFactorValid_ = false;
    bool cachedSampleRhsValid_ = false;
    std::vector<std::size_t> rowPermutation_;
    std::vector<std::size_t> columnPermutation_;
    std::vector<std::size_t> rowOffsets_;
    std::vector<std::size_t> columns_;
    std::vector<std::size_t> slotLookup_;
    std::vector<std::size_t> originalRowOffsets_;
    std::vector<std::size_t> originalColumns_;
    std::vector<std::size_t> originalSlots_;
    std::vector<std::size_t> originalDenseIndices_;
    std::vector<std::size_t> validationRows_;
    std::vector<std::size_t> nonlinearRows_;
    std::vector<std::size_t> nonlinearSlots_;
    std::vector<std::size_t> nonlinearDenseIndices_;
    std::vector<std::size_t> eliminationOffsets_;
    std::vector<std::size_t> eliminationRows_;
    std::vector<std::size_t> eliminationLowerSlots_;
    std::vector<std::size_t> eliminationTargetOffsets_;
    std::vector<std::size_t> eliminationTargets_;
    std::vector<std::size_t> mutableFactorSlots_;
    std::vector<std::size_t> diagonalSlots_;
    std::vector<double> values_;
    std::vector<double> cachedLinearValues_;
    std::vector<float> nonlinearStaticValues_;
    std::vector<double> workRhs_;
    std::vector<double> cachedSampleRhs_;
    std::vector<float> sampleBaselineRhs_;
    std::vector<double> solutionWork_;
};

} // namespace guitardsp::circuit
