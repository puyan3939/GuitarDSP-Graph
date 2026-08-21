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
// pattern, and solves the permuted-row system. If a numerical pivot becomes too
// small, solve() returns false so the caller can fall back to the dense partial-
// pivot reference solver. Columns are never permuted, so the output vector remains
// in the original MNA variable order.
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

        values_.assign(columns_.size(), 0.0f);
        workRhs_.assign(dimension_, 0.0f);
        available_ = true;
        return true;
    }

    bool solve(const std::vector<float>& denseMatrix,
               const std::vector<float>& rhs,
               std::vector<float>& solution) noexcept {
        if (!available_ || denseMatrix.size() != dimension_ * dimension_ ||
            rhs.size() != dimension_ || solution.size() != dimension_)
            return false;

        constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();
        constexpr float pivotFloor = 1.0e-14f;
        std::fill(values_.begin(), values_.end(), 0.0f);

        for (std::size_t row = 0; row < dimension_; ++row) {
            const auto originalRow = rowPermutation_[row];
            workRhs_[row] = rhs[originalRow];
            for (std::size_t column = 0; column < dimension_; ++column) {
                const float value = denseMatrix[originalRow * dimension_ + column];
                if (value == 0.0f) continue;
                const auto slot = slotLookup_[row * dimension_ + column];
                if (slot == npos) return false;
                values_[slot] = value;
            }
        }

        for (std::size_t k = 0; k < dimension_; ++k) {
            const auto diagonalSlot = slotLookup_[k * dimension_ + k];
            if (diagonalSlot == npos) return false;
            const float diagonal = values_[diagonalSlot];
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return false;

            for (std::size_t row = k + 1U; row < dimension_; ++row) {
                const auto lowerSlot = slotLookup_[row * dimension_ + k];
                if (lowerSlot == npos) continue;
                const float entry = values_[lowerSlot];
                if (entry == 0.0f) continue;
                const float factor = entry / diagonal;
                if (!std::isfinite(factor)) return false;
                values_[lowerSlot] = factor;

                for (std::size_t index = rowOffsets_[k]; index < rowOffsets_[k + 1U]; ++index) {
                    const auto column = columns_[index];
                    if (column <= k) continue;
                    const auto target = slotLookup_[row * dimension_ + column];
                    if (target == npos) return false;
                    values_[target] -= factor * values_[index];
                }
                workRhs_[row] -= factor * workRhs_[k];
            }
        }

        for (std::size_t row = dimension_; row-- > 0U;) {
            float sum = workRhs_[row];
            float diagonal = 0.0f;
            for (std::size_t index = rowOffsets_[row]; index < rowOffsets_[row + 1U]; ++index) {
                const auto column = columns_[index];
                if (column == row) diagonal = values_[index];
                else if (column > row) sum -= values_[index] * solution[column];
            }
            if (std::abs(diagonal) < pivotFloor || !std::isfinite(diagonal)) return false;
            solution[row] = sum / diagonal;
            if (!std::isfinite(solution[row])) return false;
        }
        return true;
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
        values_.clear();
        workRhs_.clear();
    }

    bool available_ = false;
    std::size_t dimension_ = 0U;
    std::size_t originalNonZeros_ = 0U;
    std::size_t factorNonZeros_ = 0U;
    std::vector<std::size_t> rowPermutation_;
    std::vector<std::size_t> rowOffsets_;
    std::vector<std::size_t> columns_;
    std::vector<std::size_t> slotLookup_;
    std::vector<float> values_;
    std::vector<float> workRhs_;
};

} // namespace guitardsp::circuit
