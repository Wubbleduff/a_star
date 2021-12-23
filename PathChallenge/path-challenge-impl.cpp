/*******************************************************************
*
*    Author: Kareem Omar
*    kareem.h.omar@gmail.com
*    https://github.com/komrad36
*
*    Last updated Dec 15, 2021
*******************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <random>
#include <unordered_set>
#include <vector>

using U32 = uint32_t;
using U64 = uint64_t;

using namespace std::chrono;

/// configuration
static constexpr U32 kRows = 120;
static constexpr U32 kCols = 230;
static constexpr U32 kStartX = 14;
static constexpr U32 kStartY = 11;
static constexpr U32 kEndX = (kCols - 1) - 14;
static constexpr U32 kEndY = (kRows - 1) - 11;

/// derived configuration
static constexpr U32 kCells = kRows * kCols;
static constexpr U32 kStartI = kStartY * kCols + kStartX;
static constexpr U32 kEndI = kEndY * kCols + kEndX;

/// your code goes here!
float FastPathFind(const U64* pWalls)
{
    std::vector<U32> m_openSet;
    m_openSet.reserve(1000);
    std::unordered_set<U32> m_visited;
    double m_f[kCells];

    for (U32 i = 0; i < kCells; ++i)
        m_f[i] = 99999999999.0;

    m_f[kStartI] = 0.0;
    m_openSet.emplace_back(kStartI);

    do
    {
        std::vector<U32>::iterator it = std::min_element(m_openSet.begin(), m_openSet.end(),
            [&](const U32 a, const U32 b) { return m_f[a] < m_f[b]; });
        const U32 eI = *it;
        const double f = m_f[eI];
        m_visited.emplace(eI);
        m_openSet.erase(it);

        if (eI == kEndI)
            return float(f);

        const U32 eX = eI % kCols;
        const U32 eY = eI / kCols;

        for (const std::pair<U32, U32>& p : std::initializer_list<std::pair<U32, U32>>{
            { eX - 1, eY }, { eX + 1, eY }, { eX, eY - 1 }, { eX, eY + 1 },
            { eX - 1, eY - 1}, { eX - 1, eY + 1 }, { eX + 1, eY - 1 }, { eX + 1, eY + 1 } })
        {
            const U32 x = p.first;
            const U32 y = p.second;
            const U32 i = y * kCols + x;
            if (x < kCols && y < kRows && m_visited.find(i) == m_visited.end() &&
                !(pWalls[i >> 6] & (1ULL << (i & 63ULL))))
            {
                if (std::find(m_openSet.begin(), m_openSet.end(), i) == m_openSet.end())
                    m_openSet.emplace_back(i);
                m_f[i] = std::fmin(m_f[i], f + ((x == eX || y == eY) ? 1.0 : 1.4142135623730951));
            }
        }
    } while (!m_openSet.empty());

    return -1.0f;
}
