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

#if defined(__clang__) || defined(__GNUC__)
#define NEVER_INLINE __attribute__((noinline))
#else
#define NEVER_INLINE __declspec(noinline)
#endif

static int RandomNonZeroInt(std::mt19937& gen, int a, int b)
{
    const int x = std::uniform_int_distribution<int>(a, b - 1)(gen);
    return x ? x : b;
}

static void GenerateWalls(U64* pWalls)
{
    for (U32 i = 0; i < (kCells + 63) >> 6; ++i)
        pWalls[i] = 0;

    std::mt19937 gen(std::random_device{}());

    const U32 numWalls = std::uniform_int_distribution<U32>(130, 150)(gen);

    for (U32 iWall = 0; iWall < numWalls; ++iWall)
    {
        // pick start point
        U32 x = std::uniform_int_distribution<U32>(0, kCols - 1)(gen);
        U32 y = std::uniform_int_distribution<U32>(0, kRows - 1)(gen);

        // pick direction and prepare traversal
        int signedDx = RandomNonZeroInt(gen, -7, 7);
        int signedDy = RandomNonZeroInt(gen, -7, 7);
        int dx = std::abs(signedDx);
        int dy = -std::abs(signedDy);
        int sx = signedDx > 0 ? 1 : -1;
        int sy = signedDy < 0 ? 1 : -1;
        int err = dx + dy;
        dx <<= 1;
        dy <<= 1;
        U32 allowWall = 0;

        for (;;)
        {
            const U32 i = y * kCols + x;

            // break if hit edge, hit wall, or small termination probability
            if (x >= kCols || y >= kRows || (!allowWall && (pWalls[i >> 6] & (1ULL << (i & 63ULL))))
                || std::uniform_real_distribution<float>(0.0f, 1.0f)(gen) < 0.013f)
                break;

            pWalls[i >> 6] |= 1ULL << (i & 63ULL);

            if (err > 0)
            {
                err += dy;
                x += U32(sx);
            }
            else
            {
                err += dx;
                y += U32(sy);
            }

            // change direction if small probability
            if (std::uniform_real_distribution<float>(0.0f, 1.0f)(gen) < 0.12f)
            {
                if (std::uniform_real_distribution<float>(0.0f, 1.0f)(gen) < 0.5f)
                {
                    signedDx = RandomNonZeroInt(gen, -7, 7);
                    dx = std::abs(signedDx);
                }
                else
                {
                    signedDy = RandomNonZeroInt(gen, -7, 7);
                    dy = -std::abs(signedDy);
                }

                sx = signedDx > 0 ? 1 : -1;
                sy = signedDy < 0 ? 1 : -1;
                err = dx + dy;
                dx <<= 1;
                dy <<= 1;

                allowWall = 3;
            }
            if (allowWall)
                --allowWall;
        }
    }

    // clear start and end
    pWalls[kStartI >> 6] &= ~(1ULL << (kStartI & 63ULL));
    pWalls[kEndI >> 6] &= ~(1ULL << (kEndI & 63ULL));
}

static float NEVER_INLINE Ref(const U64* pWalls)
{
    std::vector<U32> m_openSet;
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

float FastPathFind(const U64* pWalls);

int main()
{
    U64 walls[(kCells + 63) >> 6] = { 0 };

    double refSum = 0.0;
    double fastSum = 0.0;
    for (U64 i = 1;; ++i)
    {
        float refPathLength;
        double refTime;
        {
            const auto start = high_resolution_clock::now();
            refPathLength = Ref(walls);
            const auto end = high_resolution_clock::now();
            refTime = static_cast<double>(duration_cast<nanoseconds>(end - start).count()) * 1e-6;
        }

        float fastPathLength;
        double fastTime;
        {
            const auto start = high_resolution_clock::now();
            fastPathLength = FastPathFind(walls);
            const auto end = high_resolution_clock::now();
            fastTime = static_cast<double>(duration_cast<nanoseconds>(end - start).count()) * 1e-6;
        }

        if (!(std::fabs(refPathLength - fastPathLength) < 0.0001f))
        {
            printf("\n\nMismatch! Expected path length of %f, got %f.\n", refPathLength, fastPathLength);
            return EXIT_FAILURE;
        }

        refSum += refTime;
        fastSum += fastTime;
        const double refAvg = refSum / double(i);
        const double fastAvg = fastSum / double(i);
        const double ratio = refAvg / fastAvg;

        printf("\rVerified %llu paths, avg %f ms (%fx speedup)", static_cast<unsigned long long>(i), fastAvg, ratio);

        GenerateWalls(walls);
    }
}
