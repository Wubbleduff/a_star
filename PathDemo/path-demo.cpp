/*******************************************************************
*
*    Author: Kareem Omar
*    kareem.h.omar@gmail.com
*    https://github.com/komrad36
*
*    Last updated Dec 15, 2021
*******************************************************************/

#include <cmath>
#include <initializer_list>
#include <random>
#include <SDL.h>
#include <tuple>

using U8 = Uint8;
using U32 = Uint32;
using U64 = Uint64;

/// configuration
static constexpr U32 kRows = 120;
static constexpr U32 kCols = 230;
static constexpr U32 kCellSize = 8;
static constexpr U32 kStartX = 14;
static constexpr U32 kStartY = 11;
static constexpr U32 kEndX = (kCols - 1) - 14;
static constexpr U32 kEndY = (kRows - 1) - 11;
static constexpr double kTitleUpdateTick = 60e6;
static constexpr double kPathFindTick = 80e3;
static constexpr double kPathTraceTick = 185e4;

/// derived configuration
static constexpr U32 kWidth = kCols * kCellSize;
static constexpr U32 kHeight = kRows * kCellSize;
static constexpr U32 kCells = kRows * kCols;
static constexpr U32 kStartI = kStartY * kCols + kStartX;
static constexpr U32 kEndI = kEndY * kCols + kEndX;
static constexpr U32 kBlocks = (kCells + 63) >> 6;
static constexpr U32 kPixels = kWidth * kHeight;

/// internal constants
static constexpr double kNotPathfinding = -2.0;
static constexpr double kPathNotFound = -1.0;
static constexpr double kUnvisitedCost = 99999999999.0;

/// finite state machine implementation
#define DEFINE_FSM(FSM_NAME) class FSM_NAME : public FSM<FSM_NAME>
#define DEFINE_STATE(PARENT, STATE_NAME) class PARENT::PARENT##_##STATE_NAME : public State<PARENT>
#define REGISTER_STATE(PARENT, STATE_NAME, STATE_ID) class PARENT##_##STATE_NAME; friend class PARENT##_##STATE_NAME; \
static constexpr U32 kState##STATE_NAME = STATE_ID; \
void Go##STATE_NAME() { Go<PARENT##_##STATE_NAME>(*this, kState##STATE_NAME); } \
bool Is##STATE_NAME() const { return m_pCurState && m_pCurState == m_pStates[STATE_ID]; }

template <class T>
class State
{
public:
    State(T& fsm) : m_fsm(fsm) {}
    virtual void Start() {}
    virtual void Update(double t) { static_cast<void>(t); }
    virtual void Stop() {}
    virtual ~State() {}

protected:
    T& m_fsm;
};

template <class T>
class FSM
{
    static constexpr U64 kMaxStates = 16;

public:
    FSM() : m_pStates{}, m_pCurState(nullptr)
    {
    }

    void Update(double t)
    {
        if (m_pCurState)
            m_pCurState->Update(t);
    }

    ~FSM()
    {
        if (m_pCurState)
            m_pCurState->Stop();

        for (State<T>* pState : m_pStates)
            delete pState;
    }

protected:
    template <class U>
    void Go(T& fsm, U32 iState)
    {
        if (m_pCurState)
            m_pCurState->Stop();

        if (!m_pStates[iState])
            m_pStates[iState] = new U(fsm);

        m_pCurState = m_pStates[iState];
        m_pCurState->Start();
    }

    State<T>* m_pStates[kMaxStates];
    State<T>* m_pCurState;
};

/// RGB to ARGB8888
struct RGB
{
    RGB(U8 r, U8 g, U8 b) : m_u((U32(r) << 16) | (U32(g) << 8) | U32(b)) {}
    U32 m_u;
};

/// draw a cell into the pixel buffer
static void DrawCell(U32* pPix, RGB rgb, U32 x, U32 y)
{
    for (U32 i = y * kCellSize; i < y * kCellSize + kCellSize - 1; ++i)
    {
        for (U32 j = x * kCellSize; j < x * kCellSize + kCellSize - 1; ++j)
        {
            pPix[i * kWidth + j] = rgb.m_u;
        }
    }
};

/// path heuristic
static double Heuristic(U32 x1, U32 y1, U32 x2, U32 y2)
{
    const double diffX = std::fabs(double(x1) - double(x2));
    const double diffY = std::fabs(double(y1) - double(y2));

    return 0.4142135623730951 * std::min(diffX, diffY) + std::max(diffX, diffY);
}

/// Pathfinder state machine
DEFINE_FSM(Pathfinder)
{
public:
    Pathfinder(const U64* pWalls) : m_walls(pWalls), m_useHeuristic(true)
    {
        ResetAndGoIdle();
    }

    void Reset()
    {
        m_len = kNotPathfinding;
        for (U32 i = 0; i < kBlocks; ++i)
            m_visited[i] = 0;
        for (U32 i = 0; i < kBlocks; ++i)
            m_queued[i] = 0;
        m_pathTraceI = kEndI;
    }

    void ResetAndGoIdle()
    {
        Reset();
        GoIdle();
    }

    void ToggleHeuristic()
    {
        m_useHeuristic = !m_useHeuristic;
    }

    template <size_t n>
    void Print(char(&buf)[n]) const
    {
        char state[32];
        snprintf(state, sizeof(state), "State: %s", IsIdle() ? "Idle" : IsPathFind() ? "PathFind" : IsPathTrace() ? "PathTrace" : "<No State>");

        char heur[64];
        snprintf(heur, sizeof(heur), m_useHeuristic ? "Octile Heuristic A* with High-g Tiebreak" : "Dijkstra");

        char pathLen[32];
        if (m_len == kNotPathfinding)
            pathLen[0] = 0;
        else if (m_len == kPathNotFound)
            snprintf(pathLen, sizeof(pathLen), " | No Path");
        else
            snprintf(pathLen, sizeof(pathLen), " | Path Len: %f", m_len);

        snprintf(buf, sizeof(buf), "%s | %s%s", heur, state, pathLen);
    }

    void Draw(U32* p)
    {
        for (U32 y = 0; y < kRows; ++y)
        {
            for (U32 x = 0; x < kCols; ++x)
            {
                const U32 i = y * kCols + x;
                if (m_visited[i >> 6] & (1ULL << (i & 63ULL)))
                    DrawCell(p, { 255, 0, 255 }, x, y);
            }
        }

        U32 i = kEndI;
        while (i != m_pathTraceI)
        {
            const U32 x = i % kCols;
            const U32 y = i / kCols;
            DrawCell(p, { 50, 50, 255 }, x, y);
            i = m_path[i];
        }
    }

    REGISTER_STATE(Pathfinder, Idle, 0);
    REGISTER_STATE(Pathfinder, PathFind, 1);
    REGISTER_STATE(Pathfinder, PathTrace, 2);

protected:
    const U64* m_walls;
    U64 m_visited[kBlocks];
    U64 m_queued[kBlocks];
    U32 m_path[kCells];
    U32 m_pathTraceI;
    double m_len;
    bool m_useHeuristic;
};

/// Pathfinder state Idle
DEFINE_STATE(Pathfinder, Idle)
{
    using State<Pathfinder>::State;
};

/// Pathfinder state PathFind
DEFINE_STATE(Pathfinder, PathFind)
{
    using State<Pathfinder>::State;

public:
    virtual void Start()
    {
        m_fsm.Reset();
        m_numTicks = ~0U;
        for (U32 i = 0; i < kCells; ++i)
            m_f[i] = kUnvisitedCost;
        for (U32 i = 0; i < kCells; ++i)
            m_g[i] = kUnvisitedCost;
        m_f[kStartI] = 0.0;
        m_g[kStartI] = m_fsm.m_useHeuristic ? Heuristic(kStartX, kStartY, kEndX, kEndY) : 0.0;
        m_openSetSize = 1;
        m_openSet[0] = kStartI;
        m_fsm.m_queued[kStartI >> 6] |= 1ULL << (kStartI & 63ULL);
    }

    virtual void Update(double t)
    {
        if (m_numTicks == ~0U)
            m_startTime = t;

        const double elapsedTime = t - m_startTime;
        const U32 desiredTicks = U32(elapsedTime / kPathFindTick);

        for (; m_numTicks != desiredTicks; ++m_numTicks)
        {
            U32 bestJ = 0;
            U32 bestI = m_openSet[0];
            for (U32 j = 1; j < m_openSetSize; ++j)
            {
                const U32 i = m_openSet[j];
                if (std::make_tuple(m_g[i], -m_f[i]) < std::make_tuple(m_g[bestI], -m_f[bestI]))
                {
                    bestJ = j;
                    bestI = i;
                }
            }

            const double f = m_f[bestI];
            m_fsm.m_queued[bestI >> 6] &= ~(1ULL << (bestI & 63ULL));
            m_fsm.m_visited[bestI >> 6] |= 1ULL << (bestI & 63ULL);
            m_openSet[bestJ] = m_openSet[--m_openSetSize];

            if (bestI == kEndI)
            {
                m_fsm.GoPathTrace();
                return;
            }

            const U32 eX = bestI % kCols;
            const U32 eY = bestI / kCols;

            for (const std::pair<U32, U32>& p : std::initializer_list<std::pair<U32, U32>>{
                {eX - 1, eY}, {eX + 1, eY}, {eX, eY - 1}, {eX, eY + 1},
                {eX - 1, eY - 1}, {eX - 1, eY + 1}, {eX + 1, eY - 1}, {eX + 1, eY + 1}
            })
            {
                const U32 x = p.first;
                const U32 y = p.second;
                const U32 i = y * kCols + x;
                if (x < kCols && y < kRows && !((m_fsm.m_walls[i >> 6] | m_fsm.m_visited[i >> 6]) & (1ULL << (i & 63ULL))))
                {
                    if (!(m_fsm.m_queued[i >> 6] & (1ULL << (i & 63ULL))))
                    {
                        m_openSet[m_openSetSize++] = i;
                        m_fsm.m_queued[i >> 6] |= 1ULL << (i & 63ULL);
                    }

                    const double candidateF = f + ((x == eX || y == eY) ? 1.0 : 1.4142135623730951);
                    if (candidateF < m_f[i])
                    {
                        m_fsm.m_path[i] = bestI;
                        m_f[i] = candidateF;
                        m_g[i] = m_fsm.m_useHeuristic ? candidateF + Heuristic(x, y, kEndX, kEndY) : candidateF;
                    }
                }
            }

            if (m_openSetSize == 0)
            {
                m_fsm.m_len = kPathNotFound;
                m_fsm.GoIdle();
                return;
            }
        }
    }

protected:
    U32 m_openSet[kCells];
    U32 m_openSetSize;
    double m_f[kCells];
    double m_g[kCells];
    double m_startTime;
    U32 m_numTicks;
};

/// Pathfinder state PathTrace
DEFINE_STATE(Pathfinder, PathTrace)
{
    using State<Pathfinder>::State;

public:
    virtual void Start()
    {
        m_numTicks = ~0U;
        m_fsm.m_pathTraceI = kEndI;
        m_fsm.m_len = 0.0;
    }

    virtual void Update(double t)
    {
        if (m_numTicks == ~0U)
            m_startTime = t;

        const double elapsedTime = t - m_startTime;
        const U32 desiredTicks = U32(elapsedTime / kPathTraceTick);

        for (; m_numTicks != desiredTicks; ++m_numTicks)
        {
            const U32 newI = m_fsm.m_path[m_fsm.m_pathTraceI];
            const U32 x = m_fsm.m_pathTraceI % kCols;
            const U32 y = m_fsm.m_pathTraceI / kCols;
            const U32 newX = newI % kCols;
            const U32 newY = newI / kCols;
            m_fsm.m_len += (x == newX || y == newY) ? 1.0 : 1.4142135623730951;
            m_fsm.m_pathTraceI = newI;
            if (m_fsm.m_pathTraceI == kStartI)
            {
                m_fsm.GoIdle();
                return;
            }
        }
    }

protected:
    double m_startTime;
    U32 m_numTicks;
};

static double GetCurTime()
{
    return double(SDL_GetPerformanceCounter()) * 1e9 / double(SDL_GetPerformanceFrequency());
}

static int RandomNonZeroInt(std::mt19937& gen, int a, int b)
{
    const int x = std::uniform_int_distribution<int>(a, b - 1)(gen);
    return x ? x : b;
}

static void GenerateWalls(U64* pWalls)
{
    for (U32 i = 0; i < kBlocks; ++i)
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

static void PresetWalls(U64* pWalls)
{
    static constexpr U64 kPreset[] = {
        0x303c000000200000,0x200000000000000,0x8000000000000000,0x800000000010001,0x409800000,0xe0000000,0x40002000000000,
        0x200000000,0x800000182,0x20000000000000c,0x10001800,0x208000000180,0x1f800000600,0xf0000c0000000,0x400000000004,
        0x3000000083000,0x6380,0x100007c0018,0x204000000100000,0x30300001800000,0x1f0030000000000,0x400000000004000,
        0x7000000081800000,0x18040000,0x10000007806000,0x2000000100000400,0x100000600600020,0x300e00000000000c,
        0x100000c000000,0x7c00781800000000,0x60060000080,0x600800000,0xc00002,0x2071f8700400,0x3000000000030008,
        0x2000008000000080,0xf003000000000000,0x200000f7033,0x300400000000,0x1800002000,0x70000000800000,0xc000,
        0x8000000060180,0x2000000000000400,0x10000000003ff0,0xc0200000000000,0x3000002000002,0x70000000000,0x4000000,
        0x800000c01bec0000,0x800000,0x8000000000007000,0x9000000001c0001,0x7f2000001182,0x7000000000000,
        0xdc0002000000000,0x6f00260000078,0x307fc0,0x700000,0xf80618008800,0x18031ffe00ec0088,0x700000000000000,
        0x33018000000000,0xfe00230000000300,0xc006000ff,0x7e0000070000000,0x800016407000,0x800000300084000,0x200,
        0xd817060f0fe00007,0x618000002600004,0x3000001fe,0xfe0007000000000,0x9000011a0471fe00,0xe0c102000001,0x1c0,
        0x7c000000fe00700,0xc00000fc00004381,0xc00000601840,0xf00700000000000,0x604080000000,0x100210100000f000,
        0x303e00000,0x70000,0xe00000000c1030,0x8000000060060780,0x7000000000007f,0x304040000002000,0xe0be00e0000000,
        0x38,0x1c000007000000,0xe101c000,0x3a000c30f8e0,0x7000000000000000,0xc01e000005800000,0x8603e0000000006d,
        0x38c001,0x2000000600000000,0x11c000f00001,0x18100838800f1fe0,0x0,0xf780007f000000e0,0x6c0ffe000c0007,
        0x404020330,0xe0000000000,0xf003c000040,0x8066000e0000fe,0x101,0xe000107000e0c000,0x8000060001,0x4040200cf800,
        0x6df8000000000000,0x300000f0007f600,0x183fe0000006000,0x4000000010100c,0x8031c0000600,0x10000180000078,
        0x40419002000ff00,0x1c1800030000000,0x3c000003c00,0x780004000040,0x101044008,0x19c0001ff00018,0xc001e00,
        0x4310030000000001,0x24000c00000000c0,0xe000f0000c180000,0xc000000001,0x201184004000,0x3f0001980060000,
        0xf0007800e,0x8018000000002000,0x300000000080441,0x30060064000420,0x8000000000060,0x201302003000000,
        0x10c00e0000000,0xf0004030001,0x6600000002000000,0x80660800,0x180004000c1000f,0x8000000000004001,
        0xce2000f00000001,0x2000007018000020,0x18004000003000,0x4000c00000,0x8010c800180,0x38f0680000078f,
        0x1c00000802003000,0xe000038000001000,0x8003e00001c0001,0x80080000186013c0,0x4000180000300,0x2c0000180001b000,
        0x3c00de020001f000,0x78200200001c,0x1c6000003000038,0x70000e000003c0,0x80001c01e030f080,0xffffc3000003f800,
        0x1fffc0c0003e,0x863f300030000100,0x7e020001c7c0f,0x1000f8a0001fff00,0x0,0xdf00020c3f40008,0xe0007800001b0800,
        0x400e02803,0x3100060000000000,0xcee000fc0000c18,0xe006ff8e00030000,0x80000101,0x1030860010000,0x4018030e800ec0,
        0x605007e000f0,0xc0000000000060,0x600e200000c04608,0xc0000700f0060063,0x300000083603,0x201f020020000000,
        0x1800c1006180000,0x208c0b000007fe0,0x80300,0xc0000180000c018,0x8000020018407,0x7fc00080183c00,0x10060004000c00,
        0x31070200000400,0x306000002000008,0x1000ff0700030,0x4018000,0x8000030007c70080,0xf40004c040800000,
        0x60000000000600,0x3300200000000001,0x2000004000,0x100010001f830,0x600c000000,0x10000780080000,0x1f080000001800,
        0x400040,0x2c00000f8000801,0x4000104000180,0x1600100000620000,0x23c4000400000,0xf000e0000062,0xd800000010000c1,
        0x1000000f80040000,0x118000981000,0x400060c00027e01c,0x1000001c0000000,0x3c0400040003fe00,0xbec0700000cc000,
        0x1000302000,0xff90000040000000,0x2600006010001ff,0x181800020900e000,0xc00,0x400000000c001010,
        0x4000000190000000,0x2000c04000080,0x6040000000000,0x1000000002,0x2030070000fc,0x1800603,0xc18000,
        0x6000280000000400,0x400300800008080f,0x1020000000000000,0x1c00000000000,0x202060e001a0000,0x3000c06000,
        0x7e0c08000000,0xcc00000001c0000,0x1030000180870300,0x800,0x180000000f0fa02,0x27c0000310000000,0x3000618000040,
        0xe01fe08000000000,0x38000001,0x100f200001e4,0x40008c,0x1fc001be2018,0x6d0000000006,0x1800320000040300,
        0x78ec1fc000000000,0xf0000000000,0x100800000397800,0x30004c000,0x740f1c1c00,0x1a63000000000000,
        0x198000040204000,0xcc01c00000000060,0x7303,0x83f0004cc700000,0xc00230001d0,0x30c1b780100000,0x8c00000000000000,
        0x1dc0218700311,0x40000000001800c,0x30181fce00,0x781842100000000,0x27c101c3c30084,0xc2800100000000,0x180c,
        0x8000230038410c40,0xc1f60d7,0xc020000f00040,0xb041980000000000,0x80786f00000d8003,0x1400300000000001,0x6008000,
        0x140003810320000,0x3003fe0000,0x60000500180000,0xc80000000000001,0xe001fe070000606,0xe00000000000600,
        0x10000160,0x3008620000000,0xc000c003c018,0x60480e80000000,0x4000,0x1e040000802308,0x10001c,0x3000007f120630,
        0xf008820000000000,0xc0001c000c10003,0x7d87040000000000,0x600000018000030,0x1040000603208000,0x1800,
        0xf80147010000,0x482000030000000c,0x3018410000f00,0x7860000000000000,0x60003e00043,0xe400608000180,0xf104,
        0x8000188388000000,0xe0000001000f,0x26c1001e100102,0x300000000000,0xc00e0000026032,0x40040c000680000,0x18e0401c,
        0xb006c000380000,0xf300000426000000,0x18103c0180101000,0x80000000000000c,0x28009000,0x40403e04000031f,
        0x20204180020,0x36000000000000,0x20018f000000000e,0x80301800010,0xe018001,0x30006800000,0x2000061800c30000,
        0xfc40006100020080,0xf000000000000000,0x6080000000008000,0x3f80200800008f00,0x1f0000840,0x1f80000000,
        0x36403000000000,0x213f800080300,0x0,0x33c,0x200400007101800,0x8f8000,0x418000000000,0x878c000000000000,
        0x2000000180100001,0x80000060000000,0x100000,0x60000003e300000,0x800000040,0xc00000ffe000018,0xc00000200000000,
        0x100080000000,0xe0000c0000000200,0x20000e000,0x200000180,0x800000640020,0x3e0000e00060000,0x400000000080,
        0xdc00c0000000180,0x200000,0x600f800000f0,0x700000100000,0x80000011c010000,0x60000000000,0x400000000103e00,
        0xc040000000360300,0x200000061,0x4f8000000000000,0x8c0600140000000,0x81c18000000,0x80,0x1e0000000,0x2180c0058,
        0x20186,0xc00000000c00000,0x1e01300000000c0,0xc03100000000c3,0x6000000000000000,0x200600000000,0x10600f0c6000,
        0x180000010004000,0x3000000000,0x7a0c018000080300,0x10000000040c00,0xf0000004,0x601820000003800,
        0x1810003c100e000,0x10004000000,0x18000000000064,0x603c18010000c000,0x2000007e,0x110000004003,0x1800000c000000,
        0x70f803ff0040,0x1000800000000c,0xd800,0x8070100003000006,0x10000f00600,0xfc000004006000,0x6000010000000000,
        0xe0000020070c0003,0x2010000000004001,0x3,0x7e0003cc00004000,0x1103c000001800,0x19800000000,0x10000004000000,
        0xe004000000039980,0x47f8000,0xc600,0x380200004000001,0x30000ef00000000,0x20800000000001,0xc000000000,
        0x3870c0001,0x600001e070,0x40600000,0x7d00004000002000,0xc0078000000397,0x1000000000000800,0x8000000000018,
        0x38701f0003000,0x2000000003800,0x30400000000,0xc008000003000000,0x3000003830007,0x800000,0x6000000000006300,
        0x8080003002000000,0x3000000000780003,0x88800000000000,0x8000000c00000000,0x3c001fc20001800,0x400000000,
        0x20600000,0x3e04003000000180,0x1e0001f8,0x1000080000000100,0x200000000018,0xf00003f801000400,0x400000000000,
        0x0 };
    for (U32 i = 0; i < kBlocks; ++i)
        pWalls[i] = kPreset[i];
}

// if mouse L is down, add wall
// if mouse R is down, remove wall
static void ProcessMouse(U64* pWalls)
{
    int mouseX;
    int mouseY;
    const U32 buttonMask = SDL_GetMouseState(&mouseX, &mouseY);
    const U32 mouseCellX = U32(mouseX) / kCellSize;
    const U32 mouseCellY = U32(mouseY) / kCellSize;
    const U32 mouseI = mouseCellY * kCols + mouseCellX;
    const bool mouseLeft = buttonMask & SDL_BUTTON_LMASK;
    const bool mouseRight = buttonMask & SDL_BUTTON_RMASK;
    if (mouseLeft && !mouseRight)
    {
        const U32 x = mouseCellX;
        const U32 y = mouseCellY;
        for (const std::pair<U32, U32>& p : std::initializer_list<std::pair<U32, U32>>{
            {x, y}, {x + 1, y}, {x, y + 1}, {x + 1, y + 1}})
        {
            const U32 px = p.first;
            const U32 py = p.second;
            const U32 i = py * kCols + px;
            if (px < kCols && py < kRows && i != kStartI && i != kEndI)
                pWalls[i >> 6] |= 1ULL << (i & 63ULL);
        }
    }
    else if (mouseRight && !mouseLeft)
    {
        pWalls[mouseI >> 6] &= ~(1ULL << (mouseI & 63ULL));
    }
}

int main(int, char**)
{
    SDL_Window* pWindow = SDL_CreateWindow("Pathfind", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, kWidth, kHeight, SDL_WINDOW_HIDDEN);
    SDL_Renderer* pRenderer = SDL_CreateRenderer(pWindow, -1, SDL_RENDERER_ACCELERATED/* | SDL_RENDERER_PRESENTVSYNC*/);
    SDL_ShowWindow(pWindow);
    SDL_Texture* pTex = SDL_CreateTexture(pRenderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, kWidth, kHeight);
    U32* pPix = new U32[kPixels];

    double lastTitleUpdateTime = GetCurTime();
    double nextTitleUpdateTime = lastTitleUpdateTime + kTitleUpdateTick;
    U64 titleUpdateFrames = 0;

    U64 walls[kBlocks] = { 0 };
    PresetWalls(walls);

    Pathfinder* pPathfinder = new Pathfinder(walls);

    for (;;)
    {
        // input processing
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            switch (e.type)
            {
            case SDL_KEYDOWN:
                switch (e.key.keysym.sym)
                {
                case SDLK_ESCAPE:
                    delete[] pPix;
                    delete pPathfinder;
                    return EXIT_SUCCESS;
                case SDLK_c:
                    for (U32 i = 0; i < kBlocks; ++i)
                        walls[i] = 0;
                    pPathfinder->ResetAndGoIdle();
                    break;
                case SDLK_g:
                    GenerateWalls(walls);
                    pPathfinder->ResetAndGoIdle();
                    break;
                case SDLK_p:
                    PresetWalls(walls);
                    pPathfinder->ResetAndGoIdle();
                    break;
                case SDLK_h:
                    pPathfinder->ToggleHeuristic();
                    break;
                case SDLK_w:
                    // write walls to console
                    for (U32 i = 0; i < kBlocks; ++i)
                        printf("0x%llx,", static_cast<unsigned long long>(walls[i]));
                    break;
                case SDLK_SPACE:
                    pPathfinder->GoPathFind();
                    break;
                }
                break;
            case SDL_QUIT:
                delete[] pPix;
                delete pPathfinder;
                return EXIT_SUCCESS;
            }
        }

        // paint walls with mouse
        ProcessMouse(walls);

        const double curTime = GetCurTime();

        // update pathfinder
        pPathfinder->Update(curTime);

        // update title bar
        if (curTime > nextTitleUpdateTime)
        {
            const double fps = 1e9 * double(titleUpdateFrames) / (curTime - lastTitleUpdateTime);
            char pathfinderInfo[256];
            pPathfinder->Print(pathfinderInfo);
            char buf[512];
            if (pathfinderInfo[0])
                snprintf(buf, sizeof(buf), "Pathfind | %s | %.0f fps", pathfinderInfo, fps);
            else
                snprintf(buf, sizeof(buf), "Pathfind | %.0f fps", fps);
            SDL_SetWindowTitle(pWindow, buf);
            lastTitleUpdateTime = curTime;
            nextTitleUpdateTime += kTitleUpdateTick;
            titleUpdateFrames = 0;
        }

        ++titleUpdateFrames;

        // draw black background
        for (U32 i = 0; i < kPixels; ++i)
            pPix[i] = 0;

        // draw pathfinder
        pPathfinder->Draw(pPix);

        // draw start
        DrawCell(pPix, { 0, 255, 0 }, kStartX, kStartY);

        // draw end
        DrawCell(pPix, { 255, 0, 0 }, kEndX, kEndY);

        // draw walls
        for (U32 y = 0; y < kRows; ++y)
        {
            for (U32 x = 0; x < kCols; ++x)
            {
                const U32 i = y * kCols + x;
                if (walls[i >> 6] & (1ULL << (i & 63ULL)))
                    DrawCell(pPix, { 130, 130, 130 }, x, y);
            }
        }

        // render
        SDL_UpdateTexture(pTex, nullptr, pPix, 4 * kWidth);
        SDL_RenderCopy(pRenderer, pTex, nullptr, nullptr);
        SDL_RenderPresent(pRenderer);
    }
}
