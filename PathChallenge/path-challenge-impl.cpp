#include <vector>
#include <intrin.h>

#include <deque>
#include <assert.h>

#include <windows.h>

using U32 = uint32_t;
using U64 = uint64_t;

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

static const double SQRT_2 = 1.4142135623730951;
static const double SENTINEL = 99999999.0;

struct v2
{
    U32 x, y;

    U32 index() const
    {
        return y * kCols + x;
    }

    bool operator==(const v2 &other) const { return x == other.x && y == other.y; }
    bool operator!=(const v2 &other) const { return !(*this == other); }
};

struct Node
{
    v2 pos;
    double f_score;
};

struct PerfStats
{
    // Stats
    U64 iterations = 0;

    U32 open_list_count = 0;
    U32 max_open_list_count = 0;

    U32 max_bucket_size = 0;
    U32 max_bucket_index = 0;

    std::deque<U64> find_cheapest_latencies = {};
    std::deque<U64> explore_neighbor_latencies = {};

    U64 initialize_latency = 0;

    double min_f_score = 1000000000000.0;
    double max_f_score = 0.0;
    double sum_f_score = 0.0;
    U32 cnt_f_score = 0;
};
static PerfStats perf_stats;

#define ABS(a) ((a) < 0) ? -a : a
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
double heuristic(v2 a, v2 b)
{
    int xDiff = ABS(a.x - b.x);
    int yDiff = ABS(a.y - b.y);
    return sqrt(xDiff*xDiff + yDiff*yDiff);
}

bool is_wall(v2 node, const U64 *walls)
{
    // Find the chunk of bits that contains the wall and mask to find the wall.
    U64 wall_chunk_bits = walls[node.index() / 64];
    U64 mask = 1ULL << (node.index() % 64);
    return wall_chunk_bits & mask;
}

#define BUCKETS
#ifdef BUCKETS

#define MIN_POSSIBLE_F_SCORE 200.0
struct OpenList
{
    struct Bucket
    {
        std::vector<Node> nodes;
    };

    std::vector<Bucket> buckets;
    U32 cheapest_bucket;
};

void open_list_push(OpenList *open_list, Node node)
{
    U32 bucket_index = (U32)((node.f_score - MIN_POSSIBLE_F_SCORE) * 50.0);

    if(bucket_index >= open_list->buckets.size())
    {
        open_list->buckets.resize(bucket_index + 1);
    }

    OpenList::Bucket *bucket = &(open_list->buckets[bucket_index]);

    bool found_on_list = false;
    for(U32 i = 0; i < bucket->nodes.size(); i++)
    {
        if(bucket->nodes[i].pos == node.pos)
        {
            found_on_list = true;
            bucket->nodes[i].f_score = node.f_score;
            break;
        }
    }

    if(!found_on_list)
    {
        bucket->nodes.push_back(node);
        open_list->cheapest_bucket = MIN(open_list->cheapest_bucket, bucket_index);
    }
}

Node open_list_pop(OpenList *open_list)
{
    OpenList::Bucket *bucket = &(open_list->buckets[open_list->cheapest_bucket]);

#if 0
    // Guaranteed find the cheapest node on the list.
    Node result = bucket->nodes[0];
    U32 result_i = 0;
    for(U32 i = 1; i < bucket->nodes.size(); i++)
    {
        if(bucket->nodes[i].f_score < result.f_score)
        {
            result = bucket->nodes[i];
            result_i = i;
        }
    }
    std::swap(bucket->nodes[result_i], bucket->nodes[bucket->nodes.size() - 1]);
#endif

    Node result = bucket->nodes.back();
    bucket->nodes.pop_back();

    if(bucket->nodes.empty())
    {
        while(open_list->buckets[open_list->cheapest_bucket].nodes.empty())
        {
            open_list->cheapest_bucket++;
            if(open_list->cheapest_bucket == open_list->buckets.size())
            {
                open_list->cheapest_bucket = (U32)-1;
                break;
            }
        }
    }

    return result;
}

void open_list_clear(OpenList *open_list)
{
    open_list->cheapest_bucket = (U32)-1;
    for(U32 i = 0; i < open_list->buckets.size(); i++)
    {
        open_list->buckets[i].nodes.clear();
    }
}

bool open_list_is_empty(OpenList *open_list)
{
    return open_list->cheapest_bucket == (U32)-1;
}

#else // BUCKETS

struct OpenList
{
    std::vector<Node> nodes;
};

void open_list_push(OpenList *open_list, Node to_push)
{
    bool found_on_open_list = false;
    for(Node &node : open_list->nodes)
    {
        if(node.pos == to_push.pos)
        {
            found_on_open_list = true;
            node.f_score = to_push.f_score;
            break;
        }
    }
    if(!found_on_open_list)
    {
        open_list->nodes.push_back(to_push);
    }
}

Node open_list_pop(OpenList *open_list)
{
    Node current = open_list->nodes[0];
    for(const Node &node : open_list->nodes)
    {
        if(node.f_score < current.f_score)
        {
            current = node;
        }
    }
    // Remove current from the open list
    for(Node &node : open_list->nodes) if(node.pos == current.pos) { std::swap(node, open_list->nodes.back()); break; }
    open_list->nodes.pop_back();

    return current;
}

void open_list_clear(OpenList *open_list)
{
    open_list->nodes.clear();
}

bool open_list_is_empty(OpenList *open_list)
{
    return open_list->nodes.empty();
}

#endif // BUCKETS


void post_process()
{
#if 0
    printf("\n");

    U64 sum = 0;
    U64 min_latency = ~0;
    U64 max_latency = 0;
    for(U64 latency : perf_stats.find_cheapest_latencies)
    {
        sum += latency;
        min_latency = MIN(min_latency, latency);
        max_latency = MAX(max_latency, latency);
    }

    printf("iterations: %I64i\n", perf_stats.iterations);
    printf("open list max: %i\n", perf_stats.max_open_list_count);
    printf("initialize latency  %I64i\n", perf_stats.initialize_latency);
    printf("find cheapest\n");
    printf("    mean: %f\n", (float)sum / perf_stats.find_cheapest_latencies.size());
    printf("    min: %I64i\n", min_latency);
    printf("    max: %I64i\n", max_latency);
    printf("min f score: %f\n", perf_stats.min_f_score);
    printf("max f score: %f\n", perf_stats.max_f_score);
    printf("avg f score: %f\n", perf_stats.sum_f_score / (double)perf_stats.cnt_f_score);
    printf("max bucket_size: %i\n", perf_stats.max_bucket_size);
    printf("max bucket_index: %i\n", perf_stats.max_bucket_index);

    // while(perf_stats.find_cheapest_latencies.size() >= 1000)
    // {
    //     perf_stats.find_cheapest_latencies.pop_front();
    // }

    perf_stats = {};

    Sleep(1000);
#endif
}


/// your code goes here!
float FastPathFind(const U64* pWalls)
{
    U64 initialize_time = __rdtsc();

    // Initialize grid
    double g_scores[kCols * kRows] = {};
    for(U32 r = 0; r < kRows; r++)
    {
        for(U32 c = 0; c < kCols; c++)
        {
            U32 i = v2{c, r}.index();
            g_scores[i] = SENTINEL;
        }
    }

    v2 start = {kStartX, kStartY};
    v2 target = {kEndX, kEndY};

    static OpenList *open_list = nullptr;
    if(!open_list)
    {
        open_list = new OpenList();
    }
    open_list_clear(open_list);

    open_list_push(open_list, {start, heuristic(start, target)});
    g_scores[start.index()] = 0.0f;

    while(!open_list_is_empty(open_list))
    {
        U64 find_cheapest_time = __rdtsc();

        // Find the cheapest node on the open list
        Node current = open_list_pop(open_list);

        // Check if done
        if(current.pos == target)
        {
            post_process();
            return g_scores[current.pos.index()];
        }

        U64 explore_neighbor_time = __rdtsc();
        v2 neighbors[] =
        {
            {current.pos.x - 1, current.pos.y + 1}, {current.pos.x, current.pos.y + 1}, {current.pos.x + 1, current.pos.y + 1},
            {current.pos.x - 1, current.pos.y    },                                     {current.pos.x + 1, current.pos.y    },
            {current.pos.x - 1, current.pos.y - 1}, {current.pos.x, current.pos.y - 1}, {current.pos.x + 1, current.pos.y - 1}
        };
        for(const v2 neighbor : neighbors)
        {
            if(neighbor.x >= kCols || neighbor.y >= kRows || is_wall(neighbor, pWalls))
            {
                continue;
            }

            double neighbor_distance = ((neighbor.x == current.pos.x) || (neighbor.y == current.pos.y)) ? 1.0f : SQRT_2;
            double neighbor_new_g_score = g_scores[current.pos.index()] + neighbor_distance;

            if(neighbor_new_g_score < g_scores[neighbor.index()])
            {
                g_scores[neighbor.index()] = neighbor_new_g_score;
                double f_score = neighbor_new_g_score + heuristic(neighbor, target);
                open_list_push(open_list, {neighbor, f_score});
            }
        }
    }

    post_process();
    return -1.0;
}



