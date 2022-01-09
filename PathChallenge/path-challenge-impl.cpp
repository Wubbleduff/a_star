#include <vector>
#include <intrin.h>
#include <immintrin.h>

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

    std::deque<U64> pop_latencies = {};
    std::deque<U64> push_latencies = {};

    U64 initialize_latency = 0;

    double min_f_score = 1000000000000.0;
    double max_f_score = 0.0;
    double sum_f_score = 0.0;
    U32 cnt_f_score = 0;
};
static PerfStats perf_stats;

#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
double heuristic(v2 a, v2 b)
{
    int diff_x = a.x - b.x;
    int diff_y = a.y - b.y;
    return sqrt(diff_x*diff_x + diff_y*diff_y);
}

#define BUCKETS 1
#if BUCKETS

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
    U64 start_time = __rdtsc();

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

    // perf_stats.push_latencies.push_back(__rdtsc() - start_time);
}

Node open_list_pop(OpenList *open_list)
{
    U64 start_time = __rdtsc();

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

    perf_stats.open_list_count--;

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
    
    // perf_stats.pop_latencies.push_back(__rdtsc() - start_time);
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
    printf("iterations: %I64i\n", perf_stats.iterations);
    printf("open list max: %i\n", perf_stats.max_open_list_count);

    {
        U64 sum = 0;
        U64 min_latency = ~0;
        U64 max_latency = 0;
        for(U64 latency : perf_stats.pop_latencies)
        {
            sum += latency;
            min_latency = MIN(min_latency, latency);
            max_latency = MAX(max_latency, latency);
        }
        printf("pop latency\n");
        printf("    mean: %f\n", (float)sum / perf_stats.pop_latencies.size());
        printf("    min: %I64i\n", min_latency);
        printf("    max: %I64i\n", max_latency);
    }

    {
        U64 sum = 0;
        U64 min_latency = ~0;
        U64 max_latency = 0;
        for(U64 latency : perf_stats.push_latencies)
        {
            sum += latency;
            min_latency = MIN(min_latency, latency);
            max_latency = MAX(max_latency, latency);
        }
        printf("push latency\n");
        printf("    mean: %f\n", (float)sum / perf_stats.push_latencies.size());
        printf("    min: %I64i\n", min_latency);
        printf("    max: %I64i\n", max_latency);
    }

    while(perf_stats.pop_latencies.size() >= 1000)
    {
        perf_stats.pop_latencies.pop_front();
    }
    while(perf_stats.push_latencies.size() >= 1000)
    {
        perf_stats.push_latencies.pop_front();
    }

    perf_stats = {};

    Sleep(1000);
#endif
}


/// your code goes here!
float FastPathFind(const U64* pWalls)
{
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
        // Find the cheapest node on the open list
        Node current = open_list_pop(open_list);

        // Check if done
        if(current.pos == target)
        {
            post_process();
            return g_scores[current.pos.index()];
        }

#define WIDE 1
#if WIDE
        __m256d neighbor_distances_0 = _mm256_set_pd(1.0, SQRT_2, 1.0, SQRT_2);
        __m256d neighbor_distances_1 = _mm256_set_pd(SQRT_2, 1.0, SQRT_2, 1.0);

        __m256d current_g_score = _mm256_broadcast_sd(&(g_scores[current.pos.index()]));
        __m256d neighbor_g_scores_0 = _mm256_add_pd(current_g_score, neighbor_distances_0);
        __m256d neighbor_g_scores_1 = _mm256_add_pd(current_g_score, neighbor_distances_1);
        
        __m256d target_x = _mm256_set1_pd(target.x);
        __m256d target_y = _mm256_set1_pd(target.y);
        v2 neighbors[] =
        {
            {current.pos.x - 1, current.pos.y + 1}, {current.pos.x, current.pos.y + 1}, {current.pos.x + 1, current.pos.y + 1},
            {current.pos.x - 1, current.pos.y    },                                     {current.pos.x + 1, current.pos.y    },
            {current.pos.x - 1, current.pos.y - 1}, {current.pos.x, current.pos.y - 1}, {current.pos.x + 1, current.pos.y - 1}
        };
        __m256d neighbor_f_scores_0;
        {
            __m256d neighbors_x = _mm256_set_pd(neighbors[3].x, neighbors[2].x, neighbors[1].x, neighbors[0].x);
            __m256d neighbors_y = _mm256_set_pd(neighbors[3].y, neighbors[2].y, neighbors[1].y, neighbors[0].y);
            __m256d diff_x = _mm256_sub_pd(neighbors_x, target_x);
            __m256d diff_y = _mm256_sub_pd(neighbors_y, target_y);
            __m256d diff_x_sq = _mm256_mul_pd(diff_x, diff_x);
            __m256d diff_y_sq = _mm256_mul_pd(diff_y, diff_y);
            __m256d radicand = _mm256_add_pd(diff_x_sq, diff_y_sq);
            neighbor_f_scores_0 = _mm256_sqrt_pd(radicand);
            neighbor_f_scores_0 = _mm256_add_pd(neighbor_f_scores_0, neighbor_g_scores_0);
        }
        __m256d neighbor_f_scores_1;
        {
            __m256d neighbors_x = _mm256_set_pd(neighbors[7].x, neighbors[6].x, neighbors[5].x, neighbors[4].x);
            __m256d neighbors_y = _mm256_set_pd(neighbors[7].y, neighbors[6].y, neighbors[5].y, neighbors[4].y);
            __m256d diff_x = _mm256_sub_pd(neighbors_x, target_x);
            __m256d diff_y = _mm256_sub_pd(neighbors_y, target_y);
            __m256d diff_x_sq = _mm256_mul_pd(diff_x, diff_x);
            __m256d diff_y_sq = _mm256_mul_pd(diff_y, diff_y);
            __m256d radicand = _mm256_add_pd(diff_x_sq, diff_y_sq);
            neighbor_f_scores_1 = _mm256_sqrt_pd(radicand);
            neighbor_f_scores_1 = _mm256_add_pd(neighbor_f_scores_1, neighbor_g_scores_1);
        }

        // Serial part
        bool neighbor_checks[8] = {};
        for(int neighbor_index = 0; neighbor_index < 8; neighbor_index++)
        {
            const v2 &neighbor = neighbors[neighbor_index];
            bool neighbor_oob_cols = neighbor.x >= kCols;
            bool neighbor_oob_rows = neighbor.y >= kRows;
            if(neighbor_oob_cols || neighbor_oob_rows)
            {
                continue;
            }
            U64 wall_chunk_bits = pWalls[neighbor.index() / 64];
            U64 mask = 1ULL << (neighbor.index() % 64);
            bool neighbor_is_wall = wall_chunk_bits & mask;
            
            if(neighbor_is_wall)
            {
                continue;
            }
            neighbor_checks[neighbor_index] = true;
        }
        for(int neighbor_index = 0; neighbor_index < 4; neighbor_index++)
        {
            if(neighbor_checks[neighbor_index])
            {
                const v2 &neighbor = neighbors[neighbor_index];
                double new_g_score = neighbor_g_scores_0.m256d_f64[neighbor_index];
                if(new_g_score < g_scores[neighbor.index()])
                {
                    g_scores[neighbor.index()] = new_g_score;
                    open_list_push(open_list, {neighbor, neighbor_f_scores_0.m256d_f64[neighbor_index]});
                }
            }
        }
        for(int neighbor_index = 4; neighbor_index < 8; neighbor_index++)
        {
            if(neighbor_checks[neighbor_index])
            {
                const v2 &neighbor = neighbors[neighbor_index];
                double new_g_score = neighbor_g_scores_1.m256d_f64[neighbor_index - 4];
                if(new_g_score < g_scores[neighbor.index()])
                {
                    g_scores[neighbor.index()] = new_g_score;
                    open_list_push(open_list, {neighbor, neighbor_f_scores_1.m256d_f64[neighbor_index - 4]});
                }
            }
        }
#else
        // Calculate neighbor positions
        v2 neighbors[] =
        {
            {current.pos.x - 1, current.pos.y + 1}, {current.pos.x, current.pos.y + 1}, {current.pos.x + 1, current.pos.y + 1},
            {current.pos.x - 1, current.pos.y    },                                     {current.pos.x + 1, current.pos.y    },
            {current.pos.x - 1, current.pos.y - 1}, {current.pos.x, current.pos.y - 1}, {current.pos.x + 1, current.pos.y - 1}
        };
        // Calculate neighbor distances
        double neighbor_distances[] =
        {
            SQRT_2, 1.0, SQRT_2, 
            1.0,         1.0,
            SQRT_2, 1.0, SQRT_2
        };
        for(int neighbor_index = 0; neighbor_index < 8; neighbor_index++)
        {
            const v2 &neighbor = neighbors[neighbor_index];
            if((neighbor.x >= kCols) || (neighbor.y >= kRows))
            {
                continue;
            }
            U64 wall_chunk_bits = pWalls[neighbor.index() / 64];
            U64 mask = 1ULL << (neighbor.index() % 64);
            if(wall_chunk_bits & mask)
            {
                continue;
            }
            double g_score = g_scores[current.pos.index()] + neighbor_distances[neighbor_index];
            if(g_score < g_scores[neighbor.index()])
            {
                float f_score = g_score + heuristic(neighbor, target);
                g_scores[neighbor.index()] = g_score;
                open_list_push(open_list, {neighbor, f_score});
            }
        }
#endif
    }

    post_process();
    return -1.0;
}



