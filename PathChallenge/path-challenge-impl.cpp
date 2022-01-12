#include <vector>
#include <intrin.h>
#include <immintrin.h>

#include <deque>
#include <assert.h>

#include <math.h>
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

static const float SQRT_2 = 1.41421356f;
static const float SENTINEL = 99999999.0f;

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
    float f_score;
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

    float min_f_score = 100000.0;
    float max_f_score = 0.0;
    float sum_f_score = 0.0;
    U32 cnt_f_score = 0;
};
static PerfStats perf_stats;

//#define ABS(a) ((a < 0) ? (-a) : (a))
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
float heuristic(v2 current, v2 target)
{
#define OCTILE 0
#if OCTILE 
    int diff_x = current.x - target.x;
    diff_x = (diff_x < 0) ? -diff_x : diff_x;
    int diff_y = current.y - target.y;
    diff_y = (diff_y < 0) ? -diff_y : diff_y;
    float result = MIN(diff_x, diff_y) * SQRT_2 + MAX(diff_x, diff_y) - MIN(diff_x, diff_y);
    return result;
#else
    int diff_x = current.x - target.x;
    int diff_y = current.y - target.y;
    float result = sqrtf(diff_x*diff_x + diff_y*diff_y);
    return result;
#endif
}

#define BUCKETS 1
#if BUCKETS

#define MIN_POSSIBLE_F_SCORE 200.0f
struct OpenList
{
    struct Bucket
    {
        std::vector<Node> nodes;
    };

    std::vector<Bucket> buckets;
    U32 cheapest_bucket;
};

__declspec(noinline) void open_list_push(OpenList *open_list, Node node)
{
    U64 start_time = __rdtsc();

    float remapped_f_score = (node.f_score - MIN_POSSIBLE_F_SCORE) * 50.0f;
    U32 bucket_index = (U32)remapped_f_score;

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
    U64 start_time = __rdtsc();

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

    perf_stats.push_latencies.push_back(__rdtsc() - start_time);
}

Node open_list_pop(OpenList *open_list)
{
    U64 start_time = __rdtsc();

    Node current = open_list->nodes[0];
    U32 current_i = 0;
    for(U32 i = 1; i < open_list->nodes.size(); i++)
    {
        if(open_list->nodes[i].f_score < current.f_score)
        {
            current = open_list->nodes[i];
            current_i = i;
        }
    }
    // Remove current from the open list
    std::swap(open_list->nodes[current_i], open_list->nodes.back());
    open_list->nodes.pop_back();

    perf_stats.pop_latencies.push_back(__rdtsc() - start_time);
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
    static float g_scores[kCols * kRows] = {};
    static v2 came_from[kCols * kRows] = {};
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
        // I believe we need to use doubles here to match the expected precision of the path length.
        if(current.pos == target)
        {
            post_process();
            double dist = 0.0;
            while(current.pos != start)
            {
                v2 next = came_from[current.pos.index()];
                dist += ((current.pos.x == next.x) || (current.pos.y == next.y)) ? 1.0 : SQRT_2;
                current.pos = next;
            }
            return dist;
        }

#define WIDE 1
#if WIDE
        // Distances from the current node to it's neighbors
        __m256 neighbor_distances = _mm256_set_ps(SQRT_2, 1.0f, SQRT_2, 1.0f, 1.0f, SQRT_2, 1.0f, SQRT_2);

        // Current node and neighbor's g scores
        __m256 current_g_score = _mm256_set1_ps(g_scores[current.pos.index()]);
        __m256 neighbor_g_scores = _mm256_add_ps(current_g_score, neighbor_distances);
        
        // Neighbor coordinates
        __m256i neighbors_x = _mm256_set_epi32(current.pos.x + 1, current.pos.x, current.pos.x - 1, current.pos.x + 1, current.pos.x - 1, current.pos.x + 1, current.pos.x, current.pos.x - 1);
        __m256i neighbors_y = _mm256_set_epi32(current.pos.y - 1, current.pos.y - 1, current.pos.y - 1, current.pos.y, current.pos.y, current.pos.y + 1, current.pos.y + 1, current.pos.y + 1);

        // Create a mask for out of bounds and wall neighbors
        __m256i cols_wide = _mm256_set1_epi32(kCols);
        __m256i rows_wide = _mm256_set1_epi32(kRows);
        __m256i neighbors_oob_left = _mm256_cmpgt_epi32(neighbors_x, _mm256_set1_epi32(-1));
        __m256i neighbors_oob_right = _mm256_cmpgt_epi32(cols_wide, neighbors_x);
        __m256i neighbors_oob_bottom = _mm256_cmpgt_epi32(neighbors_y, _mm256_set1_epi32(-1));
        __m256i neighbors_oob_top = _mm256_cmpgt_epi32(rows_wide, neighbors_y);
        __m256i neighbors_valid_mask = _mm256_and_si256(neighbors_oob_left, neighbors_oob_right);
        neighbors_valid_mask = _mm256_and_si256(neighbors_valid_mask, neighbors_oob_top);
        neighbors_valid_mask = _mm256_and_si256(neighbors_valid_mask, neighbors_oob_bottom);
        // Calculate the neighbor indices
        __m256i neighbor_indices = _mm256_mullo_epi16(neighbors_y, cols_wide);
        neighbor_indices = _mm256_add_epi32(neighbor_indices, neighbors_x);


        // Mask against walls
        __m256i clamped_neighbor_indices = _mm256_min_epi32(_mm256_max_epi32(neighbor_indices, _mm256_set1_epi32(0)), _mm256_set1_epi32(kCells));
        __m256i wall_chunk_offset = _mm256_srli_epi32(clamped_neighbor_indices, 5);
        U32 *pWalls32 = (U32 *)pWalls;
        __m256i wall_chunk_bits = _mm256_set_epi32(
            pWalls32[wall_chunk_offset.m256i_i32[7]],
            pWalls32[wall_chunk_offset.m256i_i32[6]],
            pWalls32[wall_chunk_offset.m256i_i32[5]],
            pWalls32[wall_chunk_offset.m256i_i32[4]],
            pWalls32[wall_chunk_offset.m256i_i32[3]],
            pWalls32[wall_chunk_offset.m256i_i32[2]],
            pWalls32[wall_chunk_offset.m256i_i32[1]],
            pWalls32[wall_chunk_offset.m256i_i32[0]]
        );
        __m256i wall_mask_bit_offset =  _mm256_sub_epi32(neighbor_indices, _mm256_mullo_epi16(_mm256_set1_epi32(32), _mm256_srli_epi32(neighbor_indices, 5)));
        __m256i wall_mask =  _mm256_sllv_epi32(_mm256_set1_epi32(1), wall_mask_bit_offset);
        wall_mask = _mm256_and_si256(wall_chunk_bits, wall_mask);
        wall_mask = _mm256_cmpeq_epi32(wall_mask, _mm256_set1_epi32(0));

        neighbors_valid_mask = _mm256_and_si256(neighbors_valid_mask, wall_mask);

        // Calculate the neighbor's heuristic costs
        __m256 target_x = _mm256_set1_ps(target.x);
        __m256 target_y = _mm256_set1_ps(target.y);
        __m256 neighbors_xf = _mm256_cvtepi32_ps(neighbors_x);
        __m256 neighbors_yf = _mm256_cvtepi32_ps(neighbors_y);
        __m256 diff_x = _mm256_sub_ps(neighbors_xf, target_x);
        __m256 diff_y = _mm256_sub_ps(neighbors_yf, target_y);
        __m256 diff_x_sq = _mm256_mul_ps(diff_x, diff_x);
        __m256 diff_y_sq = _mm256_mul_ps(diff_y, diff_y);
        __m256 radicand = _mm256_add_ps(diff_x_sq, diff_y_sq);
        __m256 neighbor_f_scores = _mm256_sqrt_ps(radicand);
        neighbor_f_scores = _mm256_add_ps(neighbor_f_scores, neighbor_g_scores);

        // Serial part
        for(int neighbor_index = 0; neighbor_index < 8; neighbor_index++)
        {
            if(!neighbors_valid_mask.m256i_i32[neighbor_index])
            {
                continue;
            }

            float new_g_score = neighbor_g_scores.m256_f32[neighbor_index];
            U32 neighbor_i = neighbor_indices.m256i_i32[neighbor_index];
            if(new_g_score < g_scores[neighbor_i])
            {
                g_scores[neighbor_i] = new_g_score;
                came_from[neighbor_i] = current.pos;
                U32 neighbor_x = (U32)neighbors_x.m256i_i32[neighbor_index];
                U32 neighbor_y = (U32)neighbors_y.m256i_i32[neighbor_index];
                open_list_push(open_list, {v2{neighbor_x, neighbor_y}, neighbor_f_scores.m256_f32[neighbor_index]});
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
        float neighbor_distances[] =
        {
            SQRT_2, 1.0f, SQRT_2, 
            1.0f,         1.0f,
            SQRT_2, 1.0f, SQRT_2
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
            float g_score = g_scores[current.pos.index()] + neighbor_distances[neighbor_index];
            if(g_score < g_scores[neighbor.index()])
            {
                float f_score = g_score + heuristic(neighbor, target);
                g_scores[neighbor.index()] = g_score;
                came_from[neighbor.index()] = current.pos;
                open_list_push(open_list, {neighbor, f_score});
            }
        }
#endif
    }

    post_process();
    return -1.0;
}



