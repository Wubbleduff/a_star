/*******************************************************************
*
*    Author: Kareem Omar
*    kareem.h.omar@gmail.com
*    https://github.com/komrad36
*
*    Last updated Dec 15, 2021
*******************************************************************/

#include <vector>

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

#define ABS(a) ((a) < 0) ? -a : a
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
double heuristic(v2 a, v2 b)
{
    int xDiff = ABS(a.x - b.x);
    int yDiff = ABS(a.y - b.y);
    return MIN(xDiff, yDiff) * SQRT_2 + MAX(xDiff, yDiff) - MIN(xDiff, yDiff);
}

bool is_wall(v2 node, const U64 *walls)
{
    // Find the chunk of bits that contains the wall and mask to find the wall.
    U64 wall_chunk_bits = walls[node.index() / 64];
    U64 mask = 1ULL << (node.index() % 64);
    return wall_chunk_bits & mask;
}

#if 1

void print_world(const U64 *walls, const double *g_scores)
{
    printf("\n\n\n");

    printf("     ");
    for(U32 c = 0; c < kCols; c++)
    {
        printf("|");
    }
    printf("\n");

    for(U32 r = 0; r < kRows; r++)
    {
        printf("%4i ", r);
        for(U32 c = 0; c < kCols; c++)
        {
            U32 i = v2{c, r}.index();
            if(i == kStartI)
            {
                printf("S");
            }
            else if(i == kEndI)
            {
                printf("E");
            }
            else if(is_wall({c, r}, walls))
            {
                printf("W");
            }
            else if(g_scores[i] == 42424242.0)
            {
                printf("O");
            }
            else if(g_scores[i] == SENTINEL)
            {
                printf(" ");
            }
            else
            {
                printf(".");
            }
        }
        printf("\n");
    }
    printf("\n\n\n");
}

void verify(const U64 *walls, double *g_scores)
{
    //print_world(walls, g_scores);
}


#endif

/// your code goes here!
float FastPathFind(const U64* pWalls)
{
    // Initialize grid
    double g_scores[kCols * kRows] = {};
    double f_scores[kCols * kRows] = {};
    v2 came_from[kCols * kRows] = {};
    for(U32 r = 0; r < kRows; r++)
    {
        for(U32 c = 0; c < kCols; c++)
        {
            U32 i = v2{c, r}.index();
            g_scores[i] = SENTINEL;
            f_scores[i] = SENTINEL;
            came_from[i] = v2{(U32)-1, (U32)-1};
        }
    }

    v2 start = {kStartX, kStartY};
    v2 target = {kEndX, kEndY};
    std::vector<v2> open_list = { {kStartX, kStartY} };
    g_scores[start.index()] = 0.0f;
    f_scores[start.index()] = heuristic(open_list[0], target);

    while(!open_list.empty())
    {
        // Find the cheapest node on the open list
        v2 current = open_list[0];
        for(const v2 &node : open_list)
        {
            if(f_scores[node.index()] < f_scores[current.index()])
            {
                current = node;
            }
        }
        // Remove current from the open list
        for(v2 &node : open_list) if(node == current) { std::swap(node, open_list.back()); break; }
        open_list.pop_back();

        // Check if done
        if(current == target)
        {
            return g_scores[current.index()];
        }

        for(const v2 neighbor : std::vector<v2>
            {{current.x - 1, current.y + 1}, {current.x    , current.y + 1}, {current.x + 1, current.y + 1},
             {current.x - 1, current.y    },                                 {current.x + 1, current.y    },
             {current.x - 1, current.y - 1}, {current.x    , current.y - 1}, {current.x + 1, current.y - 1}})
        {
            if(neighbor.x >= kCols || neighbor.y >= kRows || is_wall(neighbor, pWalls))
            {
                continue;
            }

            double neighbor_distance = ((neighbor.x == current.x) || (neighbor.y == current.y)) ? 1.0f : SQRT_2;
            double neighbor_new_g_score = g_scores[current.index()] + neighbor_distance;

            if(neighbor_new_g_score < g_scores[neighbor.index()])
            {
                g_scores[neighbor.index()] = neighbor_new_g_score;
                f_scores[neighbor.index()] = neighbor_new_g_score + heuristic(neighbor, target);
                came_from[neighbor.index()] = current;

                bool found_on_open_list = false;
                for(v2 &node : open_list)
                {
                    if(node == neighbor)
                    {
                        found_on_open_list = true;
                        break;
                    }
                }
                if(!found_on_open_list)
                {
                    open_list.push_back(neighbor);
                }
            }
        }
    }

    verify(pWalls, g_scores);
    return -1.0;
}



