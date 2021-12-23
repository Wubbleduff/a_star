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

static const double SQRT_2 = 1.4142135623730951;

struct v2
{
    U32 x, y;

    bool operator==(const v2 &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct Node
{
    v2 pos;
    double f_score;
};

double heuristic(v2 a, v2 b)
{
    return (b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y);
    // int dx = a.x - b.x;
    // int dy = a.y - b.y;
    // dx = dx < 0.0 ? -dx : dx;
    // dy = dy < 0.0 ? -dy : dy;
    // return (dx + dy) + (SQRT_2 - 2 * 1) * ((dx < dy) ? dx : dy);
}

/*
(3, 1)

node_i = 13
chunk_i = 1
wall_bits = 0 0 0 0 0 1 0 0 
mask =      0 0 0 0 0 1 0 0 

  0 1 2 3 4 5 6 7 8 9
0 _ _ _ _ _ _ _ _ _ _
1 _ _ _ X _ _ _ _ _ _
2 _ _ _ _ _ _ _ _ _ _
3 _ _ _ _ _ _ _ _ _ _
4 _ _ _ _ _ _ _ _ _ _
5 _ _ _ _ _ _ _ _ _ _
6 _ _ _ _ _ _ _ _ _ _
7 _ _ _ _ _ _ _ _ _ _
*/
bool is_wall(v2 node, const U64 *walls)
{
    int bit_index = node.y * kRows + node.x;
    U64 wall_chunk_bits = walls[bit_index / 64];
    U64 mask_pos = (63 - (bit_index % 64));
    U64 mask = 1ULL << mask_pos;
    return wall_chunk_bits & mask;
}

void find_neighbors(v2 node, const U64 *walls, v2 out[8], double out_distances[8], int *num_out)
{
    v2 tl = {node.x - 1, node.y + 1};
    v2 t  = {node.x,     node.y + 1};
    v2 tr = {node.x + 1, node.y + 1};
    v2 ml = {node.x - 1, node.y};
    v2 mr = {node.x + 1, node.y};
    v2 bl = {node.x - 1, node.y - 1};
    v2 b  = {node.x,     node.y - 1};
    v2 br = {node.x + 1, node.y - 1};

    if(node.x >= 1 && node.y <= kRows - 1 && !is_wall(tl, walls))
    {
        out_distances[*num_out] = SQRT_2;
        out[(*num_out)++] = tl;
    }
    if(node.y <= kRows - 1 && !is_wall(t, walls))
    {
        out_distances[*num_out] = 1.0f;
        out[(*num_out)++] = t;
    }
    if(node.x <= kCols - 1 && node.y <= kRows - 1 && !is_wall(tr, walls))
    {
        out_distances[*num_out] = SQRT_2;
        out[(*num_out)++] = tr;
    }
    if(node.x >= 1 && !is_wall(ml, walls))
    {
        out_distances[*num_out] = 1.0f;
        out[(*num_out)++] = ml;
    }
    if(node.x <= kCols - 1 && !is_wall(mr, walls))
    {
        out_distances[*num_out] = 1.0f;
        out[(*num_out)++] = mr;
    }
    if(node.x >= 1 && node.y >= 1 && !is_wall(bl, walls))
    {
        out_distances[*num_out] = SQRT_2;
        out[(*num_out)++] = bl;
    }
    if(node.y >= 1 && !is_wall(b, walls))
    {
        out_distances[*num_out] = 1.0f;
        out[(*num_out)++] = b;
    }
    if(node.x <= kCols - 1 && node.y >= 1 && !is_wall(br, walls))
    {
        out_distances[*num_out] = SQRT_2;
        out[(*num_out)++] = br;
    }
}

#if 1

void print_walls(const U64 *walls)
{
    printf("\n\n\n");
    int r = 0;
    int c = 0;
    while(true)
    {
        U64 bit_index = r * kRows + c;
        U64 wall_bits = walls[bit_index / 64];
        U64 mask_asdf = (63 - (bit_index % 64));
        U64 mask = (1ULL << mask_asdf);
        if(wall_bits != 0)
        {
            int asdf=0;
            asdf++;
        }
        bool bit_set = wall_bits & mask;
        if(r == kStartY && c == kStartX)
        {
            printf("O");
        }
        else if(r == kEndY && c == kEndX)
        {
            printf("O");
        }
        else if(bit_set)
        {
            printf(".");
        }
        else
        {
            printf(" ");
        }

        c++;
        if(c >= kCols)
        {
            c = 0;
            r++;
            printf("\n");
        }
        if(r >= kRows)
        {
            break;
        }
    }
}

void verify_walls(const U64 *walls, double *g_scores)
{
    printf("\n\n\n");
    for(int r = 0; r < kRows; r++)
    {
        for(int c = 0; c < kCols; c++)
        {
            U64 bit_index = r * kRows + c;
            U64 wall_bits = walls[bit_index / 64];
            U64 mask_asdf = (63 - (bit_index % 64));
            U64 mask = (1ULL << mask_asdf);
            bool bit_set = wall_bits & mask;
            if(bit_set)
            {
                if(g_scores[bit_index] != INFINITY)
                {
                    printf("BADDDDDDDDDDDDDDDDDDD");
                }
            }
        }
    }
}

#endif

/// your code goes here!
float FastPathFind(const U64* pWalls)
{
    // while not at the end
    //     add neighbors
    //         guess distance to the end
    //         if g score is greater than current, update it
    //     move to the next cheapest node on the open list

    //print_walls(pWalls);

    // Initialize grid
    static double g_scores[kRows][kCols] = {};
    for(int r = 0; r < kRows; r++)
    {
        for(int c = 0; c < kCols; c++)
        {
            g_scores[r][c] = INFINITY;
        }
    }

    v2 target = {kEndX, kEndY};
    std::vector<Node> open_list = { {{kStartX, kStartY}, 0.0f } };
    g_scores[kStartY][kStartX] = 0.0f;

    while(!open_list.empty())
    {
        // Find the cheapest node on the open list
        Node current = open_list[0];
        for(const Node &node : open_list)
        {
            if(node.f_score < current.f_score)
            {
                current = node;
            }
        }
        // Remove current from the open list
        for(Node &node : open_list)
        {
            if(node.pos == current.pos)
            {
                std::swap(node, open_list.back());
                break;
            }
        }
        open_list.pop_back();

        int num_neighbors = 0;
        v2 neighbors[8];
        double neighbor_distances[8];
        find_neighbors(current.pos, pWalls, neighbors, neighbor_distances, &num_neighbors);
        for(int n = 0; n < num_neighbors; n++)
        {
            v2 neighbor = neighbors[n];
            double neighbor_distsance = neighbor_distances[n];

            double neighbor_new_g_score = g_scores[current.pos.y][current.pos.x] + neighbor_distsance;

            // Check if done
            if(neighbor == target)
            {
                // Done
                verify_walls(pWalls, g_scores[0]);
                return neighbor_new_g_score;
            }

            if(neighbor_new_g_score < g_scores[neighbor.y][neighbor.x])
            {
                g_scores[neighbor.y][neighbor.x] = neighbor_new_g_score;
                double f_score = neighbor_new_g_score + heuristic(neighbor, target);

                bool found_on_open_list = false;
                for(const Node &node : open_list)
                {
                    if(node.pos == neighbor)
                    {
                        found_on_open_list = true;
                        break;
                    }
                }
                if(!found_on_open_list)
                {
                    open_list.push_back({neighbor, f_score});
                }
            }
        }
    }

    verify_walls(pWalls, g_scores[0]);
    return -1.0;
}



