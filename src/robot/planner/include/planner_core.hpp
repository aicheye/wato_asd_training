#ifndef PLANNER_CORE_HPP
#define PLANNER_CORE_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

#pragma once
struct CellIndex {
    int x, y;
    CellIndex(int xx=0, int yy=0) : x(xx), y(yy) {}
    bool operator==(const CellIndex &other) const { return x==other.x && y==other.y; }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y)<<1);
    }
};

struct AStarNode {
    CellIndex idx; double f;
    AStarNode(CellIndex i, double f_score) : idx(i), f(f_score) {}
};

struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b){ return a.f > b.f; }
};

class PlannerCore {
public:
    PlannerCore() = default;
    bool plan(const std::vector<int8_t> &grid, int width, int height,
              const CellIndex &start, const CellIndex &goal,
              std::vector<CellIndex> &out_path);
};
#endif