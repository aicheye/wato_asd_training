#include "planner_core.hpp"
#include <algorithm>

bool PlannerCore::plan(const std::vector<int8_t> &grid, int width, int height,
                       const CellIndex &start, const CellIndex &goal,
                       std::vector<CellIndex> &out_path) {

    auto heuristic = [](const CellIndex &a, const CellIndex &b){ return std::hypot(a.x-b.x, a.y-b.y); };
    auto isFree = [&](const CellIndex &c){
        if(c.x<0 || c.x>=width || c.y<0 || c.y>=height) return false;
        int val = grid[c.y*width + c.x];
        return val >= 0 && val <= 90;
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    g_score[start] = 0.0;
    open_set.emplace(start, heuristic(start, goal));

    std::vector<CellIndex> dirs{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    while(!open_set.empty()){
        CellIndex current = open_set.top().idx; open_set.pop();
        if(current == goal){
            out_path.clear();
            while(came_from.find(current) != came_from.end()){
                out_path.push_back(current);
                current = came_from[current];
            }
            out_path.push_back(start);
            std::reverse(out_path.begin(), out_path.end());
            return true;
        }

        double current_g = g_score[current];
        for(auto d : dirs){
            CellIndex neighbor(current.x+d.x, current.y+d.y);
            if(!isFree(neighbor)) continue;

            double step_cost = (d.x!=0 && d.y!=0) ? std::sqrt(2.0) : 1.0;
            double penalty = grid[neighbor.y*width + neighbor.x]/25.0;
            double tentative_g = current_g + step_cost + penalty;

            if(g_score.find(neighbor)==g_score.end() || tentative_g < g_score[neighbor]){
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                open_set.emplace(neighbor, tentative_g + heuristic(neighbor, goal));
            }
        }
    }
    return false;
}