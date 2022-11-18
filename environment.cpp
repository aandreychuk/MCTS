// cppimport
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <vector>
#include <iostream>
#include <random>
#include <chrono>
#include <string>
#define OBSTACLE 1
#define TRAVERSABLE 0
namespace py = pybind11;

class environment
{
    int num_agents;
    std::vector<std::pair<int, int>> moves = {{0,0}, {-1, 0}, {1,0},{0,-1},{0,1}};
    std::vector<std::vector<int>> grid;
    std::vector<std::pair<int, int>> goals;
    std::vector<std::pair<int, int>> cur_positions;
    std::vector<std::vector<int>> made_actions;
    std::vector<bool> reached;
    std::default_random_engine engine;
public:
    explicit environment()
    {
        num_agents = 0;
    }
    void set_seed(int seed=-1)
    {
        if(seed < 0)
            engine.seed(std::chrono::system_clock::now().time_since_epoch().count());
        else
            engine.seed(seed);
    }
    void add_agent(int si, int sj, int gi, int gj)
    {
        cur_positions.push_back({si, sj});
        goals.push_back({gi, gj});
        num_agents++;
        reached.push_back(false);
    }
    void create_grid(int height, int width)
    {
        grid = std::vector<std::vector<int>>(height, std::vector<int>(width,TRAVERSABLE));
    }
    void add_obstacle(int i, int j)
    {
        grid[i][j] = OBSTACLE;
    }
    int get_num_agents()
    {
        return num_agents;
    }
    bool reached_goal(int i)
    {
        if(i >= 0 && i < num_agents)
            return reached[i];
        else
            return false;
    }
    double step(std::vector<int> actions)
    {
        std::vector<std::pair<int, int>> executed_pos;
        for(int i = 0; i < num_agents; i++) {
            if (reached[i])
            {
                executed_pos.push_back(cur_positions[i]);
                actions[i] = 0;
            }
            else
                executed_pos.emplace_back(cur_positions[i].first + moves[actions[i]].first,
                                          cur_positions[i].second + moves[actions[i]].second);
        }
        for(int i = 0; i < num_agents; i++)
            for(int j = i+1; j < num_agents; j++) {
                if (reached[i] || reached[j])
                    continue;
                if ((executed_pos[i].first == executed_pos[j].first &&
                     executed_pos[i].second == executed_pos[j].second) ||
                    (executed_pos[i].first == cur_positions[j].first &&
                     executed_pos[i].second == cur_positions[j].second))
                {
                    executed_pos[i] = cur_positions[i];
                    executed_pos[j] = cur_positions[j];
                    actions[i] = 0;
                    actions[j] = 0;
                }
            }
        double reward(0);
        for(int i = 0; i < num_agents; i++)
            if(executed_pos[i].first < 0 || executed_pos[i].first >= grid.size() ||
               executed_pos[i].second < 0 || executed_pos[i].second >= grid[0].size()
               || grid[executed_pos[i].first][executed_pos[i].second])
            {
                executed_pos[i] = cur_positions[i];
                actions[i] = 0;
            }
        for(int i = 0; i < num_agents; i++) {
            if (reached[i])
                continue;
            if(executed_pos[i].first == goals[i].first && executed_pos[i].second == goals[i].second)
            {
                reward += 1;
                reached[i] = true;
            }
        }
        made_actions.push_back(actions);
        cur_positions = executed_pos;
        return reward;
    }
    void step_back()
    {
        for(int i = 0; i < num_agents; i++)
        {
            cur_positions[i].first = cur_positions[i].first - moves[made_actions.back()[i]].first;
            cur_positions[i].second = cur_positions[i].second - moves[made_actions.back()[i]].second;
            if(cur_positions[i].first != goals[i].first || cur_positions[i].second != goals[i].second)
                reached[i] = false;
        }
        made_actions.pop_back();
    }
    std::vector<int> sample_actions(int num_actions)
    {
        bool use_move_limits = true;//TODO - retrun to parameters
        bool agents_as_obstacles = false;//TODO - return to parameters
        std::vector<int> actions;
        for(int i = 0; i < num_agents; i++)
        {
            auto action = engine() % num_actions;
            if (use_move_limits)
            {
                while (!check_action(i, action, agents_as_obstacles))
                    action = engine() % num_actions;
            }
            actions.emplace_back(action);
        }
        return actions;
    }
    bool all_done()
    {
        return num_agents == std::accumulate(reached.begin(), reached.end(), 0);
    }
    void render()
    {
        for(int i = 0; i < num_agents; i++) {
            auto c1 = cur_positions[i], c2 = goals[i];
            if(c1.first != c2.first || c1.second != c2.second)
            {
                grid[c1.first][c1.second] = i + 2;
                grid[c2.first][c2.second] = i + 2 + num_agents;
            }
        }
        for(int i = 0; i < grid.size(); i++) {
            for (int j = 0; j < grid[0].size(); j++) {
                if (grid[i][j] == 0)
                    std::cout << " . ";
                else if (grid[i][j] == 1)
                    std::cout << " # ";
                else {
                    if (grid[i][j] > num_agents + 1)
                        std::cout << "|" << grid[i][j] - 2 - num_agents << "|";
                    else
                        std::cout << " " << grid[i][j] - 2 << " ";
                    grid[i][j] = 0;
                }
            }
            std::cout<<std::endl;
        }
    };
    bool check_action(const int agent_idx, const int action, const bool agents_as_obstacles) const
    {
        const std::pair<int, int> future_position = {cur_positions[agent_idx].first + moves[action].first, cur_positions[agent_idx].second + moves[action].second};
        if (future_position.first < 0 || future_position.second < 0 || future_position.first >= grid.size() || future_position.second >= grid.size())
            return false;
        if (grid[future_position.first][future_position.second] == 1)
            return false;
        if (agents_as_obstacles)
        {
            for (int i = 0; i < num_agents; i++)
            {
                if (i != agent_idx)
                {
                    if((cur_positions[i].first == future_position.first) && (cur_positions[i].second == future_position.second))
                        return false;
                }
            }
        }
        return true;
    }
};

PYBIND11_MODULE(environment, m) {
    py::class_<environment>(m, "environment")
            .def(py::init<>())
            .def("all_done", &environment::all_done)
            .def("sample_actions", &environment::sample_actions)
            .def("step", &environment::step)
            .def("step_back", &environment::step_back)
            .def("set_seed", &environment::set_seed)
            .def("create_grid", &environment::create_grid)
            .def("add_obstacle", &environment::add_obstacle)
            .def("add_agent", &environment::add_agent)
            .def("render", &environment::render)
            .def("get_num_agents", &environment::get_num_agents)
            .def("reached_goal", &environment::reached_goal)
            ;
}

/*
<%
setup_pybind11(cfg)
%>
*/
