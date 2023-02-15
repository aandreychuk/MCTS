#include <iostream>
#include <list>
#include <vector>
#include <numeric>
#include <cmath>
#include <string>
#include <random>
#include <map>
#include "environment.h"
#include "config.h"
struct Node
{
    Node* parent;
    double w;
    std::vector<int> actions;
    int cnt;
    double q;
    std::map<std::vector<int>, Node*> child_nodes;
    Node(Node* _parent, double _w, std::vector<int> _actions)
            :parent(_parent), w(_w), actions(_actions)
    {
        cnt = 0;
        q = w;
    }
    void update_value(double value)
    {
        w += value;
        cnt++;
        q = w/cnt;
    }
    Node* get_best_child()
    {
        Node* best_child;
        int best_score(-1);
        for(auto c:child_nodes)
            if (c.second->cnt > best_score)
            {
                best_child = c.second;
                best_score = c.second->cnt;
            }
        return best_child;
    }
};

class MonteCarloTreeSearch
{
    Node* root;
    std::list<Node> all_nodes;
    Environment env;
    Config cfg;
    std::default_random_engine generator;

public:
    explicit MonteCarloTreeSearch(const std::string& fileName, int seed = -1):env(fileName, seed)
    {
        all_nodes.emplace_back(nullptr, 0, std::vector<int>(env.get_num_agents(), 0));
        root = &all_nodes.back();
    }
    double simulation()
    {
        double score(0), g(cfg.gamma), reward(0);
        int num_steps(0);
        while(!env.all_done() && num_steps < cfg.steps_limit)
        {
            reward = env.step(env.sample_actions(cfg.num_actions));
            num_steps++;
            score += reward*g;
            g *= cfg.gamma;
        }
        //std::cout<<score<<" ";
        //int md = env.get_manhattan_distance();
        for(int i = 0; i < num_steps; i++)
            env.step_back();
        //score += std::fmax(0, (env.get_manhattan_distance() - md)/100);
        return score;
    }
    double uct(Node* n) const
    {
        return n->q + cfg.uct_c*std::sqrt(2.0*std::log(n->parent->cnt)/n->cnt);
    }
    Node* expansion(Node* n)
    {
        Node* child=nullptr;
        double best_score(-1);
        for(auto c:n->child_nodes) {
            if (uct(c.second) > best_score) {
                child = c.second;
                best_score = uct(c.second);
            }
        }
        std::uniform_int_distribution<int> distribution(0, 5);
        int number = distribution(generator);
        if(number == 0 && n->child_nodes.size() < pow(5, env.get_num_agents()))
           child = nullptr;
        return child;
    }
    void selection(Node* n)
    {
        std::list<Node*> path;
        path.clear();
        Node* child = nullptr;
        while(true)
        {
            child = expansion(n);
            if(child == nullptr)
                break;
            n = child;
            path.push_back(child);
        }
        for(auto p:path)
            env.step(p->actions);
        auto actions = env.sample_actions(cfg.num_actions);
        int tries = 0;
        while(n->child_nodes.find(actions) != n->child_nodes.end() && tries < 100)
        {
            actions = env.sample_actions(cfg.num_actions);
            tries++;
        }
        if(n->child_nodes.find(actions) == n->child_nodes.end())
        {
            all_nodes.emplace_back(n, 0, actions);
            n->child_nodes[actions] = &all_nodes.back();
            child = n->child_nodes[actions];
            path.push_back(child);
            double reward = env.step(actions);
            backprop(reward, path.back());
        }
        for(auto p:path)
            env.step_back();
    }

    void backprop(double reward, Node* node)
    {
        double score = reward + simulation();
        while(node != nullptr)
        {
            node->update_value(score);
            score *= cfg.gamma;
            node = node->parent;
        }
    }
    void loop()
    {
        for (int i = 0; i < cfg.num_expansions; i++)
            selection(root);
        return;
    }

    bool act()
    {
        loop();
        for(auto ch:root->child_nodes)
            std::cout<<ch.second->q<<" ";
        std::cout<<"\n";
        env.render();
        root = root->get_best_child();
        for(auto a:root->actions)
            std::cout<<a<<" ";
        std::cout<<" - actions\n";
        for(auto p:env.cur_positions)
            std::cout<<p.first<<" "<<p.second<<"  ";
        std::cout<<" - positions\n";
        for(auto p:env.goals)
            std::cout<<p.first<<" "<<p.second<<"  ";
        std::cout<<" - goals\n";
        env.step(root->actions);
        env.terminate_agents();
        if(env.all_done())
        {
            std::cout<<"ALL DONE\n";
            env.render();
        }
        return env.all_done();
    }
};
