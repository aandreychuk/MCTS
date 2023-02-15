#ifndef MCTS_CONFIG_H
#define MCTS_CONFIG_H

struct Config
{
    double gamma = 0.999;
    int num_actions = 5;
    int num_expansions = 1000;
    double uct_c = 1.0;
    int steps_limit = 128;
};

#endif //MCTS_CONFIG_H
