import numpy as np
from pogema import pogema_v0, GridConfig
from pogema.animation import AnimationMonitor
from pydantic import BaseModel
import cppimport.import_hook
from environment import environment

class MCTSConfig(BaseModel):
    gamma: float = 0.99
    num_actions: int = 5
    num_expansions: int = 1000
    uct_c: float = 1.0


class MonteCarloTreeSearch:
    def __init__(self, cfg: MCTSConfig):
        self.cfg = cfg
        self.root: Node = Node(parent=None, action=None, value=0, num_actions=cfg.num_actions, agent_idx=0, )

    def simulation(self, env):
        score = 0.0
        g = 1.0

        dones = [False, ]
        num_steps = 0

        while not env.all_done():
            reward = env.step(env.sample_actions(5))
            num_steps += 1
            score += reward * g
            g *= self.cfg.gamma

        for _ in range(num_steps):
            env.step_back()

        return score

    def uct(self, node, ):
        return node.q + self.cfg.uct_c * np.sqrt(2.0 * np.log(node.parent.cnt) / node.cnt)

    def expansion(self, node):
        scores = []
        for idx, c in enumerate(node.child_nodes):
            if c is None:
                return idx
            # scores.append(c.get_uct())
            scores.append(self.uct(c))
        return np.argmax(scores)

    def selection(self, node, env, actions):

        agent_idx = len(actions) % env.get_num_agents()
        next_agent_idx = (agent_idx + 1) % env.get_num_agents()
        if env.reached_goal(agent_idx):
        #if not env.grid.is_active[agent_idx]:
            action = 0
        else:
            action = self.expansion(node)

        if len(actions) == env.get_num_agents():
            reward = env.step(actions)

            if env.all_done():
                score = reward
            else:
                if node.child_nodes[action] is None:
                    score = reward + self.cfg.gamma * self.simulation(env)
                    node.child_nodes[action] = Node(parent=node, action=action, value=score,
                                                    num_actions=self.cfg.num_actions, agent_idx=next_agent_idx)
                else:
                    score = reward + self.cfg.gamma * self.selection(node.child_nodes[action], env, [action])

            node.update_value(score)

            env.step_back()
        else:
            if node.child_nodes[action] is None:
                node.child_nodes[action] = Node(parent=node, action=action, value=0,
                                                num_actions=self.cfg.num_actions, agent_idx=next_agent_idx)

            score = self.selection(node.child_nodes[action], env, actions + [action])
            node.update_value(score)

        return score * self.cfg.gamma

    def loop(self, env):
        for turn in range(self.cfg.num_expansions):
            score = self.selection(self.root, env, [], )
            self.root.update_value(score)

    def act(self, env, animate):
        # running main loop
        self.loop(env)

        actions = []
        env.render()
        for agent_idx in range(env.get_num_agents()):
            # if env.grid.is_active[agent_idx]:
            #     self.loop(env)
            print(agent_idx)
            print(self.root.q)
            for c, a_m in zip(self.root.child_nodes, ['S', 'U', 'D', 'L', 'R']):
                print(f'{a_m}:', end="")
                if c is None:
                    print(0, end=" ")
                else:
                    print(c.cnt, end=" ")
            print()
            for c, a_m in zip(self.root.child_nodes, ['S', 'U', 'D', 'L', 'R']):
                print(f'{a_m}:', end="")
                if c is None:
                    print(0, end=" ")
                else:
                    print(self.uct(c), end=" ")
            print('\n', '----' * 10)
            action = self.root.get_action()
            self.root = self.root.child_nodes[action]
            actions.append(action)
        reward = env.step(actions)
        animate.step(actions)
        return env.all_done()


class Node:
    def __init__(self, parent, action, value, num_actions, agent_idx=None):

        self.action = action
        self.parent: Node = parent
        self.cnt = 1
        self.w = value
        self.q = self.w / self.cnt
        self.child_nodes: list = [None for _ in range(num_actions)]
        self.agent_idx = agent_idx
        self.positions = None

    def update_value(self, value):
        self.w += value
        self.cnt += 1
        self.q = self.w / self.cnt

    def get_action(self):
        scores = []

        for c in self.child_nodes:
            if c is None:
                scores.append(-float('inf'))
            else:
                scores.append(c.cnt)

        return np.argmax(scores)


def main():
    gc = GridConfig(size=5, num_agents=2, seed=63, density=0.3, obs_radius=2, max_episode_steps=128)
    gc = GridConfig(size=4, num_agents=2, seed=62, density=0.4, obs_radius=2, max_episode_steps=32)
    gc = GridConfig(size=4, num_agents=3, seed=42, density=0.3, obs_radius=2, max_episode_steps=32)
    gc = GridConfig(size=4, num_agents=3, seed=42, density=0.0, obs_radius=2, max_episode_steps=32)
    gc = GridConfig(size=6, num_agents=5, seed=62, density=0.1, max_episode_steps=32)
    gc = GridConfig(size=12, num_agents=4, seed=62, density=0.3, obs_radius=2)
    # gc = GridConfig(map=""".BabA""", obs_radius=2, max_episode_steps=12)
    # gc = GridConfig(size=8, num_agents=3, seed=1, density=0.2, obs_radius=5, max_episode_steps=64)
    gc.persistent = True
    gc.collision_system = 'block_both'
    env = pogema_v0(gc)
    env = AnimationMonitor(env)
    env.reset()
    cpp_env = environment()
    for i in range(env.get_num_agents()):
       cpp_env.add_agent(env.grid.positions_xy[i][0], env.grid.positions_xy[i][1],
             env.grid.finishes_xy[i][0], env.grid.finishes_xy[i][1])
    cpp_env.create_grid(len(env.grid.obstacles),len(env.grid.obstacles[0]))
    for i in range(len(env.grid.obstacles)):
        for j in range(len(env.grid.obstacles[0])):
            if env.grid.obstacles[i][j]:
                cpp_env.add_obstacle(i, j)
    mcts = MonteCarloTreeSearch(MCTSConfig(num_actions=5))
    # env.render()
    while True:
        done = mcts.act(cpp_env, env)
        if done:
            break
    cpp_env.render()

if __name__ == '__main__':
    main()