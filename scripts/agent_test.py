import gym
from src.model.QLearningAgent import QLearningAgent
from tqdm import tqdm #used for loading bars

TOTAL_EPS = 10000
EP_MAX_STEPS = 50
EVAL_FREQ = 1000

GAMMA = 0.99
ALPHA = 0.5
EPSILON = 0.1

def train():
    env = gym.make("Taxi-v3")

    agent = QLearningAgent(
        action_space=env.action_space,
        obs_space=env.observation_space,
        gamma=GAMMA,
        alpha=ALPHA,
        epsilon=EPSILON,
    )

    step_counter = 0
    max_steps = TOTAL_EPS * EP_MAX_STEPS

    total_reward = 0
    ep_rewards = []

    for eps_num in tqdm(range(1, TOTAL_EPS+1)):
        obs = env.reset()
        episodic_return = 0
        t = 0 #timestep

        while t < EP_MAX_STEPS:
            act = agent.act(obs)
            n_obs, reward, done, _ = env.step(act)
            agent.learn(obs, act, reward, n_obs, done)

            t += 1
            step_counter += 1
            episodic_return += reward
            if done:
                break

            obs = n_obs

        total_reward += episodic_return
        ep_rewards.append(episodic_return)
    
    return total_reward, agent.q_table, ep_rewards


if __name__ == "__main__":
    total_reward, q_table, ep_rewards = train()
    print()
    print(f"Total reward over training: {total_reward}\n")
    import matplotlib.pyplot as plt
    plt.plot(ep_rewards)
    plt.show()