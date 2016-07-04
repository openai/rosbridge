import logging, os, sys
import gym
import numpy as np
import cPickle as pickle

class ZeroAgent(object):
    def __init__(self, action_space):
        self.action_space = action_space

    def act(self, observation, reward, done):
        return np.zeros(self.action_space.shape)

class ConstantAgent(object):
    def __init__(self, action_space, action):
        self.action_space = action_space
        self.action = np.array(action)

    def act(self, observation, reward, done):
        return (self.action,)

class RandomAgent(object):
    def __init__(self, action_space):
        self.action_space = action_space
        self.cached_actions = []

    def act(self, observation, reward, done):
        if len(self.cached_actions) == 0:
            a = self.action_space.sample()
            for i in range(3):
                self.cached_actions.append(a)
        return self.cached_actions.pop()

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    env = gym.make('FetchRobotRGB-v0' if len(sys.argv)<2 else sys.argv[1])

    outdir = '/tmp/logobs'
    os.system('rm -rf /tmp/logobs')
    os.mkdir(outdir)

    action = None
    episode_count = 1
    max_steps = 20
    if len(sys.argv) > 2:
        episode_count = int(sys.argv[2])
    if len(sys.argv) > 3:
        max_steps = int(sys.argv[3])
    if len(sys.argv) > 4:
        action = [float(x) for x in sys.argv[4].split(',')]

    if action is not None:
        agent = ConstantAgent(env.action_space, action)
    elif 1:
        agent = RandomAgent(env.action_space)
    else:
        agent = ZeroAgent(env.action_space)

    reward = 0
    done = False

    for i in range(episode_count):
        ob = env.reset()

        allobs = [ob]
        allactions = []
        allrewards = []
        for j in range(max_steps):
            action = agent.act(ob, reward, done)
            ob, reward, done, _ = env.step(action)
            allobs.append(ob)
            allactions.append(action)
            allrewards.append(reward)
            if done:
                break
        obsf = open(os.path.join(outdir, 'ep%d.pickle' % i), 'wb')
        pickle.dump({
            'observations': allobs,
            'actions': allactions,
            'observation_space': env.observation_space,
            }, obsf)
        obsf.close()

    env.close()
