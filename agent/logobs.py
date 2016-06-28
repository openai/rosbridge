import logging, os, sys
import gym
import numpy as np
import cPickle as pickle

class LoggingAgent(object):
    def __init__(self, action_space):
        self.action_space = action_space

    def act(self, observation, reward, done):
        return self.action_space.sample()
        #return np.zeros(self.action_space.shape)

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    env = gym.make('FetchRobotRGB-v0' if len(sys.argv)<2 else sys.argv[1])

    outdir = '/tmp/logobs'
    os.mkdir(outdir)

    agent = LoggingAgent(env.action_space)

    episode_count = 1
    max_steps = 100
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
            'actions': allactions
            'observation_space': env.observation_space,
            }, obsf)
        obsf.close()
