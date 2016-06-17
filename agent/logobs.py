import logging, os, sys
import gym
import numpy as np
import cPickle as pickle

# The world's simplest agent!
class LoggingAgent(object):
    def __init__(self, action_space):
        self.action_space = action_space

    def act(self, observation, reward, done):
        return np.zeros(self.action_space.shape)

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    env = gym.make('FetchRobot-v0' if len(sys.argv)<2 else sys.argv[1])

    # You provide the directory to write to (can be an existing
    # directory, including one with existing data -- all monitor files
    # will be namespaced). You can also dump to a tempdir if you'd
    # like: tempfile.mkdtemp().
    outdir = '/tmp/logobs'
    os.mkdir(outdir)

    # This declaration must go *after* the monitor call, since the
    # monitor's seeding creates a new action_space instance with the
    # appropriate pseudorandom number generator.
    agent = LoggingAgent(env.action_space)

    episode_count = 1
    max_steps = 5
    reward = 0
    done = False

    for i in range(episode_count):
        ob = env.reset()

        allobs = []
        for j in range(max_steps):
            action = agent.act(ob, reward, done)
            obs, reward, done, _ = env.step(action)
            allobs.append(obs)
            if done:
                break
        obsf = open(os.path.join(outdir, 'ep%d.pickle' % i), 'wb')
        pickle.dump(allobs, obsf)
        obsf.close()
