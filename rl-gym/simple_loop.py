#! /usr/bin/env python
import sys
import numpy
import rospy
import std_msgs
import time
import geometry_msgs.msg
import random
from std_srvs.srv import Empty

import gym
import rl_gym
import qlearn
import liveplot

def render():
    render_skip = 0 #Skip first X episodes.
    render_interval = 50 #Show render Every Y episodes.
    render_episodes = 10 #Show Z episodes every rendering.

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

if __name__ == '__main__':

    # Make and link the registered environment (unscrewing_env.py)
    env = gym.make('gym_unscrewing_env-v0')
    env._max_episode_steps = 200

    # Data logging and plotting parameters defined
    outdir = '/tmp/gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    plotter = liveplot.LivePlot(outdir)

    last_time_steps = numpy.ndarray(0)

    # Initialise qlearn and define hyperparameters
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.2, gamma=0.85, epsilon=0.9)

    # Epsilon setup:
    initial_epsilon = qlearn.epsilon
    epsilon_discount = 0.9986

    start_time = time.time()

    # Define total amount of episodes
    total_episodes = 1000

    highest_reward = 0

    #For loop for total episodes
    for x in range(total_episodes):
        done = False
        cumulated_reward = 0

        # Reset environment and get the current state after reset.
        observation = env.reset()

        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        #render() #defined above, not env.render()

        state = ''.join(map(str, observation))


        # Inner loop for total amount of actions per episode
        for i in range(200):

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward

            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            #Log action, state, reward, and the next state after action.
            qlearn.learn(state, action, reward, nextState)

            # Reset OpenAI environement before new action
            env._flush(force=True)

            if not(done):
                state = nextState
            else:
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

        #Plot rewards every two episodes
        if x % 2 == 0:
            plotter.plot(env)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))

    #Github table content
    print ("\n|"+str(total_episodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |")

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
    print "highest_reward: ", highest_reward

    env.close()