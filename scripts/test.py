#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped #4 Angle Data to 3
from sensor_msgs.msg import LaserScan #20 LAser Data
import time
from PPO import PPO, Memory
from PIL import Image
import torch

def call_Dist(laser):    
    print(laser)
    #print(data.pose.orientation.x)
    #print(c.avg_resultx, c.avg_resulty)

# def call_Pose(data):    
    #print("{}, {}, {}, {}".format(data.pose.orientation.x))
    
# def callback(data):    
#     print("{}, {}, {}, {}".format(data.pose.orientation.x))

def listener():  
    rospy.init_node('listener', anonymous=True)
    data = PoseStamped()
    laser = LaserScan()
    laser.ranges
    data.pose.orientation
    rospy.Subscriber("/scan", laser, call_Dist) 
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, call_Pose) 
    rospy.spin()

def test():
    ############## Hyperparameters ##############
    # env_name = "LunarLander-v3"
    # # creating environment
    # env = gym.make(env_name)
    # state_dim = env.observation_space.shape[0]
    # action_dim = 4
    # creating environment
    state_dim = 29
    action_dim = 2
    render = False
    max_timesteps = 300
    n_latent_var = 64           # number of variables in hidden layer
    lr = 0.0007
    betas = (0.9, 0.999)
    gamma = 0.99                # discount factor
    K_epochs = 4                # update policy for K epochs
    eps_clip = 0.2              # clip parameter for PPO
    #############################################

    n_episodes = 1
    max_timesteps = 300
    render = True
    save_gif = False

    filename = "PPO_{}.pth".format(env_name)
    directory = "./preTrained/"
    
    memory = Memory()
    ppo = PPO(state_dim, action_dim, n_latent_var, lr, betas, gamma, K_epochs, eps_clip)
    
    ppo.policy_old.load_state_dict(torch.load(directory+filename))
    
    for ep in range(1, n_episodes+1):
        ep_reward = 0
        state = env.reset()
        for t in range(max_timesteps):
            action = ppo.policy_old.act(state, memory)
            state, reward, done, _ = env.step(action)
            ep_reward += reward
            if render:
                env.render()
            if save_gif:
                 img = env.render(mode = 'rgb_array')
                 img = Image.fromarray(img)
                 img.save('./gif/{}.jpg'.format(t))  
            if done:
                break
            
        print('Episode: {}\tReward: {}'.format(ep, int(ep_reward)))
        ep_reward = 0
        env.close()
    
if __name__ == '__main__':
    listener()
    #test()
    
    
