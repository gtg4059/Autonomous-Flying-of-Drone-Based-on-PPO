#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped #4 Angle Data to 3
from sensor_msgs.msg import LaserScan, Imu #20 LAser Data
import torch
import torch.nn as nn
from torch.distributions import MultivariateNormal
from math import *
#import gym
import numpy as np
from squaternion import Quaternion
#from torch.utils.tensorboard import SummaryWriter

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []

    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]


class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std):
        super(ActorCritic, self).__init__()
        # action mean range -1 to 1
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, action_dim),
            nn.Tanh()
        )
        # critic
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, 1)
        )
        self.action_var = torch.full((action_dim,), action_std * action_std).to(device)

    def forward(self):
        raise NotImplementedError

    def act(self, state, memory):
        action_mean = self.actor(state)
        cov_mat = torch.diag(self.action_var).cpu()

        dist = MultivariateNormal(action_mean, cov_mat)
        action = dist.sample()
        action_logprob = dist.log_prob(action)

        memory.states.append(state)
        memory.actions.append(action)
        memory.logprobs.append(action_logprob)

        return action.detach()

    def evaluate(self, state, action):
        action_mean = self.actor(state)

        action_var = self.action_var.expand_as(action_mean)
        cov_mat = torch.diag_embed(action_var).cpu()

        dist = MultivariateNormal(action_mean, cov_mat)

        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()
        state_value = self.critic(state)

        return action_logprobs, torch.squeeze(state_value), dist_entropy


class PPO:
    def __init__(self, state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip):
        self.lr = lr
        self.betas = betas
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs

        self.policy = ActorCritic(state_dim, action_dim, action_std).to(device)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr, betas=betas)

        self.policy_old = ActorCritic(state_dim, action_dim, action_std).to(device)
        self.policy_old.load_state_dict(self.policy.state_dict())

        self.MseLoss = nn.MSELoss()

    def select_action(self, state, memory):
        state = torch.FloatTensor(state.reshape(1, -1)).to(device)
        return self.policy_old.act(state, memory).cpu().data.numpy().flatten()

    def update(self, memory):
        # Monte Carlo estimate of rewards:
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        # Normalizing the rewards:
        rewards = torch.tensor(rewards).to(device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)

        # convert list to tensor
        old_states = torch.squeeze(torch.stack(memory.states).to(device), 1).detach()
        old_actions = torch.squeeze(torch.stack(memory.actions).to(device), 1).detach()
        old_logprobs = torch.squeeze(torch.stack(memory.logprobs), 1).to(device).detach()

        # Optimize policy for K epochs:
        for _ in range(self.K_epochs):
            # Evaluating old actions and values :
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)

            # Finding the ratio (pi_theta / pi_theta__old):
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss:
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy

            # take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()

        # Copy new weights into old policy:
        self.policy_old.load_state_dict(self.policy.state_dict())


class Node():
    def __init__(self):
        # Params
        self.LaserData=np.zeros(12) #12
        self.TargetPolar = 0
        self.UWBPos=(0,0) #2
        self.RPY=[0,0,0]
        self.Dir=[0,0]
        self.Mag=0
        self.TargetDist=0
        self.TargetPos = [1.6,3.6]
        # Node cycle rate (in Hz).
        loop_rate = rospy.Rate(1000)
        string = String()
        laser = LaserScan()
        imu = Imu()
        Posedata = PoseStamped() 
        Veldata = TwistStamped()
        # Publishers
        self.pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=None)
        
        # Subscribers
        rospy.Subscriber("/UWBPosition", String, self.callback_Pos) 
        rospy.Subscriber("/scan", LaserScan, self.callback_range) #RayinformContain
        rospy.Subscriber("/mavros/imu/data", Imu, self.callback_RPY) #CrntAngle
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.callback_Vel) #CrntDir

    def callback_Pos(self,string):   
        #self.str = ""
        self.UWBPos=tuple(map(float, string.data.split(',')))
        targetdir = np.array([self.TargetPos[0]-self.UWBPos[0],self.TargetPos[1]-self.UWBPos[1]])
        self.TargetDist = np.linalg.norm(targetdir)/(1+abs(np.linalg.norm(targetdir)))
        self.TargetPolar = (atan2(self.Dir[1],self.Dir[0]) - atan2(targetdir[1],targetdir[0]))/pi
        #print("targetdir:",targetdir,"Dir:",self.Dir)
        #print("UWB:",self.UWBPos,"TPOLAR:",self.TargetPolar,"TDist:",self.TargetDist)
    def callback_Vel(self, Veldata):
        n = np.array([Veldata.twist.linear.x, Veldata.twist.linear.y])
        self.Mag = np.linalg.norm(n)
        self.Dir = n/self.Mag
        #print("MAG:",self.Mag,"DIR:",self.Dir)
    def callback_range(self, laser):
        for i in range(12):
            self.LaserData[i] = laser.ranges[i*20]
        #print("Laser:",tuple(self.LaserData))    
    def callback_RPY(self, imu): 
        q = Quaternion(imu.orientation.w,imu.orientation.x,\
            imu.orientation.y,imu.orientation.z)
        e = q.to_euler(degrees=True)
        self.RPY = [e[0]/180, e[1]/180, e[2]/180]
        #self.Pos=(eq[0],eq[1],eq[2])*pi/180
        #print("Pose:",self.RPY) 

    # def Start(self): 
    #     P = PoseStamped() 
    #     q = Quaternion.from_euler(action, 5, 0, degrees=True)
    #     P.pose.orientation.x = q.x
    #     P.pose.orientation.y = q.y
    #     P.pose.orientation.z = q.z
    #     P.pose.orientation.w = q.w
    #     self.pub.publish(P)
    #     rospy.spin()
 


def main():
    ############## Hyperparameters ##############
    render = False
    solved_reward = 300  # stop training if avg_reward > solved_reward
    log_interval = 20  # print avg reward in the interval
    max_episodes = 10000  # max training episodes
    max_timesteps = 1500  # max timesteps in one episode

    update_timestep = 4000  # update policy every n timesteps
    action_std = 0.5  # constant std for action distribution (Multivariate Normal)
    K_epochs = 80  # update policy for K epochs
    eps_clip = 0.2  # clip parameter for PPO
    gamma = 0.99  # discount factor

    lr = 0.0003  # parameters for Adam optimizer
    betas = (0.9, 0.999)

    random_seed = None
    #############################################

    # Set the number of actions or action size
    action_dim = 3
    # Set the size of state observations or state size
    state_dim = 19

    # filename and directory to load model from
    filename = "PPO_continuous_DR.pth"
    directory = "./scripts/"

    memory = Memory()
    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    ppo.policy_old.load_state_dict(torch.load(directory + filename))
    time_step = 0
    rospy.init_node('listener', anonymous=True) 
    nd = Node()

    # training loop
    for i_episode in range(1, max_episodes + 1):
        #env.reset()
        #step_result = env.get_step_result(group_name)
        state=[]
        state.extend(nd.LaserData)
        state.append(nd.TargetPolar)
        state.append(nd.TargetDist)
        state.append(nd.Mag)
        state.extend(nd.Dir)
        state.extend(nd.TargetPos)
        state = np.array(state)

        for t in range(max_timesteps):
            time_step += 1
            action = ppo.select_action(state, memory)
            
            P = PoseStamped() 
            q = Quaternion.from_euler(np.clip(action[0]*0.3,-0.05,0.05), np.clip(action[1]*0.3,-0.05,0.05), 0, degrees=True)
            P.pose.orientation.x = q.x
            P.pose.orientation.y = q.y
            P.pose.orientation.z = q.z
            P.pose.orientation.w = q.w
            nd.pub.publish(P)

            if time_step > (action[2]+1)*10+1:
                state=[]
                state.extend(nd.LaserData)
                state.append(nd.TargetPolar)
                state.append(nd.TargetDist)
                state.append(nd.Mag)
                state.extend(nd.Dir)
                state.extend(nd.TargetPos)
                state = np.array(state)
                print(P) 
            my_node.loop_rate.sleep()



if __name__ == '__main__':
    main()


