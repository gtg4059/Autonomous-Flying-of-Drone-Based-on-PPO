#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped #4 Angle Data to 3
from sensor_msgs.msg import LaserScan, Imu #20 LAser Data
from mavros_msgs.msg import AttitudeTarget, State, Thrust
from mavros_msgs.srv import SetMode, CommandBool
#from mavros_msgs.srv import CommandBool, SetMode
import time
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
        cov_mat = torch.diag(self.action_var).to(device)

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
        cov_mat = torch.diag_embed(action_var).to(device)
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
        rewards = torch.FloatTensor(rewards).to(device)
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
        self.RPY=[0,0]
        self.Dir=[0,0]
        self.Mag=0
        self.TargetDist=0
        self.TargetPos = [1.6,3.6]
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(50)
        string = String()
        laser = LaserScan()
        imu = Imu()
        Veldata = TwistStamped()
        state = State()
        self.current_state = State() 
        self.offb_set_mode = SetMode
        # Publishers
        #self.pub = rospy.Publisher("/mavros/setpoint_attitude/attitude", PoseStamped, queue_size=10)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_attitude/attitude", PoseStamped, queue_size=10)
        self.local_thr_pub = rospy.Publisher("/mavros/setpoint_attitude/thrust", Thrust, queue_size=10) 
        
        # Subscribers
        rospy.Subscriber("/UWBPosition", String, self.callback_Pos) 
        rospy.Subscriber("/scan", LaserScan, self.callback_range) #RayinformContain
        rospy.Subscriber("/mavros/imu/data", Imu, self.callback_RPY) #CrntAngle
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.callback_Vel) #CrntDir
        rospy.Subscriber("/mavros/state",State,self.callback_state)
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
        self.current_state = None

    def callback_Pos(self,string):   
        #self.str = ""
        self.UWBPos=tuple(map(float, string.data.split(',')))
        targetdir = np.array([self.TargetPos[0]-self.UWBPos[0],self.TargetPos[1]-self.UWBPos[1]])
        targetdirAngle = atan2(targetdir[1],targetdir[0]) if atan2(targetdir[1],targetdir[0])>=0 else atan2(targetdir[1],targetdir[0])+tau
        self.TargetDist = np.linalg.norm(targetdir)/(1+abs(np.linalg.norm(targetdir)))*2-1
        crntAngle = atan2(self.Dir[1],self.Dir[0]) if atan2(self.Dir[1],self.Dir[0])>=0 else atan2(self.Dir[1],self.Dir[0])+tau
        self.TargetPolar = targetdirAngle - crntAngle if targetdirAngle - crntAngle<pi else targetdirAngle - crntAngle - tau
        #print("targetdir:",targetdir,"Dir:",self.Dir)
        #print("UWB:",self.UWBPos,"TPOLAR:",self.TargetPolar,"TDist:",self.TargetDist)
    def callback_Vel(self, Veldata):
        n = np.array([Veldata.twist.linear.x, Veldata.twist.linear.y])
        self.Mag = np.linalg.norm(n)
        self.Dir = n/self.Mag
        #print("MAG:",self.Mag,"DIR:",self.Dir)
    def callback_range(self, laser):
        for i in range(12):
            #self.LaserData[i] = laser.ranges[i*20]
            if isinf(laser.ranges[(i*20+180)%360])==False and laser.ranges[(i*20+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20+180)%360]
            elif isinf(laser.ranges[(i*20-1+180)%360])==False and laser.ranges[(i*20-1+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20-1+180)%360]
            elif isinf(laser.ranges[(i*20+1+180)%360])==False and laser.ranges[(i*20+1+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20+1+180)%360]
            elif isinf(laser.ranges[(i*20-2+180)%360])==False and laser.ranges[(i*20-2+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20-2+180)%360]
            elif isinf(laser.ranges[(i*20+2+180)%360])==False and laser.ranges[(i*20+2+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20+2+180)%360]
            elif isinf(laser.ranges[(i*20-3+180)%360])==False and laser.ranges[(i*20-3+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20-3+180)%360]
            elif isinf(laser.ranges[(i*20+3+180)%360])==False and laser.ranges[(i*20+3+180)%360]!=0:
                self.LaserData[i] = laser.ranges[(i*20+3+180)%360]
            else: 
                self.LaserData[i] = 0.5
        #print("Laser:",tuple(self.LaserData))    
    def callback_RPY(self, imu): 
        q = Quaternion(imu.orientation.w,imu.orientation.x,\
            imu.orientation.y,imu.orientation.z)
        e = q.to_euler(degrees=True)
        self.RPY = [e[0]/180, e[1]/180]
        #self.Pos=(eq[0],eq[1],eq[2])*pi/180
        #print("Pose:",self.RPY) 
    def callback_state(self, state):
        self.current_state = state




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

    rospy.init_node('listener', anonymous=True) 
    nd = Node()

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
    T = Thrust()
    T.thrust = 0.25
    pose = PoseStamped()
    q = Quaternion.from_euler(0, 0, 0, degrees=True)
    pose.pose.orientation.w = q.w
    pose.pose.orientation.x = q.x
    pose.pose.orientation.y = q.y
    pose.pose.orientation.z = q.z
    prev_state = nd.current_state
    for i in range(100):
        nd.local_thr_pub.publish(T)
        nd.local_pos_pub.publish(pose)
        nd.loop_rate.sleep()
    print("ready")
    # while nd.state.mode == "STABILIZED":
    #     nd.loop_rate.sleep()
    # training loop
    for i_episode in range(1, max_episodes + 1):
        #env.reset()
        #step_result = env.get_step_result(group_name)
        state=[]
        state.extend(nd.LaserData)#nd.LaserData
        state.append(nd.TargetPolar/tau)
        state.append(nd.TargetDist)
        state.append(nd.Mag/20)
        state.extend(nd.Dir)
        state.extend(nd.RPY)
        # state.extend([5,5,5,5,5,5,5,5,5,5,10,10])#nd.LaserData
        # state.append(0)
        # state.append(3.6)
        # state.append(nd.Mag/20)
        # state.extend(nd.Dir)
        # state.extend(nd.RPY)
        state = np.array(state)
        last_request = rospy.get_rostime()
        for t in range(max_timesteps):
            
            
            time_step += 1
            action = ppo.select_action(state, memory)
            now = rospy.get_rostime()
            if nd.current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                nd.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not nd.current_state.armed and (now - last_request > rospy.Duration(5.)):
                    nd.arming_client(True)
                    last_request = now 
            roll=-1*np.clip(action[0]*0.3,-0.05,0.05)*180/pi
            pitch=np.clip(action[1]*0.3,-0.05,0.05)*180/pi
            T = Thrust()
            T.thrust = 0.2
            pose = PoseStamped()
            q = Quaternion.from_euler(roll, pitch, 90, degrees=True)
            pose.pose.orientation.w = q.w
            pose.pose.orientation.x = q.x
            pose.pose.orientation.y = q.y
            pose.pose.orientation.z = q.z
            pose.header.stamp = rospy.Time.now()
            T.header.stamp = rospy.Time.now()
            nd.local_pos_pub.publish(pose)
            nd.local_thr_pub.publish(T)
            if time_step > (action[2]+1)*10+1:
                state=[]
                state.extend(nd.LaserData)#nd.LaserData
                state.append(nd.TargetPolar/tau)
                state.append(nd.TargetDist)
                state.append(nd.Mag/20)
                state.extend(nd.Dir)
                state.extend(nd.RPY)
                # state.extend([5,5,5,5,5,5,5,5,5,5,10,10])#nd.LaserData
                # state.append(0)
                # state.append(3.6)
                # state.append(nd.Mag/20)
                # state.extend(nd.Dir)
                # state.extend(nd.RPY)
                state = np.array(state)
                print(roll,pitch) 
                time_step=0
                nd.loop_rate.sleep()


if __name__ == '__main__':
    main()


