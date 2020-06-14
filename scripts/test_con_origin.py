import gym
from PPO_continuous import PPO, Memory
from PIL import Image
import torch

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class Node():
    def __init__(self):
        # Params
        self.LaserData=np.zeros(12) #12
        self.TargetPolar = 0
        self.UWBPos=(0,0) #2
        self.RPY=(0,0,0)
        self.Dir=(0,0)
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
        #self.pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=None)
        
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
        print("UWB:",self.UWBPos,"TPOLAR:",self.TargetPolar,"TDist:",self.TargetDist)
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
        self.RPY = (e[0]/180, e[1]/180, e[2]/180) 

def test():
    ############## Hyperparameters ##############
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    n_episodes = 3          # num of episodes to run
    max_timesteps = 1500    # max timesteps in one episode
    render = True           # render the environment
    save_gif = False        # png images are saved in gif folder
    
    # filename and directory to load model from
    filename = "PPO_continuous_DR.pth"
    directory = "./scripts/"

    action_std = 0.5        # constant std for action distribution (Multivariate Normal)
    K_epochs = 80           # update policy for K epochs
    eps_clip = 0.2          # clip parameter for PPO
    gamma = 0.99            # discount factor
    
    lr = 0.0003             # parameters for Adam optimizer
    betas = (0.9, 0.999)
    #############################################
    
    memory = Memory()
    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    model_trt = TRTModule()
    ppo.policy_old.load_state_dict(model_trt.load_state_dict(torch.load(directory + filename)))
    
    for ep in range(1, n_episodes+1):
        state = nd.LaserData+nd.TargetPolar+nd.TargetDist+nd.Mag+nd.Dir+nd.TargetPos#step_result.obs[0]
        for t in range(max_timesteps):
            time_step += 1
            action = ppo.select_action(state, memory)
            if time_step > action[3]:
                state = 0 #step_result.obs[0][0]  # get the next states for each unity agent in the environment
                my_node.loop_rate.sleep()
            
    
if __name__ == '__main__':
    test()