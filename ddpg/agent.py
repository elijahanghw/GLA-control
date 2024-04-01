import torch
import torch.autograd
import torch.optim as optim
import torch.nn as nn

from networks import *
from utils import *

class Agent:
    def __init__(self, env, nodes=16, a_lr=5e-4, c_lr=1e-3, gamma=0.99, tau=5e-3, mem_size=100000):
        self.num_states = env.state_space.shape[0]
        self.num_actions = env.action_space.shape[0]
        self.gamma = gamma
        self.tau = tau
        
        # self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.device = torch.device('cpu')
        print(f"Training on: {self.device}")

        # Actor and critic network
        self.actor = Actor(self.num_states, nodes, self.num_actions).to(self.device)
        self.actor_target = Actor(self.num_states, nodes, self.num_actions).to(self.device)

        self.critic = Critic(self.num_states + self.num_actions, nodes, self.num_actions).to(self.device)
        self.critic_target = Critic(self.num_states + self.num_actions, nodes, self.num_actions).to(self.device)

        for target_param, param in zip(self.actor_target.parameters(), self.actor.parameters()):
            target_param.data.copy_(param.data)

        for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
            target_param.data.copy_(param.data)

        # Training
        self.memory = Memory(mem_size)
        self.critic_criterion = nn.MSELoss()
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=a_lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=c_lr)

    def get_action(self, state):
        state = Variable(torch.from_numpy(state).float().unsqueeze(0)).to(self.device)
        self.actor.eval()
        action = self.actor.forward(state)
        action = action.cpu().detach().numpy()[0]

        return action
        
    def update(self, batch_size):
        states, actions, rewards, next_states, done = self.memory.sample(batch_size)

        states = np.array(states)
        actions = np.array(actions)
        rewards = np.array(rewards)
        next_states = np.array(next_states)
        done = np.array(done)

        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(next_states).to(self.device)
        done = torch.FloatTensor(done).to(self.device)
    
        # Critic loss        
        Qvals = self.critic.forward(states, actions)
        next_actions = self.actor_target.forward(next_states)
        next_Q = self.critic_target.forward(next_states, next_actions)
        Qprime = rewards + self.gamma * next_Q
        critic_loss = self.critic_criterion(Qvals, Qprime)

        # update critic network
        self.critic.train()
        self.critic_optimizer.zero_grad()
        critic_loss.backward() 
        self.critic_optimizer.step()

        # Actor loss
        self.critic.eval()
        mu = self.actor.forward(states)
        self.actor.train()
        policy_loss = -self.critic.forward(states, mu)
        policy_loss = torch.mean(policy_loss)
        
        # update actor network
        self.actor_optimizer.zero_grad()
        policy_loss.backward()
        self.actor_optimizer.step()

        # update target networks 
        for target_param, param in zip(self.actor_target.parameters(), self.actor.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))
       
        for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
            target_param.data.copy_(param.data * self.tau + target_param.data * (1.0 - self.tau))

    def save_actor_critic(self):
        torch.save(self.actor.state_dict(), 'ddpg/actor.pth')
        torch.save(self.critic.state_dict(), 'ddpg/critic.pth')

