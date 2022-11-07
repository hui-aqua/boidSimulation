import numpy as np

class Boids:
    def __init__(self, number):
        self.b_num=number
        # generate a random number between[0,1]
        self.b_position=np.random.rand(number,3)
        self.b_speed=np.random.normal(0, 1, size=(number, 3))
        self.b_min_dis=0.5   # m
        self.dis_to_other,none=self.update_dis()
        self.visible_range=2 # m

    def update_dis(self):
        min_dis_to_other=np.array([0.0]*self.b_num)
        index_to_nearest=[]
        for i in range(self.b_num):
            
            b_position_copy=self.b_position.copy()
            b_position_copy-=self.b_position[i]
            b_dis=np.linalg.norm(b_position_copy,axis=1)
            b_dis_sort=np.sort(b_dis)
            min_dis_to_other[i]=b_dis_sort[1]
            index_to_nearest.append(np.where(b_dis==b_dis_sort[1])[0][0])
        return min_dis_to_other, index_to_nearest
        


    def initBoids_box(self,x_min,x_max,y_min,y_max,z_min,z_max):
        self.b_position[:,0] *=abs(x_max-x_min)
        self.b_position[:,0] +=x_min
        self.b_position[:,1] *=abs(y_max-y_min)
        self.b_position[:,1] +=y_min
        self.b_position[:,2] *=abs(z_max-z_min)
        self.b_position[:,2] +=z_min
    

    # separation
    # Each boid attempts to avoid running into other boids. 
    # If two or more boids get too close to one another (i.e. within one another's protected range), 
    # they will steer away from one another. They will do so in the following way:  
    def separation(self):
        min_dis_to_other, index_to_nearest=self.update_dis()
        # print(min_dis_to_other)
        # print(index_to_nearest)
        min_dis_vector=self.b_position-self.b_position[index_to_nearest]
        min_dis_vector[min_dis_to_other>self.b_min_dis]*=0.0
        # print(min_dis_vector)
        k=1   # avoid factor is a tunable parameter
              # act like a stiffness
        self.b_speed+=min_dis_vector*k
    # Alignment        
    
    
    def boundary(self):
        margin=self.visible_range  # a tunable parameter
        turnFactor = 1   # a tunable parameter
        radius=15 # m
        height=5 # m

        # side wall
        dis_2_00_vector=self.b_position[:,:1]-np.array([[0,0]]*self.b_num)
        dis_2_00=np.linalg.norm(dis_2_00_vector,axis=1)
        self.b_speed[dis_2_00>radius-margin][:,:1]-=turnFactor*dis_2_00_vector/dis_2_00
        
        # top wall
        dis_2_top=self.b_position[:,2]-height
        
        self.b_speed[dis_2_top>0][:,2]-=turnFactor*abs(dis_2_top)
        
        # bottom wall
        dis_2_top=self.b_position[:,2]-0.0
        
        self.b_speed[dis_2_top<margin][:,2]+=turnFactor*abs(dis_2_top)
        
        
        
    def update_position(self,dt):
        self.b_position+=self.b_speed*dt
        