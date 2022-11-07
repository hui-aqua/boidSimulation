import scr.writeVTK as st
import scr.boids as bs


 
fish=bs.Boids(50)
fish.initBoids_box(-5,5,-5,5,0,2)
print(fish.b_speed)
fish.separation()






run_time=50 #s
dt=0.001    #s
for i in range(int(run_time/dt)):

    fish.update_position(dt)
    if i % 20 == 0:        # write result for 50 frames per second  
        st.write_point('/ami/fish'+str(i),fish.b_position.tolist())
