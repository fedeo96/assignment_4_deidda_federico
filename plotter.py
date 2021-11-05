import numpy as np
import matplotlib.pyplot as plt

x=[]
y=[]
xgt=[]
ygt=[]
xrmse=[]
yrmse=[]
time=[]
fig, (ax1, ax2, ax3,ax4) = plt.subplots(4)
#myfile<< best_particle.x<< " "<< best_particle.y<< " " <<gt_x << " "<<gt_y<<" "<<RMSE(0)<<" "<< RMSE(1)<<" " <<duration.count()<<'\n';
with open('res.txt','r') as file:
    # reading each line    
    next(file)
    # reading each line    
    for line in file:
        # reading each word        
        word = line.split()
        x.append(float(word[0]))
        y.append(float(word[1])) 
        xgt.append(float(word[2]))
        ygt.append(float(word[3])) 
        xrmse.append(float(word[4]))
        yrmse.append(float(word[5])) 
        time.append(float(word[6])) 
        
ax1.plot(x, y)
ax1.scatter(xgt, ygt,color='green', s=5)

t=[i for i in range(len(xrmse))]
ax2.plot(t, xrmse)
ax3.plot(t, yrmse)
t=[i for i in range(len(time))]
ax4.plot(t, time)
plt.show()
