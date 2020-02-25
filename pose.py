import sys
import re
import numpy as np
import matplotlib.pyplot as plt


Homo_pose_All = [] 
pose_file_perfix = './../decompose_H_pose'
delimiters = "[", ", ", "],[", "]\n"
regexPattern = '|'.join(map(re.escape, delimiters))
for i in range(4):
    file_name = pose_file_perfix+str(i)+'.txt'
    with open(file_name,'r') as f:
        Homo_pose = []
        for line in f:
            sub_list = re.split(regexPattern,line)
            sub_list = list(map(float,sub_list[1:-1]))
            Homo_pose.append(sub_list)
        Homo_pose_All.append(Homo_pose)

Homo_pose_All = np.array(Homo_pose_All)
print('Homo_pose_All.shape: ', Homo_pose_All.shape)
        
PNP_pose = []
PNP_file_name = './../pnp_pose.txt'
with open(PNP_file_name,'r') as f:
    for line in f:
        sub_list = re.split(regexPattern,line)
        sub_list = list(map(float,sub_list[1:-1]))
        PNP_pose.append(sub_list)
PNP_pose = np.array(PNP_pose)
print('PNP_pose.shape',PNP_pose.shape)

# Diffz_pnp = PNP_pose[-1,2] - PNP_pose[0,2]
# Diffz_Homo = Homo_pose_All[0,-1,2] - Homo_pose_All[0,0,2]
# scale_z = Diffz_pnp/Diffz_Homo
# print('scale_z:',scale_z)
# Diffx_pnp = PNP_pose[-1,0] - PNP_pose[0,0]
# Diffx_Homo = Homo_pose_All[0,-1,0] - Homo_pose_All[0,0,0]
# scale_x = Diffx_pnp/Diffx_Homo
# print('scale_x:',scale_x)



mark = ['^','.','x','+']
axis_name = ['x','y','z']
fig_t, axs_t = plt.subplots(3,1)
for j in range(len(axs_t)):
    for i in range(Homo_pose_All.shape[0]):
        axs_t[j].plot(range(Homo_pose_All.shape[1]), Homo_pose_All[i,:,j],label = 'sol_'+str(i), marker = mark[i])
        axs_t[j].set(xlabel='frame',ylabel=axis_name[j]+'_axis')
        axs_t[j].grid(True)
    axs_t[j].plot(range(PNP_pose.shape[0]), PNP_pose[:,j], label ="PNP")

handles, labels = axs_t[len(axs_t)-1].get_legend_handles_labels()
fig_t.legend(handles, labels, loc='lower center', ncol=4)      
fig_t.suptitle('decomposedH_pose_t', fontsize = 10)
fig_t.tight_layout() 


angle_name = ['pitch','yaw','roll']
fig_R, axs_R = plt.subplots(3,1)
for j in range(len(axs_R)):
    for i in range(Homo_pose_All.shape[0]):
        axs_R[j].plot(range(Homo_pose_All.shape[1]), Homo_pose_All[i,:,j+3],label = 'sol_'+str(i), marker = mark[i])
        axs_R[j].set(xlabel='frame',ylabel=angle_name[j]+' deg')
        axs_R[j].grid(True)
    axs_R[j].plot(range(PNP_pose.shape[0]), PNP_pose[:,j+3], label ="PNP")

handles, labels = axs_R[len(axs_R)-1].get_legend_handles_labels()
fig_R.legend(handles, labels, loc='lower center', ncol=4)      
fig_R.suptitle('decomposedH_pose_R', fontsize = 10)
fig_R.tight_layout() 

plt.show()