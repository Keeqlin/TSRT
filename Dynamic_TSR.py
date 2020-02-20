import sys
import re
import numpy as np
import matplotlib.pyplot as plt


rect_pt = ['L_up','R_up','R_low','L_low','center']
fig_size = (8,4)


def Diff(Data):
    Diff = Data - np.vstack((Data[0,:],Data[:-1,:])) 
    return Diff

def Position(Data,pt_num,figure_name, save_fig = False):
    fig, axs = plt.subplots(1,2, figsize=fig_size)
    frame_num = Data.shape[0]
    for i in range(1,pt_num+1):
        axs[0].plot(range(Data.shape[0]), Data[:,i*2-1],label=rect_pt[i-1])
        axs[1].plot(range(Data.shape[0]), Data[:,i*2],label=rect_pt[i-1])
    axs[0].set(xlabel='frame',ylabel='$pixel$',title='x_axis')
    axs[1].set(xlabel='frame',ylabel='$pixel$',title='y_axis')
    # TODO unify yticks
    # print(axs[0].get_yticks())
    # print(axs[1].get_yticks())
    # ytick_interval = ax[0].get_yticks()[1]-ax[0].get_yticks()[0]
    axs[1].set_yticks(axs[0].get_yticks())
    for ax in axs:
        ax.grid(True)
        ax.legend()
    fig.suptitle(figure_name, fontsize = 10)
    fig.tight_layout() 
    if save_fig:
        fig.savefig(figure_name+'.png')

def Velocity(Data,pt_num,figure_name,save_fig = False):
    fig, axs = plt.subplots(1,2, figsize=fig_size)
    vData = Diff(Data)
    for i in range(1,pt_num+1):
        axs[0].plot(range(Data.shape[0]), vData[:,i*2-1],label=rect_pt[i-1])
        axs[1].plot(range(Data.shape[0]), vData[:,i*2],label=rect_pt[i-1])

    axs[0].set(xlabel='frame',ylabel='$pixel/frame$',title='x_axis')
    axs[1].set(xlabel='frame',ylabel='$pixel/frame$',title='y_axis')
    axs[1].set_yticks(axs[0].get_yticks())
    for ax in axs:
        ax.grid(True)
        ax.legend()
    fig.suptitle(figure_name, fontsize = 10)
    fig.tight_layout() 
    if save_fig:
        fig.savefig(figure_name+'.png')


def Acceleration(Data,pt_num,figure_name,save_fig = False):
    fig, axs = plt.subplots(1,2, figsize=fig_size)
    aData = Diff(Diff(Data))
    for i in range(1,pt_num+1):
        axs[0].plot(range(Data.shape[0]), aData[:,i*2-1],label=rect_pt[i-1])
        axs[1].plot(range(Data.shape[0]), aData[:,i*2],label=rect_pt[i-1])
    axs[0].set(xlabel='frame',ylabel='$pixel/frame^2$',title='x_axis')
    axs[1].set(xlabel='frame',ylabel='$pixel/frame^2$',title='y_axis')
    axs[1].set_yticks(axs[0].get_yticks())
    for ax in axs:
        ax.grid(True)
        ax.legend()
    fig.suptitle(figure_name, fontsize = 10)
    fig.tight_layout()
    if save_fig:
        fig.savefig(figure_name+'.png')


# load .txt and preprocessing 
gt_file = sys.argv[1]
ground_truth = []
pt_num = 0

delimiters = ",[", ", ", "],[", "]\n"
regexPattern = '|'.join(map(re.escape, delimiters))
with open(gt_file,'r') as f:    
    for line in f:
        line = str(line)
        sub_list = re.split(regexPattern,line)
        if len(sub_list) == 10:
            int_sub_list = list(map(int, sub_list[0:len(sub_list)-1]))
            pt_num = int(len(int_sub_list)/2)
            int_sub_list.append( sum(int_sub_list[i] for i in range(1,len(int_sub_list),2))/(pt_num) ) #center x
            int_sub_list.append( sum(int_sub_list[i] for i in range(2,len(int_sub_list),2))/(pt_num) ) #center y
            ground_truth.append(int_sub_list)   
            pt_num += 1   # add center pt 

f.close()
txt_del = ".txt"
txt_reg = '|'.join(map(re.escape, txt_del))
gt_file = re.split(txt_reg,gt_file)[0]
ground_truth = np.array(ground_truth)
print('ground_truth.shape:',ground_truth.shape)


# Draw data
Position(ground_truth,pt_num,'Postion_'+gt_file)
Velocity(ground_truth,pt_num,'Velocity_'+gt_file)
Acceleration(ground_truth,pt_num,'Acceleratoin_'+gt_file)
plt.show()


