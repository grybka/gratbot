#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class TimeSeries:


    def __init__(self,times=np.array([],dtype=np.float),data=np.array([],dtype=np.float),title="",ylabel=""): #datatype defaults to float
        self.times=np.array(times)
        self.data=np.array(data)
        self.title=title
        self.ylabel=ylabel

    def __len__(self):
        return len(self.times)

    def append(self,t,x):
        self.times=np.append(self.times,t)
        self.data=np.append(self.data,x)

    def extract_bracketing_elements(self,target_timeseries,before_time_min=0,before_time_max=1,after_time_min=0,after_time_max=1):
        my_matches_before=TimeSeries()
        my_matches_after=TimeSeries()
        target_matches=TimeSeries()
        indices_before=np.searchsorted(self.times,target_timeseries.times-before_time_min)-1
        indices_after=np.searchsorted(self.times,target_timeseries.times+after_time_min)
        for i in range(len(indices_before)):
            if indices_after[i]>=len(self.times) or indices_before[i]<0 or target_timeseries.times[i]-self.times[indices_before[i]] > before_time_max or self.times[indices_after[i]]-target_timeseries.times[i] > after_time_max:
                pass
            else:
                target_matches.append(target_timeseries.times[i],target_timeseries.data[i])
                my_matches_before.append(self.times[indices_before[i]],self.data[indices_before[i]])
                my_matches_after.append(self.times[indices_after[i]],self.data[indices_after[i]])
        return target_matches,my_matches_before,my_matches_after




    def find_elems_just_after_times(self,times,timewindow=1):
        indices=np.searchsorted(self.times,times)
        return TimeSeries(self.times[indices],self.data[indices],title=self.title,ylabel=self.ylabel)

    def find_elems_just_before_times(self,times):
        indices=np.searchsorted(self.times,times)
        while len(indices)!=0 and indices[0]==0:
            print("this probably won't work")
            indices=np.delete(indices[0])
        if len(indices)==0:
            return TimeSeries()
        return TimeSeries(self.times[indices-1],self.data[indices-1],title=self.title,ylabel=self.ylabel)

    def plot(self,start_time=None,stop_time=None):
        fig, ax = plt.subplots()
        ax.set(xlabel='time', ylabel=self.ylabel,title=self.title)
        if start_time is not None:
            plt.xlim(start_time,stop_time)
        ax.plot(self.times,self.data)
        #ax.vlines(turns_t,np.min(headings),np.max(headings),color="C1")
        fig.show()

class TrackData:
    def __init__(self):
        self.times=np.array([],dtype=np.float)
        self.xywh=[]
        self.last_update=[]
        self.labels=[]

    def append(self,t,xywh,last_update,label):
        self.times=np.append(self.times,t)
        self.xywh.append(xywh)
        self.last_update.append(last_update)
        self.labels=np.append(self.labels,label)

    def get_last_update_x_timeseries(self):
        return TimeSeries(self.times,self.last_update[:,0])
