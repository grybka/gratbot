import numpy as np
import uncertainties
from uncertainties import ufloat
from uncertainties.umath import *
from uncertainties import unumpy

class BayesianArray:
    def __init__(self,vals=None,cov=None,ndim=1):
        if np.any(vals)==None:
            self.vals=np.zeros(ndim)
        else:
            self.vals=np.array(vals)
        if np.any(cov)==None:
            self.covariance=np.eye(len(self.vals))
        else:
            self.covariance=np.array(cov)

    def __add__(self,toadd):
        return BayesianArray(self.vals+toadd.vals,self.covariance+toadd.covariance)

    def updated(self,x):
        mu_1=self.vals
        mu_2=x.vals
        sigma_1=self.covariance
        sigma_2=x.covariance
        sumsigmainv=np.linalg.inv(sigma_1+sigma_2)
        mu_new=np.dot(sigma_2,np.dot(sumsigmainv,mu_1))+np.dot(sigma_1,np.dot(sumsigmainv,mu_2))
        sigma_new=np.dot(sigma_1,np.dot(sumsigmainv,sigma_2))
        return BayesianArray(mu_new,sigma_new)

    def get_as_ufloat(self):
        return np.array(uncertainties.correlated_values(self.vals,self.covariance))

    def __str__(self):
        return str(self.vals)+"("+str(self.covariance)+")"

    def min_covariance(self,min_cov):
        for i in range(len(self.vals)):
            if self.covariance[i][i]<min_cov[i]:
                self.covariance[i][i]=min_cov[i]
        return self

    def get_error_ellipse(self,var1=0,var2=1):
        cov=np.array( [ [self.covariance[var1][var1],self.covariance[var1][var2]],[self.covariance[var2][var1],self.covariance[var2][var2]]])
        w, v = np.linalg.eigh(cov)
        order = w.argsort()[::-1]
        w, v = w[order], v[:,order]
        theta = np.degrees(np.arctan2(*v[:,0][::-1]))
        return self.vals[var1],self.vals[var2],2.*np.sqrt(w[0]),2*np.sqrt(w[1]),theta

    @staticmethod
    def from_ufloats(ufloatarray):
        return BayesianArray(unumpy.nominal_values(ufloatarray),uncertainties.covariance_matrix(ufloatarray))
