import time
import board
import busio
import adafruit_lis3mdl
from tqdm import tqdm
from scipy.optimize import curve_fit

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lis3mdl.LIS3MDL(i2c)

print("OK wave the thing around")


datax=[]
datay=[]
dataz=[]
for i in tqdm(range(1000)):
    bx,by,bz=sensor.magnetic
    datax.append(bx)
    datay.append(by)
    dataz.append(bz)

print("Fitting")

def fit_func(x,bx,by,bz,b0):
    return (x[0]-bx)**2+(x[1]-by)**2+(x[2]-bz)**2-b0**2

ydata=np.zeros(len(datax))
popt, pcov = curve_fit(fit_func, [datax,datay,dataz] , ydata)

for i in range(len(popt)):
    print("{} +- {}".format(popt[i],np.sqrt(pcov[i][i])))




# (X+x)^2+(Y+y)^2+(Z+z)^2 - L^2 = 0
# 2(X+x) dX
