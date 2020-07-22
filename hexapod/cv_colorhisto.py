import cv2 as cv2
import matplotlib.pyplot as plt
import numpy as np

def init_plot():
    # Initialize plot.
    bins=100
    fig, ax = plt.subplots()
    ax.set_title('Histogram (RGB)')
    ax.set_xlabel('Bin')
    ax.set_ylabel('Frequency')
    # Initialize plot line object(s). Turn on interactive plotting and show plot.
    lw = 3
    alpha = 0.5
    lineR, = ax.plot(np.arange(bins), np.zeros((bins,)), c='r', lw=lw, alpha=alpha)
    lineG, = ax.plot(np.arange(bins), np.zeros((bins,)), c='g', lw=lw, alpha=alpha)
    lineB, = ax.plot(np.arange(bins), np.zeros((bins,)), c='b', lw=lw, alpha=alpha)
    ax.set_xlim(0, bins-1)
    ax.set_ylim(0.001, 1)
    ax.set_yscale("log")
    plt.ion()
    plt.show()
    return fig,lineR,lineG,lineB


def update_plot(myframe,fig,lineR,lineB,lineG):
    bins=100
    numPixels = np.prod(myframe.shape[:2])
    #cv2.imshow('RGB', myframe)
    hsv = cv2.cvtColor(myframe, cv2.COLOR_BGR2HSV)
    #(b, g, r) = cv2.split(myframe)
    (h, s, v) = cv2.split(hsv)
    #histogramR = cv2.calcHist([r], [0], None, [bins], [0, 255]) / numPixels
    histogramR = cv2.calcHist([h], [0], None, [bins], [0, 255]) / numPixels
    #histogramG = cv2.calcHist([g], [0], None, [bins], [0, 255]) / numPixels
    histogramG = cv2.calcHist([s], [0], None, [bins], [0, 255]) / numPixels
    #histogramB = cv2.calcHist([b], [0], None, [bins], [0, 255]) / numPixels
    histogramB = cv2.calcHist([v], [0], None, [bins], [0, 255]) / numPixels
    lineR.set_ydata(histogramR)
    lineG.set_ydata(histogramG)
    lineB.set_ydata(histogramB)
    fig.canvas.draw()
