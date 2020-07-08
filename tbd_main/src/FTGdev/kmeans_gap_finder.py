from math import pi
import numpy as np
import datetime as dt
import sys
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

data = np.load('scan_hists.npy')

# np.rad2deg(math.tan(.3/.75)) calcs angle needed for .3 m width gap at denom dist away

for nn in range(0,data.shape[0],10):
    fdat = data[nn,:]
    
    mini = fdat[::2]#.copy()  # explicit copy, O(n)
    # diffs = abs(np.diff(fdat))
    diffs = np.diff(mini)
    top = max(fdat)
    ran = max(fdat) - min(fdat)
    print(top,ran)
    dat = fdat[fdat > top-ran/2]
    inds = np.where(fdat > top-ran/2)[0]
    #inds = np.asarray(range(len(dat)))
    dat = np.concatenate((inds.reshape(-1,1),dat.reshape(-1,1)),1)
    # create kmeans object
    clusts = 3
    kmeans = KMeans(n_clusters=clusts)
    # fit kmeans object to data
    kmeans.fit(dat)
    # print location of clusters learned by kmeans object
    #print(kmeans.cluster_centers_)
    # save new clusters for chart
    y_km = kmeans.fit_predict(dat)
    #print(y_km)

    plt.figure(1)
    plt.clf()
    plt.plot(fdat,'o')
    for c in range(clusts):
        plt.plot(dat[y_km==c,0],dat[y_km==c,1],'o')


    plt.ylim( [ 0 , 36 ] )

    #plt.pause(0.1)
    
    plt.figure(2)
    plt.clf()
    plt.plot(diffs,'o')
    plt.ylim( [ -1 , 1] )
    plt.pause(0.01)
    #plt.show()


