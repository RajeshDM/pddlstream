import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import kalepy as kale

from kalepy.plot import nbshow
from icecream import ic
from mpl_toolkits.mplot3d import Axes3D

'''
NUM = int(1e4)
np.random.seed(12345)
# Combine data from two different PDFs
_d1 = np.random.normal(4.0, 1.0, NUM)
_d2 = np.random.lognormal(0, 0.5, size=NUM)
ic (len(_d1))
ic (len(_d2))
data = np.concatenate([_d1, _d2])
ic (len(data))

ic (data)
# Calculate the "true" distribution
xx = np.linspace(0.0, 7.0, 100)[1:]
yy = 0.5*np.exp(-(xx - 4.0)**2/2) / np.sqrt(2*np.pi)
yy += 0.5 * np.exp(-np.log(xx)**2/(2*0.5**2)) / (0.5*xx*np.sqrt(2*np.pi))

ic (xx)
ic (en(yy) )

# Reconstruct the probability-density based on the given data points.
points, density = kale.density(data, probability=True)

# Plot the PDF
plt.plot(points, density, 'k-', lw=2.0, alpha=0.8, label='KDE')

# Plot the "true" PDF
plt.plot(xx, yy, 'r--', alpha=0.4, lw=3.0, label='truth')

# Plot the standard, histogram density estimate
plt.hist(data, density=True, histtype='step', lw=2.0, alpha=0.5, label='hist')

plt.legend()
plt.show()

#nbshow()

samples = kale.resample(data)
ic (len(samples))
'''
xs = []
ys = []
n = 10000
data_folder = 'data/'
file_name = '2d_unity_poses_with_rotation_'
obj_ids = ['treasure_chest_large','trophy','6939183f-c90f-43d4-a9fa-35210461a4f5','sturdy_box_small']
for i in range(len(obj_ids)-3):
    x = np.load(data_folder + file_name + obj_ids[i]+'.npy',allow_pickle=True)
    y = np.zeros([n, 10], np.float32)
    y[:, i] = 1
    xs.append(x)
    ys.append(y)
    #ixs.append(np.asarray([i]*n))

y = np.concatenate(ys, 0)
data = np.concatenate(xs, 0).T

#ic (data[:100])

# Load some random-ish three-dimensional data
#np.random.seed(9485)
#data = kale.utils._random_data_3d_02(num=3e3)

ic (data.shape)


# Construct a KDE
kde = kale.KDE(data)

# Construct new data by resampling from the KDE
#resamp = kde.resample(size=1e3)

# Plot the data and distributions using the builtin `kalepy.corner` plot
#corner, h1 = kale.corner(kde, quantiles=[0.5, 0.9])
#h2 = corner.clean(resamp, quantiles=[0.5, 0.9], dist2d=dict(median=False), ls='--')
#kale.corner(kde)
resamp = kde.resample(size=1e3)
#resamp = kale.resample(data)
ic (resamp.shape)

# Plot the data and distributions using the builtin `kalepy.corner` plot
'''
corner, h1 = kale.corner(kde, quantiles=[0.5, 0.9])
h2 = corner.clean(resamp, quantiles=[0.5, 0.9], dist2d=dict(median=False), ls='--')

corner.legend([h1, h2], ['input data', 'new samples'])
'''
#corner.legend([h1, h2], ['input data', 'new samples'])
#plt.show()
old_data= data.T

#plt.plot(old_data[:,0],old_data[:,1],'ro')
#plt.plot(resamp[0],resamp[1],'bo')
fig = plt.figure()
ax = Axes3D(fig)
#ax.scatter(old_data[:,0],old_data[:,1],old_data[:,2])
ax.scatter(resamp[0],resamp[1],resamp[2])
