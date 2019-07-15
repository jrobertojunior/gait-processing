import matplotlib.pyplot as plt
from scipy.signal import find_peaks, savgol_filter
import numpy as np
import csv
from numpy import genfromtxt

data = genfromtxt('alana-01-inter.csv', delimiter=',')
data = np.delete(data, slice(1), 1)
data = data.reshape(data.shape[0])
print(data.shape)
plt.plot(data)

y_hat = savgol_filter(data, 15, 3)

peaks, _ = find_peaks(y_hat)
plt.plot(peaks, y_hat[peaks], "x")
plt.plot(y_hat, color="red")

plt.show()
