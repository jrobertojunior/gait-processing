import matplotlib.pyplot as plt
import csv
from scipy.signal import find_peaks

x = []
y = []

with open('alana-01-inter.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(int(row[0]))
        y.append(float(row[1]))


plt.plot(x, y, marker='o')

plt.title('Data from the CSV File: People and Expenses')

plt.xlabel('Number of People')
plt.ylabel('Expenses')

plt.show()
