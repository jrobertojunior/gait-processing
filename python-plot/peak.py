import matplotlib.pyplot as plt
from scipy.signal import find_peaks, savgol_filter
import numpy as np
import csv
from numpy import genfromtxt


def main():
    recording = 'alana-01'

    # ankle_dists = get_csv(recording, "-ankledists.csv")
    # print(ankle_dists)
    ankles_raw, ankles = handle_file(
        recording, "-ankledists.csv")
    plot_two(ankles, ankles_raw, color="red")

    l_ankle_raw, l_ankle = handle_file(recording, "-leftdist.csv")
    plot_two(l_ankle, l_ankle_raw, color="blue")

    r_ankle_raw, r_ankle = handle_file(recording, "-rightdist.csv")
    plot_two(r_ankle, r_ankle_raw, color="green")

    # spine_raw, spine_filt = handle_file(recording, "-spinepos.csv")
    # print(spine_raw[0][0])

    # peaks, _ = find_peaks(ankles)
    # valleys, _ = find_peaks(ankles * -1)
    # crit = np.concatenate((peaks, valleys), axis=0)

    crit_ankles = critical_points(ankles)
    plt.plot(crit_ankles, ankles[crit_ankles], "x")

    crit_l = critical_points(l_ankle)
    plt.plot(crit_l, l_ankle[crit_l], "x")

    crit_r = critical_points(r_ankle)
    plt.plot(crit_r, r_ankle[crit_r], "x")

    # print(crit_l)

    # plt.plot(valleys, ankles[valleys], "x")

    # crit_l_ankle = critical_points(l_ankle)
    # plt.plot(crit_l_ankle, l_ankle[crit_ankles], "x")

    # crit_r_ankle, _ = critical_points(r_ankle)
    # plt.plot(crit_r_ankle, r_ankle[crit_r_ankle], "x")

    # ankles_step = np.empty(shape=ankles_raw.shape)

    # for i in crit_ankles:
    #     ankles_step[i] = l_ankle[closest_peak(
    #         i, crit_l_ankle)] - r_ankle[closest_peak(i, crit_r_ankle)]
    # print("({}) {}".format(i, ankles_step[i]*100))

    # plt.plot(ankles_step, color="grey")
    # first_pos = l_ankle_pos[0]
    # for pos in l_ankle_pos:
    #     print("{} {}".format(pos, first_pos))
    #     # print(pos - first_pos)

    plt.show()


def handle_file(recording, filename):
    raw_data = get_csv(recording, filename)
    filtered_data = savgol_filter(raw_data, 17, 5)

    return raw_data, filtered_data


def get_csv(recording, filename):
    return genfromtxt(
        '../recordings/{}{}.'.format(recording, filename))


def plot_two(primary, secondary, color="red"):
    plt.plot(primary, color=color)
    plt.plot(secondary, color=color, alpha=0.2)


def critical_points(array):
    peaks, _ = find_peaks(array)
    valleys, _ = find_peaks(array * -1)
    crit = np.concatenate((peaks, valleys), axis=0)

    return crit


def closest_peak(index, peaks):
    min_diff = float("inf")
    for peak in peaks:
        diff = index - peak
        if diff != 0 and diff < min_diff:
            min_diff = diff

    return min_diff


if __name__ == "__main__":
    main()
