# -*- codeing = utf-8 -*-
# @Time :2022/9/20 21:46
# @Author: PATTON
# @File : main.py
# @Software: PyCharm

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

if __name__ == "__main__":

    latitude = []
    longitude = []
    altitude = []
    utm_easting = []
    utm_northing = []
    filename = open("topic2.yaml", "r")
    for line in filename:
        line = line.strip()
        # print(line)
        if "#" in line:
            continue
        elif "latitude" in line:
            latitude.append(line)
        elif "longitude" in line:
            longitude.append(line)
        elif "altitude" in line:
            altitude.append(line)
        elif "utm_easting" in line:
            utm_easting.append(line)
        elif "utm_northing" in line:
            utm_northing.append(line)

    latitude_ = []
    for lati in latitude:
        result = ""
        for c in lati:
            if c.isdigit() or c == '-' or c == '.':
                result += c
            else:
                continue;
        latitude_.append(float(result))

    longitude_ = []
    for longi in longitude:
        result = ""
        for c in longi:
            if c.isdigit() or c == '-' or c == '.':
                result += c
            else:
                continue;
        longitude_.append(float(result))

    altitude_ = []
    for alti in altitude:
        result = ""
        for c in alti:
            if c.isdigit() or c == '-' or c == '.':
                result += c
            else:
                continue;
        altitude_.append(float(result))

    utm_easting_ = []
    for utm_e in utm_easting:
        result = ""
        for c in utm_e:
            if c.isdigit() or c == '-' or c == '.':
                result += c
            else:
                continue;
        utm_easting_.append(float(result))

    utm_northing_ = []
    for utm_n in utm_northing:
        result = ""
        for c in utm_n:
            if c.isdigit() or c == '-' or c == '.':
                result += c
            else:
                continue;
        utm_northing_.append(float(result))


    # print("Latitude: ", latitude_)
    # print("Longitude: ", longitude_)
    print("Altitude: ", altitude_)
    # print("Utm_easting: ", utm_easting_)
    # print("Utm_northing: ", utm_northing_)
    # print(len(altitude_))
    # print(len(longitude_))
    # print(len(latitude_))


    count = len(altitude_)
    # plt.style.use('_mpl-gallery')
    # x = count
    # y = latitude_

    #2D scatter plot
    df = pd.DataFrame({'x': range(count), 'y': altitude_ })
    # df = pd.DataFrame({'x': utm_northing_, 'y': utm_easting_})
    plt.plot('x', 'y', data=df, linestyle='none', marker='o')


    # 3D scatter plot
    # fig = plt.figure()
    # ax = plt.axes(projection = '3d')
    # ax.scatter(utm_easting_,utm_northing_,altitude_)
    # ax.set_title('3D scatter plot')


    plt.show()





    # fig.savefig("walking_latitute.png")


    # fig, ax = plt.subplots()
    # sizes = np.random.uniform(15, 80, len(x))
    # colors = np.random.uniform(15, 80, len(x))
    # ax.scatter(x, y, s=sizes, c=colors, vmin=0, vmax=100)
    # plt.xlabel("counters")
    # plt.ylabel("meters")

    # sns.set(style='dark-grid')
    # tips = sns.load_dataset(latitude_)
    # sns.relplot(x='test_times', y='latitude', col='time',hue='latitude', data=tips)


