import pandas as pd
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import math 

df = pd.DataFrame({
    'x': [1.3*5280, 0.6*5280, 4.6*5280, 1.3*5280, 4.3*5280, 4.8*5280, 0.6*5280, 5*5280, 3.1*5280, 2*5280, 3.2*5280, 4.1*5280, 3.3*5280, 3.3*5280, 0.7*5280, 4.1*5280, 1.2*5280, 0.7*5280, 0.3*5280, 4.3*5280, 2*5280, 0.2*5280, 0.5*5280, 0.7*5280, 4*5280],
    'y': [1.1*5280, 4.2*5280, 4.0*5280, 4.2*5280, 0.5*5280, 1.3*5280, 0.2*5280, 2.1*5280, 1.3*5280, 2.5*5280, 3.9*5280, 4.7*5280, 0.6*5280, 4.5*5280, 2.8*5280, 2.6*5280, 1*5280, 3.2*5280, 0.3*5280, 4.7*5280, 0*5280, 1.7*5280, 4.2*5280, 0.4*5280, 2.9*5280]
})

kmeans = KMeans(n_clusters=4)
kmeans.fit(df)

labels = kmeans.predict(df)
centroids = kmeans.cluster_centers_

print(centroids)
dist_list = []
dist_list.append([13200, 13200])
for i in range(len(centroids)):
    dist_list.append(centroids[i])
for j in range(len(dist_list)):
    if j <= len(dist_list) - 2:
        distance = round(math.hypot((dist_list[j][0] - dist_list[j+1][0]), (dist_list[j][1] - dist_list[j+1][1])))
        print(distance/5280 * 1.61)

fig = plt.figure()

colmap = {1: 'r', 2: 'g', 3: 'b', 4: 'k', 5: 'y'}

colors = map(lambda x: colmap[x+1], labels)
colors1 = list(colors)
plt.scatter(df['x'], df['y'], color=colors1, alpha=0.5, edgecolor='k')
for idx, centroid in enumerate(centroids):
    plt.scatter(*centroid, color=colmap[idx+1])
# plt.xlim(0, 30000)
# plt.ylim(0, 30000)
plt.savefig('k2.pdf')
plt.show()
