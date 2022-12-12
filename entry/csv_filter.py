# A simple Python program which filters out all stations not within the
# Champaign-Urbana area. This was used to generate champaign_urbana_data.csv.

import pandas as pd

data = pd.read_csv("data/alt_fuels_stations.csv")

long = data['Longitude']
lat = data['Latitude']

long_filter = (long < -88.1) * (long > -88.4)
lat_filter = (lat < 40.3) * (lat > 40.0)


print(data[long_filter * lat_filter])