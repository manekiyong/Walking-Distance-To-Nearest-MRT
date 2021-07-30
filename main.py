import re
import json
import pandas as pd
import math
import os

import osmnx as ox
import networkx as nx
ox.config(use_cache=True, log_console=True)

def getCentroid(arr):
    if len(arr) == 3 and type(arr[0])==float:
        return arr[1], arr[0]
    else:
        noOfCoords=0
        SumLat=0
        SumLong = 0
        for a in arr:
            aLat, aLong = getCentroid(a)
            SumLat+=aLat
            SumLong+=aLong
            noOfCoords+=1
        avgLat = SumLat/noOfCoords
        avgLong = SumLong/noOfCoords
        return avgLat, avgLong


def walkingDist(graph, orig_node, mrt_node):
    length = nx.shortest_path_length(G=graph, source=orig_node, target=mrt_node, weight='length')
    return length


def getRoute(G, orig_node, target_node, LatLong = True):  # LatLong = True: [Lat, Long]; False: [Long, Lat]
    route = nx.shortest_path(G, orig_node, target_node, 'length')
    routeLatLong = []
    route.insert(0, orig_node)
    for i in route:
        latlong = []
        node = G.nodes[i]
        if LatLong:
            latlong.append(node['y'])
            latlong.append(node['x'])
        else:
            latlong.append(node['x'])
            latlong.append(node['y'])
        routeLatLong.append(latlong)
    return str(routeLatLong)

def findNearestMRT3(graph, lat, long, MRTdf):
    shortestDist = 99999
    shortestDistIndex = 0
    result = False
    r = 0.019
    orig_node = ox.distance.nearest_nodes(graph, long, lat)
    count=0
    while count==0:
        for i, rows in MRTdf.iterrows():
            MRTLat = rows['Centroid'][0]
            MRTLong = rows['Centroid'][1]
            if(abs(MRTLat-lat)<r and abs(MRTLong-long)<r): # Find the number of stations that has lat/long of +- 0.019 (Approx 1.9km) of the HDB's coordinate
                count+=1
        if(count==0):
            r+=0.01
    print("MRT found within", str(r), "Stations found:", count)
    for i, rows in MRTdf.iterrows():
        MRTLat = rows['Centroid'][0]
        MRTLong = rows['Centroid'][1]
        if(abs(MRTLat-lat)>r or abs(MRTLong-long)>r): # If the MRT station is further than the 3.8km square, then skip. 
            continue
        MRT_node = rows['NodeID']
        curDist = walkingDist(graph, orig_node, MRT_node)
        if(curDist < shortestDist):
            shortestDist = curDist
            shortestDistIndex = i
            nearestMRTNode = MRT_node
    shortestRoute = getRoute(graph, orig_node, nearestMRTNode)
    return shortestDist, MRTdf.loc[shortestDistIndex], shortestRoute

def getFileName(fileType):
    arr = os.listdir()
    arr.sort()
    count = 1
    fileMap = []
    for i in arr:
        if(i[-len(fileType):]!=fileType):
            continue
        print(str(count)+")", i)
        fileMap.append(i)
        count+=1
    print("0) Exit")
    choice = int(input())
    while choice < 0 or choice > len(fileMap):
        print("Please enter a valid input: ")
        choice = int(input())
    if choice == 0:
        exit()
    fileName = fileMap[choice-1]
    return fileName


if __name__ == "__main__":
    #Select File
    print("Select MRT geojson File:")
    geojsonFileName = getFileName('.geojson')
    print("Select HDB Address Map .csv File:")
    csvFileName = getFileName('.csv')

    #Load selected files
    with open(geojsonFileName) as f:
        data = json.load(f)
    addrMap = pd.read_csv(csvFileName, index_col = 0)

    # Load Map using Bounding Box coordinates of Singapore.
    G = ox.graph.graph_from_bbox(1.4809, 1.1304753, 103.58, 104.0120359, network_type='walk')

    # Reconstruct GeoJSON format into Dataframe (later exported as CSV) and finding all the MRT's OSMNX node ID. 
    MRTdf = pd.DataFrame()
    for i in data['features']:
        temp_dict = {'id': i['properties']['Name']}
        for j in i['properties']['Description'].split("<th>")[1:]:
            header = j.split("</th>")[0]
            res = re.search('<td>(.*)</td>', j)
            val = res.group(1)
            temp_dict[header]=val
        temp_dict['Polygon'] = i['geometry']['coordinates']
        centroid = getCentroid(i['geometry']['coordinates'])
        temp_dict['Centroid'] = centroid
        stnNode = ox.distance.nearest_nodes(G, centroid[1], centroid[0])
        temp_dict['NodeID']=stnNode
        MRTdf = MRTdf.append(temp_dict, ignore_index=True)

    # Find all address's nearest MRT station
    count = 0
    totalLen = len(addrMap)
    addrMap["Shortest Path"]=""
    for i, rows in addrMap.iterrows():
        dist, nearestRow, shortestRoute = findNearestMRT3(G, rows['Latitude'], rows['Longitude'], MRTdf)
        addrMap.at[i, 'Distance to MRT(m)'] = dist
        addrMap.at[i, 'Nearest MRT'] = nearestRow['NAME']
        addrMap.at[i, 'Shortest Path'] = shortestRoute
        count+=1
        print("Progess:", str(count)+"/"+str(totalLen))

    # Export Results
    MRTExportFileName = geojsonFileName[:-8]+".csv"
    MRTdf.to_csv(MRTExportFileName)
    csvExportFileName = csvFileName[:-4]+"-WalkDist.csv"
    addrMap.to_csv(csvExportFileName)