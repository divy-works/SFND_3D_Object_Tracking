import os
import csv
import math
import pdb

list_of_files = os.listdir("../build/")

list_of_ttc_files = []
for file in list_of_files:
    if file.endswith("_measurementLog.csv"):
        list_of_ttc_files.append(file)

print(list_of_ttc_files)
matchingPointsDict = {}
det_desc_list = []

for file in list_of_ttc_files:
    with open("../build/" + file) as csvfile:
        inputfile = csv.reader(csvfile, delimiter=",")
        endIndex = file.find("_measurementLog.csv")

        det_desc = file[0:endIndex]
        det_desc_list.append(det_desc)

        matchingPointsDict[det_desc] = {}

        for row in inputfile:
            try:
                imageIndex = int(row[0])
                numberOfMatchingPoints = int(row[1])
                matchingPointsDict[det_desc][imageIndex] = numberOfMatchingPoints
            except:
                pass

with open("MatchingPointsStats.csv", 'w') as csvfile: 
    csvfile.write("detector descriptor combination, number of frames, average matching points")
    csvfile.write("\n")
    for det_desc in det_desc_list:
        row_string = det_desc + ","
        numberOfFrames = 0
        sum_num_matching_points = 0

        for key, value in matchingPointsDict[det_desc].items():
            if key < 50:
                sum_num_matching_points += value
                numberOfFrames += 1

        row_string += str(numberOfFrames) + ","
        if numberOfFrames > 0 :
            row_string += str(sum_num_matching_points / numberOfFrames)

        csvfile.write(row_string)
        csvfile.write("\n")

csvfile.close()