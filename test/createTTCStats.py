import os
import csv
import math
import pdb

list_of_files = os.listdir("../build/")

list_of_ttc_files = []
for file in list_of_files:
    if file.endswith("_ttc_measurement.csv"):
        list_of_ttc_files.append(file)

print(list_of_ttc_files)
ttc_dictionary = {}
det_desc_list = []
for file in list_of_ttc_files:
    with open("../build/" + file) as csvfile:
        inputfile = csv.reader(csvfile, delimiter=",")
        endIndex = file.find("_ttc_measurement.csv")

        det_desc = file[0:endIndex]
        det_desc_list.append(det_desc)

        ttc_dictionary[det_desc] = {}

        for row in inputfile:
            try:
                camera_ttc = float(row[2])
                if not math.isinf(camera_ttc) and not math.isnan(camera_ttc):
                    imageIndex = int(row[0])
                    ttc_dictionary[det_desc][imageIndex] = camera_ttc
            except:
                pass


with open("TTCStats.csv", 'w') as csvfile: 
    csvfile.write("detector descriptor combination, number of frames, average ttc")
    csvfile.write("\n")
    for det_desc in det_desc_list:
        row_string = det_desc + ","
        numberOfFrames = 0
        sum_ttc = 0

        for key, value in ttc_dictionary[det_desc].items():
            if key < 50:
                sum_ttc += value
                numberOfFrames += 1

        row_string += str(numberOfFrames) + ","
        if numberOfFrames > 0 :
            row_string += str(sum_ttc / numberOfFrames)

        csvfile.write(row_string)
        csvfile.write("\n")

csvfile.close()

with open("CombinedTTC.csv", 'w') as csvfile:
    header = "image index"
    for det_desc in det_desc_list:
        header += ", " + det_desc
    csvfile.write(header)
    csvfile.write("\n")

    for imgIndex in range(1,51):
        
        row_string = str(imgIndex)

        for det_desc in det_desc_list:
            try:
                row_string += ", " + str(ttc_dictionary[det_desc][imgIndex])
            except:
                row_string += ","
            
        csvfile.write(row_string)
        csvfile.write("\n")
        