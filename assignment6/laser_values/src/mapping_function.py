#!/usr/bin/env python
import numpy as np
import sys
import collections
from math import sqrt


def mappingFunction(anglesList, deg):
    Angles = collections.namedtuple('Angles', ['original', 'actual'])
    rad = deg*(np.pi/180)
    x_vals = []
    y_vals = []
    for x in anglesList:
        x_vals.append(x.original)
        y_vals.append(x.actual)
    #print(x_vals)
    #print(y_vals)
    z=np.polyfit(x_vals, y_vals,3)
    #print(z)
    f=np.poly1d(z)
    #print(str(f))
    y = np.polyval(f,rad)
    #print(y)
    result = y * (180/np.pi)
    return result

def printTable(anglesList):
    print("Original Angle", '\t', "Actual Angle")
    print("--------------", '\t', "------------")
    for x in anglesList:
        print(x.original, '\t', x.actual)

Angles = collections.namedtuple('Angles', ['original', 'actual'])
anglesList = []
temp1 = Angles((0*(np.pi/180)), actual = 0.70862077)
temp2 = Angles((30*(np.pi/180)), actual = 0.47012)
temp3 = Angles((60*(np.pi/180)), actual = 0.21957)
temp4 = Angles((90*(np.pi/180)), actual = 0.07710)
temp5 = Angles((120*(np.pi/180)), actual = 0.04258)
temp6 = Angles((150*(np.pi/180)), actual = 0.3160)
temp7 = Angles((179*(np.pi/180)), actual = 0.4821790)

anglesList.append(temp1)
anglesList.append(temp2)
anglesList.append(temp3)
anglesList.append(temp4)
anglesList.append(temp5)
anglesList.append(temp6)
anglesList.append(temp7)

printTable(anglesList)
print(mappingFunction(anglesList,50))
