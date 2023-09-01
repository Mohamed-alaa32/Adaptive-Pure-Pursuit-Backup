#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
from pandas import *

# reading CSV file
# data = read_csv("/home/mohammed/fsaiworkspace/src/formula/src/navigation/adaptive_pure_pursuit/testing/points.csv")
data = read_csv(
    "/home/mohammed/fsaiworkspace/src/formula/src/navigation/adaptive_pure_pursuit/testing/xlist.csv"
)
print(data[data.columns[0]].tolist())
# print(data.columns[0])
# converting column data to list
# month = data['month_number'].tolist()
# fc = data['facecream'].tolist()
# fw = data['facewash'].tolist()
# tp = data['toothpaste'].tolist()
# sh = data['shampoo'].tolist()
