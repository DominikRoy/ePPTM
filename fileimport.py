import logging
import datetime
import json
from pssh.clients import ParallelSSHClient
from pssh.config import HostConfig
import pandas as pd
#from deepface.commons.distance import findThreshold
import csv
try:
    with open("/home/domg/Documents/PPTM/extension/pptm-uav-poc/log/powertop_2024-05-25_18-14-54.csv", 'r') as file:
            rows = list(csv.reader(file, delimiter=';'))
            # get the software power consumers overview
            begin = rows.index([' *  *  *   Overview of Software Power Consumers   *  *  *'])
            end = rows.index([' *  *  *   Device Power Report   *  *  *'])
            headers = rows[begin+2]
            headers.append("None")
            print(rows[begin+3:end-5])
            print(rows[begin+3:end-4])
            df = pd.DataFrame(rows[begin+3:end-2],)
            print(df)
            # get the energy consumption readings of cos_dist
            # fixme: hardcoded exec name, should be read from config but powertop doesn't print full name
            mask = df[6].str.contains('python3.9')
            mask.fillna(False, inplace=True)
            energy_series = df[mask][7]

            # combine readings from all threads and compute total energy (E = P * T)
            total_watts = 0
            for row in energy_series:
                value, unit = row.split()
                value = float(value)
                if unit == 'mW': 
                    value*=0.001
                elif unit == 'uW':
                    value*=0.000001
                elif unit == 'W':
                    pass
                else:
                    print(None)
                total_watts += value
            total_joules = total_watts * 200



finally:

    print(energy_series, total_joules)