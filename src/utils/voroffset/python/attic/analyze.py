# System libs
import os
import json

# Third party libs
import numpy
import pandas
from pandas.io.json import json_normalize

# Local libs
import common

formatters = {
    'time': lambda x: '{:,d}s {:d}ms'.format(*map(int, divmod(x, 1000))),
}

def main():
    data_file = 'comparison.json'
    json_filename = os.path.join(common.result_folder, data_file)
    with open(json_filename) as f:
        json_data = json.load(f)
    db = []
    for entry in json_data:
        x, y = entry['grid_size']
        del entry['grid_size']
        del entry['model_name']
        entry['grid_size_x'] = x
        entry['grid_size_y'] = y
        db.append(entry)
    df = json_normalize(db)

    gb = df.groupby(['num_threads', 'num_dexels', 'model', 'radius_relative'])
    print(gb.mean()['time'].to_string(float_format=formatters['time']))


if __name__ == "__main__":
    main()
