# System libs
import os
import json
import math
import itertools
import subprocess

# Third party libs
import numpy

# Local libs
import common
import matplotlib.pyplot as plt

# Ranges for the batch test
array_num_dexels = [256, 512]
array_num_threads = [1, 4]
array_radii = [0.0125, 0.025, 0.05, 0.1]

map_radii = {
    "buddha.ply": [0.0144573, 0.0289147, 0.0578294, 0.115659],
    "filigree.ply": [0.0177713, 0.0355427, 0.0710854, 0.142171],
    "octa-flower.ply": [0.0197374, 0.0394748, 0.0789496, 0.157899],
    "vase-lion.ply": [0.0172792, 0.0345585, 0.069117, 0.138234],
}

def comparasion_dilation(input_models, output_json):
    """
    Comparasion with the method mentioned in Wang's paper

    Args:
        input_models (list): List of input models to test on.
        output_json (str): Destination file for the result data.
    """
    database = []
    # Results for different number of dexels and different radii
    for model_path in input_models:
        model_name = os.path.split(model_path)[1]
        assert(model_name in map_radii.keys())
        for num_dexels in array_num_dexels:
            for num_threads in array_num_threads:
                for i, r in enumerate(array_radii):
                    r_mm = map_radii[model_name][i]
                    # Dilation
                    args_dilation = [
                        common.exe_path,
                        '--input', model_path,
                        '--num_dexels', str(num_dexels),
                        '--radius', str(r_mm),
                        '--padding', str(math.ceil(r_mm * num_dexels)),
                        '--num_thread', str(num_threads),
                        '--json', common.tmp_json,
                        '--output', 'tmp.ply',
                        '--radius_in_mm',
                        '--force']
                    subprocess.check_call(args_dilation)
                    with open(common.tmp_json, 'r') as f:
                        entry = json.load(f)
                        entry['radius_relative'] = r
                        entry['radius_mm'] = r_mm
                        entry['model'] = model_name
                        database.append(entry)
    # Write results in a file
    json_filename = os.path.join(common.result_folder, output_json)
    common.ensure_folder_exists(json_filename)
    with open(json_filename, 'w') as f:
        f.write(json.dumps(database, indent=4))

if __name__ == "__main__":
    data_file = 'comparison.json'
    comparasion_dilation(common.Dataset.compare.models(), data_file)
