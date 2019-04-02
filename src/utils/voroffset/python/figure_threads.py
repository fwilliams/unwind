#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import matplotlib.ticker as plticker

# Ranges for the batch test
array_num_dexels = numpy.round(numpy.geomspace(256, 1024, num=7)).astype(int)
array_num_threads = [1, 3, 6]
array_radii = [0.0125, 0.025, 0.0375, 0.05, 0.0625, 0.075, 0.0875, 0.1]
array_method = ["ours", "brute_force"]


def batch_test_num_dexels(input_models, output_json):
    """
    Batch test the effect of varying dexel sizes, with a radius of 5% num_dexels.

    Args:
        input_models (list): List of input models to test on.
        output_json (str): Destination file for the result data.
    """
    r = 0.05  # Dilation radius (relative)
    database = {}
    # Results for different number of dexels
    for model_path in input_models:
        model_name = os.path.split(model_path)[1]
        model_data = []
        for n in array_num_dexels:
            for method in array_method:
                for k in array_num_threads:
                    args = [
                        common.exe_path,
                        '--input', model_path,
                        '--num_dexels', str(n),
                        '--radius', str(r * n),
                        '--padding', str(math.ceil(r * n)),
                        '--num_thread', str(k),
                        '--json', common.tmp_json,
                        '--force',
                        '--method', method]
                    print(' '.join(args))
                    subprocess.check_call(args)
                    with open(common.tmp_json, 'r') as f:
                        entry = json.load(f)
                        entry['radius_relative'] = r
                        model_data.append(entry)
        database[model_name] = model_data
    # Write results in a file
    json_filename = os.path.join(common.result_folder, output_json)
    common.ensure_folder_exists(json_filename)
    with open(json_filename, 'w') as f:
        f.write(json.dumps(database, indent=4))


def batch_test_radius(input_models, output_json):
    """
    Batch test the effect of varying radii, with a number of dexels of 512.

    Args:
        input_models (list): List of input models to test on.
        output_json (str): Destination file for the result data.
    """
    n = 512  # Number of dexels
    database = {}
    # Results for different radii
    for model_path in input_models:
        model_name = os.path.split(model_path)[1]
        model_data = []
        for r in array_radii:
            for method in array_method:
                for k in array_num_threads:
                    args = [
                        common.exe_path,
                        '--input', model_path,
                        '--num_dexels', str(n),
                        '--radius', str(r * n),
                        '--padding', str(math.ceil(r * n)),
                        '--num_thread', str(k),
                        '--json', common.tmp_json,
                        '--force',
                        '--method', method]
                    print(' '.join(args))
                    subprocess.check_call(args)
                    with open(common.tmp_json, 'r') as f:
                        entry = json.load(f)
                        entry['radius_relative'] = r
                        model_data.append(entry)
        database[model_name] = model_data
    # Write results in a file
    json_filename = os.path.join(common.result_folder, output_json)
    common.ensure_folder_exists(json_filename)
    with open(json_filename, 'w') as f:
        f.write(json.dumps(database, indent=4))


def plot_curves(file_name, type_name, xlabel, ylabel='time', show_ylabel=True, show_legend=True):
    json_file = os.path.join(common.result_folder, file_name)
    with open(json_file, 'r') as f:
        database = json.load(f)
    # Get list of x entries
    x_array = set()
    for entries in database.values():
        x_array = x_array | set(entry[xlabel] for entry in entries)
    x_array = sorted(x_array)
    # Get list of y entries
    y_arrays = {}
    for i, entries in enumerate(database.values()):
        for entry in entries:
            key = (entry['method'], entry['num_threads'])
            if key not in y_arrays:
                y_arrays[key] = numpy.zeros((len(database.keys()), len(x_array)))
            j = x_array.index(entry[xlabel])
            y = entry[ylabel]
            y_arrays[key][i, j] = (y / 1000 if ylabel == 'time' else y)
    # Draw plot
    colors = itertools.cycle(('C0', 'C1'))
    markers = itertools.cycle(('*', '^', 'o'))  # ',', +', '.', 'o', '*')
    linestyles = itertools.cycle((':', '--', '-'))
    # plt.figure()
    fig, ax = common.newfig(1)
    old_type = None
    color = next(colors)
    show_conv = (type_name == 'loglog')
    for key, y_array in y_arrays.items():
        nt = key[1]
        name = ('Ours' if key[0] == 'ours' else 'Brute force') + ' with ' + str(nt) + ' thread'
        name = name + 's' if nt > 0 else None
        if old_type != key[0]:
            color = next(colors)
            show_conv = (type_name == 'loglog')
        common.region_plot(x_array, y_array, color, color, name, next(linestyles), next(markers), show_conv, show_legend)
        show_conv = False
        old_type = key[0]
    if type_name == 'loglog':
        plt.loglog()
        ax.xaxis.set_major_formatter(plticker.LogFormatter(labelOnlyBase=True))
        ax.xaxis.set_minor_formatter(plticker.LogFormatter(labelOnlyBase=False
            , minor_thresholds=(4, 0.5)))
    plt.xlabel(common.legends[xlabel], fontsize='large')
    if show_ylabel:
        plt.ylabel(common.legends[ylabel], fontsize='large')
    if show_legend or (type_name == 'loglog'):
        plt.legend(loc='upper left')
    result_plot = os.path.join(common.result_folder,
                               ylabel + '_' + xlabel + '_' + type_name + '.pdf')
    common.savefig(result_plot)


def main():
    data_file = 'num_dexels.json'
    # batch_test_num_dexels(common.Dataset.basic.models(), data_file)
    plot_curves(data_file, 'linear', 'num_dexels', 'time', True, True)
    plot_curves(data_file, 'loglog', 'num_dexels', 'time', False, False)

    data_file = 'radius.json'
    # batch_test_radius(common.Dataset.basic.models(), data_file)
    plot_curves(data_file, 'linear', 'radius_relative', 'time', True, False)
    plot_curves(data_file, 'loglog', 'radius_relative', 'time', False, False)


if __name__ == "__main__":
    main()
