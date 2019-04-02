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
array_num_dexels = numpy.round(numpy.geomspace(256, 2048, num=8)).astype(int)
array_num_threads = [1, 3, 6]
array_radii = [0.025, 0.05]


def batch_test_erosion_dilation(input_models, output_json):
    """
    Batch test erosion vs dilation on various dexel sizes, with 2 different radii.

    Args:
        input_models (list): List of input models to test on.
        output_json (str): Destination file for the result data.
    """
    database = {}
    # Results for different number of dexels and different radii
    for model_path in input_models:
        model_name = os.path.split(model_path)[1]
        model_data = []
        for n in array_num_dexels:
            for r in array_radii:
                # Dilation
                args_dilation = [common.exe_path,
                                 '--input', model_path,
                                 '--num_dexels', str(n),
                                 '--radius', str(r * n),
                                 '--padding', str(math.ceil(r * n)),
                                 '--num_thread', str(3),
                                 '--json', common.tmp_json,
                                 '--force']
                subprocess.check_call(args_dilation)
                with open(common.tmp_json, 'r') as f:
                    entry = json.load(f)
                    entry['radius_relative'] = r
                    model_data.append(entry)
                # Erosion
                args_erosion = [common.exe_path,
                                '--input', model_path,
                                '--num_dexels', str(n),
                                '--radius', str(r),
                                '--padding', str(math.ceil(r * n)),
                                '--num_thread', str(3),
                                '--json', common.tmp_json,
                                '--force',
                                '--apply', 'erosion']
                subprocess.check_call(args_erosion)
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


def plot_curves(file_name, type_name, show_ylabel=True, show_legend=True):
    json_file = os.path.join(common.result_folder, file_name)
    xlabel = 'num_dexels'
    ylabel = 'time'
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
            key = (entry['operation'], entry['radius_relative'])
            if key not in y_arrays:
                y_arrays[key] = numpy.zeros((len(database.keys()), len(x_array)))
            j = x_array.index(entry[xlabel])
            y_arrays[key][i, j] = entry[ylabel] / 1000
    # Draw plot
    colors = itertools.cycle(('C0', 'C1', 'C2', 'C3'))
    markers = itertools.cycle(('o', '*', 'd', '^'))  # ',', +', '.', 'o', '*')
    linestyles = itertools.cycle(('-', '--', '-.', ':'))
    # plt.figure()
    fig, ax = common.newfig(1)
    show_conv = (type_name == 'loglog')
    for key, y_array in sorted(y_arrays.items()):
        r = key[1]
        c = next(colors)
        e = key[0].title()
        name = '{}, $r = {}$'.format(e, r)
        common.region_plot(x_array, y_array, c, c, name, next(linestyles), next(markers), show_conv)
    if type_name == 'loglog':
        plt.loglog()
        ax.xaxis.set_major_formatter(plticker.LogFormatter(labelOnlyBase=True))
        ax.xaxis.set_minor_formatter(plticker.LogFormatter(labelOnlyBase=False
            , minor_thresholds=(4, 0.5)))
    plt.xlabel(common.legends[xlabel], fontsize='large')
    if show_ylabel:
        plt.ylabel(common.legends[ylabel], fontsize='large')
    # if show_legend:
    plt.legend(loc='upper left')
    result_plot = os.path.join(common.result_folder,
                               ylabel + '_' + os.path.splitext(file_name)[0] + '_' + type_name + '.pdf')
    # plt.savefig(result_plot)
    common.savefig(result_plot)


if __name__ == "__main__":
    data_file = 'dilation_vs_erosion.json'
    # batch_test_erosion_dilation(common.Dataset.basic.models(), data_file)
    plot_curves(data_file, 'linear', True, True)
    plot_curves(data_file, 'loglog', False, False)
