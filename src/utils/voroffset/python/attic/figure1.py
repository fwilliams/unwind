import os
import json
import glob
import math
import getpass
import subprocess
import matplotlib.pyplot as plt
# import seaborn

# data/
# data/models/
# data/results/
# data/results/num_threads
# data/results/num_dexels
# data/results/radius
# data/plots/num_threads
# data/plots/num_dexels
# data/plots/radius

from common import *

array_num_dexels = range(64, 512, 32)
array_num_threads = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
array_radii = [0.0125, 0.025, 0.0375, 0.05, 0.0625, 0.075, 0.0875, 0.1]
array_method = ["brute_force", "reverse_brute_force"]


for output_dir in [model_folder, json_folder_num_threads, json_folder_num_dexels,
                   json_folder_radius, plot_folder_num_threads,
                   plot_folder_num_dexels, plot_folder_radius]:
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)


def temp_opener(name, flag, mode=0o777):
    return os.open(name, flag | os.O_TEMPORARY, mode)


def region_plot(x_label, y_label, x_array, y_array_1, y_array_2, path_name, num_models):
    if num_models > 0:
        plt.figure()
        mid_time_cost = []
        min_time_cost = []
        max_time_cost = []

        mid_time_cost_brute_force = []
        min_time_cost_brute_force = []
        max_time_cost_brute_force = []

        for i in range(len(x_array)):
            record = []
            mean = 0
            std_dev = 0
            for k in range(num_models):
                record.append(y_array_1[k * len(x_array) + i])
                mean = mean + y_array_1[k * len(x_array) + i]
            mean = mean / len(record)
            for x in record:
                std_dev = std_dev + math.pow(x - mean, 2)
            std_dev = math.sqrt(std_dev / len(record))
            min_time_cost.append(mean - std_dev)
            max_time_cost.append(mean + std_dev)
            mid_time_cost.append(mean)

        l1 = plt.plot(x_array, mid_time_cost, 'k', color='#CC4F1B')
        plt.fill_between(x_array, min_time_cost, max_time_cost,
                         alpha=0.5, edgecolor='#CC4F1B', facecolor='#FF9848')
        if len(y_array_2) > 0:
            for i in range(len(x_array)):
                record_brute_force = []
                mean = 0
                std_dev = 0
                for k in range(num_models):
                    record_brute_force.append(y_array_2[k * len(x_array) + i])
                    mean = mean + y_array_2[k * len(x_array) + i]
                mean = mean / len(record_brute_force)
                for x in record_brute_force:
                    std_dev = std_dev + math.pow(x - mean, 2)
                std_dev = math.sqrt(std_dev / len(record_brute_force))
                record_brute_force.sort()
                min_time_cost_brute_force.append(mean - std_dev)
                max_time_cost_brute_force.append(mean + std_dev)
                mid_time_cost_brute_force.append(mean)

            l2 = plt.plot(x_array, mid_time_cost_brute_force,
                          'k', color='#3F7F4C')
            plt.fill_between(x_array, min_time_cost_brute_force, max_time_cost_brute_force,
                             alpha=0.5, edgecolor='#3F7F4C', facecolor='#7EFF99')
            plt.legend((l1[0], l2[0]), ('Our algorithm',
                                        'Brute force'), loc='upper right')
        else:
            plt.legend(l1[0], 'Our algorithm', loc='upper right')
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        result_plot = os.path.join(path_name, y_label + '_' + x_label + '_result.png')
        plt.savefig(result_plot)


def compute_for_threads_num():
    all_models = []
    for ext in ('*.stl', '*.obj'):
        all_models.extend(glob.glob(os.path.join(model_folder, ext)))
    x = 0
    # results for different number of threads
    for models in all_models:
        x = x + 1
        if x == 200:
            break
        model_name = os.path.split(models)[1]
        data = []
        result_json = os.path.join(
            json_folder_num_threads, os.path.splitext(model_name)[0] + '.json')
        input_model = os.path.join(model_folder, model_name)
        for n in array_num_threads:
            # call cmd line c++ code
            for method in array_method:
                args = [exe_path, '--input', input_model, '--num_dexels', str(512), '--radius', str(
                        0.025 * 512), '--padding', str(math.ceil(0.025 * 512)), '--num_thread', str(n), 
                        '--json', tmp_json, '--force', '--method', method, '-x']
                print(args)
                subprocess.check_call(args)
                with open(tmp_json, 'r') as f:
                    result = json.load(f)
                data.append(result)
        with open(result_json, 'w') as f:
            f.write(json.dumps(data, indent=4))


def compute_for_dexels_num():
    all_models = []
    for ext in ('*.stl', '*.obj'):
        all_models.extend(glob.glob(os.path.join(model_folder, ext)))
    # results for different number of dexels
    for models in all_models:
        model_name = os.path.split(models)[1]
        data = []
        result_json = os.path.join(
            json_folder_num_dexels, os.path.splitext(model_name)[0] + '.json')
        input_model = os.path.join(model_folder, model_name)
        for n in array_num_dexels:
            # call cmd line c++ code
            for method in array_method:
                args = [exe_path, '--input', input_model, '--num_dexels', str(
                        n), '--radius', str(0.05 * n), '--padding', str(math.ceil(0.05 * n)), 
                        '--num_thread', str(1), '--json', tmp_json, '--force', '--method', method, '-x']
                print(args)
                subprocess.check_call(args)
                with open(tmp_json, 'r') as f:
                    result = json.load(f)
                data.append(result)
        with open(result_json, 'w') as f:
            f.write(json.dumps(data, indent=4))


def compute_for_radius():
    all_models = []
    for ext in ('*.stl', '*.obj'):
        all_models.extend(glob.glob(os.path.join(model_folder, ext)))
    # results for different radii
    for models in all_models:
        model_name = os.path.split(models)[1]
        data = []
        result_json = os.path.join(
            json_folder_radius, os.path.splitext(model_name)[0] + '.json')
        input_model = os.path.join(model_folder, model_name)
        for n in array_radii:
            # call cmd line c++ code
            for method in array_method:
                args = [exe_path,
                        '--input', input_model, '--num_dexels', str(512), '--radius', str(n * 512), 
                        '--padding', str(math.ceil(n * 512)), '--num_thread', str(12), '--json', tmp_json, 
                        '--force', '--method', method, '-x']
                print(args)
                subprocess.check_call(args)
                with open(tmp_json, 'r') as f:
                    result = json.load(f)
                data.append(result)
        with open(result_json, 'w') as f:
            f.write(json.dumps(data, indent=4))


def plot_for_threads_num():
    json_folder = os.path.join(json_folder_num_threads, '', '*.json')
    count = 0
    time_cost = []
    time_cost_brute_force = []
    xlabel = 'num_threads'
    ylabel = 'time_cost'
    for json_file in glob.glob(json_folder):
        count = count + 1
        # result_plot = os.path.join(plot_folder_num_threads,os.path.splitext(plot_name)[0]+'.png')
        with open(json_file, 'r') as f:
            result = json.load(f)
        for i in range(len(array_num_threads)):  # number of threads
            time_cost.append(result[2 * i]['time_cost'] /
                             result[0]['time_cost'])
            time_cost_brute_force.append(result[2 * i + 1]['time_cost'] / result[1]['time_cost'])
    region_plot(xlabel, ylabel, array_num_threads, time_cost,
                time_cost_brute_force, plot_folder_num_threads, count)


def plot_for_dexels_num():
    json_folder = os.path.join(json_folder_num_dexels, '', '*.json')
    # num_dexels = []
    time_cost = []
    time_cost_brute_force = []
    count = 0
    xlabel = 'num_dexels'
    ylabel = 'time_cost'
    for json_file in glob.glob(json_folder):
        count = count + 1
        plot_name = os.path.split(json_file)[1]
        # result_plot = os.path.join(plot_folder_num_threads,os.path.splitext(plot_name)[0]+'.png')
        with open(json_file, 'r') as f:
            result = json.load(f)
        for i in range(len(array_num_dexels)):  # number of threads
            time_cost.append(result[2 * i]['time_cost'])
            time_cost_brute_force.append(result[2 * i + 1]['time_cost'])
            # num_dexels.append(result[i][0]['num_voxels'])
    region_plot(xlabel, ylabel, array_num_dexels, time_cost,
                time_cost_brute_force, plot_folder_num_dexels, count)


def plot_for_radius():
    json_folder = os.path.join(json_folder_radius, '', '*.json')
    time_cost = []
    time_cost_brute_force = []
    count = 0
    xlabel = 'num_radii'
    ylabel = 'time_cost'
    for json_file in glob.glob(json_folder):
        # plot_name = os.path.split(json_file)[1]
        count = count + 1
        with open(json_file, 'r') as f:
            result = json.load(f)
        for i in range(len(array_radii)):  # number of threads
            time_cost.append(result[2 * i]['time_cost'])
            time_cost_brute_force.append(result[2 * i + 1]['time_cost'])
    region_plot(xlabel, ylabel, array_radii, time_cost,
                time_cost_brute_force, plot_folder_radius, count)


def plot_error_threads():
    json_folder = os.path.join(json_folder_num_threads, '', '*.json')
    count = 0
    error = []
    xlabel = 'num_threads'
    ylabel = 'error'
    for json_file in glob.glob(json_folder):
        count = count + 1
        # result_plot = os.path.join(plot_folder_num_threads,os.path.splitext(plot_name)[0]+'.png')
        with open(json_file, 'r') as f:
            result = json.load(f)
        for i in range(len(array_num_threads)):  # number of threads
            error.append(result[2 * i]['xor_volume'])
    region_plot(xlabel, ylabel, array_num_threads, error,
                [], plot_folder_num_threads, count)


def plot_error_dexels():
    json_folder = os.path.join(json_folder_num_dexels, '', '*.json')
    # num_dexels = []
    error = []
    count = 0
    xlabel = 'num_dexels'
    ylabel = 'error'
    for json_file in glob.glob(json_folder):
        count = count + 1
        plot_name = os.path.split(json_file)[1]
        # result_plot = os.path.join(plot_folder_num_threads,os.path.splitext(plot_name)[0]+'.png')
        with open(json_file, 'r') as f:
            result = json.load(f)
        for i in range(len(array_num_dexels)):  # number of threads
            error.append(result[2 * i]['xor_volume'])
    region_plot(xlabel, ylabel, array_num_dexels, error,
                [], plot_folder_num_dexels, count)


def plot_error_radius():
    json_folder = os.path.join(json_folder_radius, '', '*.json')
    error = []
    count = 0
    xlabel = 'num_radii'
    ylabel = 'error'
    for json_file in glob.glob(json_folder):
        # plot_name = os.path.split(json_file)[1]
        count = count + 1
        with open(json_file, 'r') as f:
            result = json.load(f)
        for i in range(len(array_radii)):  # number of threads
            error.append(result[2 * i]['xor_volume'])
    region_plot(xlabel, ylabel, array_radii, error,
                [], plot_folder_radius, count)


if __name__ == "__main__":
    # compute_for_threads_num()
    # plot_for_threads_num()
    # plot_error_threads()
    compute_for_dexels_num()
    plot_for_dexels_num()
    # plot_error_dexels()
    # compute_for_radius()
    # plot_for_radius()
    # plot_error_radius()
