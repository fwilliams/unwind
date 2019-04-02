#!/usr/bin/env python
# -*- coding: utf-8 -*-

# System libs
import os
import math
import glob
import getpass
from enum import Enum

# Third party libs
import numpy
import matplotlib

def figsize(scale):
    fig_width_pt = 252.9449                       # Get this from LaTeX using \the\textwidth
    inches_per_pt = 1.0/72.27                     # Convert pt to inch
    golden_mean = (numpy.sqrt(5.0)-1.0)/2.0       # Aesthetic ratio (you could change this)
    fig_width = fig_width_pt*inches_per_pt*scale  # width in inches
    fig_height = fig_width*golden_mean            # height in inches
    fig_size = [fig_width,fig_height]
    return fig_size

# Configure matplotlib
# http://bkanuka.com/articles/native-latex-plots/
matplotlib.use("pgf")
pgf_with_custom_preamble = {
    # "font.family": "serif",         # use serif/main font for text elements
    "text.usetex": True,            # use inline math for ticks
    "font.size": 10,
    "axes.labelsize": 10,           # LaTeX default is 10pt font.
    "font.size": 10,
    "legend.fontsize": 8,           # Make the legend/label fonts a little smaller
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.linewidth": 0.5,
    "xtick.major.width": 0.5,
    "ytick.major.width": 0.5,
    "xtick.minor.width": 0.25,
    "ytick.minor.width": 0.25,
    "patch.linewidth": 0.5,
    "lines.linewidth": 1,
    "lines.markersize": 1,
    "figure.figsize": figsize(1),   # default fig size of 0.9 textwidth
    "pgf.rcfonts": False,           # don't setup fonts from rc parameters
    "pgf.preamble": [
        # "\\usepackage[largesc]{newpxtext}",
        # "\\usepackage{newpxmath}",
    ]
}

matplotlib.rcParams.update(pgf_with_custom_preamble)
import matplotlib.pyplot as plt

# Hard-coded root data folder
project_folder = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
exe_path = os.path.join(project_folder, 'build/offset3d')
data_folder = os.path.join(project_folder, 'data')
if not os.path.exists(exe_path):
    raise FileNotFoundError("Executable file not found. Please compile or update hard-coded path:\n" + exe_path)
if not os.path.exists(data_folder):
    raise FileNotFoundError("Data folder not found. Please download the dataset or update hard-coded path:\n" + data_folder)

# Relative folders and files
result_folder = os.path.join(data_folder, "results")
tmp_json = "foo.json"

# Captions for the plots
legends = {"time": "Time (s)",
           "num_threads": "Number of Threads",
           "radius": "Dilation Radius (Number of Dexels)",
           "radius_relative": "Dilation Radius",
           "num_dexels": "Grid Size"}


def newfig(width):
    plt.clf()
    fig = plt.figure(figsize=figsize(width))
    ax = fig.add_subplot(111)
    return fig, ax


def savefig(filename):
    # plt.savefig('{}.pgf'.format(filename), bbox_inches='tight')
    plt.savefig('{}'.format(filename), bbox_inches='tight')


# Different datasets and their models
class Dataset(Enum):
    basic = 'basic'
    extra = 'extra'

    def models(self):
        """
        List 3D models in a given dataset folder.

        Returns:
            list: List of filename of 3D models present in the dataset folder.
        """
        folder_name = os.path.join(data_folder, 'models', self.value)
        models = []
        for ext in ('*.stl', '*.obj', '*.ply', '*.off'):
            models.extend(glob.glob(os.path.join(folder_name, ext)))
        return models


def ensure_folder_exists(filepath):
    """
    Ensure parent folders exists before writing a new file.

    Args:
        filepath (str): Input file path.
    """
    dirname = os.path.dirname(filepath)
    if not os.path.exists(dirname):
        os.makedirs(dirname)


def convergence_rate(x, y):
    x, y = zip(*sorted(zip(x, y))[:])
    w = numpy.polyfit(numpy.log(x[:]), numpy.log(y[:]), 1)
    p = numpy.poly1d(w)
    z = numpy.exp(p(numpy.log(x)))
    return w[0]


def region_plot(x_array, y_array, curve_color, face_color, curve_name, curve_style, point_maker, show_conv=False, show_legend=True):
    """
    Plot a region curve, where the region drawn has lower and upper bounds equals
    to the standard deviation of the input dataset.

    Args:
        x_array (list): Input labels on the X axis.
        y_array (numpy.array): Input labels on the Y axis, where each line represents a different sample. It must have a number of columns = len(x_array).
    """
    assert (len(x_array) == y_array.shape[1]), "Dimension mismatch between plotted arrays"
    x_array = numpy.array(x_array)[1:]
    y_array = numpy.array(y_array)[:,1:]
    y_mean = numpy.mean(y_array, axis=0)
    y_stdev = numpy.std(y_array, axis=0)

    w = convergence_rate(x_array, y_mean)
    if not show_legend:
        curve_name = None
    if show_conv:
        # print(curve_name, w)
        curve_name = "$\mu = {:.2f}$".format(w)

    plt.plot(x_array, y_mean, color=curve_color, label=curve_name,
             linestyle=curve_style, marker=point_maker)
    plt.fill_between(x_array, y_mean - y_stdev, y_mean + y_stdev, alpha=0.10,
                     edgecolor=curve_color, facecolor=face_color, linewidth=0.1)

    return w


def main_test():
    my_dataset = Dataset.basic
    print(my_dataset.models())


if __name__ == '__main__':
    main_test()
