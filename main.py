import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from baidu_apollo_plot import plot_scenario

plot_scenario(
    "testdata/gen_0_sce_6.00000",
    "data/maps/borregas_ave/base_map.bin",
    "out"
)