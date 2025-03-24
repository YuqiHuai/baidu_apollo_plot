import argparse
import json
from pathlib import Path
from typing import List

from rich_argparse import RichHelpFormatter

from baidu_apollo_plot import plot_scenario
from apollo.map_service import load_map_service
from apollo.vehicle_param import VehicleParam


def set_up_parser():
    parser = argparse.ArgumentParser(
        description='Apollo scenario analysis with extendable oracles',
        formatter_class=RichHelpFormatter,
    )

    parser.add_argument(
        '-v',
        '--vehicle',
        type=str,
        nargs=1,
        required=True,
        help='Specify path of vehicle_param protobuf file',
    )
    parser.add_argument(
        '-m',
        '--map',
        type=str,
        nargs=1,
        required=True,
        help='Specify path of the HD map',
    )

    parser.add_argument('record', help='Record to plot')
    parser.add_argument('out', help='Location to save gif')

    return parser


def main():
    # oracle_manager = OracleExtensionManager()
    # print(oracle_manager.available_extensions)

    parser = set_up_parser()

    # Verify arguments
    args = parser.parse_args()
    args_dict = vars(args)

    vehicle_param_file = Path(args_dict.get('vehicle')[0])
    map_file = Path(args_dict.get('map')[0])

    record_file = Path(args_dict.get('record'))
    out_file = Path(args_dict.get('out'))


    # load dependencies and run tests
    map_service = load_map_service(map_file)
    vehicle_param = VehicleParam.load_from_file(vehicle_param_file)

    plot_scenario(
        "testdata/scenario_0.resim_1.00000", "data/maps/borregas_ave/base_map.bin", "out"
    )