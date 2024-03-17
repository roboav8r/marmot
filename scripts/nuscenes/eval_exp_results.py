#!/usr/bin/env python3

import os
import json
import subprocess
import argparse
import re

from pathlib import Path

# from ament_index_python.packages import get_package_share_directory


class EvalExpResults():
    def __init__(self,args):
        # self.results_dir = os.path.join(get_package_share_directory('ros_tracking'),args.results_dir)
        self.results_dir = os.path.join(Path(__file__).parent.parent, args.results_dir)
        self.split = args.split
        self.nuscenes_dir = args.nuscenes_dir
        self.dataset = args.dataset

    def evaluate_results(self):

        file_object = open(os.path.join(self.results_dir, "evaluated_results.txt"), "w")
        acc_file_object = open(os.path.join(self.results_dir, "accuracy.csv"), "w")
        acc_file_object.write('exp_config,split,amota,bicycle_amota,bus_amota,car_amota,moto_amota,ped_amota,trailer_amota,truck_amota\n')

        # Iterate through all .json files in results directory
        for (dirpath, _, filenames) in os.walk(self.results_dir,topdown=True):

            for file in filenames:
                # Ignore non-result files
                if os.path.splitext(file)[-1] != '.json':
                    continue

                # Get dataset for the evaluation split
                parent_dir = os.path.split(dirpath)[-1]
                if parent_dir in ['mini_val','mini_train']:
                    dataset = "v1.0-mini"
                elif parent_dir in ['val','train']:
                    dataset = "v1.0-trainval"
                else:
                    continue

                print("Evaluating experiment case: " + os.path.splitext(file)[0])
                file_object.write("EXPERIMENT CASE: " + os.path.splitext(file)[0] + "; EVAL SPLIT: " + parent_dir + "\n")

                proc = subprocess.run(["python3","scripts/nuscenes/evaluate.py", 
                    "--eval_set", parent_dir,
                    "--version", dataset,
                    "--dataroot", self.nuscenes_dir,
                    os.path.join(dirpath,file)], capture_output=True, text=True)

                # Capture the relevant parts of stdout and write to a text file
                final_results = re.search("(### Final results ###\n)(.*)(Per-class results:\n)(.*)(bicycle)(\s*)([\w\.\w]*)(.*)(bus)(\s*)([\w\.\w]*)(.*)(car)(\s*)([\w\.\w]*)(.*)(motorcy)(\s*)([\w\.\w]*)(.*)(pedestr)(\s*)([\w\.\w]*)(.*)(trailer)(\s*)([\w\.\w]*)(.*)(truck)(\s*)([\w\.\w]*)(.*)(Aggregated results:\n)(AMOTA)(\s*)([\w\.\w]*)(.*)(AMOTP.*)(Eval time:)(.*)(Rendering curves)",proc.stdout,re.MULTILINE|re.DOTALL)
                file_object.write(final_results.group())             
                file_object.write("\n\n")

                # Write to accuracy results .CSV
                acc_file_object.write(os.path.splitext(file)[0]+ ',' + parent_dir + ',' + final_results.group(36) +',' + final_results.group(7) + ',' + final_results.group(11) + ',' + final_results.group(15) + ',' + final_results.group(19) + ',' + final_results.group(23) + ',' + final_results.group(27) + ',' + final_results.group(31) + '\n')
                print("Complete. \n")

        file_object.close()
        acc_file_object.close()

def main(args=None):
    parser = argparse.ArgumentParser()
    home_dir = Path.home()

    parser.add_argument(
        "--nuscenes-dir",
        "-n",
        default=home_dir / "nuscenes",
        help="Path to nuscenes data directory",
    )
    parser.add_argument(
        "--results-dir",
        "-r",
        default= "../results",
        help="Relative path from ros_tracking/scripts to results.json files",
    )

    parser.add_argument(
        "--dataset",
        "-d",
        default="v1.0-mini",
        help="NuScenes dataset: v1.0-mini, v1.0-trainval, v1.0-test",
    )
    parser.add_argument(
        "--split",
        "-s",
        default="mini_val",
        help="NuScenes dataset split: mini_train, mini_val, train, val, test",
    )

    args = parser.parse_args()

    # Initialize converter object
    eval_exp_res = EvalExpResults(args)

    eval_exp_res.evaluate_results()


if __name__ == "__main__":
    main()