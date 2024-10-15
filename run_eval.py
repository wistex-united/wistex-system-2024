
import argparse
import subprocess
import os
import wandb
import shutil

DATA_FILE = "Config/Scenes/tempData.txt"
CONFIG_FILE = "Config/rl.cfg"

'''
A script to run an evaluation on SimRobot using specific arguments: 

# To override rl.cfg using a config file directly

python3 run_eval.py --rl-cfg="heuristic.cfg" --scene-type="random" --iter=1

# To override rl.cfg using command line arguments (uses default.cfg as base)

python3 run_eval.py --scene-type="random" --iter=1 num_robots=3 role_switch_distance=0.2

'''
def get_parser():
    parser = argparse.ArgumentParser(description="Extract metrics from a scene")
    parser.add_argument('--rl-cfg', type=str, default='default.cfg', required = False, help = "RL Config file to evaluate")
    parser.add_argument('--rl-cfg-folder', type=str, default='rl_cfgs', help = "RL Config file to evaluate")
    parser.add_argument("--template", type=str, default="ThreeRobots", help="Name of the scene file: Found in Config/Scenes")
    parser.add_argument("--scene-type", type=str, default="random", help="Name of the output file")
    parser.add_argument("--iter", type=int, default=1, help="Number of iterations to run")
    return parser


def extract_metric():
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE) as f:
            data = f.read()
        
        success, time = data.split(" ")
        return float(success), float(time)
    else:
        print("Data file not found")
        return None, None    
def run_randomization(template, scene_type):
    exit_code = subprocess.run([
        f"python3" , 
        f"randomize_scenes.py",  
        f"--template={template}",
        f"--type={scene_type}"
    ])
    return exit_code

def run_SimRobot(template):
    # TODO: Add support for MAC
    exit_code = subprocess.run([
        "Build/Linux/SimRobot/Develop/SimRobot", 
        f"Config/Scenes/{template}.ros2",  
    ])
    return exit_code

# Remove file # 
def delete_file(file):
    os.remove(file)

# Copy file #
def copy_file(file, target_path):
    shutil.copy(file, target_path)
    

def override_arguments(rest, config_file):
    with open(config_file, "r") as f:
        lines = f.readlines()
    
    for arg in rest:
        key, value = arg.split("=")
        found_k = False
        for i, line in enumerate(lines):
            if key == line.split("=")[0]:
                found_k = True
                lines[i] = f"{key}={value}\n"
                print(f"Overriding {key} with {value}")
                break

        if not found_k:
            raise ValueError(
                f"Ovrerride argument {key} not found in config file {config_file}"
            )

    with open(config_file, "w") as f:
        f.writelines(lines)

    
if __name__ == "__main__":
    parser = get_parser()
    args, rest = parser.parse_known_args()

    full_path = os.path.join(args.rl_cfg_folder, args.rl_cfg)
    
    copy_file(full_path, CONFIG_FILE)
    override_arguments(rest, CONFIG_FILE)
    
    total_success = 0
    avg_time = 0
    for i in range(args.iter):
        run_randomization(args.template, args.scene_type)
        run_SimRobot(args.template)
        success, time = extract_metric()

        if success is None:
            continue
        delete_file(DATA_FILE)
        total_success += success

        avg_time += time
    
    run_name = args.rl_cfg.split(".")[0]
    print('-------Final Results-------')
    print(f"Config file: {args.rl_cfg}")
    print(f"Evaluation type: {args.type}")
    print(f"Success rate: {total_success/args.iter}")
    print(f"Average time: {avg_time/args.iter}") 