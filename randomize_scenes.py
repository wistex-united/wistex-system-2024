import random

# Template scene path # 
template_name = "josh_ros2_scene"
template_path = f"Config/Templates/{template_name}.txt"

output_file = f"Config/Scenes/Randomized/{template_name}.ros2"

# Extract a Random floating point value # 
def get_random_float_in_range(min_val: float, max_val: float) -> float:
    range = max_val - min_val
    return random.random() * range + min_val

# Open a Template Path #
with open(template_path) as f:
    template = f.read()


# Write to an output file #
print("Output file:", output_file)
with open(output_file, "w") as f:
    f.write(template.format(
        robotX=get_random_float_in_range(-4.5, 4.5),
        robotY=get_random_float_in_range(-3, 3),
        ballX=get_random_float_in_range(-4.5, 4.5),
        ballY=get_random_float_in_range(-3, 3),
    ))