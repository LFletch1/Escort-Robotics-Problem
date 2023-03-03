from EscortProblem import EscortProblem

def main():
    env_files = ["Envs/tetris_env.json",
                "Envs/rooms.json",
                "Envs/rooms2.json",
                "Envs/rooms3.json",
                "Envs/new_env.json",
                "Envs/interesting.json",
                "Envs/new_env3.json",
                "Envs/pursuit_fail.json"]
    for env_file in env_files:
        path_file = env_file.replace("Envs","solution_paths").replace(".json", "_path.txt")
        file = open(path_file, 'r')
        file.readline()
        escort_start_x, escort_start_y = file.readline().strip().replace(" ","").replace("(","").replace(")","").split(":")[1].split(",")
        escort_start_pos = (float(escort_start_x), float(escort_start_y))
        file.readline()
        vip_goal_x, vip_goal_y = file.readline().strip().replace(" ","").replace("(","").replace(")","").split(":")[1].split(",")
        VIP_goal_pos = (float(vip_goal_x), float(vip_goal_y))

        lines = file.readlines()
        path = []
        for line in lines:
            x, y = line.strip().split(",")
            path.append((float(x),float(y)))

        prob = EscortProblem(env_file, escort_start_pos, VIP_goal_pos)
        # prob.show_path(path)
        prob.show_path(path, save_fig=True, filename=env_file.replace("Envs","solution_svgs").replace(".json", "_solution.svg"))


if __name__ == "__main__":
    main()