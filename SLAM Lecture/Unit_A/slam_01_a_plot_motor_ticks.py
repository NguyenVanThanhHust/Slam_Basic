import matplotlib.pyplot as plt

if __name__ == '__main__':
    # Read file and plot position
    with open('robot4_motors.txt') as robo_file:
        left_move = list()
        right_move = list()
        for each_line in robo_file:
            sp = each_line.split()
            left_move.append(int(sp[2]))
            right_move.append(int(sp[6]))
        plt.plot(left_move)
        plt.plot(right_move)
        plt.savefig('slam_01_a.png')