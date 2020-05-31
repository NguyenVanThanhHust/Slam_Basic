# Plot the ticks from the left and right motor
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Read all ticks fo left and right motor and plot it
    with open("robot4_motors.txt") as fp:
        left_move = list()
        right_move = list()
        for line in fp:
            sp = line.split()
            left_move.append(int(sp[2]))
            right_move.append(int(sp[6]))
        plt.plot(left_move)
        plt.plot(right_move)
        plt.savefig('slam_01_a_plot_motor.png')

	

