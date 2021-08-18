import matplotlib.animation as animation
import matplotlib.pyplot as plt

file = open("cmake-build-debug/out.txt", 'r')
states = []
for line in file.readlines():
    state = [float(x) for x in line.split()]
    states.append(state)

dt = 0.01
L = 1

fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(autoscale_on=False, xlim=(-L, L), ylim=(-L, 1.))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


def animate(i):
    state = states[i]
    base = state[0:3]
    leg = state[3:6]
    foot = state[6:9]

    x = [base[0], leg[0], foot[0]]
    y = [base[2], leg[2], foot[2]]
    line.set_data(x, y)

    time_text.set_text(time_template % (i * dt))

    return line, time_text


ani = animation.FuncAnimation(fig, animate, len(states), interval=dt * 1000, blit=True)
plt.show()
