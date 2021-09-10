import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

file = open("cmake-build-debug/out.txt", 'r')
states = []
for line in file.readlines():
    state = [float(x) for x in line.split()]
    states.append(state)

dt = 0.01
L = 0.3

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(autoscale_on=False, xlim=(-2., 2.), ylim=(-1., 3.))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


def animate(i):
    state = states[i]
    theta = state[2]
    phi = state[3]

    base = np.array([state[1], state[0] + 0.6])
    leg = base + L * np.array([np.sin(theta), -np.cos(theta)])
    foot = leg + L * np.array([np.sin(theta + phi), -np.cos(theta + phi)])

    x = [base[0], leg[0], foot[0]]
    y = [base[1], leg[1], foot[1]]
    line.set_data(x, y)

    time_text.set_text(time_template % (i * dt))

    return line, time_text


ani = animation.FuncAnimation(fig, animate, len(states), interval=dt * 1000, blit=True)
# ani.save("animation.mp4")
plt.show()
