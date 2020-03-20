from matplotlib import pyplot as plt
from matplotlib import rc
rc('font', **{'family': 'serif', 'serif': ['Cardo']})
rc('text', usetex=True)

timings = []
with open("timing.txt") as timing_file:
    for line in timing_file:
        timings.append(float(line))

ax = plt.plot(timings, 'bo-', markersize=2, linewidth=0.1)
plt.hlines(50, -100, 4000, colors='r', linestyle='dashdot')
plt.xlim((0, len(timings)))
plt.xlabel("Optimization Round", fontsize=30)
plt.ylabel("Runtime [ms]", fontsize=30)
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)
plt.show()
print(timings)