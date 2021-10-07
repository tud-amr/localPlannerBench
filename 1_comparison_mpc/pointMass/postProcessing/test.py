import subprocess

subprocess.run(["ls", "-l"])
subprocess.run(["./createPlots", "../results/testSeries/mpc_20211007_111108", "10"])
