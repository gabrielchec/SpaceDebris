import os
h = [10, 100, 1000, 10000, 100000]
for i in h:
    #print(i)
    os.system("python3  main.py " + str(i) + "  > output.txt")
