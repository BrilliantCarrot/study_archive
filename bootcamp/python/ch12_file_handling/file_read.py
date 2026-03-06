path = "./file.txt"
f = open(path,'r')
lines = f.readlines()
for line in lines:
    print(line, end='')
f.close()
