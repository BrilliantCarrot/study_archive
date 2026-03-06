path = "./file.txt"
f = open(path, 'a')
for i in range(10,16):
    data = "\n%d번째 줄입니다." % i
    f.write(data)
f.close()