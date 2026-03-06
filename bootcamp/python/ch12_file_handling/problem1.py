import os
print(os.getcwd())

path = r"계좌.txt"
mode = "w"
encoding = "utf-8"

# with open(path, mode, encoding=encoding) as file:
#     file.write("ㅇㅇㅇ\n")
#     file.write("ㅁㅁㅁ\n")
#     file.write("ㅎㅎㅎ\n")

# with open("계좌.txt", 'r') as file:
#     lines = file.readlines()


acc_list = []

with open("계좌.txt", "r", encoding="utf-8") as file:
    for line in file:
        line = line.strip()
        space_idx = line.find(" ") 
        account_num = line[space_idx+1:]
        acc_list.append(account_num)

print(acc_list)