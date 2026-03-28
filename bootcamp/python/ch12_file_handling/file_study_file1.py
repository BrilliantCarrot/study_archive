# '계좌1.txt' 파일을 생성한 후,  
#  예금주와 계좌번호를 파일에 작성하세요. 
#  예) 
#  김삿갓 597-89-000089 
#  이수근 343-64-000064 
#  박혁거세 136-97-000097

# with open("계좌1.txt", 'w', encoding='utf-8') as f:
#     f.write("김삿갓 597-89-000089\n")
#     f.write("이수근 343-64-000064\n")
#     f.write("박혁거세 136-97-000097\n")


# 앞서 생성한 file 폴더의 '계좌1.txt' 읽어서 계좌번호를 리스트로 반환 
# 및 저장하세요.

# account_dic = {}

# with open('계좌1.txt', 'r', encoding='utf-8') as f:
#     for line in f:
#         data = line.strip().split()
#         account_dic[data[0]] = data[1]


# print(account_dic)


# 앞서 생성한 file 폴더의 '계좌1.txt' 파일에 다음 예금주와 계좌번호를 
# 추가하세요. 
# 강호동 147-12-002093 
# 유재석 146-22-102093

# with open("계좌1.txt", 'a', encoding='utf-8') as f:
#     f.write("강호동 147-12-002093\n")
#     f.write("유재석 146-22-102093\n")

# 피자 주문 프로그램을 주문 내역 파일로 저장하기 
# 터미널을 통해 주문 입력 
# 주문 내역은 order.txt 파일로 저장 
# 다음과 같이 주문 내역 쓰기 
# 주문 내역:  
# - 치즈피자 (3200원) x 3 9600원 
# 주문 내역 읽어 화면에 출력하기

# # 피자 가격 저장함
# pizza_name = "치즈피자"
# pizza_price = 3200

# # 터미널에서 주문 수량 입력받음
# count = int(input("치즈피자 주문 수량 입력: "))

# # 총 금액 계산함
# total = pizza_price * count

# # 주문 내역 문자열 만듦
# order_text = f"- {pizza_name} ({pizza_price}원) x {count} {total}원\n"

# # order.txt 파일에 주문 내역 저장함
# with open("order.txt", "w", encoding="utf-8") as f:
#     f.write("주문 내역: \n")
#     f.write(order_text)

# # order.txt 파일 읽어서 화면에 출력함
# with open("order.txt", "r", encoding="utf-8") as f:
#     result = f.read()

# print(result)

# 피자 주문 프로그램을 주문 내역 파일로 저장하기 
# 터미널을 통해 주문 입력 
# 주문 내역은 order.txt 파일로 저장 
# 다음과 같이 주문 추가하기 
# 주문 내역:  
# - 치즈피자 (3200원) x 3 9600원 
# - 사이다 (1500원) x 2 3000원 
# 주문 내역 읽어 화면에 출력하기


drink_name = "사이다"
drink_price = 1500

# pizza_num = int(input("치즈피자 주문 수량 입력: "))
drink_num = int(input("음료 주문 수량 입력: "))

# pizza_total = pizza_price * pizza_num
drink_total = drink_price * drink_num

# order_text1 = f"- {pizza_name} ({pizza_price}원) x {pizza_num} {pizza_total}원\n"
order_text2 = f"- {drink_name} ({drink_price}원) x {drink_num} {drink_total}원\n"

with open("order.txt", "a", encoding="utf-8") as f:
    f.write(order_text2)

# # order.txt 파일 읽어서 화면에 출력함
# with open("order.txt", "r", encoding="utf-8") as f:
#     lines = f.readlines()

# for line in lines:
#     print(line, end='')

# 또는 밑에것을 이용
with open("order.txt", "r", encoding="utf-8") as f:
    result = f.read()
print(result)