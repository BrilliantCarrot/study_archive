import re

text = "The mission of the Python Software Foundation"
re1 = re.compile("Python")
result1 = re1.findall(text)
print(result1)

print('='*40)

print("===== finditer =====")

text = "3pyt8hon 2love 7coding"

# [a-z]+ : 소문자가 1개 이상 연속된 덩어리들을 찾음
# 만약 [a-z]*를 쓰면 소문자가 없어도 성공이기에, 숫자, 빈 문자열, 문자열 끝도 매치
p = re.compile("[a-z]+")

# finditer는 반복 가능한(iterable) 객체를 반환합니다.
result = p.finditer(text)

print(f"입력 문구: {text}\n")

for match_obj in result:
    # span(): (시작 인덱스, 끝 인덱스) 튜플
    # group(): 매치된 실제 문자열
    start, end = match_obj.span()
    print(f"찾은 단어: '{match_obj.group()}'")
    print(f"위치 정보: {start}번 인덱스부터 {end}번 인덱스 전까지")
    print(f"확인: {text[start:end]}") # 슬라이싱으로 직접 확인
    print("-" * 20)

