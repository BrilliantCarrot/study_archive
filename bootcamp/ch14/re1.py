# 정규 표현식
# 문자열 패턴 정의해서 검색, 검사, 치환, 추출 등을 할 수 있게해주는 문자열 처리 규칙

import re

# 정규 표현식을 정의해서 컴파일 함수 인자로 전달하면
# 패턴 객체를 반환 함
# 패턴 객체: 검색 대상 문자열에서 패턴의 발견을 도와주는 객체
# match: 문자열 처음(중요)부터 정규식과 매치되는지 조사함
# 시작이 일치해야 함
# search: 문자열 전체를 훑음
# findall: 문자열 전체에서 모든 일치를 찾음
# 메타 문자: 별도의 의미가 담겨있는 문자

pattern = re.compile("[b]")
# [a-z]+에서 +가 의미하는거
# 정규 표현식에서 +는 "앞에 있는 문자가 1번 이상 반복됨"을 의미
print(pattern.match("banana"))

# 축악 후 형태
# 매치객체 = re.match("정규표현식", "검색대상문자열")

m = re.match("[a-z]+", "python")
print(m)


# result = pattern.match("banana")
result = pattern.search("banana")
print(result)

# 이 경우엔 b 포함, a 포함 다 있으므로 match 됨
p2 = pattern = re.compile("[ab]")
print(p2.match("banana"))
print(p2.match("apple"))

print("p3")
p3 = re.compile("[ab][ab]")
print(p3.match("banana"))
print(p3.match("aa"))
print(p3.match("apple"))

print("p4")
p4 = re.compile("[\^0-5]")
print(p4.match("3banana"))
print(p4.match("7banana"))
print(p4.match("banana"))
print(p4.match("^banana")) # ^이 쓰임
print(p4.match("aa"))
print(p4.match("apple"))

print("===== p5 =====")
p5 = re.compile("[a-z]")
print(p5.match("3banana"))
print(p5.match("7banana"))
print(p5.match("banana"))
print(p5.match("^banana"))
print(p5.match("aa"))
print(p5.match("zpple"))

print("===== p6 =====")
p6 = re.compile("3.a") # 3과 a사이 어떠한 문자라도 한 번 나올 수 있음
print(p6.match("3xyza"))
print(p6.match("3xa"))
print(p6.match("banana"))

print("===== p7-1 =====")
p7 = re.compile("bo{2}b") # o가 2번 반복되면 match
print(p7.match("bob")) # o가 1번 나오므로 unmatch
print(p7.match("boob")) # o가 2번 반복되므로 match
print(p7.match("booob")) # 0가 3번 반복되므로 unmatch
print("===== p7-2 =====")
p7 = re.compile("bo{,2}b") # 2번 이하 반복까지 match
print(p7.match("bob")) # o가 1번 반복됨으로 match
print(p7.match("boob")) # o가 2번 반복됨으로 match
print(p7.match("booob")) # o가 3번 반복됨으로 unmatch
print("===== p7-3 =====")
p7 = re.compile("bo{3,5}b") # 2번 이하 반복까지 match
print(p7.match("bob")) # o가 1번 있으므로 unmatch
print(p7.match("boob")) # o가 2번 있으므로 unmatch
print(p7.match("booob")) # o가 3번 있으므로 match
print(p7.match("boooob")) # o가 4번 있으므로 match
print(p7.match("booooob")) # o가 5번 있으므로 match

print("===== p8-1 =====")
p8 = re.compile("hello$")
print(p8.match("world, hello"))
print(p8.match("hello"))
print(p8.search("world, hello"))
print(p8.findall("world, hello"))
print("===== p8-2 =====")
p8 = re.compile("^hello")
print(p8.match("hello"))
print(p8.match("hello, world"))

print("===== p9 =====")
# 매치 객체 메서드 종류 및 기능
p9 = re.compile("[a-z]+")
r9 = p9.match("python")

print(r9.group())
print(r9.span())
print(r9.start())
print(r9.end())

print("===== p10 =====")
r10 = re.compile("[a-z]*")
print(r10.match("3pyt8hon"))
# 아무 것도 없어도 조건 만족(0글자 매칭 가능)
# *의 믜미: 앞에 있는 패턴이 0번 이상 반복됨을 뜻. 즉, 소문자가 없어도 성공으로 간주
# [a-z]에 해당하지 않지만, *는 0번 반복도 허용하므로 
# 시작 지점에서 소문자가 0개 있다"고 판단하고 즉시 매치를 종료
import re

# 1. 특정 문자가 0번 이상 반복되는 경우
# 'b'가 없어도 되고, 여러 개 있어도 됩니다.
p1 = re.compile("ab*c")
print(f"ac: {p1.match('ac')}")      # <re.Match... group='ac'> (0번)
print(f"abc: {p1.match('abc')}")    # <re.Match... group='abc'> (1번)
print(f"abbbc: {p1.match('abbbc')}")# <re.Match... group='abbbc'> (3번)

print("-" * 20)

# 2. 공백이 있을 수도, 없을 수도 있는 경우 (\s*)
# 단어 사이의 공백 유무에 상관없이 매칭하고 싶을 때 유용합니다.
p2 = re.compile("Apple\s*Pie")
print(f"Case 1: {p2.match('ApplePie').group()}")   # 공백 0개
print(f"Case 2: {p2.match('Apple  Pie').group()}") # 공백 2개

print("-" * 20)

# 3. match() vs findall() 차이 (중요!)
# match는 시작부터 확인하지만, findall은 문자열 전체에서 찾아냅니다.
p3 = re.compile("[a-z]*")
text = "123apple456"

# match는 숫자로 시작하면 빈 문자열('')을 반환하고 끝납니다.
print(f"Match 결과: '{p3.match(text).group()}'") 

# findall은 문자열 전체를 훑으며 '0번 이상 반복되는 소문자'를 다 찾습니다.
# 소문자가 없는 구간은 빈 문자열('')로 반환됩니다.
print(f"Findall 결과: {p3.findall(text)}") 
# 결과: ['', '', '', 'apple', '', '', '', '']
