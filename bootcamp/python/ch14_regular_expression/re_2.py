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

print("=" * 30)
# !!!시험에서 역슬래시 문제는 안나옴, 옵션 부터는 안나옴
# re.VERBOSE: 정규식에 공백과 주석을 허용해서 복잡한 패턴을 읽기 쉽게 만들어주는 플래그
# 이메일 주소 유효성 검사 - re.VERBOSE 없이
pattern_normal = r'[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}'

# 이메일 주소 유효성 검사 - re.VERBOSE 사용
pattern_verbose = re.compile(r"""
    [a-zA-Z0-9._%+-]+   # 로컬 파트 (@ 앞부분): 영문, 숫자, 특수문자 허용
    @                   # @ 기호
    [a-zA-Z0-9.-]+      # 도메인 이름: 영문, 숫자, 하이픈, 점 허용
    \.                  # 점 (이스케이프 필요)
    [a-zA-Z]{2,}        # 최상위 도메인 (TLD): 영문 2자 이상
""", re.VERBOSE)

# 테스트 데이터
emails = [
    "user@example.com",       # 정상
    "hello.world@gmail.com",  # 정상
    "invalid-email",          # 비정상
    "missing@dot",            # 비정상
    "test.user@company.co.kr" # 정상
]

print("=== 이메일 유효성 검사 ===")
for email in emails:
    match = pattern_verbose.fullmatch(email)
    status = "✅ 유효" if match else "❌ 무효"
    print(f"{status}: {email}")

