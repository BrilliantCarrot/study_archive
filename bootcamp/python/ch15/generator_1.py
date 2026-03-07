# 제너레이터(Generator)는 이터레이터(Iterator)를 만드는 가장 쉽고 간편한 방법입니다. 모든 제너레이터는 이터레이터이지만, 모든 이터레이터가 제너레이터인 것은 아닙니다. 
# 1. 구현 방식의 차이
# 이터레이터: 클래스 내부에 __iter__와 __next__ 메서드를 직접 정의해야 합니다. 질문자님이 위에서 작성하신 코드가 전형적인 이터레이터 방식입니다.
# 제너레이터: 일반 함수처럼 작성하되, return 대신 yield 키워드를 사용합니다. 파이썬이 자동으로 __iter__와 __next__를 구현해주므로 코드가 훨씬 간결합니다. 

# 2. 상태 관리 (State Management)
# 이터레이터: self.position과 같이 현재 어디까지 진행됐는지 상태를 변수로 직접 저장하고 관리해야 합니다.
# 제너레이터: 함수가 실행되다가 yield를 만나면 그 지점의 상태(로컬 변수 등)를 자동으로 기억하고 멈춥니다. 다음 호출 시 멈췄던 지점부터 다시 시작합니다. 

# 3. 메모리 효율성
# 둘 다 데이터를 한꺼번에 메모리에 올리지 않고 필요할 때마다 하나씩 생성하는 게으른 연산(Lazy evaluation)을 수행하여 메모리를 절약합니다.
# 다만, 제너레이터는 클래스 구조 없이 함수만으로 동작하므로 구현이 더 가볍고 가독성이 좋습니다. 

def simple_generator():
    yield 'a'
    yield 'b'
    yield 'c'

g = simple_generator()
print(type(g))

for item in g:
    print(item)

print("__iter__" in dir(g))
print("__next__" in dir(g))

print('='*30)

def mygen():
    for i in range(1,1000):
        result = i * i
        yield result

gen = mygen()
print(next(gen))
print(next(gen))
print(next(gen))

print('='*30)

# 컴프리헨션
# 간결하고 효율적인 방식으로 새로운 시퀀스를 생성할 수 있는 문법
# 반복문과 조건문을 활용하여 리스트, 딕셔러니 등을 생성하는 데 사용

numbers = [1,2,3,4]
squared = [x**2 for x in numbers]
print(squared)

numbers = [1,2,3,4]
squared = [x**2 for x in numbers if x%2 == 0] # %2를 먼저 한 다음에 컴프리헨션 수행
print(squared)
print('-'*10)
numbers = [1,2,3,4]
squared = [x**2+1 for x in numbers if x%2 == 0] # %2를 먼저 한 다음에 컴프리헨션 수행
print(squared)

print('-'*20,"딕셔너리 컴프리헨션")

squared_dict = {x : x**2 for x in range(1,5)}
print(squared_dict)
squared_dict = {x : x**2 for x in range(5) if x%2 == 0} # 마찬가지로 조건문 적용 가능
print(squared_dict)

print('-'*20)

# 제너레이터 생성 방법2
# 제너레이터 컴프리헨션
# (표현식 for 요소 in 이터러블 if 조건)



# 제너레이터 활용하기
# 작성하신 코드는 제너레이터 표현식(Generator Expression)의 핵심인 '지연 평가(Lazy Evaluation)'를 보여주는 아주 좋은 예시입니다.
# 이 코드의 핵심은 list_job을 만드는 시점에 함수가 실행되는 것이 아니라, next()를 호출하는 시점에 함수가 실행된다는 점입니다.
# 1. 코드 흐름 분석
# list_job = (longtime_job() ...):
# 괄호 ()를 사용하여 제너레이터를 생성했습니다.
# 중요한 점은 이때 longtime_job() 함수가 아직 실행되지 않는다는 것입니다. 그저 "앞으로 호출하면 실행할 준비가 된 예약권" 5개를 만든 상태입니다.
# print(next(list_job)):
# next()가 호출되는 순간, 제너레이터 내부의 첫 번째 루프(i=0)가 돌아갑니다.
# 드디어 longtime_job()이 호출되어 job start가 출력되고 1초를 쉽니다.
# 함수가 반환한 "done"을 next가 받아서 print로 화면에 출력합니다.
# print('='*30)
# 3. 리스트 컴프리헨션과의 결정적 차이
# 만약 괄호를 대괄호 []로 바꾼다면 결과가 완전히 달라집니다.
# 리스트 [longtime_job() for i in range(5)]:
# 변수에 할당하는 즉시 함수 5개를 연속으로 실행합니다. (총 5초 소요)
# 메모리에 결과값 5개를 모두 담아둡니다.
# 제너레이터 (longtime_job() for i in range(5)):
# 변수에 할당할 때는 아무 일도 안 일어납니다. (0초 소요)
# next()를 부를 때마다 딱 한 번씩만 함수를 실행합니다. 메모리에는 현재 실행할 단계의 정보만 보관합니다.

import time

def longtime_job():
    print("job start")
    time.sleep(1)
    return "done"

list_job = (longtime_job() for i in range(5))
print(next(list_job))
# job start
# 1초 대기
# done

print('='*30)
print("제너레이터 연습")
print('='*30)

print("방법 1: yield 함수 이용(제너레이터 함수)")
def square_generator(n):
    for i in range(1, n+1):
        yield i ** 2
for num in square_generator(5):
    print(num)

print("방법2: 제너레이터 표현식")
generator = (i ** 2 for i in range(1, 6))
for num in generator:
    print(num)

print("방법3: 리스트 컴프리헨션")
n = int(input("n을 입력하세요: "))
result = [i ** 2 for i in range(1, n + 1)]
for num in result:
    print(num)

print("방법4: map, lmabda, range를 이용")
result = map(lambda x: x ** 2, range(1, 6))
for num in result:
    print(num)

print("방법5: iterator 클래스 - 모든 제너레이터는 이터레이터이다")
class SquareIterator:
    def __init__(self, n):
        self.n = n
        self.current = 1

    def __iter__(self):
        return self

    def __next__(self):
        if self.current > self.n:
            raise StopIteration
        value = self.current ** 2
        self.current += 1
        return value

n = int(input("n을 입력하세요: "))

for num in SquareIterator(n):
    print(num)


print("방법6: iterable 클래스")
class SquareIterator:
    def __init__(self, n):
        self.n = n
        self.current = 1

    def __next__(self):
        if self.current > self.n:
            raise StopIteration
        value = self.current ** 2
        self.current += 1
        return value

class SquareIterable:
    def __init__(self, n):
        self.n = n

    def __iter__(self):
        return SquareIterator(self.n)

n = int(input("n을 입력하세요: "))

for num in SquareIterable(n):
    print(num)
