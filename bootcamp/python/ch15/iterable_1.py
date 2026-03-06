a = [1,2,3]
# next(a) 'list' object is not an iterator
# 이터레이터 생성 방법
ia = iter(a)
type(ia)
print(next(ia))
print(next(ia))
print(next(ia))
# print(next(ia)) # stop iteration 발생
# 이터레이터는 for문이나 next로 값을 한 번 읽으면, 다시 값을 읽을 수는 없음
for i in ia:
    print(i)