a = [1,2,3]
# next(a) 'list' object is not an iterator
# 리스트는 이터러블이지만 이터레이터는 아님
# 즉, 반복 가능하다고 해서 이터레이터는 아님
# 하지만 반복 가능하다면 iter 함수를 이용해 이터레이터로 변환 가능
# 이터레이터 생성 방법
ia = iter(a)
# 리스트는 이터러블이므로 iter를 이용하여 이터레이터 생성 가능
type(ia)
print(next(ia))
print(next(ia))
print(next(ia))
# print(next(ia)) # 이것을 한번 더 실행하면 stop iteration 발생
# 왜냐면 이터레이터는 for문이나 next로 값을 한 번 읽으면, 다시 값을 읽을 수는 없음
for i in ia:
    print(i)