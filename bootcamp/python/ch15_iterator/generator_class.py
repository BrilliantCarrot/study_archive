gen = (i * i for i in range(1,100))
print(type(gen))
for i in gen:
    print(i)

# 위와 같음
class MyIterator:
    def __init__(self):
        self.data = 1

    # __iter__가 없으면: TypeError: 'MyIterator' object is not iterable 에러를 발생시킵니다.
    def __iter__(self):
        return self
    
    def __next__(self):
        result = self.data * self.data
        if self.data >= 100:
            print("self.data 100 넘음")
            raise StopIteration
        
        self.data = self.data + 1

        return result
    

iter = MyIterator()
print(type(iter))
for i in iter:
    print(i)