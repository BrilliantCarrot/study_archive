# class MyIterator:
#     def __init__(self,data):
#         self.data = data
#         self.position = 0
#     def __iter__(self):
#         return self
#     def __next__(self):
#         if self.position >= len(self.data):
#             raise StopIteration
#         result = self.data[self.position]
#         self.position += 1
#         return result
    
# if __name__ == "__main__":
#     i = MyIterator([1,2,3]) # i 객체 생성
#     for item in i:
#         print(item)
    

class MyIterator:
    def __init__(self,data):
        self.data = data
        self.position = len(self.data) -1
    def __iter__(self):
        return self
    def __next__(self):
        if self.position < 0:
            raise StopIteration
        result = self.data[self.position]
        self.position -= 1
        return result

if __name__ == "__main__":
    i = MyIterator([1,2,3]) # i 객체 생성
    for item in i:
        print(item)
    


print(next(i))