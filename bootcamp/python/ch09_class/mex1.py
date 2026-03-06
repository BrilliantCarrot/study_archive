# mex1.py file
# 파이썬 모듈 임포트시 특정 클래스, 함수, 변수를 명시하느 이유
# 1. 코드 간결화(가독성)
# 2. 이름공간 충돌 방지
# 3. 어떤 기능을 사용하는 지 명확하게 구분 가능

class Cvalue:
  def __init__(self):
    self.lista = []
  def add(self, num):
    self.lista.append(num)
  def fprintf(self):
    print(self.lista)

def plus(a,b):
  print("이건 외부에서 import 해도 보여요!")
  c = a + b
  return c

if __name__ == "__main__":  # 이 파일을 직접 실행했을 때에만 밑에 코드를 실행하라
# 내가 직접 실행할 때만 돌아가게 할 코드를 설정하는 안전장치
  print("이건 mex1.py를 직접 실행했을 때만 보여요!")
  p1 = Cvalue()
  p1.add(1)
  p1.add(2)
  p1.add(3)
  p1.fprintf()