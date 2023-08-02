import casadi as ca
import sympy as sp
import sympy.physics.mechanics as me
from multiprocessing import Process, Queue

class PicklableCasadiObject:
    def __init__(self):
        self.dm = ca.DM([1,2,3])
        self.fun = ca.Function('fun', [], [self.dm])
        self.sp_var = sp.symbols('x')
        self.me_var = me.dynamicsymbols('x')

def run_once(pickle: PicklableCasadiObject, output: Queue):
    output.put(["Picklable!", pickle.dm, pickle.fun, pickle.sp_var])

def main():
    pickle = PicklableCasadiObject()
    output = Queue()
    process = Process(target=run_once, args=(pickle, output))
    process.start()

    result = output.get()
    print(result)

if __name__ == "__main__":
    main()