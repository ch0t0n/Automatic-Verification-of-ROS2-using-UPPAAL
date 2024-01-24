

# Queue using arrays

A = [-1 for i in range(5)]
fl = 0

def enqueue(p):
    global A, fl
    for i,v in enumerate(A):
        if (v == -1) and (fl == 0):
            A[i] = p
            break
        if i==4:
            fl = 1
            # print('full')
    if fl == 1:
        for i in range(4):
            A[i] = A[i+1]
        A[4] = p

enqueue(3)
print(A)
enqueue(4)
print(A)
enqueue(5)
print(A)
enqueue(6)
print(A)
enqueue(7)
print(A)
enqueue(8)
print(A)
enqueue(9)
print(A)
enqueue(0)
print(A)

enqueue(11)
print(A)
