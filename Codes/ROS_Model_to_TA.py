import ast

with open("ros_model.txt") as file:
    r = file.read()
    file.close()

P = ast.literal_eval(r)

num_pub = len(P['Pub'])
num_sub = len(P['Sub'])
num_top = len(P['Topic'])

i = 0
for pub in P['Pub']:
    tim_or_sub = P['MT'][pub]
    if 't' in tim_or_sub:
        pub_dict = {}
        
        r = P['Timer'][tim_or_sub]

        g = 1
        print('timer = ', tim_or_sub)
    else:
        print('subscriber = ', tim_or_sub)



