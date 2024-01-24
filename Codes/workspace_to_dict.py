from tkinter import filedialog
from tkinter import *
import os
import re

def browse_button():
    # Allow user to select a directory and store it in global var
    # called folder_path
    filename = filedialog.askdirectory()
    publishers = 0
    subscribers = 0
    timers = 0

    pubs_set, subs_set = set(), set()

    output_dict = {}

    for root, dirs, files in os.walk(filename):        
        for file in files:
            if file.endswith(".py"):
                f_launch = open(os.path.join(root, file), "r")
                data = f_launch.read()
                pubs = data.count("create_publisher(")
                subs = data.count("create_subscription(")
                tims = data.count("create_timer(")
                publishers += pubs
                subscribers += subs
                timers += tims

                pattern = re.compile(r'create_publisher\(.*,.*,.*\)')
                if pubs >= 0:
                    match = pattern.search(data)
                    print(match)
    
    for i in range(publishers):
        n = 'p'+str(i+1)
        pubs_set.add(n)

    for i in range(subscribers):
        n = 's'+str(i+1)
        subs_set.add(n)


    output_dict["Pub"] = pubs_set
    output_dict["Sub"] = subs_set
    output_dict["Timer"] = timers

    print(output_dict)


root = Tk()
root.title("ROS Program to network of TA")
# Set geometry(widthxheight)
root.geometry('250x100')


folder_path = StringVar()
lbl1 = Label(master=root,text="Select path for the ros program:")
lbl1.grid(row=0, column=1)
button2 = Button(text="Browse", command=browse_button)
button2.place(relx=0.5, rely=0.5, anchor=CENTER)

mainloop()



