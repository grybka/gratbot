import time

#global_screen=None #should be a curses thing later
global_screen=[]

def gprint(text):
    if len(global_screen)==0:
        print(text)
    else:
        global_screen[0].add_text_to_top(text)



def gprint_low(text):
    if len(global_screen)==0:
        print(text)
    else:
        global_screen[0].add_text_to_bottom(text)
