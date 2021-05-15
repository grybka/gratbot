import curses
import time
import threading

class SplitScreenDisplay:
    def __init__(self,stdscr):

        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        #self.save_filename = "logs/screen_log_{}.txt".format(self.start_timestr)
        self.save_filename = "logs/screen_log_latest.txt".format(self.start_timestr)
        self.f=open(self.save_filename,"w")
        height,width = stdscr.getmaxyx()
        self.width=width
        self.top_lines=[]
        self.bottom_lines=[]
        self.top_lock=threading.Lock()
        self.bottom_lock=threading.Lock()

        curses.halfdelay(1)
        h2=int((height-1)/2)
        self.top_height=h2
        self.bottom_height=h2
        self.top_window=curses.newwin(self.top_height,width,0,0)
        self.bottom_window=curses.newwin(self.bottom_height,width,self.top_height,0)
        self.entry_window=curses.newwin(1,width,2*h2,0)
        self.entry_string=""

    def end(self):
        self.f.close()

    def redraw_top(self):
        self.top_window.clear()
        on_line=self.top_height-1
        for line in reversed(self.top_lines):
            print(line)
            self.top_window.addstr(on_line,0,line)
            on_line-=1
        self.top_window.refresh()

    def redraw_bottom(self):
        self.bottom_window.clear()
        on_line=self.bottom_height-1
        for line in reversed(self.bottom_lines):
            self.bottom_window.addstr(on_line,0,line)
            on_line-=1
        self.bottom_window.refresh()

    def add_text_to_bottom(self,text):
        with self.bottom_lock:
            text=text.replace('\n',' ')
            while len(text)>=self.width-1:
                self.bottom_lines.append(text[0:self.width-2])
                text=text[self.width-2:]
            self.bottom_lines.append(text)
            while len(self.bottom_lines)>self.bottom_height:
                self.bottom_lines.pop(0)
            self.redraw_bottom()

    def add_text_to_top(self,text):
        with self.top_lock:
            try:
                self.f.write(text+"\n")
                text=text.replace('\n',' ')
                while len(text)>=self.width-1:
                    self.top_lines.append(text[0:self.width-2])
                    text=text[self.width-2:]
                self.top_lines.append(text)
                while len(self.top_lines)>self.top_height:
                    self.top_lines.pop(0)
                self.redraw_top()
            except:
                #file wasn't open.  Fine.
                pass

    def allow_input(self):
        ch=self.entry_window.getch()
        if ch==10:
            toret=self.entry_string
            self.entry_string=""
            self.entry_window.clear()
            return toret
        else:
            if ch!=curses.ERR:
                if ch==8: #KEY_BACKSPACE
                    self.entry_string=self.entry_string[0:-1]
                elif ch==9: #tab
                    pass
                else:
                    self.entry_string+=chr(ch)
                #self.add_text_to_bottom("{}".format(int(ch)))
                self.entry_window.clear()
                self.entry_window.addstr(self.entry_string)
                self.entry_window.refresh()
            return None

#screen=curses.initscr()
#screen.refresh()

if __name__=="__main__":
    def myfunc(stdscr):
        display=SplitScreenDisplay(stdscr)
        display.add_text_to_top("Top 1")
        display.add_text_to_top("Top 2")
        display.add_text_to_top("Top 3")
        display.add_text_to_bottom("Bottom 1")
        display.add_text_to_bottom("Bottom 2")
        display.add_text_to_bottom("Bottom 3")

        last_key=""
        start=time.time()
        while True:
            res=display.allow_input()
            if res is not None:
                display.add_text_to_top(res)
                if res=="quit":
                    break
            if time.time()-start>10:
                break
    curses.wrapper(myfunc)
