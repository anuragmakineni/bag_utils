#!/usr/bin/env python
import rosbag
import Tkinter
import tkFileDialog


class SplitBag():
    def __init__(self, input_path, output_path):
        self.bag_in = rosbag.Bag(input_path)
        self.output_path = output_path

    def get_topics(self):
        return self.bag_in.get_type_and_topic_info()[1].keys()

    def split(self, topics):
        print("Spliting: " + str(topics))
        with rosbag.Bag(self.output_path, 'w') as bag_out:
            for topic, msg, t in self.bag_in.read_messages(topics):
                bag_out.write(topic, msg, t)


class GUI(Tkinter.Frame):
    def __init__(self, root, split):
        Tkinter.Frame.__init__(self, root)
        self.root = root
        self.split = split

        self.states = []
        self.available_topics = None
        self.create_boxes()

        Tkinter.Button(self.root, text='SPLIT', command=lambda: self.prep_split()).pack()

    def create_boxes(self):
        self.available_topics = self.split.get_topics()
        # display available topics
        for x in range(len(self.available_topics)):
            self.states.append(Tkinter.IntVar())
            l = Tkinter.Checkbutton(self.root, text=self.available_topics[x], variable=self.states[x])
            l.pack(anchor='w')

    def prep_split(self):
        desired_topics = []
        for x in range(len(self.states)):
            if self.states[x].get():
                desired_topics.append(self.available_topics[x])
        self.split.split(desired_topics)
        exit()

if __name__ == '__main__':
    root = Tkinter.Tk()
    root.wm_title("Available Topics")
    root.withdraw()

    input = tkFileDialog.askopenfilename(defaultextension='.bag')
    output = tkFileDialog.asksaveasfilename(defaultextension='.bag')

    root.deiconify()

    split = SplitBag(input, output)

    GUI(root, split).pack()
    root.mainloop()
