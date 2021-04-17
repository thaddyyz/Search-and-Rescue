#!/usr/bin/env python
import Tkinter as tk
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %i', data.data)
	dist=((data.data>>2) & 0b0000000001111111)
	labelr = tk.Label(master=c, text=dist,bg="red", font=("ubuntu",20,'bold'))#comment out
	labelr.place(x=335, y=30)#comment out

	if((data.data&0b0000000000000011)==1):#red
		labelc = tk.Label(master=c, text="  RED   ",bg="red", font=("ubuntu",20,'bold'))
	elif((data.data&0b0000000000000011)==2):#green
		labelc = tk.Label(master=c, text="GREEN",bg="green", font=("ubuntu",20,'bold'))
	elif((data.data&0b0000000000000011)==3):#green
		labelc = tk.Label(master=c, text="  OFF   ",bg="white", font=("ubuntu",20,'bold'))
	else:	
		labelc = tk.Label(master=c, text=" none ",bg="white", font=("ubuntu",20,'bold'))
	labelc.place(x=160, y=100)

	if(((data.data>>13)&0b0000000000000001)==1):#IR l
		coord = 330, 20, 270, 120#xpos,ypos,width with respect to x
		arcl = c.create_arc(coord, start=100, extent=70, fill="red")#start angle, angle of arc
	else:
		coord = 330, 20, 270, 120#xpos,ypos,width with respect to x
		arcl = c.create_arc(coord, start=100, extent=70, fill="light green")#start angle, angle of arc
	if(((data.data>>12)&0b0000000000000001)==1):#IR r
		coord = 430, 20, 370, 120#xpos,ypos,width with respect to x
		arcr = c.create_arc(coord, start=10, extent=70, fill="red")
	else:
		coord = 430, 20, 370, 120#xpos,ypos,width with respect to x
		arcr = c.create_arc(coord, start=10, extent=70, fill="light green")
	if(((data.data>>11)&0b0000000000000001)==1):#IR bl
		coord = 330, 220, 270, 120#xpos,ypos,width with respect to x
		arclb = c.create_arc(coord, start=190, extent=70, fill="red")
	else:
		coord = 330, 220, 270, 120#xpos,ypos,width with respect to x
		arclb = c.create_arc(coord, start=190, extent=70, fill="light green")
	if(((data.data>>10)&0b0000000000000001)==1):#IR br
		coord = 430, 220, 370, 120#xpos,ypos,width with respect to x
		arcrb = c.create_arc(coord, start=280, extent=70, fill="red")#start angle, angle of arc
	else:
		coord = 430, 220, 370, 120#xpos,ypos,width with respect to x
		arcrb = c.create_arc(coord, start=280, extent=70, fill="light green")#start angle, angle of arc
	if(((data.data>>9)&0b0000000000000001)==1):#IR rear	
		coord = 400, 220, 300, 120#xpos,ypos,width with respect to x
		arcr = c.create_arc(coord, start=235, extent=70, fill="red")#rear
	else:
		coord = 400, 220, 300, 120#xpos,ypos,width with respect to x
		arcr = c.create_arc(coord, start=235, extent=70, fill="light green")#rear
win =tk.Tk()
c = tk.Canvas(win, bg="white", height=250, width=450)
coord = 430, 20, 370, 120#xpos,ypos,width with respect to x
arcr = c.create_arc(coord, start=10, extent=70, fill="light green")

coord = 330, 20, 270, 120#xpos,ypos,width with respect to x
arcl = c.create_arc(coord, start=100, extent=70, fill="light green")#start angle, angle of arc

coord = 330, 220, 270, 120#xpos,ypos,width with respect to x
arclb = c.create_arc(coord, start=190, extent=70, fill="light green")

coord = 430, 220, 370, 120#xpos,ypos,width with respect to x
arcrb = c.create_arc(coord, start=280, extent=70, fill="light green")#start angle, angle of arc

coord = 400, 220, 300, 120#xpos,ypos,width with respect to x
arcr = c.create_arc(coord, start=235, extent=70, fill="light green")#rear

#coord = 390, 220, 310, 120#xpos,ypos,width with respect to x
arcr = c.create_rectangle(300,160,400,80, fill="purple")#alex
c.pack()

win.title("ALEX GUI")
mylabel=tk.Label(win,text="Distance in CM",font=("ubuntu",12))
mylabel.place(x=295,y=0)

label1 = tk.Label(master=c, text="I", bg="red", font=("ubuntu",20,'bold'))
label1.place(x=50, y=40)
label2 = tk.Label(master=c, text="U", bg="red", font=("ubuntu",20,'bold'))
label2.place(x=15, y=40)
label3 = tk.Label(master=c, text="<", bg="red", font=("ubuntu",20,'bold'))
label3.place(x=50, y=160)
label4 = tk.Label(master=c, text="O", bg="red", font=("ubuntu",20,'bold'))
label4.place(x=85, y=40)
label5 = tk.Label(master=c, text="K", bg="red", font=("ubuntu",20,'bold'))
label5.place(x=50, y=100)
label6 = tk.Label(master=c, text=">", bg="red", font=("ubuntu",20,'bold'))
label6.place(x=85, y=160)
label7 = tk.Label(master=c, text="M", bg="red", font=("ubuntu",20,'bold'))
label7.place(x=15, y=160)
label8 = tk.Label(master=c, text="J (colour)", bg="yellow", font=("ubuntu",20,'bold'))
label8.place(x=130, y=40)
label9 = tk.Label(master=c, text="L (speed)", bg="orange", font=("ubuntu",20,'bold'))
label9.place(x=130, y=160)

colour="ALEX"
labela = tk.Label(master=c, text=colour,bg="blue", font=("ubuntu",20,'bold'))
labela.place(x=315, y=100)

dist=2#comment out
labelr = tk.Label(master=c, text=dist,bg="red", font=("ubuntu",20,'bold'))#comment out
labelr.place(x=335, y=30)#comment out


rospy.init_node('listener', anonymous=True)
rospy.Subscriber('chatter', Int16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

#if __name__ == '__main__':
#    listener()
win.mainloop()

