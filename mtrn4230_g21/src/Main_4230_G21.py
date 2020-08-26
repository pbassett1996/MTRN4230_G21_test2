#!/usr/bin/env python
#Project: MTRN4230 Group Project
#Group: G21
#Date: 29/07/2020
#About: The following function coordinates the integration of the object detection and trajectory planner objects as well as the Graphical User Interface.

# General Libraries
import numpy as np
import math, random, time, os

# ROS
import rospy, sys, tf
from geometry_msgs.msg import *
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo

#custom pick msg created for the project
from mtrn4230_g21.msg import Pick

# OpenCV
import cv2, cv_bridge
from cv_bridge import CvBridgeError

# Robot Arm
from gazebo_msgs.srv import *

# Tkinter, GUI
import Tkinter as tk
import ttk
from PIL import Image
# Project imports
from computer_vision.ObjectDetection_4230_G21 import ObjectDetection
from trajectory_planning.TrajectoryPlanner_4230_G21 import MoveItCartesianPath


models_folder = "../models"

colour = "all"
shape = "all"
kinect_height = 1.5 #meters above the ground
pos_array_x  = np.zeros(20)
pos_array_y = np.zeros(20)
object_num = 0
pick_objs = False
pos = []

ObjectDict = {
    "red cube" : 2,
    "blue box" :2,
    "green cube" :3,
    "yellow cylinder" : 3
}

def StringToInts(val):
    num = 0
    if (val != ""):
        num = int(val)
    
    return num

#Colour and shape identifier
def setVars(OD):
    global colour, shape, pick_objs, object_num, pos
    
    if(greenCube_var.get() != "" or redCube_var.get() != "" or blueBox_var.get() != "" or yellCyl_var.get() != ""):
        colour = "all"
        shape = "all"
        ObjectDict = {
            "red cube" : StringToInts(redCube_var.get() ),
            "blue box" :StringToInts(blueBox_var.get()),
            "green cube" :StringToInts(greenCube_var.get()),
            "yellow cylinder" : StringToInts(yellCyl_var.get())
        }
        OD.UserInput(colour, shape, ObjectDict, True)
    else :
        colour = col.get()
        shape = shp.get()
        ObjectDict = {
            "red cube" : 0,
            "blue box" :0,
            "green cube" :0,
            "yellow cylinder" : 0
        }
        OD.UserInput(colour, shape, ObjectDict, False)

    #print "Picking up", str(colour), "objects of", str(shape), "shape/s."
    
    time.sleep(3)
    object_num = 0
    pos = OD.get_coordinates() #Get object coordinates
    pick_objs = True

#Delete objects in Gazebo simulation
#Source: http://wiki.ros.org/rospy/Overview/Services
def delete_objects(number, delete, flag):
    if flag == True:
        item_name = "kinect1"
        d(item_name)
    for num in xrange(0, number+1):
        item_name = "Object_{0}".format(num)
        delete(item_name)
        time.sleep(0.1)
        
#Spawn objects in Gazebo simulation
#Source: http://wiki.ros.org/rospy/Overview/Services
def spawn_objects(number, spawn, obj1, obj2, obj3, obj4, orientation):
    print("Spawning objects into robot's field of view")
    global pos_array_x
    global pos_array_y
    obj_pos = []

    
    for num in xrange(0,number+1):
        flag = False
        while flag == False:
            flag2 = False
            pos_x = random.uniform(0.2,0.7)
            pos_y = random.uniform(0.1,0.7)
            for i in range(0, len(obj_pos)+1):
                if(len(obj_pos) == 0):
                    obj_pos.append([pos_x, pos_y])
                    break
                elif(float(math.sqrt((pos_x-obj_pos[i][0])*(pos_x-obj_pos[i][0])+(pos_y-obj_pos[i][1])*(pos_y-obj_pos[i][1]))) < 0.1):
                    break
                elif((i+2) > len(obj_pos)):
                    obj_pos.append([pos_x, pos_y])
                    flag = True
                    break
                   
        pos_array_y[num] = pos_y
        pos_array_x[num] = pos_x
        item_name = "Object_{0}".format(num)
        item_pose = Pose(Point(x=pos_x, y = pos_y, z = 0), orientation)
        if(num >= 0 and num <= round(number/4)):
            spawn(item_name, obj1, "", item_pose, "world")
        elif(num >= round(number/4) and num <= 2*round(number/4)):
            spawn(item_name, obj2, "", item_pose, "world")
        elif(num >= 2*round(number/4) and num <= 3*round(number/4)):
            spawn(item_name, obj3, "", item_pose, "world")
        else:
            spawn(item_name, obj4, "", item_pose, "world")
        time.sleep(0.1)

#Close program
def close_windows(num, d):
    print("Closing down")
    delete_objects(num, d, False)
    cv2.destroyAllWindows()
    root.destroy()
    quit()

#Reset UI vals
def reset_UI():
    col.set("none")
    shp.set("none")
    greenCube_var.set("") 
    redCube_var.set("") 
    blueBox_var.set("")
    yellCyl_var.set("")



#Used to reset the objects in the kinects field of view
def reset_obj(num, s, d , obj1, obj2, obj3, obj4, orientation, OD):
    print("Resetting objects")
    global object_num, pick_objs
    reset_UI()
    OD.reset()
    pick_objs = False
    delete_objects(num, d, False)
    time.sleep(0.1)
    spawn_objects(num, s,obj1, obj2, obj3, obj4, orientation)
    object_num = 0

#Recrusive function that controls the motion planner using the object detection output   
def run(OD,pub, count, root, flag):
    global object_num, pick_objs, pos
    count += 0.1
    
    #If the arm is not moving and we have pressed "go"
    #TODO: wait for robot to finish task before retasking
    if(pick_objs == True):
        if(object_num < len(pos)):
            #send this to the pick place node
            #MP.pickup(pos[object_num][0], pos[object_num][1], 0.1)
            msg = Pick()
            msg.x = pos[object_num][0]
            msg.y = pos[object_num][1]
            msg.depth = pos[object_num][2]
            rospy.loginfo(msg)
            pub.publish(msg)

            object_num += 1

    if count < 100:
        root.after(100, run, OD,pub, count, root, flag)

def nullCmd():
    pass

if __name__ == "__main__":
    rospy.init_node('Main')
    pub = rospy.Publisher('pick_point_publisher', Pick,queue_size=100)

    #Block until service is available
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/get_model_state")

    #Quaternion orientation of kinect and objects
    orient_kinect = Quaternion(*tf.transformations.quaternion_from_euler(0,1.57,0))
    orient_obj = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

    #Create objects for spawn and delete roservice
    s_kinect = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    s_obj = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    d = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    states = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)


    #Open URDF and SDF model files
    with open(os.path.join(models_folder, "blue_box.urdf"), "r") as f:
        blue_box = f.read()
    with open(os.path.join(models_folder, "red_box.urdf"), "r") as f:
        red_box = f.read()
    with open(os.path.join(models_folder, "green_cube.urdf"), "r") as f:
        green_cube = f.read()
    with open(os.path.join(models_folder, "yellow_cylinder.urdf"), "r") as f:
        yellow_cylinder = f.read()
    with open(os.path.join(models_folder, "kinect.sdf"), "r") as f:
        kinect = f.read()

    #Spawn the kinect into the world
    model = states('kinect1', "link")
    if(model.success == False):
        item_pose = Pose(Point(x=0.5, y=0.3, z = kinect_height), orient_kinect)
        s_kinect("kinect1", kinect, "",item_pose, "world")
        time.sleep(1)
        
    #Spawn the objects for detection
    obj_num = 15
    spawn_objects(obj_num, s_obj, blue_box, red_box, green_cube, yellow_cylinder, orient_obj)
    
    try:
        OD = ObjectDetection(colour, shape, ObjectDict)
    except KeyboardInterrupt:
        print "Shutting down."

    root = tk.Tk()
    root.title("MTRN4230 G21")
    root.geometry("500x500")
    root.configure(background='white smoke')

    col = tk.StringVar()
    col.set("none")
    shp = tk.StringVar()
    shp.set("none")

    title = tk.Text(root)
    title.insert(tk.INSERT,"MTRN4230 G21")

    titLable = tk.Label(root, text="MTRN4230 G21",bg='white smoke')
    titLable.place(relx=0.25, rely=0.05)
    titLable.config(font=("Space", 20))

    #colour drop down menu
    colLabel = tk.Label(root, text="Option 1: Choose colours and/or shapes to be picked", bg = 'white smoke')
    colLabel.place(relx=0.1, rely = 0.2)
    colLabel2 = tk.Label(root, text = "Colour", bg = 'white smoke')
    colLabel2.place(relx = 0.1, rely = 0.3)
    colorDrop = tk.OptionMenu(root, col,'none','red', 'blue', 'green', 'yellow', 'all')
    colorDrop.config(fg='white', bg='slate gray', width=10, borderwidth=5)
    colorDrop["menu"].config(fg='white', bg='slate gray')
    colorDrop.place(relx=0.2,rely=0.275)

    #shape drop down menu
    shapeLabel = tk.Label(root, text="Shape", bg = 'white smoke')
    shapeLabel.place(relx=0.55, rely = 0.3)
    shapeDrop = tk.OptionMenu(root, shp, 'none','cube', 'box', 'cylinder', 'all')
    shapeDrop.config(fg='white', bg='slate gray', width=10, borderwidth=5)
    shapeDrop["menu"].config(fg='white', bg='slate gray')
    shapeDrop.place(relx=0.65,rely=0.275)    

    #Object combination selection
    objLabel = tk.Label(root, text="Option 2: Choose a combination of objects to be picked", bg = 'white smoke')
    objLabel.place(relx=0.1, rely = 0.425)

    #Red Cube
    redCube_var = tk.StringVar()
    redCube = tk.Entry(root, textvariable = redCube_var, width = 5)
    redCube.place(relx=0.1, rely = 0.525)

    canvas1 = tk.Canvas(root, width =40, height = 40, bg = 'white smoke',bd=0, highlightthickness=0, relief='ridge')
    canvas1.place(relx = 0.3, rely = 0.5)
    
    img_redCube = tk.PhotoImage(file = "RedCube.png")
    canvas1.create_image(20,20, image = img_redCube)

    #Green Cube
    greenCube_var = tk.StringVar()
    greenCube = tk.Entry(root,textvariable = greenCube_var, width = 5)
    greenCube.place(relx=0.6, rely = 0.525)

    canvas2 = tk.Canvas(root, width =40, height = 40, bg = 'white smoke',bd=0, highlightthickness=0, relief='ridge')
    canvas2.place(relx = 0.8, rely = 0.5)
    
    img_greenCube = tk.PhotoImage(file = "GreenCube.png")
    canvas2.create_image(20,20, image = img_greenCube)

    #Blue Box
    blueBox_var = tk.StringVar()
    blueBox = tk.Entry(root,textvariable = blueBox_var, width = 5)
    blueBox.place(relx=0.1, rely = 0.675)

    canvas3 = tk.Canvas(root, width =75, height = 40, bg = 'white smoke',bd=0, highlightthickness=0, relief='ridge')
    canvas3.place(relx = 0.3, rely = 0.65)
    
    img_blueBox = tk.PhotoImage(file = "BlueBox.png")
    canvas3.create_image(37,20, image = img_blueBox)

    #Yellow Cylinder
    yellCyl_var = tk.StringVar()
    yellCyl = tk.Entry(root,textvariable = yellCyl_var, width = 5)
    yellCyl.place(relx=0.6, rely = 0.675)

    canvas4 = tk.Canvas(root, width =40, height = 50, bg = 'white smoke',bd=0, highlightthickness=0, relief='ridge')
    canvas4.place(relx = 0.8, rely = 0.65)

    img_yellCyl = tk.PhotoImage(file = "YellowCylinder.png")
    canvas4.create_image(20,25, image = img_yellCyl)
  

    #start button
    startButton = tk.Button(root, text="START", command = lambda: setVars(OD), width=7, height=2, fg="white", bg="green4",borderwidth=3)
    startButton.place(relx=0.1,rely=0.8)
    #resetbutton
    resetButton = tk.Button(root, text="RESET", command = lambda: reset_obj(obj_num, s_obj, d, blue_box, red_box, green_cube, yellow_cylinder, orient_obj, OD), width=7,height=2, fg="white", bg="orange",borderwidth=3)
    resetButton.place(relx=0.3,rely=0.8)

    #clear values button
    stopButton = tk.Button(root, text="CLEAR", command = lambda: reset_UI(), width=7,height=2, fg="white", bg="orange",borderwidth=3)
    stopButton.place(relx=0.5,rely=0.8)

    #stop button
    stopButton = tk.Button(root, text="STOP", command = lambda: close_windows(obj_num, d), width=7,height=2, fg="white", bg="red3",borderwidth=3)
    stopButton.place(relx=0.7,rely=0.8)

   
    
    # slider
    #quantity = tk.Scale(root, from_=0, to=10, length=250, tickinterval=1, orient=tk.HORIZONTAL, width=20)
    #quantity.place(x=200,y=200)

    time.sleep(0.1)
    count = 0
    flag = True
    run(OD,pub, count, root,flag)

    root.mainloop()
    #rospy.spin()
