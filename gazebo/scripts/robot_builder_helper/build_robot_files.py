from nodes import Node, NodeMatrix
import sys
#This vavlue is used in creating the "Node Matrix" which I use as a higher level representation of the robot configuration
MAX_ROBOT_SIZE = 5


def create_x_launch(nodeMatrix, ID):
	pass

def create_one_x_mount(nodeMatrix, ID):
	f = open('one_'+str(ID)+'_mount.launch', 'w+')
	s = '''<?xml version="1.0"?>

<launch>
    <arg name="robot_name"/>
    <arg name="init_pose"/>
    <!-- icreate robot model -->
    <node name="create_model_mount" args="$(arg init_pose) -file $(find weaselball_description)/meshes/''' + str(ID) +'''/model.sdf -sdf -model $(arg robot_name)" pkg="gazebo_ros" type="spawn_model"/>
</launch>'''
	f.write(s)

def create_x_model_sdf(nodeMatrix, ID):
	pass

def create_x_model_config(nodeMatrix, ID):
	f = open('model.config', 'w+')
	s = '''<?xml version="1.0" ?>
<model>
    <name>''' + str(ID) +  '''</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name>Justin Wasserman</name>
        <email></email>
    </author>
    <description></description>
</model>'''
	f.write(s)

def create_nodes(robot_ID):
	#This will need to be done by hand for now until an algorithm can be made to do it
	nodes_L = []
	nodeMatrix = NodeMatrix(MAX_ROBOT_SIZE)
	mid_x = int(MAX_ROBOT_SIZE / 2)
	mid_y = int(MAX_ROBOT_SIZE / 2)
	if(robot_ID == 1):
		#1
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
	elif(robot_ID == 2):
		#2
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
	elif(robot_ID == 3):
		#3_straight
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+2] = 1	
	elif(robot_ID == 4):
		#3_L
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x-1][mid_y] = 1	
	elif(robot_ID == 5):
		#3_backwords_L
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x+1][mid_y] = 1	
	elif(robot_ID == 6):
		#4_straight
		nodematrix.Matrix[mid_x][mid_y] = 1	
		nodematrix.Matrix[mid_x][mid_y+1] = 1	
		nodematrix.Matrix[mid_x][mid_y+2] = 1	
		nodematrix.Matrix[mid_x][mid_y-1] = 1	
	elif(robot_ID == 7):
		#4_L
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+2] = 1	
		nodeMatrix.Matrix[mid_x+1][mid_y] = 1	
	elif(robot_ID == 8):
		#4_backwords_L
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+2] = 1	
		nodeMatrix.Matrix[mid_x-1][mid_y] = 1	
	elif(robot_ID == 9):
		#4_T
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+2] = 1	
		nodeMatrix.Matrix[mid_x+1][mid_y+1] = 1	
	elif(robot_ID == 10):
		#4_square
		nodeMatrix.Matrix[mid_x][mid_y] = 1	
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x+1][mid_y+1] = 1	
		nodeMatrix.Matrix[mid_x+1][mid_y] = 1	
	else:
		print("[Error] Can not create the robot of this ID right now!")
	return nodeMatrix

	
		

#For the sake of parsing here are the ID to Robot conversions
# 1 = 1, 2 = 2, 3 = 3_straight, 4 = 3_L, 5 = 3_backwords_L, 6 = 4_straight, 7 = 4_L, 8 = 4_backwords_L, 9 = 4_T , 10 =4_square
if __name__ == "__main__":
	#Parse the input for the ID of the robot 
	robotID = int(sys.argv[1])
	#Get the nodes of the robot
	nodeMatrix = create_nodes(robotID)
	#create files
	create_x_launch(nodeMatrix, robotID)
	create_one_x_mount(nodeMatrix, robotID)
	create_x_model_sdf(nodeMatrix, robotID)
	create_x_model_config(nodeMatrix, robotID)
