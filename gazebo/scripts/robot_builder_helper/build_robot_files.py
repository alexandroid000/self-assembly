from nodes import Node, NodeMatrix
import sys
#This vavlue is used in creating the "Node Matrix" which I use as a higher level representation of the robot configuration
MAX_ROBOT_SIZE = 11

def create_x_script(robotID):
	f = open("run.sh", "w+")
	s1 = '''#!/usr/bin/env bash

killall gzserver
killall gzclient

#Get Workspace Path
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
WORKSPACE_PATH="$(dirname "$SCRIPT_PATH")"
cwd=$PWD
source $WORKSPACE_PATH/scripts/Robot_Config.txt

source $WORKSPACE_PATH/devel/setup.sh
#Upload the test file to make sure everything is connected fine
cd $WORKSPACE_PATH/data/collections
chmod +x upload.sh
rm *.csv
rm *.txt


touch testUpload
echo $(date) >> testUpload
sh ./upload.sh s3://weaselball-data/''' + str(robotID) + '''

cd $cwd
export GAZEBO_MODEL_PATH=$WORKSPACE_PATH/src/weaselball_description/meshes:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$WORKSPACE_PATH/src/weaselball_gazebo/worlds:$GAZEBO_RESOURCE_PATH

roslaunch weaselball_gazebo ''' + str(robotID) + '''.launch init_cond:=$doit

if [ "$UPLOAD_DATA" -eq "1" ]; then
	cd $WORKSPACE_PATH/data/collections
	chmod +x upload.sh
	sh ./upload.sh s3://weaselball-data/''' + str(robotID) + '''
	if [ "$DELETE_AFTER_UPLOAD" -eq "1" ]; then
		rm *.csv
	fi
fi


killall gzserver
killall gzclient
echo "''' + str(robotID) + ''' Finished"

if [ "$RUNNING_ON_AWS" -eq "1" ]; then
    sudo shutdown now
fi
'''
	f.write(s1)

def create_x_launch(nodeMatrix,node_positions, ID, GUI):
	f = open(str(ID) + ".launch", 'w+')
	s1 = '''<launch>
 <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find weaselball_gazebo)/worlds/simulate.world"/>
    <arg name="gui" value="''' + str(GUI) + '''"/>
    <arg name="paused" value="true"/>
  </include>

'''
	#Create Weaselballs
	s2_list = []
	for i, position in enumerate(node_positions):
		s_temp = '''<group ns="weasel''' + str(i) + '''">
<include file="$(find weaselball_gazebo)/launch/include/one_robot.launch">
<arg name="robot_name" value="swarmbot''' + str(i) + '''"/>
<arg name="init_pose" value="-x ''' + str(position[0])+ ''' -y ''' + str(position[1]) +''' -z 0.03 -R 1.1 -P 1.2 -Y 1.3"/>
</include>
</group>

'''
		s2_list.append(s_temp)
	s2 = ''.join(s2_list)

	#Create the weaselball structure
	s3 = '''<group ns="mount0">
<include file="$(find weaselball_gazebo)/launch/include/one_''' + str(ID) + '''_mount.launch">
<arg name="robot_name" value="mount0"/>
<arg name="init_pose" value="-x -0.0 -y -0.0 -z 0.01"/>
</include>
</group>

</launch>
'''
	f.write(s1 + s2 + s3)



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

def create_x_model_sdf(nodeMatrix, node_positions, ID):
	f = open('model.sdf', 'w+')
	s1 = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='""" + str(ID) + """'>
"""
	s2_list = []
	for i, position in enumerate(node_positions):
		s_temp = """      <link name='mountlink_""" + str(i) + """'>
        <collision name='coll""" + str(i) + """'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://mount/meshes/mount_collision.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <inertial>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <inertia>
          <ixx>0.00055873</ixx>
          <ixy>0.0000</ixy>
          <ixz>0.00000</ixz>
          <iyy>0.00055873</iyy>
          <iyz>0.00000/iyz></iyz>
          <izz>0.0010789</izz>
          </inertia>
          <mass>0.028</mass>
        </inertial>
        <visual name='mountvisual""" + str(i) + """'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://mount/meshes/mount.dae</uri>
            </mesh>
          </geometry>
        </visual>
		   <sensor name='contactSensor' type='contact'>
		   <plugin name="bumpSensor" filename="libbumpSensor.so"> </plugin>
		      <contact>
			    <collision>coll""" + str(i) + """</collision>
		      </contact>
		   </sensor>
		<pose frame=''>""" + str(position[0]) + """ """ +str(position[1]) + """ 0 0 -0 0</pose>
      </link>
"""
		s2_list.append(s_temp)
	s2 = ''.join(s2_list)

	s3_list = []
	joint_positions = []
	#Find all joint pairs
	for i in range(len(node_positions)):
		for j in range(len(node_positions)):
			if i == j:
				continue
			if (i,j) in joint_positions or (j,i) in joint_positions:
				continue
			joint_positions.append( (i,j) )

	for i, pair in enumerate(joint_positions):
		s_temp = """    <joint name='mountlink_JOINT_""" + str(i) + """' type='fixed'>
      <parent>mountlink_""" + str(pair[0]) +"""</parent>
      <child>mountlink_""" + str(pair[1]) + """</child>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </limit>
          <suspension>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </suspension>
        </ode>
      </physics>
    </joint>
"""
		s3_list.append(s_temp)
	s3 = ''.join(s3_list)
	s4 = """    <static>0</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>"""
	f.write(s1 + s2 + s3 + s4)





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

def set_node_positions(nodeMatrix):
	node_position_l = []
	mount_diameter = 0.0554 * 2
	for x in range(MAX_ROBOT_SIZE):
		for y in range(MAX_ROBOT_SIZE):
			if(nodeMatrix.Matrix[x][y] == 1):
				node_position_l.append(((x-int(MAX_ROBOT_SIZE/2))*mount_diameter, ((y-int(MAX_ROBOT_SIZE/2))*mount_diameter)))
	return node_position_l

#RRT Build robot Structure
#Given a number of robots we want to build at least, create a robot structure using RRT like algorithm
#I wonder what the distribution of number of robots is...
def build_RRTBot(minimum_robots):
    if (MAX_ROBOT_SIZE * MAX_ROBOT_SIZE < minimum_robots):
        print("[build_RRTBot] Can't build robot, not enough cells. Increa MAX_ROBOT_SIZE")
    nodeMatrix = NodeMatrix(MAX_ROBOT_SIZE)
    cells = range(MAX_ROBOT_SIZE)
    total_robots = 0
    #Start robot tree in the middle
    nodeMatrix.Matrix[MAX_ROBOT_SIZE/2][MAX_ROBOT_SIZE/2] = 1
    while(total_robots < minimum_robots):
        #Choose a random point in the discrete space to build the robot
        chooseFlag = 1
        while(chooseFlag == 1):
            x,y = random.choice(cells), random.choice(cells)
            if(nodeMatrix.Matrix[x][y] != 1):
                chooseFlag = 0
        nodeMatrix.Matrix[x][y] = 1
        x_end, y_end = closestBot((x,y), nodeMatrix)
        path = getPath((x,y),(x_end, y_end))
        for points in path:
            nodeMatrix.Matrix[point[0], point[1]] = 1

#Helper function for build_RRTBot
#Gets the closest robot (Manhattan distance) to given robot
def getClosestBot(start, design):
    distances = {}
    for i in range(MAX_ROBOT_SIZE):
        for j in range(MAX_ROBOT_SIZE):
            if design.Matrix[i][j] == 1:
                distances[(i,j)] = (i-start[0]) + (j-start[1])
    ret = min(distances, key=distances.get)
    if len(ret) == 0:
        print("[Robot Builder] Something went wrong, no other robots found!")
        return
    elif len(ret) == 1:
        return ret
    else:
        return ret[0]



#Helper function for build_RRTBot
#Given a start discrete (x,y) and finish (x,y), find a path between the 2 points.
def getPath(start, finish):
    if start == finish:
        return start
    explored = []
    queue = [start]

    while queue:
        path = queue.pop(0)
        node = path[-1]
        if node not in explored:
            neighbours = [] 
            if (node[0] + 1 < MAX_ROBOT_SIZE):
                neighbours.append(node[0]+1,node[1])
            if (node[0] - 1 >= 0):
                neighbours.append(node[0]-1, node[1])
            if (node[1] + 1 < MAX_ROBOT_SIZE):
                neighbours.append(node[0], node[1] + 1)
            if (node[1] - 1 >= 0):
                neighbours.append(node[0], node[1] - 1)
            for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                if neighbour == goal:
                    return new_path

            explored.append(node)
    print("[getPath] No Path found!")


    

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
		nodeMatrix.Matrix[mid_x][mid_y] = 1
		nodeMatrix.Matrix[mid_x][mid_y+1] = 1
		nodeMatrix.Matrix[mid_x][mid_y+2] = 1
		nodeMatrix.Matrix[mid_x][mid_y-1] = 1
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



if __name__ == "__main__":
    nodeMatrix = build_RRTBot(55):  

#For the sake of parsing here are the ID to Robot conversions
# 1 = 1, 2 = 2, 3 = 3_straight, 4 = 3_L, 5 = 3_backwords_L, 6 = 4_straight, 7 = 4_L, 8 = 4_backwords_L, 9 = 4_T , 10 =4_square
#if __name__ == "__main__":
#	#Parse the input for the ID of the robot
#	if(len(sys.argv) == 1 or len(sys.argv) == 2):
#		print("[ERROR] Please enter a robot to build")
#		print(sys.argv)
#		exit()
#	robotID = int(sys.argv[1])
#	GUI = sys.argv[2]
#	#Get the nodes of the robot
#	nodeMatrix = create_nodes(robotID)
#	node_positions = set_node_positions(nodeMatrix)
#	if(len(node_positions) == 0):
#		print("[ERROR] Incorrect Robot ID set")
#		exit()
#	#create files
#	print("[Debug] Building Files")
#	create_x_launch(nodeMatrix, node_positions, robotID, GUI)
#	create_one_x_mount(nodeMatrix, robotID)
#	create_x_model_sdf(nodeMatrix, node_positions, robotID)
#	create_x_model_config(nodeMatrix, robotID)
#	create_x_script(robotID)
