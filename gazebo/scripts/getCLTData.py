#the purpose of this file is to have a script that will collect data by collecting data about the different size RRTBots. This script will take in a range of RRTBot sizes and then record the data on Gazebo and send it to AWS.
import subprocess
import shutil
def moveGeneratedFile():
    shutil.move("New_Robot_Config.txt", "Robot_Config.txt")

#This will only setup the minimum needed to get all of the data onto AWS. Everything else should be setup by the user.
def setup_robot_config():
    with open("Robot_Config.txt", "rt") as fin:
        with open("New_Robot_Config.txt", "wt") as fout:
            for line in fin:
                if("UPLOAD_DATA=" in line):
                    fout.write("UPLOAD_DATA="+str(1) + "\n")
                elif("LARGE_ROBOT_GENERATOR=" in line):
                    fout.write("LARGE_ROBOT_GENERATOR="+str(1) + "\n")
                #We want to turn this off otherwise this will shut down the computer...
                elif("RUNNING_ON_AWS=" in line):
                    fout.write("RUNNING_ON_AWS=="+str(0) + "\n")
                else:
                    fout.write(line)
    moveGeneratedFile()

def replace_K_large(newK):
    with open("Robot_Config.txt", "rt") as fin:
        with open("New_Robot_Config.txt", "wt") as fout:
            for line in fin:
                if("K_LARGE=" in line):
                    fout.write("K_LARGE="+str(int(newK)) + "\n")
                elif("N_LARGE=" in line):
                    fout.write("N_LARGE="+str(int(newK)) + "\n")
                else:
                    fout.write(line)
                    
    moveGeneratedFile()

if __name__ == "__main__":
    #Setup initial script
    setup_robot_config()
    #Chose RRTBot size range to test over
    robots_to_run = range(1,3) 
    #Run Tests!
    for robot in robots_to_run:
        #Modify script to have correct number of robots
        replace_K_large(robot)
        #Run the build of the environment
        subprocess.call(["./setupEnvironment.sh"])
        #Run ./run.sh
        subprocess.call(["./run.sh"])

