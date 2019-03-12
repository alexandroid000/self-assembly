#the purpose of this file is to have a script that will collect data by collecting data about the different size/structured weaselballs. This script will take in a range of the weaselball labels we would like to run and then records the data on Gazebo and send it to AWS.
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
                    fout.write("LARGE_ROBOT_GENERATOR="+str(0) + "\n")
                #We want to turn this off otherwise this will shut down the computer after each run, so it'd only run once
                elif("RUNNING_ON_AWS=" in line):
                    fout.write("RUNNING_ON_AWS=="+str(0) + "\n")
                else:
                    fout.write(line)
    moveGeneratedFile()

def replace_Robot_To_Run(newK):
    with open("Robot_Config.txt", "rt") as fin:
        with open("New_Robot_Config.txt", "wt") as fout:
            for line in fin:
                if("ROBOT_TO_RUN=" in line):
                    fout.write("ROBOT_TO_RUN="+str(int(newK)) + "\n")
                else:
                    fout.write(line)
                    
    moveGeneratedFile()

if __name__ == "__main__":
    #Setup initial script
    setup_robot_config()
    #Chose RRTBot size range to test over
    robots_to_run = range(3,8) 
    #Run Tests!
    for robot in robots_to_run:
        #Modify script to have correct number of robots
        replace_Robot_To_Run(robot)
        #Run the build of the environment
        subprocess.call(["./setupEnvironment.sh"])
        #Run ./run.sh
        subprocess.call(["./run.sh"])

