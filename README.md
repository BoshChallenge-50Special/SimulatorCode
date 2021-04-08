# SimulatorCode


# Code to Use behaviour tree

To use in main.py

'''
elif(run_type=="ros"):
  #pro=Processer(cam, car, sem, gps, bno)
  #pro.start()
  #os.system("rosrun startup_package test_python3.py --venv src/startup_package/src/SimulatorCode/venv &")
  print(os.system("rosrun startup_package processer_tree.py &"))
  while not rospy.is_shutdown():
    sleep(0.5)
  os.system("pkill -f tree.py")
'''


Pacchetti che forse vanno insallati

sudo apt install -y python3 python3-dev python3-pip build-essential
pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg
pip3 install py-trees
