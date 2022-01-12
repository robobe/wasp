# Gazebo
- Install gazebo11 from repository
- Install dev library libgazebo11-dev

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo11
sudo apt-get install libgazebo11-dev
```
# Reference
- [Install Gazebo using Ubuntu packages](http://gazebosim.org/tutorials?tut=install_ubuntu)