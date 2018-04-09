# How to connect to IFI Drift-supported ROS Virtual Machine
## Semester: Spring 2018

Drift has just set up virtual machines running Ubuntu 16.04 and ROS Kinetic for the participants if INF3480/4380 course. **You can access them only if you are officially enrolled for the course!**

## Option 1 - Regular Client (Recommended)

Regular client (recommended) - Drift is working on distributing the VMware Horizon client to our RHEL client machines, but for now I can install it on individual machines: Add server: `vdi-apcon-prod.uio.no` Then you can login using your university credentials.

You can download VM Ware Horizon client here: https://my.vmware.com/web/vmware/info?slug=desktop_end_user_computing/vmware_horizon_clients/4_0

## Option 2 - Web Interface

You can use browser-based web client. Just navigate to https://vdi-apcon-prod.uio.no/ (redirect: http://ros.ifi.uio.no) and login with your university credentials. It is a test-setup and some issues, especially with login can occurr.

# Screenlock

If the screen is locked, just login again using your university password.

## !!!IMPORTANT!!! - Power off and virtual machine state - Make Backups of your catkin workspace

Due to security risks the administration rights are not given, so "sudo" commands will not work. **Furthermore, the state of the virtual machine after shutting down or connection breaking might not be saved!** So it's very important for you to make backups of your work before ending the session! **The best option is to make backup of your whole catkin workspace**. We recommend to use git, but just transferring the whole catkin workspace to your own computer is good enough as well.
