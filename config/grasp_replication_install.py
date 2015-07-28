#! /usr/bin/env python

import os
import datetime
import time

if __name__ == "__main__":
	print "Installing package dependencies"
	pkgs = "sox chrony isc-dhcp-server openssh-client openssh-server gnome-terminal"
	os.system("apt-get install " + pkgs)

	print "Installing DHCP and chrony configuration files."
	if os.path.exists("/etc/chrony/chrony.conf"):
		print "Detected chrony config file. Creating backup."
		now = str(datetime.date.fromtimestamp(time.time()))
		os.system("mv /etc/chrony/chrony.conf /etc/chrony/chrony.conf." + now)
	
	os.system("cp ./chrony.conf /etc/chrony/")
	os.system("service chrony restart")

	# To be done later... DHCP
#	if os.path.exists("/etc

	if os.path.exists("~/.ssh"):
		print "Detected existing ssh setup. Backing up..."
		now = str(datetime.date.fromtimestamp(time.time()))
		os.system("mv ~/.ssh ~/.ssh" + now)
		print "Original ssh file renamed to .ssh" + now

	os.system("mv ./.ssh ~/")
	os.system("service ssh restart")

	print "It is a good idea to check that the time on the robot is the same as the time on the client computer. user 'date -u'"

	print "Adding WAM hostname to hosts file."
	os.system("echo '192.168.109.100	WAM' >> /etc/hosts")
