ROS dev container for vscode
----------------------------
Packed with:
- Preconfigured docker image for ROS development.
- Browser accessible X11 server to display gazebo, rviz, rqt (runs on Windows/Mac).
- Tasks definition to run catkin_make, roscore, rviz commands.

![screenshot](https://user-images.githubusercontent.com/18067/58605055-8dc84980-82d1-11e9-8ee5-dc969fcb2ae1.png)

How to use this dev container
-----------------------------
First, you have to install VS Code and Docker for Windows/Mac:
- https://code.visualstudio.com/
- https://docs.docker.com/docker-for-mac/
- https://docs.docker.com/docker-for-windows/

After you installed required softwares:

1. Install ["Remote Development" extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) on your VS Code.
2. Clone this repository by using git command.
3. Click on the quick actions status bar item (green icon) in the lower left corner of the VS Code.
4. Select "Remote-Containers: Open Folder in Container..." from the command list that appears, and open the root folder of the project you just cloned.
5. You need to wait a while for container to come up (only required once).

For detailed instructions, see:
https://code.visualstudio.com/docs/remote/containers

How to open X11 server screen
-----------------------------

1. Wait for the container to start.
2. Open http://localhost:3000/ using your favorite browser.

If you are using Docker Toolbox, open the following URL instead:

http://192.168.99.100:3000/

If you want browser screen to be integrated with VS Code, use [Browser Preview for VS Code extension](https://marketplace.visualstudio.com/items?itemName=auchenberg.vscode-browser-preview).

Created by
----------
Yosuke Matsusaka (MID Academic Promotions, Inc.)