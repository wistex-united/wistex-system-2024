# Instructions on installing VM for linux
## ARM Macs
Download UTM from here: `https://mac.getutm.app`
Download ARM Ubuntu from here: `https://ubuntu.com/download/server/arm`

Follow these instructions to install: `https://www.youtube.com/watch?v=MVLbb1aMk24&t=311s`
NOTE: To get a GUI since this is a server install you have to run (The video has a different command based on tasksel ignore that): 
`sudo apt install ubuntu-desktop^` (The ^ IS important)
and
`sudo reboot`
You can likely do most of it yourself but after you install you will encounter a flashing cursor. To fix go to the video and it shows you the fix.



Ubuntu server has an issue with using all space allocated (You'll only have 32GB no matter how much you give it in the VM. 

To fix this go to terminal (type individual commands):
`sudo lvm`

`lvextend -l +100%FREE /dev/ubuntu-vg/ubuntu-lv`

`exit`

`sudo resize2fs /dev/ubuntu-vg/ubuntu-lv`

`df -h`

(Source for reference: `https://askubuntu.com/questions/1106795/ubuntu-server-18-04-lvm-out-of-space-with-improper-default-partitioning`)

For better display settings:
`Settings -> Displays -> Resolution -> Set to 1920x1200`

After this you can follow normal BHuman code setup here:
`https://docs.b-human.de/coderelease2023/getting-started/initial-setup/`
