# Test ethercat environment for linuxcnc using halcmd.
cd scripts
. ./rip-environment

cd ..
cd bin
halcmd loadusr -W /home/user/linuxcnc_distro_files/linuxcnc/bin/lcec_conf /home/user/linuxcnc_distro_files/linuxcnc/rtlib/ethercat-conf.xml
halcmd show

halcmd loadrt threads name1=base-thread fp1=0 period1=30000
halcmd loadrt lcec

halcmd net ec-slaves-responding lcec.slaves-responding
halcmd net ec-link-up lcec.link-up
halcmd net ec-all-op lcec.all-op

halcmd addf lcec.read-all base-thread
halcmd addf lcec.write-all base-thread

halcmd start
halshow # or: halshow


# Clear linux hal environment
halrun -U
