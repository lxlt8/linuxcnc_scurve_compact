# disable the do you want to close dialog
root_window.tk.call("wm","protocol",".","WM_DELETE_WINDOW","destroy .")

# set the view of the preview
# valid views are view_x view_y view_y2 view_z view_z2 view_p
commands.set_view_z()

# set the size of startwindow
#root_window.tk.call("wm","geometry",".","1600x1600")








##########################################################
########       LOAS_LAST FILE   PY3 for 2.9   ############
########   In ini file under [DISPLAY]  ##################
########     LOAD_LASTFILE = 	YES       ################
##########################################################

loadlast = inifile.find('DISPLAY', 'LOAD_LASTFILE')
if loadlast == "YES" :
    load_lastfile = True
else:
    load_lastfile = False


lastfile = ""
recent = ap.getpref('recentfiles', [], repr)
if len(recent):
    lastfile = recent.pop(0)

code = []
addrecent = True
if args:
    initialfile = args[0]
elif "AXIS_OPEN_FILE" in os.environ:
    initialfile = os.environ["AXIS_OPEN_FILE"]
elif inifile.find("DISPLAY", "OPEN_FILE"):
    initialfile = inifile.find("DISPLAY", "OPEN_FILE")
elif os.path.exists(lastfile) and load_lastfile:
    initialfile = lastfile
    print ("Loading ") 
    print (initialfile)
elif lathe:
    initialfile = os.path.join(BASE, "share", "axis", "images","axis-lathe.ngc")
    addrecent = False
else:
    initialfile = os.path.join(BASE, "share", "axis", "images", "axis.ngc")
    addrecent = False

if os.path.exists(initialfile):
    open_file_guts(initialfile, False, addrecent)
